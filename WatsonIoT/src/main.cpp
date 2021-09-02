#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ETH.h>
#include <time.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Adxl355.h>            // forked from https://github.com/markrad/esp32-ADXL355
#include <math.h>
#include <esp_https_ota.h>
#include <esp_task_wdt.h>
#include <SPIFFS.h>
#include "config.h"
#include "dlcert.h"
#include "semver.h"             // from https://github.com/h2non/semver.c
#include <cppQueue.h>
#include "soc/soc.h"            // Enable/Disable BrownOut detection
#include "soc/rtc_cntl_reg.h"

// Watson IoT connection details
static char MQTT_HOST[48];            // ORGID.messaging.internetofthings.ibmcloud.com
static char MQTT_DEVICEID[30];        // Allocate a buffer large enough for "d:orgid:devicetype:deviceid"
static char MQTT_ORGID[7];            // Watson IoT 6 character orgid
#ifdef MQTT_LOCALBROKER
#define MQTT_PORT        1883         // Secure MQTT 8883 / Insecure MQTT 1883
#else
#define MQTT_PORT        8883         // Secure MQTT 8883 / Insecure MQTT 1883
#endif
#define MQTT_TOKEN       "OpenEEW-sens0r"   // Watson IoT DeviceId authentication token
#define MQTT_DEVICETYPE  "OpenEEW"    // Watson IoT DeviceType
#define MQTT_USER        "use-token-auth"
#define MQTT_TOPIC            "iot-2/evt/status/fmt/json"
#define MQTT_TOPIC_ALARM      "iot-2/cmd/earthquake/fmt/json"
#define MQTT_TOPIC_SAMPLERATE "iot-2/cmd/samplerate/fmt/json"
#define MQTT_TOPIC_FWCHECK    "iot-2/cmd/firmwarecheck/fmt/json"
#define MQTT_TOPIC_SEND10SEC  "iot-2/cmd/10secondhistory/fmt/json"
#define MQTT_TOPIC_SENDACCEL  "iot-2/cmd/sendacceldata/fmt/json"
#define MQTT_TOPIC_RESTART    "iot-2/cmd/forcerestart/fmt/json"
#define MQTT_TOPIC_THRESHOLD  "iot-2/cmd/threshold/fmt/json"
#define MQTT_TOPIC_FACTORYRST "iot-2/cmd/factoryreset/fmt/json"
char deviceID[13];

// Store the Download Server PEM and Digicert CA and Root CA in SPIFFS
// If an OTA firmware upgrade is required, the binary is downloaded from a secure server
#define BLUEMIX_CERT_PEM_FILE      "/mybluemix-net-chain.pem"
//#define DOWNLOAD_CERT_PEM_FILE   "/github-com-chain.pem"
#define DOWNLOAD_CERT_PEM_FILE     "/openeew-com-chain.pem"
#define WATSON_IOT_PLATFORM_CA_PEM "/messaging.pem"

// Timezone info
#define TZ_OFFSET -5  // (EST) Hours timezone offset to GMT (without daylight saving time)
#define TZ_DST    60  // Minutes timezone offset for Daylight saving

// MQTT objects
void callback(char* topic, byte* payload, unsigned int length);
#ifdef MQTT_LOCALBROKER
WiFiClient	 wifiClient;    // Insecure MQTT
#else
WiFiClientSecure wifiClient;    // Secure MQTT
#endif
PubSubClient mqtt(MQTT_HOST, MQTT_PORT, callback, wifiClient);

// Activation
bool OpenEEWDeviceActivation();
bool FirmwareVersionCheck( char *, String );
void PerformFirmwareOTA( String, const char *, uint );
void SetTimeESP32();
void SendLiveData2Cloud();
void Send10Seconds2Cloud();

// ETH_CLOCK_GPIO17_OUT - 50MHz clock from internal APLL inverted output on GPIO17 - tested with LAN8720
#ifdef ETH_CLK_MODE
#undef ETH_CLK_MODE
#endif
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#ifdef  PRODUCTION_BOARD
#define ETH_POWER_PIN    2  // Ethernet on production board
#else
#define ETH_POWER_PIN   -1  // Ethernet on prototype board
#endif
// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE        ETH_PHY_LAN8720
// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR        0
// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN     23
// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN    18

// Network variables
Preferences prefs;
String _ssid;    // your network SSID (name) - loaded from NVM
String _pswd;    // your network password    - loaded from NVM
int networksStored;
static bool bEthConnected  = false;
static bool bEthConnecting = false;
static bool bWiFiConnected  = false;
static bool bNetworkInterfaceChanged = false;

// --------------------------------------------------------------------------------------------
// ADXL Accelerometer
void IRAM_ATTR isr_adxl();

int32_t Adxl355SampleRate = 31;  // Reporting Sample Rate [31,125]

int8_t CHIP_SELECT_PIN_ADXL = 15;
#ifdef  PRODUCTION_BOARD
int8_t ADXL_INT_PIN = 35; // ADXL is on interrupt 35 on production board
#else
int8_t ADXL_INT_PIN = 2;  // ADXL is on interrupt 2 on prototype board
#endif
Adxl355::RANGE_VALUES range = Adxl355::RANGE_VALUES::RANGE_2G;
Adxl355::ODR_LPF odr_lpf;
Adxl355::STATUS_VALUES adxstatus;
Adxl355 adxl355(CHIP_SELECT_PIN_ADXL);
SPIClass *spi1 = NULL;

long fifoOut[32][3];
bool fifoFull = false;
int  fifoCount = 0;
int STA_len = 32;    // can change to 125
int LTA_len = 320;   // can change to 1250
int QUE_len = LTA_len + STA_len;

// --------------------------------------------------------------------------------------------
// Variables to hold accelerometer data
// 10 second FIFO queue for STA / LTA algorithm
typedef struct AccelXYZ {
  double x; double y; double z;
} AccelReading ;
cppQueue StaLtaQue( sizeof( AccelReading ), 352, FIFO );   // 11 seconds of Accelerometer data
uint32_t numSecsOfAccelReadings = 0;

// --------------------------------------------------------------------------------------------
// SmartConfig
int  numScannedNetworks();
int  numNetworksStored();
void readNetworkStored(int netId);
void storeNetwork(String ssid, String pswd);
void clearNetworks();
bool WiFiScanAndConnect();
bool startSmartConfig();

// --------------------------------------------------------------------------------------------
// NeoPixel LEDs
#include <Adafruit_NeoPixel.h>
#define LED_PIN 16
#define LED_COUNT 3
//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
void NeoPixelStatus( int );
void NeoPixelBreathe();
bool breathedirection = true;
int  breatheintensity = 1;

// Map the OpenEEW LED status colors to the Particle Photon status colors
#define LED_OFF           0
#define LED_CONNECTED     1 // Cyan breath
#define LED_FIRMWARE_OTA  2 // Magenta
#define LED_CONNECT_WIFI  3 // Green
#define LED_CONNECT_CLOUD 4 // Cyan fast
#define LED_LISTEN_WIFI   5 // Blue
#define LED_WIFI_OFF      6 // White
#define LED_SAFE_MODE     7 // Magenta breath
#define LED_FIRMWARE_DFU  8 // Yellow
#define LED_ERROR         9 // Red
#define LED_ORANGE       10 // Orange

// --------------------------------------------------------------------------------------------
// Buzzer Alarm
bool bStopEarthquakeAlarm = false;
void EarthquakeAlarm( int );
void AlarmBuzzer();
int freq = 4000;
int channel = 0;
int resolution = 8;
int io = 5;

// --------------------------------------------------------------------------------------------
// STA/LTA Algorithm globals
bool  bPossibleEarthQuake = false;
double        thresh  = 4.0;
double     stalta[3]  = { 0, 0, 0 };
double     sample[3]  = { 0, 0, 0 };
double  sampleSUM[3]  = { 0, 0, 0 };
double      ltSUM[3]  = { 0, 0, 0 };
double    sample1[3]  = { 0, 0, 0 };
double LTAsample1[3]  = { 0, 0, 0 };
double     offset[3]  = { 0, 0, 0 };
double  sampleABS[3]  = { 0, 0, 0 };
double    sample1ABS  = 0;
double LTAsample1ABS  = 0;
double       stav[3]  = { 0, 0, 0 };
double       ltav[3]  = { 0, 0, 0 };


// --------------------------------------------------------------------------------------------
void IRAM_ATTR isr_adxl() {
  fifoFull = true;
  //fifoCount++;
}


void StartADXL355() {
  // odr_lpf is a global
  adxl355.start();
  delay(1000);

  // Calibrating the ADXL355 can cause brownouts
  NeoPixelStatus( LED_OFF ); // turn off the LED to reduce power consumption
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  if (adxl355.isDeviceRecognized()) {
    Serial.println("Initializing sensor");
    adxl355.initializeSensor(range, odr_lpf, debug);
    Serial.println("Calibrating sensor");
    adxl355.calibrateSensor(20, debug); // This has been increased to make traces start closer to zero
    Serial.println("ADXL355 Accelerometer activated");

    bool bDiscardInitialADXLreadings = true ;
    while( bDiscardInitialADXLreadings ) {
      adxstatus = adxl355.getStatus();
      if (adxstatus & Adxl355::STATUS_VALUES::FIFO_FULL) {
        adxl355.readFifoEntries( (long *)fifoOut ) ;
        bDiscardInitialADXLreadings = false;
      }
    }
    Serial.println("ADXL355 Accelerometer first samples discarded");
  }
  else {
    Serial.println("Unable to get accelerometer");
  }
  Serial.println("Finished accelerometer configuration");
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1); //enable brownout detector
}


// Handle subscribed MQTT topics - Alerts and Sample Rate changes
void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<100> jsonMQTTReceiveDoc;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] : ");

  payload[length] = 0; // ensure valid content is zero terminated so can treat as c-string
  Serial.println((char *)payload);
  DeserializationError err = deserializeJson(jsonMQTTReceiveDoc, (char *)payload);
  if (err) {
    Serial.print(F("deserializeJson() failed with code : "));
    Serial.println(err.c_str());
  } else {
    JsonObject cmdData = jsonMQTTReceiveDoc.as<JsonObject>();
    if ( strcmp(topic, MQTT_TOPIC_ALARM) == 0 ) {
      // {Alarm:[true|test|false]}
      String AlarmType = cmdData["Alarm"].as<String>() ;
      Serial.println( "Alarm received: " + AlarmType );
      if ( AlarmType.equalsIgnoreCase("true") ) {
        // Sound the Buzzer & Blink the LED RED
        bStopEarthquakeAlarm = false;
        EarthquakeAlarm( LED_ERROR);
        bStopEarthquakeAlarm = false;
      } else if ( AlarmType.equalsIgnoreCase("test") ) {
        // Sound the Buzzer & Blink the LED ORANGE
        bStopEarthquakeAlarm = false;
        EarthquakeAlarm( LED_ORANGE );
        bStopEarthquakeAlarm = false;
      }else if ( AlarmType.equalsIgnoreCase("false") ) {
        bStopEarthquakeAlarm = true;
      }
    } else if ( strcmp(topic, MQTT_TOPIC_FWCHECK) == 0 ) {
      // Remote message received to check for new firmware
      // If a device is running for many months it might fall behind on the version of the
      // firmware it is running. As part of the ESP32 power up / activation process,
      // the board does a firmware version check. If there is a newer firmware version
      // it initiates an OTA firmware update. That only happens on startup.
      // A board that has been running for a long time might be stranded on an old version.
      // The administrator/device owner has asked the board to check for a firmware update.
      // That would let us dynamically update sensors if there was a security / software flaw.
      NeoPixelStatus( LED_FIRMWARE_OTA ); // Firmware OTA - Magenta
      OpenEEWDeviceActivation();
    } else if ( strcmp(topic, MQTT_TOPIC_SEND10SEC) == 0 ) {
      // Send 10 seconds of accelerometer history
      Serial.println("Send 10 seconds of accelerometer history to the cloud");
      Send10Seconds2Cloud() ;
    } else if ( strcmp(topic, MQTT_TOPIC_SENDACCEL) == 0 ) {
      // Start sending live accelometer data to the cloud. The payload asks for n seconds of data
      numSecsOfAccelReadings = cmdData["LiveDataDuration"].as<uint32_t>();
      Serial.print("Send live accelometer data to the cloud (secs):");
      Serial.println( numSecsOfAccelReadings );
    } else if ( strcmp(topic, MQTT_TOPIC_SAMPLERATE) == 0 ) {
      // Set the ADXL355 Sample Rate
      int32_t NewSampleRate = 0;
      bool    SampleRateChanged = false ;

      NewSampleRate = cmdData["SampleRate"].as<int32_t>(); // this form allows you specify the type of the data you want from the JSON object
      if( NewSampleRate == 31 ) {
        // Requested sample rate of 31 is valid
        Adxl355SampleRate = 31;
        SampleRateChanged = true;
        odr_lpf = Adxl355::ODR_LPF::ODR_31_25_AND_7_813;
      } else if ( NewSampleRate == 125 ) {
        // Requested sample rate of 125 is valid
        Adxl355SampleRate = 125;
        SampleRateChanged = true;
        odr_lpf = Adxl355::ODR_LPF::ODR_125_AND_31_25;
      } else if ( NewSampleRate == 0 ) {
        // Turn off the sensor ADXL
        Adxl355SampleRate = 0;
        SampleRateChanged = false; // false so the code below doesn't restart it
        Serial.println("Stopping the ADXL355");
        adxl355.stop();
        StaLtaQue.flush() ; // flush the Queue
        strip.clear();  // Off
        strip.show();
      } else {
        // invalid - leave the Sample Rate unchanged
      }

      Serial.print("ADXL355 Sample Rate has been changed:");
      Serial.println( Adxl355SampleRate );
      //SampleRateChanged = false;
      Serial.println( SampleRateChanged ) ;
      if( SampleRateChanged ) {
        Serial.println("Changing the ADXL355 Sample Rate");
        adxl355.stop();
        delay(1000);
        Serial.println("Restarting");
        StartADXL355();
        breatheintensity = 1;
        breathedirection = true;
      }
      jsonMQTTReceiveDoc.clear();
    } else if ( strcmp(topic, MQTT_TOPIC_THRESHOLD) == 0 ) {
      // Override the `thresh` global
      char newthreshmsg[50];
      snprintf( newthreshmsg, 49, "Previous STA/LTA Shake Threshold : %5.2f", thresh);
      Serial.println(newthreshmsg);
      thresh = cmdData["ThresholdOverride"].as<double>();
      snprintf( newthreshmsg, 49, "Override STA/LTA Shake Threshold : %5.2f", thresh);
      Serial.println(newthreshmsg);
    } else if ( strcmp(topic, MQTT_TOPIC_FACTORYRST) == 0 ) {
      // Remote message received to factory reset the device
      Serial.println("Remote message received to factory reset the device.");
      clearNetworks();
      Serial.println("Restarting Device...");
      esp_restart();
    } else if ( strcmp(topic, MQTT_TOPIC_RESTART) == 0 ) {
      Serial.println("Restarting Device...");
      esp_restart();
    } else {
      Serial.println("Unknown command received");
    }
  }
}


void PerformFirmwareOTA( String firmware_ota_url, const char *DownloadServerPemChain, uint pemSize ) {
  Serial.write((const unsigned char*)DownloadServerPemChain,pemSize);

  // Increase the watchdog timer before starting the firmware upgrade
  // The download and write can trip the watchdog timer and the old firmware
  // will abort / reset before the new firmware is complete.
  esp_task_wdt_init(15,0);
  NeoPixelStatus( LED_OFF ); // turn off the LED to reduce power consumption
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.println("Starting OpenEEW OTA firmware upgrade...");
  esp_http_client_config_t config = {0};
  config.url = firmware_ota_url.c_str() ;
  config.cert_pem = DownloadServerPemChain ;
  esp_err_t ret = esp_https_ota(&config);
  if (ret == ESP_OK) {
      Serial.println("OTA upgrade downloaded. Restarting...");
      WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1); //enable brownout detector
      esp_restart();
  } else {
      esp_task_wdt_init(5,0);
      Serial.println("The OpenEEW OTA firmware upgrade failed : ESP_FAIL");
  }
}


bool FirmwareVersionCheck( char *firmware_latest, String firmware_ota_url ) {
  semver_t current_version = {};
  semver_t latest_version = {};
  char VersionCheck[55];
  bool bFirmwareUpdateRequiredOTA = false;

  if (semver_parse(OPENEEW_FIRMWARE_VERSION, &current_version)
    || semver_parse(firmware_latest, &latest_version)) {
    Serial.println("Invalid semver string");
    return false;
  }

  int resolution = semver_compare(latest_version, current_version);

  if (resolution == 0) {
    snprintf(VersionCheck,54,"Version %s is equal to: %s", firmware_latest, OPENEEW_FIRMWARE_VERSION);
  }
  else if (resolution == -1) {
    snprintf(VersionCheck,54,"Version %s is lower than: %s", firmware_latest, OPENEEW_FIRMWARE_VERSION);
  }
  else {
    snprintf(VersionCheck,54,"Version %s is higher than: %s", firmware_latest, OPENEEW_FIRMWARE_VERSION);
    bFirmwareUpdateRequiredOTA = true;
  }
  Serial.println(VersionCheck);

  if( bFirmwareUpdateRequiredOTA ) {
    // OTA upgrade is required
    Serial.println("An OTA upgrade is required. Download the new OpenEEW firmware :");
    Serial.println(firmware_ota_url);
    // Launch an OTA upgrade
    NeoPixelStatus( LED_FIRMWARE_OTA ); // blink magenta

    if( SPIFFS.begin(false) ) {
      char DownloadServerCert[30];
      if( firmware_ota_url.indexOf("mybluemix.net") >=0 ) {
        strncpy(DownloadServerCert, BLUEMIX_CERT_PEM_FILE,  30);
      } else {
        strncpy(DownloadServerCert, DOWNLOAD_CERT_PEM_FILE, 30);
      }
      Serial.printf("Opening Server PEM Chain : %s\r\n", DownloadServerCert);
      if( SPIFFS.exists( DownloadServerCert )) {
        File pemfile = SPIFFS.open( DownloadServerCert );
        if( pemfile ) {
          char *DownloadServerPemChain = nullptr;
          size_t pemSize = pemfile.size();
          DownloadServerPemChain = (char *)malloc(pemSize);
          if( pemSize != pemfile.readBytes(DownloadServerPemChain, pemSize) ) {
            Serial.printf("Reading %s pem server certificate chain failed.\r\n",DownloadServerCert);
            Serial.println("Failback to inline download server certificate.");
            PerformFirmwareOTA( firmware_ota_url, DownLoadOpenEEWPem, sizeof(DownLoadOpenEEWPem));
          } else {
            Serial.printf("Read %s pem server certificate chain from SPIFFS\r\n",DownloadServerCert);
            PerformFirmwareOTA( firmware_ota_url, DownloadServerPemChain, pemSize);
          }
          free( DownloadServerPemChain );
        } else {
          Serial.println("Failed to open server pem chain.");
          Serial.println("Failback to inline download server certificate.");
          PerformFirmwareOTA( firmware_ota_url, DownLoadOpenEEWPem, sizeof(DownLoadOpenEEWPem));
        }
        pemfile.close();
      } else {
        Serial.printf("The %s pem server certificate file does not exist.\r\n",DownloadServerCert);
        Serial.println("The SPIFFS filesystem might be empty.");
        Serial.println("Failback to inline download server certificate.");
        PerformFirmwareOTA( firmware_ota_url, DownLoadOpenEEWPem, sizeof(DownLoadOpenEEWPem));
      }
    } else {
      Serial.println("An error has occurred while mounting SPIFFS");
      Serial.println("Failback to inline download server certificate.");
      PerformFirmwareOTA( firmware_ota_url, DownLoadOpenEEWPem, sizeof(DownLoadOpenEEWPem));
    }
  }
  // Free allocated memory when we're done
  semver_free(&current_version);
  semver_free(&latest_version);
  return true;
}


// Call the OpenEEW Device Activation endpoint to retrieve MQTT OrgID
bool OpenEEWDeviceActivation() {
  // OPENEEW_ACTIVATION_ENDPOINT "https://device-mgmt.openeew.com/activation?ver=1"
  // $ curl -i  -X POST -d '{"macaddress":"112233445566","firmware_device":"1.0.0"}'
  //    -H "Content-type: application/JSON" https://device-mgmt.openeew.com/activation?ver=1
  Serial.println("Contacting the OpenEEW Device Activation Endpoint :");
  Serial.println(OPENEEW_ACTIVATION_ENDPOINT);

  HTTPClient http;
  // Domain name with URL path or IP address with path
  http.begin( OPENEEW_ACTIVATION_ENDPOINT );

   // HTTP request with a content type: application/json
  http.addHeader("Content-Type", "application/json");

  // Construct the serialized http request body
  // '{"macaddress":"112233445566","lat":40.00000,"lng":-74.00000,"firmware_device":"1.0.0"}'
  DynamicJsonDocument httpSendDoc(120);
  String httpRequestData;
  httpSendDoc["macaddress"]      = deviceID;
  httpSendDoc["firmware_device"] = OPENEEW_FIRMWARE_VERSION;
  // Serialize the entire string to be transmitted
  serializeJson(httpSendDoc, httpRequestData);
  Serial.print("Sending Device Activation : ");
  Serial.println(httpRequestData);

  int httpResponseCode = http.POST(httpRequestData);

  Serial.print("HTTP Response code: ");
  Serial.println(httpResponseCode);
  if( httpResponseCode == 200 ) {  // Success
    // Get the response payload
    // Ex {"org":"5yrusp","firmware_latest":"1.1.0","firmware_ota_url":"https://download.firmware.com/openeew.bin"}
    String payload = http.getString();
    Serial.print("HTTP post response payload: ");
    Serial.println( payload );
    http.end();  // free resources

    DynamicJsonDocument ReceiveDoc(200);
    DeserializationError err = deserializeJson(ReceiveDoc, payload );
    if (err) {
      Serial.print(F("deserializeJson() failed with code : "));
      Serial.println(err.c_str());
      return false;
    } else {
      JsonObject ActivationData = ReceiveDoc.as<JsonObject>();
      char firmware_latest[20];  // Hold a version string as long as 10.0.0-alpha.beta
      String firmware_ota_url;

      strncpy(MQTT_ORGID, ActivationData["org"], sizeof(MQTT_ORGID) );
      Serial.print("OpenEEW Device Activation directs MQTT data from this sensor to : ");
      Serial.println(MQTT_ORGID);

      strncpy(firmware_latest, ActivationData["firmware_latest"], sizeof(firmware_latest) );
      firmware_ota_url = ActivationData["firmware_ota_url"].as<String>();
      FirmwareVersionCheck(firmware_latest, firmware_ota_url);
    }
    return true ;
  } else {        // Failed to successfully contact endpoint
    http.end();   // free resources
    Serial.println("Device Activation failed. Waiting...");
    return false;
  }
}


void Connect2MQTTbroker() {
  if( bNetworkInterfaceChanged ) {
    mqtt.disconnect();
    bNetworkInterfaceChanged = false;
  }
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    NeoPixelStatus( LED_CONNECT_CLOUD ); // blink cyan
    // Attempt to connect / re-connect to IBM Watson IoT Platform
    // These params are globals assigned in setup()
    if( mqtt.connect(MQTT_DEVICEID, MQTT_USER, MQTT_TOKEN) ) {
  //if( mqtt.connect(MQTT_DEVICEID) ) { // No Token Authentication
      Serial.println("MQTT Connected");
      mqtt.subscribe(MQTT_TOPIC_ALARM);
      mqtt.subscribe(MQTT_TOPIC_SAMPLERATE);
      mqtt.subscribe(MQTT_TOPIC_FWCHECK);
      mqtt.subscribe(MQTT_TOPIC_SEND10SEC);
      mqtt.subscribe(MQTT_TOPIC_SENDACCEL);
      mqtt.subscribe(MQTT_TOPIC_RESTART);
      mqtt.subscribe(MQTT_TOPIC_THRESHOLD);
      mqtt.subscribe(MQTT_TOPIC_FACTORYRST);
      mqtt.setBufferSize(2000);
      mqtt.loop();
    } else {
      Serial.println("MQTT Failed to connect!");
      delay(5000);
    }
  }
}


template<typename L> void loadFromFile(const char* fname, L&& load) {
  if (SPIFFS.exists(fname)) {
    File f = SPIFFS.open(fname);
    load(f, f.size());
    f.close();
  } else {
    Serial.printf("The certificate does not exist: %s\r\n",fname);
  }
}


void loadCertificates(WiFiClientSecure* client) {
  SPIFFS.begin();
  loadFromFile(WATSON_IOT_PLATFORM_CA_PEM, [client](Stream& stream, size_t size){return client->loadCACert(stream, size);});
  //loadFromFile("/client.cert.pem", [client](Stream& stream, size_t size){return client->loadCertificate(stream, size);});
  //loadFromFile("/private.key.pem", [client](Stream& stream, size_t size){return client->loadPrivateKey(stream, size);});
  SPIFFS.end();
}


void Send10Seconds2Cloud() {
  // DynamicJsonDocument is stored on the heap
  // Allocate a ArduinoJson buffer large enough to 10 seconds of Accelerometer trace data
  DynamicJsonDocument historydoc(16384);
  JsonObject payload      = historydoc.to<JsonObject>();
  JsonArray  alltraces    = payload.createNestedArray("traces");
  JsonObject acceleration = alltraces.createNestedObject();

  // Load the key/value pairs into the serialized ArduinoJSON format
  payload["device_id"] = deviceID ;
  payload["device_t"]  = time(nullptr);

  // Generate an array of json objects that contain x,y,z arrays of 32 floats.
  // [{"x":[],"y":[],"z":[]},{"x":[],"y":[],"z":[]}]
  AccelReading AccelRecord ;
  for( uint16_t idx=0; idx < StaLtaQue.getCount(); idx++ ) {
    if( StaLtaQue.peekIdx( &AccelRecord, idx) ) {
      //char reading[75];
      //snprintf( reading, 74, "[ x=%3.3f , y=%3.3f , z=%3.3f ]", AccelRecord.x, AccelRecord.y, AccelRecord.z);
      //Serial.println(reading);

      acceleration["x"].add(AccelRecord.x);
      acceleration["y"].add(AccelRecord.y);
      acceleration["z"].add(AccelRecord.z);
    }
  }

  // Serialize the History Json object into a string to be transmitted
  //serializeJson(historydoc,Serial);  // print to console
  static char historymsg[16384];
  serializeJson(historydoc, historymsg, 16383);

  int jsonSize = measureJson(historydoc);
  Serial.print("Sending 10 seconds of accelerometer readings in a MQTT packet of size: ");
  Serial.println( jsonSize );
  mqtt.setBufferSize( (jsonSize + 50 ));  // increase the MQTT buffer size

  // Publish the message to MQTT Broker
  if (!mqtt.publish(MQTT_TOPIC, historymsg)) {
    Serial.println("MQTT Publish failed");
  } else {
    NeoPixelStatus( LED_CONNECTED ); // Success - blink cyan
  }

  mqtt.setBufferSize( 2000 );  // reset the MQTT buffer size
  historydoc.clear();
}


void SendLiveData2Cloud() {
  // variables to hold accelerometer data
  // DynamicJsonDocument is stored on the heap
  DynamicJsonDocument jsonDoc(3000);
  JsonObject payload      = jsonDoc.to<JsonObject>();
  JsonArray  traces       = payload.createNestedArray("traces");
  JsonObject acceleration = traces.createNestedObject();

  // Load the key/value pairs into the serialized ArduinoJSON format
  payload["device_id"] = deviceID;
  payload["device_t"]  = time(nullptr);

  // Generate an array of json objects that contain x,y,z arrays of 32 floats.
  // [{"x":[],"y":[],"z":[]},{"x":[],"y":[],"z":[]}]
  AccelReading AccelRecord ;
  // Send the last 32 records (or less) from the queue
  uint16_t idx = StaLtaQue.getCount() ;
  if( idx >= 32 ) {
    idx = idx - 32;
  }
  for( ; idx < StaLtaQue.getCount(); idx++ ) {
    if( StaLtaQue.peekIdx( &AccelRecord, idx) ) {
      //char reading[75];
      //snprintf( reading, 74, "[ x=%3.3f , y=%3.3f , z=%3.3f ]", AccelRecord.x, AccelRecord.y, AccelRecord.z);
      //Serial.println(reading);

      acceleration["x"].add(AccelRecord.x);
      acceleration["y"].add(AccelRecord.y);
      acceleration["z"].add(AccelRecord.z);
    }
  }

  // Serialize the current second Json object into a string to be transmitted
  static char msg[2000];
  serializeJson(jsonDoc, msg, 2000);
  Serial.println(msg);

  int jsonSize = measureJson(jsonDoc);
  Serial.print("Sending 1 second of accelerometer readings in a MQTT packet of size: ");
  Serial.println( jsonSize );
  mqtt.setBufferSize( (jsonSize + 50 ));  // increase the MQTT buffer size

  // Publish the message to MQTT Broker
  if (!mqtt.publish(MQTT_TOPIC, msg)) {
    Serial.println("MQTT Publish failed");
  } else {
    NeoPixelStatus( LED_CONNECTED ); // Success - blink cyan
  }

  mqtt.setBufferSize( 2000 );  // reset the MQTT buffer size
  jsonDoc.clear();
}


void NetworkEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_WIFI_READY:    // 0
      Serial.println("ESP32 WiFi interface ready");
      break;
    case SYSTEM_EVENT_STA_START:     // 2
      Serial.println("ESP32 WiFi started");
      WiFi.setHostname("openeew-sensor-wifi");
      break;
    case SYSTEM_EVENT_SCAN_DONE:
      Serial.println("Completed scan for access points");
      break;
    case SYSTEM_EVENT_STA_CONNECTED: // 4
      Serial.println("ESP32 WiFi connected to AP");
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi access point");
      bWiFiConnected = false;
      break;
    case SYSTEM_EVENT_STA_GOT_IP:    // 7
      Serial.println("ESP32 station got IP from connected AP");
      Serial.print("Obtained IP address: ");
      Serial.println( WiFi.localIP() );
      if( bEthConnected ) {
        Serial.println("Ethernet is already connected");
      }
      break;
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth / wifi hostname here
      ETH.setHostname( "openeew-sensor-eth" );
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      bEthConnecting = true;
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      bEthConnected = true;

      // Switch the MQTT connection to Ethernet from WiFi (or initially)
      // Preference the Ethernet wired interface if its available
      // Disconnect the MQTT session
      if( mqtt.connected() ){
        Serial.println("Previously connected to WiFi, try to switch the MQTT connection to Ethernet");
        bNetworkInterfaceChanged = true;
        // No need to call mqtt.setClient(ETH); because ETH is a ETHClient which is not the same class as WiFi client
        // Connect2MQTTbroker(); // The MQTT reconnect will be handled by the main loop()
      }
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      bEthConnected = false;
      // Disconnect the MQTT client
      if( mqtt.connected() ){
        Serial.println("Previously connected to Ethernet, try to switch the MQTT connection to WiFi");
        bNetworkInterfaceChanged = true;
      }
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      bEthConnected = false;
      break;
    case SYSTEM_EVENT_STA_STOP:
      Serial.println("WiFi Stopped");
      NeoPixelStatus( LED_WIFI_OFF ); // White
      break;
    case SYSTEM_EVENT_AP_STOP:
      Serial.println("ESP32 soft-AP stop");
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      Serial.println("a station connected to ESP32 soft-AP");
      break;
    default:
      Serial.print("Unhandled Network Interface event : ");
      Serial.println(event);
      break;
  }
}


time_t periodic_timesync;
// MQTT SSL requires a relatively accurate time between broker and client
void SetTimeESP32() {
  time_t now = time(nullptr);
  Serial.print("Before time sync: ");
  Serial.print(ctime(&now));

  // Set time from NTP servers
  configTime(TZ_OFFSET * 3600, TZ_DST * 60, "time.nist.gov", "pool.ntp.org");
  Serial.print("Waiting for time");
  while(time(nullptr) <= 100000) {
    NeoPixelStatus( LED_FIRMWARE_DFU ); // blink yellow
    Serial.print(".");
    delay(100);
  }
  unsigned timeout = 5000;
  unsigned start = millis();
  while (millis() - start < timeout) {
      now = time(nullptr);
      if (now > (2019 - 1970) * 365 * 24 * 3600) {
          break;
      }
      delay(100);
  }
  delay(1000); // Wait for time to fully sync

  Serial.print("\nAfter time sync : ");
  now = time(nullptr);
  Serial.print(ctime(&now));
  periodic_timesync = now;     // periodically resync the time to prevent drift
}


void setup() {
  // Start serial console
  Serial.begin(115200);
  Serial.setTimeout(2000);
  while (!Serial) { }
  Serial.println();
  Serial.println("OpenEEW Sensor Application");

  // Starting the ESP with the LEDs on can cause brownouts
  NeoPixelStatus( LED_OFF ); // turn off the LED to reduce power consumption
  strip.setBrightness(50);  // Dim the LED to 20% - 0 off, 255 full bright

  // Start Network connections
  WiFi.onEvent(NetworkEvent);

  // Start the ETH interface, if it is available, before WiFi
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
  delay(5000);
  if( bEthConnecting ) {
    uint EthTimeOut = 20;
    while( !bEthConnected && EthTimeOut ) {
      Serial.println("Waiting for Ethernet to start...");
      // Wait for 10 seconds for Ethernet to start
      // ESP32 does not work with some older 10/100 Ethernet switches
      EthTimeOut--;
      delay( 500 );
    }
    if( EthTimeOut == 0 ) {
      Serial.println("Ethernet interface did not return an IP address within 10 seconds. Skipping.");
    }
  }

  WiFi.mode(WIFI_STA);
  bWiFiConnected = WiFiScanAndConnect();
  if( !bWiFiConnected )  {
    // If the sensor has been registered in the past
    // at least one WiFi network will have been stored in NVM
    // and if the Ethernet cable is connected, do not
    // loop in SmartConfig, just use the hardwired connection.
    if( numNetworksStored() && bEthConnected ) {
      Serial.println("Previously registered device, use hardwired Ethernet connection.");
    } else {
      while( !startSmartConfig() ) {
        // loop in SmartConfig until the user provides
        // the correct WiFi SSID and password
      }
    }
  } else {
    Serial.println("WiFi Connected");
  }

  byte mac[6];                     // the MAC address of your Wifi shield
  WiFi.macAddress(mac);

  // Output this ESP32 Unique WiFi MAC Address
  Serial.print("WiFi MAC: ");
  Serial.println(WiFi.macAddress());

  // Use the reverse octet Mac Address as the MQTT deviceID
  //snprintf(deviceID,13,"%02X%02X%02X%02X%02X%02X",mac[5],mac[4],mac[3],mac[2],mac[1],mac[0]);
  snprintf(deviceID,13,"%02X%02X%02X%02X%02X%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.println(deviceID);

  // Set the time on the ESP32
  SetTimeESP32();

  // Call the Activation endpoint to retrieve this OpenEEW Sensor details
  while( ! OpenEEWDeviceActivation() ) {
    // Loop forever, waiting for activation success
    NeoPixelStatus( LED_CONNECT_CLOUD ); // blink Cyan
    delay(10000);
  }

  // Dynamically build the MQTT Device ID from the Mac Address of this ESP32
  // MQTT_ORGID was retreived by the OpenEEWDeviceActivation() function
  //snprintf(MQTT_DEVICEID,30,"d:%s:%s:%02X%02X%02X%02X%02X%02X",MQTT_ORGID,MQTT_DEVICETYPE,mac[5],mac[4],mac[3],mac[2],mac[1],mac[0]);
  snprintf(MQTT_DEVICEID,30,"d:%s:%s:%02X%02X%02X%02X%02X%02X",MQTT_ORGID,MQTT_DEVICETYPE,mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.println(MQTT_DEVICEID);

#ifdef MQTT_LOCALBROKER
  snprintf(MQTT_HOST,48,MQTT_LOCALBROKER);  // Enter the IP address of the MQTT broker on your local subnet
#else
  snprintf(MQTT_HOST,48,"%s.messaging.internetofthings.ibmcloud.com",MQTT_ORGID);  // Centrally managed

  loadCertificates( &wifiClient );      // Load the Watson IoT messaging.pem CA Cert from SPIFFS
#endif

  char mqttparams[100]; // Allocate a buffer large enough for this string ~95 chars
  snprintf(mqttparams, 99, "MQTT_USER:%s  MQTT_TOKEN:%s  MQTT_DEVICEID:%s", MQTT_USER, MQTT_TOKEN, MQTT_DEVICEID);
  Serial.println(mqttparams);

#if OPENEEW_SAMPLE_RATE_125
  odr_lpf = Adxl355::ODR_LPF::ODR_125_AND_31_25;
#endif

#if OPENEEW_SAMPLE_RATE_31_25
  odr_lpf = Adxl355::ODR_LPF::ODR_31_25_AND_7_813;
#endif

  pinMode(ADXL_INT_PIN, INPUT);
  pinMode(CHIP_SELECT_PIN_ADXL, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ADXL_INT_PIN), isr_adxl, FALLING);

  spi1 = new SPIClass(HSPI);
  adxl355.initSPI(*spi1);
  StartADXL355();

  ledcSetup(channel, freq, resolution);
  ledcAttachPin(io, channel);
  pinMode(io, OUTPUT);
  digitalWrite(io, LOW); // turn off buzzer

  // Connect to MQTT - IBM Watson IoT Platform
  Connect2MQTTbroker();
}


void loop() {
  mqtt.loop();
  // Confirm Connection to MQTT - IBM Watson IoT Platform
  Connect2MQTTbroker();

  //====================== ADXL Accelerometer =====================
  if (fifoFull)  {
    fifoFull = false;
    adxstatus = adxl355.getStatus();

    if (adxstatus & Adxl355::STATUS_VALUES::FIFO_FULL) {
      // Keep track of the heap in case heap fragmentation returns
      //Serial.println( xPortGetFreeHeapSize() );
      int numEntriesFifo = adxl355.readFifoEntries( (long *)fifoOut ) ;
      if ( numEntriesFifo != -1 ) {
        // Declare one AccelReading structure for this iteration of loop()
        // so it doesn't need to go in and out of scope in various for() loops below
        //   typedef struct AccelXYZ {
        //     double x; double y; double z;
        //   } AccelReading ;
        AccelReading AccelRecord;

        // [{"x":[9.479,0],"y":[0.128,-1.113],"z":[-0.185,123.321]},{"x":[9.479,0],"y":[0.128,-1.113],"z":[-0.185,123.321]}]
        double gal;
        double x, y, z;
        for (int i = 0; i < numEntriesFifo; i++) {
          gal = adxl355.valueToGals(fifoOut[i][0]);
          x = round(gal*1000)/1000;
          AccelRecord.x = x;

          gal = adxl355.valueToGals(fifoOut[i][1]);
          y = round(gal*1000)/1000;
          AccelRecord.y = y;

          gal = adxl355.valueToGals(fifoOut[i][2]);
          z = round(gal*1000)/1000;
          AccelRecord.z = z;

          StaLtaQue.push(&AccelRecord);
        }

        // Do some STA / LTA math here...
        char mathmsg[65];
        snprintf(mathmsg, 64, "Calculating STA/LTA from %d accelerometer readings", StaLtaQue.getCount());
        //Serial.println(mathmsg);
        if( StaLtaQue.isFull() ) {
          /////////////////// find offset ////////////////
          int queCount = StaLtaQue.getCount( );

          for (int idx = 0; idx < queCount; idx++) {
            if( StaLtaQue.peekIdx( &AccelRecord, idx) ) {
              sample[0] = AccelRecord.x;
              sample[1] = AccelRecord.y;
              sample[2] = AccelRecord.z;
              for (int j = 0; j < 3; j++) {
                sampleSUM[j] += sample[j];
              }
            }
          }
          for (int j = 0; j < 3; j++) {
            offset[j]  = sampleSUM[j] / (QUE_len);
          }

          /////////////////// find lta /////////////////
          sampleSUM[0] = 0;
          sampleSUM[1] = 0;
          sampleSUM[2] = 0;
          for (int idx = 0; idx < LTA_len; idx++) {
            if( StaLtaQue.peekIdx( &AccelRecord, idx) ) {
              sampleABS[0] = abs( AccelRecord.x - offset[0] );
              sampleABS[1] = abs( AccelRecord.y - offset[1] );
              sampleABS[2] = abs( AccelRecord.z - offset[2] );
              for (int j = 0; j < 3; j++) {
                sampleSUM[j] += sampleABS[j];
              }
            }
          }
          for (int j = 0; j < 3; j++) {
            ltav[j]  = sampleSUM[j] / (LTA_len);
          }

          //////////////////// find sta ///////////////////////
          sampleSUM[0] = 0;
          sampleSUM[1] = 0;
          sampleSUM[2] = 0;
          for (int idx = LTA_len-STA_len ; idx < LTA_len; idx++) {
            if( StaLtaQue.peekIdx( &AccelRecord, idx) ) {
              sampleABS[0] = abs( AccelRecord.x - offset[0] );
              sampleABS[1] = abs( AccelRecord.y - offset[1] );
              sampleABS[2] = abs( AccelRecord.z - offset[2] );
              for (int j = 0; j < 3; j++) {
                sampleSUM[j] += sampleABS[j];
              }
            }
          }
          for (int j = 0; j < 3; j++) {
            stav[j]    = sampleSUM[j] / STA_len;
            stalta[j]  =      stav[j] / ltav[j];
            if ( bPossibleEarthQuake==false ) {
              if ( stalta[j] >= thresh ) {
                // Whoa - STA/LTA algorithm detected some anomalous shaking
                Serial.printf("STA/LTA = %f = %f / %f (%i)\n", stalta[j], stav[j], ltav[j], j );
                bPossibleEarthQuake = true ;
              }
            }
          }

          //// find STA/LTA for the other 31 samples but without doing the summing again

          for (int idx = LTA_len+1; idx < QUE_len; idx++) {
            if( StaLtaQue.peekIdx( &AccelRecord, idx) ) {
              sample[0] = AccelRecord.x;
              sample[1] = AccelRecord.y;
              sample[2] = AccelRecord.z;
            }
            if( StaLtaQue.peekIdx( &AccelRecord, idx-STA_len) ) {
              sample1[0] = AccelRecord.x;
              sample1[1] = AccelRecord.y;
              sample1[2] = AccelRecord.z;
            }
            if( StaLtaQue.peekIdx( &AccelRecord, idx-LTA_len) ) {
              LTAsample1[0] = AccelRecord.x;
              LTAsample1[1] = AccelRecord.y;
              LTAsample1[2] = AccelRecord.z;
            }
            for (int j = 0; j < 3; j++) {
              sampleABS[j]  = abs(sample[j]     - offset[j]);
              sample1ABS    = abs(sample1[j]    - offset[j]);
              LTAsample1ABS = abs(LTAsample1[j] - offset[j]);
              stav[j]   += ( sampleABS[j] - sample1ABS)    /  STA_len;
              ltav[j]   += ( sampleABS[j] - LTAsample1ABS) /  LTA_len;
              stalta[j]  = stav[j] / ltav[j];
              if ( bPossibleEarthQuake==false ) {
                if ( stalta[j] >= thresh ) {
                  // Whoa - STA/LTA algorithm detected some anomalous shaking
                  Serial.printf("STA/LTA = %f = %f / %f (%i)\n", stalta[j], stav[j], ltav[j], j );
                  bPossibleEarthQuake = true ;
                }
              }
            }
          }
        }

        if( numSecsOfAccelReadings > 0 ) {
          SendLiveData2Cloud();
          numSecsOfAccelReadings-- ;
          bPossibleEarthQuake=false;
        } else if( bPossibleEarthQuake ) {
          // The STA/LTA algorithm detected some anomalous shaking
          // If this is continued shaking, the above SendLiveData2Cloud()
          // function has already sent current accelerometer data
          // so don't send it again.
          bPossibleEarthQuake=false;

          // Start sending 5 minutes of live accelerometer data
          Serial.println("Start sending 5 minutes of live accelerometer data");
          numSecsOfAccelReadings = 300 ;

          // Send the previous 10 seconds of history to the cloud
          Send10Seconds2Cloud();
        }

        // When this loop is done, drop 32 records off the queue
        if( StaLtaQue.isFull() ) {
          for( int i=0; i < 32; i++ )
            StaLtaQue.drop();
        }

        //Switch the direction of the LEDs
        breathedirection = breathedirection ? false : true;
      }

    // Keep track of the heap in case heap fragmentation returns
    //Serial.println( xPortGetFreeHeapSize() );
    }
  }

  if( adxstatus )
    NeoPixelBreathe();

  if( (time(nullptr) - periodic_timesync) > RESYNCTIME ) {
    // Resync the ESP32 time once a day so that MQTT and Seismology time is accurate
    SetTimeESP32();
  }

  delay(100);
}


//================================= WiFi Handling ================================
//Scan networks in range and return how many are they.
int numScannedNetworks() {
  int n = WiFi.scanNetworks();
  Serial.println("WiFi Network scan done");
  if (n == 0)  {
    Serial.println("No networks found");
  }
  else {
    Serial.print(n);
    Serial.println(" network(s) found");
//#if LOG_L2
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
      delay(10);
    }
//#endif
  }
  return n;
}


//Return how many networks are stored in the NVM
int numNetworksStored() {
  prefs.begin("networks", true);
  networksStored = prefs.getInt("num_nets");
  Serial.print("Stored networks : ");
  Serial.println(networksStored);
  prefs.end();

  return networksStored;
}


//Each network as an id so reading the network stored with said ID.
void readNetworkStored(int netId)
{
  Serial.println("Reading stored networks from NVM");

  prefs.begin("networks", true);
  String idx;
  idx = "SSID" + (String)netId;
  _ssid = prefs.getString(idx.c_str(), "");
  idx = "key" + (String)netId;
  _pswd = prefs.getString(idx.c_str(), "");
  prefs.end();

  Serial.print("Found network ");
  Serial.print(_ssid);
  Serial.print(" , ");
  DEBUG_L2(_pswd);  // off by default
  Serial.println("xxxxxx");
}


//Save a pair of SSID and PSWD to NVM
void storeNetwork(String ssid, String pswd)
{
  Serial.print("Writing network to NVM: ");
  Serial.print(ssid);
  Serial.print(",");
  Serial.println(pswd);

  prefs.begin("networks", false);
  int aux_num_nets = prefs.getInt("num_nets");
  Serial.print("Stored networks in NVM: ");
  Serial.println(aux_num_nets);
  aux_num_nets++;
  String idx;
  idx = "SSID" + (String)aux_num_nets;
  prefs.putString(idx.c_str(), ssid);
  idx = "key" + (String)aux_num_nets;
  prefs.putString(idx.c_str(), pswd);
  prefs.putInt("num_nets", aux_num_nets);
  prefs.end();
  Serial.print("Device has ");
  Serial.print(aux_num_nets);
  Serial.println(" networks stored in NVM");
}


//Clear all networks stored in the NVM, force SmartConfig
void clearNetworks() {
  Serial.println("Clear all stored networks from NVM");
  prefs.begin("networks", false);
  if( prefs.clear() ) {
    Serial.println("All networks have been erased");
  } else {
    Serial.println("Failed to clear WiFi networks");
  }
  prefs.end();
}


//Joins the previous functions, gets the stored networks and compares to the available, if there is a match and connects, return true
//if no match or unable to connect, return false.
bool WiFiScanAndConnect()
{
  int num_nets = numNetworksStored();
  int num_scan = numScannedNetworks();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  for (int i = 1; i < (num_nets + 1); i++) {
    readNetworkStored(i);

    for (int j = 0; j < num_scan; j++) {
      if (_ssid == WiFi.SSID(j)) {
        //Serial.print("Status from connection attempt");
        WiFi.begin(_ssid.c_str(), _pswd.c_str());
        WiFi.setSleep(false);
        unsigned long t0 = millis();

        while (WiFi.status() != WL_CONNECTED && (millis() - t0) < CONNECTION_TO) {
          NeoPixelStatus( LED_LISTEN_WIFI ); // blink blue
          delay(1000);
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("WiFi was successfully connected");
          return true;
        }
        else {
          Serial.println("There was a problem connecting to WiFi");
        }
      }
      else {
        Serial.println("Got no match for network");
      }
    }
  }
  Serial.println("Found no matches for saved networks");
  return false;
}


// Executes the Smart config routine and if connected, will save the network for future use
bool startSmartConfig()
{
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP_STA);
  WiFi.beginSmartConfig();
  int RouterDownTimeOut = 0;

  // Wait for SmartConfig packet from mobile
  Serial.print("Waiting for SmartConfig or Router to restart" );
  while( !WiFi.smartConfigDone() ) {
    delay(500);
    Serial.print("Waiting for SmartConfig or WiFi Router to recover (10 minutes ~ 650 ticks): " );
    Serial.println( RouterDownTimeOut );
    NeoPixelStatus( LED_LISTEN_WIFI );  // blink blue
    if( RouterDownTimeOut < 650 ) {
      RouterDownTimeOut++;
    } else if( !numNetworksStored() ) {
      // There are no stored WiFi network credentials, keep waiting
      RouterDownTimeOut = 0;
    } else {
      // Maybe the router was down and this device normally connects to a network.
      // If there's a power outage in the building and power is restored, the device comes up first/fast.
      // The wifi router might not yet be available. The device goes into SmartConfig polling mode and never recovers.
      // Eventually the network router comes up but the device is stuck because its looping here.
      // A restart 10 minutes later would bring it back to normal.
      // This could be a common problem in the field. Don't dispatch someone just to power cycle the sensor.
      Serial.println("Router Recovery? Restart OpenEEW device to retry saved networks");
      esp_restart();
    }

    if( bEthConnected ) {
      // Ethernet cable was connected during SmartConfig
      if( numNetworksStored() ) {
        Serial.println("Previously registered device, Skip SmartConfig, Use hardwired Ethernet connection.");
        // Skip SmartConfig
        WiFi.stopSmartConfig();
        return true;
      }
    }
  }

  Serial.println("SmartConfig received.");
  for( int i=0;i<4;i++){
    delay(500);
    NeoPixelStatus( LED_CONNECT_WIFI ); // Success - blink green
  }

  // Wait for WiFi to connect to AP
  Serial.println("Waiting for WiFi");
  unsigned long t0 = millis();
  while( WiFi.status() != WL_CONNECTED && (millis() - t0) < CONNECTION_TO)  {
    delay(500);
    Serial.print(".");
    NeoPixelStatus( LED_LISTEN_WIFI );  // blink blue
  }
  if (WiFi.status() == WL_CONNECTED) {
    _ssid = WiFi.SSID();
    _pswd = WiFi.psk();
    Serial.print("Smart Config done, connected to: ");
    Serial.print(_ssid);
    Serial.print(" with psswd: ");
    Serial.println("xxxxxx");
    DEBUG_L2(_pswd)  // off by default
    storeNetwork(_ssid, _pswd);
    NeoPixelStatus( LED_CONNECT_WIFI ); // Success - blink green
    return true;
  }
  else {
    Serial.println("SmartConfig received an incorrect WiFi password.");
    WiFi.stopSmartConfig();
    return false;
  }
}


void NeoPixelStatus( int status ) {
  // Turn leds off to cause a blink effect
  strip.clear();  // Off
  strip.show(); // This sends the updated pixel color to the hardware.
  delay(400);   // Delay for a period of time (in milliseconds).

  switch( status ) {
    case LED_OFF :
      strip.clear();  // Off
      break;
    case LED_CONNECTED :
      strip.fill( strip.Color(0,255,255), 0, 3);  // Cyan breath
      Serial.println("LED_CONNECTED - Cyan");
      break;
    case LED_FIRMWARE_OTA :
      strip.fill( strip.Color(255,0,255), 0, 3);  // Magenta
      Serial.println("LED_FIRMWARE_OTA - Magenta");
      break;
    case LED_CONNECT_WIFI :
      strip.fill( strip.Color(0,255,0), 0, 3);  // Green
      Serial.println("LED_CONNECT_WIFI - Green");
      break;
    case LED_CONNECT_CLOUD :
      strip.fill( strip.Color(0,255,255), 0, 3);  // Cyan fast
      Serial.println("LED_CONNECT_CLOUD - Cyan");
      break;
    case LED_LISTEN_WIFI :
      strip.fill( strip.Color(0,0,255), 0, 3);  // Blue
      Serial.println("LED_LISTEN_WIFI - Blue");
      break;
    case LED_WIFI_OFF :
      strip.fill( strip.Color(255,255,255), 0, 3);  // White
      Serial.println("LED_WIFI_OFF - White");
      break;
    case LED_SAFE_MODE :
      strip.fill( strip.Color(255,0,255), 0, 3);  // Magenta breath
      Serial.println("LED_SAFE_MODE - Magenta");
      break;
    case LED_FIRMWARE_DFU :
      strip.fill( strip.Color(255,255,0), 0, 3);  // Yellow
      Serial.println("LED_FIRMWARE_DFU - Yellow");
      break;
    case LED_ORANGE :
      strip.fill( strip.Color(255,165,0), 0, 3);  // Red
      Serial.println("LED_ORANGE - Orange");
      break;
    case LED_ERROR :
      strip.fill( strip.Color(255,0,0), 0, 3);  // Red
      Serial.println("LED_ERROR - Red");
      break;
    default :
      strip.clear();  // Off
      break;
  }
  strip.show(); // Send the updated pixel color to the hardware
}


void NeoPixelBreathe() {
  if( breatheintensity < 0)
    breatheintensity = 0;
  strip.setBrightness( breatheintensity );  // slow breathe the LED
  // Serial.printf("Brightness is %d\n",breatheintensity);
  strip.fill( strip.Color(0,255,255), 0, 3);
  strip.show();

  // Increase or decrease the LED intensity
  breathedirection ? breatheintensity++ : breatheintensity-- ;
}


// Sound the Buzzer & Blink the LED
void EarthquakeAlarm( int AlarmLEDColor ) {
  Serial.println("Earthquake Alarm!");
  strip.setBrightness(255);       // The breathe intensity might have the brightness low
  for( int i=0;i<10;i++) {
    if( !bStopEarthquakeAlarm ) {
      delay(500);
      NeoPixelStatus( AlarmLEDColor ); // Alarm - blink red or orange
      AlarmBuzzer();
    }
    mqtt.loop();  // Process any incoming MQTT topics (which might stop the alarm)
  }
  strip.setBrightness( breatheintensity );  // reset the brightness to the prior intensity
  digitalWrite(io, LOW); // turn off buzzer
}


// Generate Buzzer sounds
void AlarmBuzzer() {
  ledcWrite(channel, 50);
  delay(100);
  ledcWrite(channel, 500);
  delay(100);
  ledcWrite(channel, 2000);
  delay(100);
  ledcWrite(channel, 4000);
  delay(100);
}
