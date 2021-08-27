#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Update.h>
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
#include "secrets.h"            // certificates for AWS IoT
#include <cppQueue.h>
#include "soc/soc.h"            // Enable/Disable BrownOut detection
#include "soc/rtc_cntl_reg.h"


// IoT connection details
const int MQTT_PORT = 8883;

const char MQTT_SUB_TOPIC[] = "mysubtopic";

const char MQTT_TOPIC[] = "iot-2/evt/status/fmt/json"; //waveform data
const char MQTT_TOPIC_ALARM[] = "iot-2/cmd/earthquake/fmt/json";
const char MQTT_TOPIC_SAMPLERATE[] = "iot-2/cmd/samplerate/fmt/json";
const char MQTT_TOPIC_FWCHECK[] = "iot-2/cmd/firmwarecheck/fmt/json";
const char MQTT_TOPIC_SEND10SEC[] = "iot-2/cmd/10secondhistory/fmt/json";
const char MQTT_TOPIC_SENDACCEL[] = "iot-2/cmd/sendacceldata/fmt/json";
const char MQTT_TOPIC_RESTART[] = "iot-2/cmd/forcerestart/fmt/json";
const char MQTT_TOPIC_THRESHOLD[] = "iot-2/cmd/threshold/fmt/json";
const char MQTT_TOPIC_FACTORYRST[] = "iot-2/cmd/factoryreset/fmt/json";

char deviceID[13];

//void messageReceived(char* topic, byte* payload, unsigned int length);

WiFiClientSecure net;
WiFiClient nethttp;
PubSubClient client(net);

// OTA
long contentLength = 0; // response from S3
bool isValidContentType = false;

String host = S3_FIRMWARE_BUCKET; // Host => bucket-name.s3.region.amazonaws.com
int port = 80; // Non https. For HTTPS 443. As of today, HTTPS doesn't work.
String bin = S3_FILE; // bin file name with a slash in front.

//String host = "grillo-firmware.s3.amazonaws.com"; // Host => bucket-name.s3.region.amazonaws.com
//int port = 80; // Non https. For HTTPS 443. As of today, HTTPS doesn't work.
//String bin = "/S3_OTA.ino.esp32.bin"; // bin file name with a slash in front.

String getHeaderValue(String header, String headerName) {
  return header.substring(strlen(headerName.c_str()));
}

//bool FirmwareVersionCheck( char *, String );
//void PerformFirmwareOTA( String, const char *, uint );
void NTPConnect();
void SendLiveData2Cloud();
void Send10Seconds2Cloud();

// Ethernet variables
#ifdef ETH_CLK_MODE
#undef ETH_CLK_MODE
#endif
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#define ETH_POWER_PIN    2  // Ethernet on production board
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_ADDR        0
#define ETH_MDC_PIN     23
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
int8_t ADXL_INT_PIN = 35; // ADXL is on interrupt 35 on production board
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
// ADXL
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

// --------------------------------------------------------------------------------------------
// OTA Logic 
void execOTA() {
  Serial.println("Connecting to: " + String(host));
  // Connect to S3
  if (nethttp.connect(host.c_str(), port)) {
    // Connection Succeed.
    // Fetching the bin
    Serial.println("Fetching Bin: " + String(bin));

    // Get the contents of the bin file
    nethttp.print(String("GET ") + bin + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Cache-Control: no-cache\r\n" +
                 "Connection: close\r\n\r\n");

    // Check what is being sent
    //    Serial.print(String("GET ") + bin + " HTTP/1.1\r\n" +
    //                 "Host: " + host + "\r\n" +
    //                 "Cache-Control: no-cache\r\n" +
    //                 "Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (nethttp.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println("Client Timeout !");
        nethttp.stop();
        return;
      }
    }

    while (nethttp.available()) {
      // read line till /n
      String line = nethttp.readStringUntil('\n');
      // remove space, to check if the line is end of headers
      line.trim();

      // if the the line is empty,
      // this is end of headers
      // break the while and feed the
      // remaining `client` to the
      // Update.writeStream();
      if (!line.length()) {
        //headers ended
        break; // and get the OTA started
      }

      // Check if the HTTP Response is 200
      // else break and Exit Update
      if (line.startsWith("HTTP/1.1")) {
        if (line.indexOf("200") < 0) {
          Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
          break;
        }
      }

      // extract headers here
      // Start with content length
      if (line.startsWith("Content-Length: ")) {
        contentLength = atol((getHeaderValue(line, "Content-Length: ")).c_str());
        Serial.println("Got " + String(contentLength) + " bytes from server");
      }

      // Next, the content type
      if (line.startsWith("Content-Type: ")) {
        String contentType = getHeaderValue(line, "Content-Type: ");
        Serial.println("Got " + contentType + " payload.");
        if (contentType == "application/octet-stream") {
          isValidContentType = true;
        }
      }
    }
  } else {
    // Connect to S3 failed
    // May be try?
    // Probably a choppy network?
    Serial.println("Connection to " + String(host) + " failed. Please check your setup");
    // retry??
    // execOTA();
  }

  // Check what is the contentLength and if content type is `application/octet-stream`
  Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // check contentLength and content type
  if (contentLength && isValidContentType) {
    // Check if there is enough to OTA Update
    bool canBegin = Update.begin(contentLength);

    // If yes, begin
    if (canBegin) {
      Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
      // No activity would appear on the Serial monitor
      // So be patient. This may take 2 - 5mins to complete
      size_t written = Update.writeStream(nethttp);

      if (written == contentLength) {
        Serial.println("Written : " + String(written) + " successfully");
      } else {
        Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?" );
        // retry??
        // execOTA();
      }

      if (Update.end()) {
        Serial.println("OTA done!");
        if (Update.isFinished()) {
          Serial.println("Update successfully completed. Rebooting.");
          ESP.restart();
        } else {
          Serial.println("Update not finished? Something went wrong!");
        }
      } else {
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    } else {
      // not enough space to begin OTA
      // Understand the partitions and
      // space availability
      Serial.println("Not enough space to begin OTA");
      nethttp.flush();
    }
  } else {
    Serial.println("There was no content in the response");
    nethttp.flush();
  }
}



// Handle subscribed MQTT topics - Alerts and Sample Rate changes
void messageReceived(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  if(payload[0] == '1'){
       NeoPixelStatus( LED_FIRMWARE_OTA ); // Firmware OTA - Magenta
       Serial.print("Firmware update start...");
       execOTA();  
       }
}

void pubSubErr(int8_t MQTTErr)
{
  if (MQTTErr == MQTT_CONNECTION_TIMEOUT)
    Serial.print("Connection tiemout");
  else if (MQTTErr == MQTT_CONNECTION_LOST)
    Serial.print("Connection lost");
  else if (MQTTErr == MQTT_CONNECT_FAILED)
    Serial.print("Connect failed");
  else if (MQTTErr == MQTT_DISCONNECTED)
    Serial.print("Disconnected");
  else if (MQTTErr == MQTT_CONNECTED)
    Serial.print("Connected");
  else if (MQTTErr == MQTT_CONNECT_BAD_PROTOCOL)
    Serial.print("Connect bad protocol");
  else if (MQTTErr == MQTT_CONNECT_BAD_CLIENT_ID)
    Serial.print("Connect bad Client-ID");
  else if (MQTTErr == MQTT_CONNECT_UNAVAILABLE)
    Serial.print("Connect unavailable");
  else if (MQTTErr == MQTT_CONNECT_BAD_CREDENTIALS)
    Serial.print("Connect bad credentials");
  else if (MQTTErr == MQTT_CONNECT_UNAUTHORIZED)
    Serial.print("Connect unauthorized");
}

void connectToMqtt(bool nonBlocking = false)
{
  Serial.print("MQTT connecting ");
  while (!client.connected())
  {
    if (client.connect(deviceID))
    {
      Serial.println("connected!");
      if (!client.subscribe(MQTT_SUB_TOPIC))
        pubSubErr(client.state());
    }
    else
    {
      Serial.print("failed, reason -> ");
      pubSubErr(client.state());
      if (!nonBlocking)
      {
        Serial.println(" < try again in 5 seconds");
        delay(5000);
      }
      else
      {
        Serial.println(" <");
      }
    }
    if (nonBlocking)
      break;
  }
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
  client.setBufferSize( (jsonSize + 50 ));  // increase the MQTT buffer size

  // Publish the message to MQTT Broker
  if (!client.publish(MQTT_TOPIC, historymsg)) {
    Serial.println("MQTT Publish failed");
  } else {
    NeoPixelStatus( LED_CONNECTED ); // Success - blink cyan
  }

  client.setBufferSize( 2000 );  // reset the MQTT buffer size
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
  client.setBufferSize( (jsonSize + 50 ));  // increase the MQTT buffer size

  // Publish the message to MQTT Broker
  if (!client.publish(MQTT_TOPIC, msg)) {
    Serial.println("MQTT Publish failed");
  } else {
    NeoPixelStatus( LED_CONNECTED ); // Success - blink cyan
  }

  client.setBufferSize( 2000 );  // reset the MQTT buffer size
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
      if( client.connected() ){
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
      if( client.connected() ){
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

unsigned long lastMillis = 0;
time_t now;
time_t nowish = 1510592825;

void NTPConnect(void)
{
  Serial.print("Setting time using SNTP");
  configTime(TIME_ZONE * 3600, DST * 3600, "pool.ntp.org", "time.nist.gov");
  now = time(nullptr);
  while (now < nowish)
  {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("done!");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
}

void connectToWiFi(String init_str)
{
  if (init_str != emptyString)
    Serial.print(init_str);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  if (init_str != emptyString)
    Serial.println("ok!");
}

void checkWiFiThenMQTT(void)
{
  connectToWiFi("Checking WiFi");
  connectToMqtt();
}

unsigned long previousMillis = 0;
const long interval = 5000;

void checkWiFiThenMQTTNonBlocking(void)
{
  connectToWiFi(emptyString);
  if (millis() - previousMillis >= interval && !client.connected()) {
    previousMillis = millis();
    connectToMqtt(true);
  }
}

void checkWiFiThenReboot(void)
{
  connectToWiFi("Checking WiFi");
  Serial.print("Rebooting");
  ESP.restart();
}




void setup() {
  // Start serial console
  Serial.begin(115200);
  delay(5000);
  Serial.println();
  Serial.println(); 

  NeoPixelStatus( LED_OFF ); // turn off the LED to reduce power consumption
  strip.setBrightness(50);  // Dim the LED to 20% - 0 off, 255 full bright

  // Start the ETH interface, if it is available, before WiFi
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
  delay(5000);
  if( bEthConnecting ) {
    while( !bEthConnected ) {
      Serial.println("Waiting for Ethernet to start...");
      delay( 500 );
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
  snprintf(deviceID,13,"%02X%02X%02X%02X%02X%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.println(deviceID);
  WiFi.setHostname(deviceID);

  // Set the time on the ESP32
  NTPConnect();

  //char mqttparams[100]; // Allocate a buffer large enough for this string ~95 chars
  
  net.setCACert(cacert);
  net.setCertificate(client_cert);
  net.setPrivateKey(privkey);

  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(messageReceived);

  // Connect to MQTT
  connectToMqtt();

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
}


void loop() {

  now = time(nullptr);  
  if (!client.connected())
  {
    checkWiFiThenMQTT();
    //checkWiFiThenMQTTNonBlocking();
    //checkWiFiThenReboot();
  }
  else
  {
    client.loop();

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
   }

  if( adxstatus )
    NeoPixelBreathe();

  delay(10);
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
      strip.setBrightness(20);
      strip.fill( strip.Color(0,255,255), 0, 3);  // Cyan breath - dim
      Serial.println("LED_CONNECTED - Cyan");
      break;
    case LED_FIRMWARE_OTA :
      strip.setBrightness(255);
      strip.fill( strip.Color(255,0,255), 0, 3);  // Magenta
      Serial.println("LED_FIRMWARE_OTA - Magenta");
      break;
    case LED_CONNECT_WIFI :
      strip.setBrightness(255);
      strip.fill( strip.Color(0,255,0), 0, 3);  // Green
      Serial.println("LED_CONNECT_WIFI - Green");
      break;
    case LED_CONNECT_CLOUD :
      strip.setBrightness(20);
      strip.fill( strip.Color(0,255,255), 0, 3);  // Cyan fast
      Serial.println("LED_CONNECT_CLOUD - Cyan");
      break;
    case LED_LISTEN_WIFI :
      strip.setBrightness(255);
      strip.fill( strip.Color(255,165,0), 0, 3);  // Orange
      Serial.println("LED_LISTEN_WIFI - Orange");
      break;
    case LED_WIFI_OFF :
      strip.setBrightness(255);
      strip.fill( strip.Color(255,255,255), 0, 3);  // White
      Serial.println("LED_WIFI_OFF - White");
      break;
    case LED_SAFE_MODE :
      strip.setBrightness(255);
      strip.fill( strip.Color(255,0,255), 0, 3);  // Magenta breath
      Serial.println("LED_SAFE_MODE - Magenta");
      break;
    case LED_FIRMWARE_DFU :
      strip.setBrightness(255);
      strip.fill( strip.Color(255,255,0), 0, 3);  // Yellow
      Serial.println("LED_FIRMWARE_DFU - Yellow");
      break;
    case LED_ORANGE :
      strip.setBrightness(255);
      strip.fill( strip.Color(255,165,0), 0, 3);  // Red
      Serial.println("LED_ORANGE - Orange");
      break;
    case LED_ERROR :
      strip.setBrightness(255);
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
    client.loop();  // Process any incoming MQTT topics (which might stop the alarm)
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

