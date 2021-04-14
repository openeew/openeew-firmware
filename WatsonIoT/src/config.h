// --------------------------------------------------------------------------------------------
//        UPDATE CONFIGURATION TO MATCH YOUR ENVIRONMENT
// --------------------------------------------------------------------------------------------
//#define OPENEEW_ACTIVATION_ENDPOINT "https://openeew-devicemgmt.mybluemix.net/activation?ver=1"
#define OPENEEW_ACTIVATION_ENDPOINT "https://device-mgmt.openeew.com/activation?ver=1"
// VERSION uses convention semver.org
//  1.3.9 < 1.4.0 < 1.4.1-alpha1 < 1.4.1-alpha2 < 1.4.1
#define OPENEEW_FIRMWARE_VERSION    "1.5.0"

// Run this firmware with a MQTT Broker on a local subnet
// One way of setting up a local subnet MQTT Broker, see
// https://www.digitalocean.com/community/tutorials/how-to-install-and-secure-the-mosquitto-mqtt-messaging-broker-on-ubuntu-18-04
// Comment this Define to send data to local Cloud
//#define MQTT_LOCALBROKER "192.168.1.101"


#define OPENEEW_SAMPLE_RATE_125 false
#define OPENEEW_SAMPLE_RATE_31_25 true

#define debug true

#define LOG_L2 false //Needs to be true to have logging about deep wifi info

#define DEBUG_IL(x) \
    if (debug)      \
        Serial.print(x);
#define DEBUG(x) \
    if (debug)   \
        Serial.println(x);

#define DEBUG_IL_L2(x) \
    if (LOG_L2)        \
        Serial.print(x);
#define DEBUG_L2(x) \
    if (LOG_L2)     \
        Serial.println(x);

#define CONNECTION_TO 6000    //ms
#define RECONNECTION_TO 10000 //ms

#define PRODUCTION_BOARD 1
