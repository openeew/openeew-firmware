
// Add timezone, if not UTC
int8_t TIME_ZONE = 0; // UTC
uint8_t DST = 0;

// debugging
#define debug true
#define LOG_L2 false // Needs to be true to have logging about deep wifi info

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

#define CONNECTION_TO 15000   // ms
#define RECONNECTION_TO 15000 // ms
#define RESYNCTIME 86400      // Resync the ESP32 time once a day
#define MQTT_CONNECTION_LIMIT 60000

#define PRODUCTION_BOARD 1
#define OPENEEW_SAMPLE_RATE_125 false
#define OPENEEW_SAMPLE_RATE_31_25 true

