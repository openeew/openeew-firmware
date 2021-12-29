

// ========= Add Station Infomation for this sensor ===========================================
// This needs to be done for every sensor you flash
char user[40] = "AMEIRA";   // add user code,eg JBIEBER
char network[10] = "GR";     // add 2 digit network code, eg GR
char station[10] = "GR002";  // add 5 digit station code, eg ST001


// ========================== add your MQTT credentials =======================================
const char MQTT_HOST[] = "192.168.0.60";
const int MQTT_PORT = 1883;


// ========================== if hardcoding Wifi, add here ====================================
// Hardcode wifi credentials (to use them hardcodewifi needs to be set to true)
bool hardcodewifi = false;
const char ssid[] = "";
const char pass[] = "";
