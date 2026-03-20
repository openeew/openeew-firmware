/***
 * System level definitions for the 
 * 
 */


#include "definitions.h"
#include "esp_log.h"
#include "esp_sntp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sys/time.h"



//JSON lib
#include <cjson/cJSON.h>
//NTP Time Stamp Servers from espidf lib 
#include "esp_sntp.h"


//TRACK THE NUMBER OF BOOTS IN THE SYSTEM
RTC_DATA_ATTR static int boot_count = 0;

typedef struct {
    char* MQTT_ADDRESS; 
    char* MQTT_USERNAME; 
    char* MQTT_PASSWORD; 
    uint16_t PORT; 
} CONFIG_MQTT_T; 

typedef struct {
    char* SSID; 
    char* PASSWORD; 
} CONFIG_WIFI_T; 

/**
 * @brief configs for the earthquake system
 */
typedef struct {
    bool EARTHQUAKE_ALARM_STATE; 
    uint8_t EARTHQUAKE_INDICATOR; // LED COLOR FOR THE EARTHQUAKE 
} EARTHQUAKE_CONFIG_T; 

/**
 * @brief Function to raise earth quake alarm 
 * @param earthquake returns 
 */
bool RaiseEarthQuakeAlarm(EARTHQUAKE_CONFIG_T* earthquake); 


/**
 * @brief 
 *        Configuration for the devices timestamp 
 */
typedef struct {
    double deviceTime; 
    int8_t TIME_ZONE = 0; 
    uint8_t DST = 0; 

} TIMESTEP_T; 

/**
 * @brief 
 *        DEFINITION OF NETWORK STATUS 
 *        This includes the status of WIFI and Ethernet connections
 * 
 */
typedef struct {
    bool ETHCONNECTED; 
    bool ETHCONNECTING; 
    bool WIFICONNECTED; 
    bool NETWORKINTERFACECHANGED;

} NETWORKSTATUS_T; 

/**
 * @brief 
 *      struct holds general information about the SEISMI DEVICE 
 */
typedef struct {
       char* DEVICENAME;  //NAME OF THE DEVICE 
       char* DEVICEID; 
       char* MACADDRESS; //Mac address of the system 
       char* IP;         // IP address of the system
       char* USER;       // code for the user 
       char STATION[5];    // 5 digit station code 
       char NETWORK[2];    // 2 Digit station code 
} SYSTEM_INFO_T; 

/**
 * @brief 
 *       Centralised location of the module's status 
 */
typedef struct  {
    NETWORKSTATUS_T* networkStatus; 
    SYSTEM_INFO_T* systemInfo;      // General information about the system 
    ACCEL_T* accelerationInfo; 
    const char* TAG; 
} SYSTEM_STATUS_T; 

/**
 * @brief 
 *      Function initializes the system information
 *      It allocates memory based on system defintions then appends the information to the system 
 */
bool systemInit(); 



/**
 * @brief 
 *      Variable holding the accelerometer data points
 */
typedef struct 
{
    float x;
    float y;
    float z;
} ACCEL_T;


/**
 * @brief function passes a string and returns JSON file 
 * @param json - pass the string of the JSON object 
 * @return cJson - the json string is parsed and a JSON object is returned 
 * 
 */
cJSON* parse_string(const char* json); 


/**
 * @brief extract value string from JSON object 
 * @param json - pointer to the json object 
 *        objname - Key of the JSON object
 * 
 * @return returns constant reference to the string of the value of the JSON object * 
 */
const char* getJSONObjectItem(cJSON* json, const char* objname); 

/**
 * @brief deletes the JSON object. For every JSON object created 
 *        ensure the json object is released.i.e. manual garbage collection 
 * @return bool true - when the json object is cleared 
 *         bool false - when the operation fails
 */
bool freeJSONObj(cJSON* json); 


// DECLARE GLOBAL VARIABLES 
extern CONFIG_MQTT_T CONFIG_MQTT; 
extern CONFIG_WIFI_T CONFIG_WIFI; 
extern EARTHQUAKE_CONFIG_T EARTHQUAKE; 
extern NETWORKSTATUS_T NETWORKSTATUS; 

extern SYSTEM_STATUS_T SYSTEM_STATUS; 



/**
 * @brief 
 *        System Queue definition 
 */


extern QueueHandle_t sensorDataQueue; 
extern QueueHandle_t systemStatusQueue; 
extern QueueHandle_t alarmInfoQueue; 

//Semaphore to synchronize access to system status 
extern SemaphoreHandle_t xSemaphore_systemStatus; 
//Semaphore to synchronize access to system alarm 
extern SemaphoreHandle_t xSemaphore_systemAlarm; 
//Semaphore to synchronize access to system communication channel 
extern SemaphoreHandle_t xSemaphore_systemCommChannel; 

/**
 * @brief 
 *      Initialize system semaphores used for synchronization of resources 
 * @return 
 *         True - Successful creation of semaphores 
 *         False - Failed to create system Semaphores
 */
bool systemSemaphoreInit(); 



/**
 * @brief 
 *        Function initializes all the system queues 
 * @return 
 *        return true when initialization of Queues is complete 
 *        returns false when initialization of System Queues is false 
 */
bool systemQueueInit();


/**
 * @brief 
 *        Function to connect to SNTP servers across the world. 
 *        Generates timestamp 
 * 
 */

#include "esp_sntp.h"

bool init_sntp(); 
static void sntp_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data); 


/**
 * @brief 
 *         Returns UTC time in second 
 */
char* getLocalTime(); 
void wifi_init_sta(); 
static void event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data)