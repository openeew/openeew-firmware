
#include "definitions.h"

CONFIG_MQTT_T CONFIG_MQTT = {
    .MQTT_ADDRESS = "",
    .MQTT_USERNAME = "", 
    .MQTT_PASSWORD = "", 
    .PORT = 0,
}; 

CONFIG_WIFI_T CONFIG_WIFI = {
    .SSID = "", 
    .PASSWORD = ""
}; 

EARTHQUAKE_CONFIG_T EARTHQUAKE = {
    .EARTHQUAKE_ALARM_STATE = false, 
    .EARTHQUAKE_INDICATOR = 0, 
}; 

NETWORKSTATUS_T NETWORKSTATUS = {
    .ETHCONNECTED = false, 
    .ETHCONNECTING = false, 
    .WIFICONNECTED = false, 
    .NETWORKINTERFACECHANGED = false
}; 

SYSTEM_INFO_T SYSTEM_INFO {
    .DEVICENAME = "SeismicDevice", 
    .DEVICEID = "", 
    .MACADDRESS= "", 
    .IP = "", 
    .USER = "", 
    .STATION = "", 
    .NETWORK = ""
}; 

ACCEL_T acceleration = {
    .x = 0, 
    .y = 0, 
    .z = 0 
}

SYSTEM_STATUS_T SYSTEM_STATUS = {
    .networkStatus = &NETWORKSTATUS, 
    .systemInfo = &SYSTEM_INFO, 
    .accelerationInfo = &acceleration, 
    .TAG = "MAIN", 
}; 


cJSON* parse_string(const char* json)
{
    cJSON* obj_json = cJSON_Parse(json); 
    if(obj_json == NULL)
    {
        ESP_LOGE(TAG, "[parse_string] Failed to parse JSON object"); 
        return 1; 
    }
}

const char* getJSONObjectItem(cJSON* json, const char* objname)
{
    cJSON* obj = cJSON_GetObjectItem(json, objname);
    return obj->valuestring; 
}


bool freeJSONObj(cJSON* json)
{
    cJsON_delete(json); 
    return true; 
}

bool raiseEarthQuakeAlarm(EARTHqUAKER_CONFIG_T* earthquake)
{

  //use locks to modify the state of the alarm 
  ESP_LOGI("RaiseEarthQuakeAlarm", "Earthquake Alarm!"); 
//   strip.setBrightness(255); // The breathe intensity might have the brightness low
//   for (int i = 0; i < 10; i++)
//   {
//     if (EarthquakeAlarmBool)
//     {
//       delay(500);
//       NeoPixelStatus(AlarmLEDColor); // Alarm - blink red or orange
//       AlarmBuzzer();
//     }
//     client.loop(); // Process any incoming MQTT topics (which might stop the alarm)
//   }
//   strip.setBrightness(breatheintensity); // reset the brightness to the prior intensity
//   digitalWrite(io, LOW);     
}




QueueHandle_t sensorDataQueue = NULL; 
QueueHandle_t systemInfoQueue = NULL; 
QueueHandle_t alarmInfoQueue = NULL;  

bool systemQueueInit()
{
    sensorDataQueue = xQueueCreate(1, sizeof(ACCEL_T)); 
    systemStatusQueue = xQueueCreate(1, sizeof(SYSTEM_STATUS_T)); 
    alarmInfoQueue = xQueueCreate(1, sizeof(EARTHQUAKE_CONFIG_T)); 
    if (sensorDataQueue == NULL || systemStatusQueue == NULL || alarmInfoQueue== NULL)
    {
        //error in creation of queues 
        ESP_LOGE(SYSTEM_STATUS.TAG, "System Queue initialization Failed!"); 
        return false; 
    }
    ESP_LOGI(SYSTEM_STATUS.TAG, "Initialization of system Queue complete");
    return true; 
}


static void sntp_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)id;
    const esp_netif_sntp_time_sync_t *evt = (const esp_netif_sntp_time_sync_t *)data;
    if (evt) {
        char ts[64];
        time_t t = evt->tv.tv_sec;
        struct tm tm_utc;
        gmtime_r(&t, &tm_utc);
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm_utc);
        ESP_LOGI("SNTP HANDLER", "SNTP event: time synced (UTC): %s.%06ld", ts, (long)evt->tv.tv_usec);
    } else {
        ESP_LOGI("SNTP HANDLER", "SNTP event: time synced (no timeval provided)");
    }
}



bool init_sntp()
{
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_SNTP_EVENT, NETIF_SNTP_TIME_SYNC, &sntp_event_handler, NULL));

    ESPLOGI("INIT SNTP", "Initializing the SNTP service"); 
    ESPLOGI("Init SNTP", "System time in UTC0\r\n"); 
    setenv("TZ","UTC0", 1); 
    tzset(); 

   
    
    sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    sntp_setservername(0, "time.google.com");
    sntp_setservername(1, "pool.ntp.com");
    esp_sntp_init();

}

char* getLocalTime()
{
    char strftime_buf[64]; 
    time_t now; 
    struct tm timeinfo; 

    localtime_rr(&now, &timeinfo); 
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo); 
    ESPLOGI("LocalTime", "System local time is %s", strftime_buf); 
    return strftime_buf; 
}


bool systemSemaphoreInit()
{
    SemaphoreHandle_t xSemaphore_systemStatus = vSemaphoreCreateBinary( xSemaphore_systemStatus );
    SemaphoreHandle_t xSemaphore_systemAlarm = vSemaphoreCreateBinary( xSemaphore_systemStatus );
    SemaphoreHandle_t xSemaphore_systemCommChannel = vSemaphoreCreateBinary( xSemaphore_systemCommChannel );
    if(xSemaphore_systemStatus == NULL || xSemaphore_systemAlarm == NULL || xSemaphore_systemCommChannel == NULL)
    {
        ESPLOGE("Semaphore Creation", "Failed to create semaphores "); 
        return false; 
    }
    ESPLOGI("Semaphore Creation", "Completed Initialization of Semaphores"); 
    return true; 
}

#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY


static EventGroupHandle_t s_wifi_event_group;

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

bool wifi_init_sta()
{
    s_wifi_event_group = xEventGroupCreate();

    esp_err_t err = esp_netif_init();
    if(err) return false; 


    esp_err_t err = esp_event_loop_create_default();
    if(err) return false; 

    esp_err_t err = esp_netif_create_default_wifi_sta();
    if(err) return false; 

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if(err) return false; 


    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_err_t err = esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&event_handler,NULL,&instance_any_id);
    if(err) return false; 
    esp_err_t err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,&event_handler,NULL, &instance_got_ip);
    if(err) return false; 

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI.SSID,
            .password = CONFIG_WIFI.PASSWORD,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("WIFI INIT", "connected to ap SSID:%s password:%s",
                 CONFIG_WIFI.SIID, CONFIG_WIFI.PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI("WIFI INIT", "Failed to connect to SSID:%s, password:%s",
                 CONFIG_WIFI.SSID, CONFIG_WIFI.PASSWORD);
    } else {
        ESP_LOGE("WIFI INIT", "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);



    //updte the wifi status of the device 
    SYSTEM_STATUS->networkStatus.WIFICONNECTED = true; 
}

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

bool systemInit()
{
    ESP_ERROR_CHECK( nvs_flash_init()); 
    ESP_ERROR_CHECK(esp_net_if_init()); 
    //create system event loop allow for posting and receiving of events 
    ESP_ERROR_CHECK(esp_event_loop_create_default()); 
    //Start system initialization with creation of Queues 
    ESPLOGI("System Init", "Initialized nvs_flash, esp_net and event loop"); 

    if(!systemQueueInit())
    {
        ESPLOGE("System Init", "Failed to initialize system Queues"); 
        return false; 
    }
    if(!systemSemaphoreInit())
    {
        ESPLOGE("System Init", "Failed to initialize system Semaphores"); 
        return false; 
    } 
   
    //wifi connected 
    if(SYSTEM_STATUS->networkStatus.WIFICONNECTED == false && SYSTEM_STATUS->networkStatus.ETHCONNECTED == false ) {
        ESPLOGI("SYS INIT", "STARTING WIFI INITIALIZATION"); 
        bool wifi_state = wifi_init_sta(); 
        if (!wifi_state)
        {
            ESPLOGE("SYSTEM INIT", "Failed to Initialized WIFI service"); 
        }
        //change the hostname of the device 
        esp_netif_set_hostname(netif, SYSTEM_STATUS->systemInfo.DEVICENAME);
        ESPLOGI("SYS INIT", "System Hostname is %s", SYSTEM_STATUS->systemInfo.DEVICENAME); 
        // Indicate the FLAG to show WIFI is connected  
        SYSTEM_STATUS->networkStatus.WIFICONNECTED = true; 
    }

    //
    if(SYSTEM_STATUS->networkStatus.WIFICONNECTED == false && SYSTEM_STATUS->networkStatus.ETHCONNECTED == false ) {
        ESPLOGI("SYSTEM INIT", "Ethernet handler added to the system "); 
        esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL); 
        // register Ethernet event handler (to deal with user specific stuffs when event like link up/down happened)
        SYSTEM_STATUS->networkStatus.ETHCONNECTED = true; 
    } 



}
