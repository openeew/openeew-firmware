#include "/definitions/definitions.h"
#include "/tasks/task.h"


void app_main()
{   
    ++boot_count; 
    ESPLOGI(SYSTEM_STATUS.TAG, "Current Boot count for the device is %d \r\n", boot_count); 
    systemInit(); 
}