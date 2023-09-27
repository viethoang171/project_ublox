/**
 * @file bee_App_main.c
 * @author nguyen__viet_hoang
 * @date 21 September 2023
 * @brief app_main for ublox project
 */

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "driver/uart.h"
#include "hal/gpio_types.h"
#include <string.h>

#include "bee_Uart.h"
#include "bee_Sht3x.h"
#include "bee_Lena_r8.h"

static void mqtt_vPublish_task()
{
    static TickType_t last_time_publish = 0;
    lena_vConfigure_credential();
    lena_vConnect_mqtt_broker();
    for (;;)
    {
        if (xTaskGetTickCount() - last_time_publish > pdMS_TO_TICKS(BEE_TIME_PUBLISH_DATA_SENSOR))
        {
            lena_vPublish_data_sensor();
            last_time_publish = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main()
{
    uart_vCreate();
    sht3x_start();
    xTaskCreate(mqtt_vPublish_task, "mqtt_vPublish_task", 1024 * 3, NULL, 2, NULL);
    xTaskCreate(mqtt_vSubscribe_command_server_task, "mqtt_vSubscribe_command_server_task", 1024 * 3, NULL, 1, NULL);
}