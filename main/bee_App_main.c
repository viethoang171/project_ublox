#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "driver/uart.h"
#include "hal/gpio_types.h"
#include <string.h>

#include "bee_Uart.h"
#include "bee_Sht3x.h"
#include "bee_Lena_r8.h"
static void vTask_publish()
{
    for (;;)
    {
        lena_vPublish_data_sensor();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main()
{
    uart_vCreate();
    sht3x_start();
    xTaskCreate(vTask_publish, "vTask_publish", 1024 * 3, NULL, 1, NULL);
}