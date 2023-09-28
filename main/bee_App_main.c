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

void app_main()
{
    uart_vCreate();
    sht3x_start();
    mqtt_vLena_r8_start();
}