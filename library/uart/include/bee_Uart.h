/**
 * @file bee_Uart.h
 * @author nguyen__viet_hoang
 * @date 25 June 2023
 * @brief module uart, API create for others functions, config Pins for UART
 */
#ifndef BEE_UART_H
#define BEE_UART_H
#include "driver/gpio.h"

#define UART_TX GPIO_NUM_27
#define UART_RX GPIO_NUM_26

#define EX_UART_NUM UART_NUM_1
#define BUF_SIZE 1024

void uart_vCreate();

#endif