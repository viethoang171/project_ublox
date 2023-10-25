/*****************************************************************************
 *
 * @file 	bee_rs485.h
 * @author 	tuha
 * @date 	6 Sep 2023
 * @brief	module for project rs485 communication
 *
 ***************************************************************************/

#ifndef BEE_RS485
#define BEE_RS485

#define TX_PIN (17)
#define RX_PIN (16)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define RTS_PIN (4)

// CTS is not used in RS485 Half-Duplex Mode
#define CTS_PIN (UART_PIN_NO_CHANGE)

#define BUF_SIZE (4096)
#define BAUD_RATE (115200)

// Read packet timeout
#define PACKET_READ_TICS (100 / portTICK_PERIOD_MS)
#define RX_TASK_STACK_SIZE (4096)
#define RX_TASK_PRIO (10)
#define UART_PORT_2 (2)

// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define RX_READ_TOUT (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks

#define CRC8_POLYNOMIAL 0x31

#define ADDRESS_SLAVE_1 0X01
#define ADDRESS_SLAVE_2 0X02
#define ADDRESS_SLAVE_3 0X03

void rs485_init();

void RX_task(void *pvParameters);

void TX(const int port, const char *str, uint8_t length);

uint16_t MODBUS_CRC16(const unsigned char *buf, unsigned int len);

char *read_holding_registers(uint8_t slave_addr);

#endif

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/