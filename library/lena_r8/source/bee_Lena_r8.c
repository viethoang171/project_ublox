/**
 * @file bee_Lena_r8.c
 * @author nguyen__viet_hoang
 * @date 21 September 2023
 * @brief module SIM LENA-R8 with API, init, connect MQTT broker, configure parameters and publish/subscribe data sensor
 */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>

#include "bee_cJSON.h"
#include "bee_Sht3x.h"
#include "bee_Uart.h"
#include "bee_Led.h"
#include "bee_Lena_r8.h"

#define SIZE_QUEUE_TASK_SUB 30

QueueHandle_t queue_message_response;

float f_Sht3x_temp;
float f_Sht3x_humi;

static uint8_t u8Trans_code = 0;
static uint8_t u8Mac_address[6] = {0xb8, 0xd6, 0x1a, 0x6b, 0x2d, 0xe8};
static char message_publish[200];
static char message_publish_content_for_publish_mqtt_binary[200];
static char mac_address[13];

static void mqtt_vCreate_content_message_json_data(uint8_t u8Flag_temp_humi, float f_Value)
{
    char *message_json_publish;
    u8Trans_code++;
    cJSON *root;

    root = cJSON_CreateObject();
    if (root != NULL)
    {
        cJSON_AddItemToObject(root, "thing_token", cJSON_CreateString(mac_address));
        cJSON_AddItemToObject(root, "cmd_name", cJSON_CreateString("Bee.data"));

        if (u8Flag_temp_humi == FLAG_TEMPERATURE)
        {
            cJSON_AddItemToObject(root, "object_type", cJSON_CreateString("temperature"));
        }
        else
        {
            cJSON_AddItemToObject(root, "object_type", cJSON_CreateString("humidity"));
        }

        cJSON_AddItemToObject(root, "values", cJSON_CreateNumber(f_Value));
        cJSON_AddItemToObject(root, "trans_code", cJSON_CreateNumber(u8Trans_code));
    }
    message_json_publish = cJSON_Print(root);
    cJSON_Delete(root);

    if (message_json_publish != NULL)
    {
        snprintf(message_publish, 200, "AT+UMQTTC=9,0,0,%s,%d\r\n", BEE_TOPIC_PUBLISH, strlen(message_json_publish));
        snprintf(message_publish_content_for_publish_mqtt_binary, 200, "%s\r\n", message_json_publish);
    }
}

static void lena_vConfigure_credential()
{
    char command_AT[200] = {};

    // config client Id
    snprintf(command_AT, 200, "AT+UMQTT=0,%s\r\n", BEE_MQTT_CLIENT_ID);
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));

    // config IP broker and port
    snprintf(command_AT, 200, "AT+UMQTT=3,%s,%s\r\n", BEE_MQTT_BROKER_URL, BEE_BROKER_PORT);
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));

    // config broker user name and password
    snprintf(command_AT, 200, "AT+UMQTT=4,%s,%s\r\n", BEE_USER_NAME, BEE_USER_PASSWORD);
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
}

static void lena_vConnect_mqtt_broker()
{
    char command_AT[200] = {};

    // CGACT
    snprintf(command_AT, 200, "AT+CGACT=1,1\r\n");
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));

    // AT connect
    snprintf(command_AT, 200, "AT+UMQTTC=1\r\n");
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));

    // create AT command to subscribe topic on broker
    snprintf(command_AT, 200, "AT+UMQTTC=4,0,%s\r\n", BEE_TOPIC_SUBSCRIBE);
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
}

static void lena_vPublish_data_sensor()
{
    snprintf(mac_address, sizeof(mac_address), "%02x%02x%02x%02x%02x%02x", u8Mac_address[0], u8Mac_address[1], u8Mac_address[2], u8Mac_address[3], u8Mac_address[4], u8Mac_address[5]);

    // publish temperature value
    mqtt_vCreate_content_message_json_data(FLAG_TEMPERATURE, f_Sht3x_temp);
    uart_write_bytes(EX_UART_NUM, message_publish, strlen(message_publish));
    uart_write_bytes(EX_UART_NUM, message_publish_content_for_publish_mqtt_binary, 200);

    // publish humidity value
    mqtt_vCreate_content_message_json_data(FLAG_HUMIDITY, f_Sht3x_humi);
    uart_write_bytes(EX_UART_NUM, message_publish, strlen(message_publish));
    uart_write_bytes(EX_UART_NUM, message_publish_content_for_publish_mqtt_binary, 200);
}

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

static void mqtt_vRead_response_task()
{
    char list_message_subscribe[200] = {};
    char command_AT[200] = {};
    char message_json[200] = {};
    char message_urc[200] = {};

    for (;;)
    {
        // If broker publish message for module
        uart_read_bytes(EX_UART_NUM, list_message_subscribe, 1000, (TickType_t)0);

        if (strstr(list_message_subscribe, "+UUMQTTC: 6") != NULL)
        {
            snprintf(command_AT, 200, "AT+UMQTTC=6,1\r\n");
            uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));

            // printf("--------------%s------------------\n", list_message_subscribe);
            // printf("--------------%s------------------\n", message_json);
            list_message_subscribe[0] = '\0';

            output_vToggle(LED_CONNECTED_BROKER);
            snprintf(message_publish, 200, "AT+UMQTTC=9,0,0,%s,%d\r\n", BEE_TOPIC_PUBLISH, 200);
            if (strstr(list_message_subscribe, "\"temperature\"") != NULL)
            {
                snprintf(message_publish_content_for_publish_mqtt_binary, 200,
                         "{\"thing_tokern\":\"b8d61a6b2de8\","
                         "\"cmd_name\":\"Bee.data\","
                         "\"object_type\":\"temperature\","
                         "\"values\":%f,"
                         "\"trans_code\":%d}\r\n",
                         f_Sht3x_temp, u8Trans_code);
            }

            else if (strstr(list_message_subscribe, "\"humidity\"") != NULL)
            {
                snprintf(message_publish_content_for_publish_mqtt_binary, 200,
                         "{\"thing_tokern\":\"b8d61a6b2de8\","
                         "\"cmd_name\":\"Bee.data\","
                         "\"object_type\":\"humidity\","
                         "\"values\":%f,"
                         "\"trans_code\":%d}\r\n",
                         f_Sht3x_humi, u8Trans_code);
            }
            printf("%s\n", message_publish_content_for_publish_mqtt_binary);
            uart_write_bytes(EX_UART_NUM, message_publish, strlen(message_publish));
            uart_write_bytes(EX_UART_NUM, message_publish_content_for_publish_mqtt_binary, 200);
            message_publish[0] = '\0';
            message_publish_content_for_publish_mqtt_binary[0] = '\0';
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void mqtt_vLena_r8_start()
{
    xTaskCreate(mqtt_vPublish_task, "mqtt_vPublish_task", 1024 * 3, NULL, 2, NULL);
    xTaskCreate(mqtt_vRead_response_task, "mqtt_vRead_response_task", 1024 * 3, NULL, 3, NULL);
}