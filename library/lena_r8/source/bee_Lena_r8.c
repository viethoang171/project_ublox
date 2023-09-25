/**
 * @file bee_Lena_r8.c
 * @author nguyen__viet_hoang
 * @date 21 September 2023
 * @brief module SIM LENA-R8 with API, init, connect MQTT broker, configure parameters and publish/subscribe data sensor
 */
#include "freertos/queue.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>

#include "bee_cJSON.h"
#include "bee_Sht3x.h"
#include "bee_Uart.h"
#include "bee_Lena_r8.h"

#define SIZE_QUEUE_TASK_SUB 30

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
        snprintf(message_publish, 200, "AT+UMQTTC=9,0,0,\"VB/DMP/VBEEON/BEE/SMH/b8d61a6b2de8/telemetry\",%d\r\n", strlen(message_json_publish));
        printf("%s\n", message_publish);
        snprintf(message_publish_content_for_publish_mqtt_binary, 200, "%s\r\n", message_json_publish);
    }
}

static void mqtt_vSubscribe_command_server_task(void *params)
{
    char message_subscribe[200];

    QueueHandle_t queue, queue_host_main;
    queue = xQueueCreate(SIZE_QUEUE_TASK_SUB, sizeof(message_subscribe));
    queue_host_main = xQueueCreate(SIZE_QUEUE_TASK_SUB, sizeof(message_subscribe));
    for (;;)
    {
        if (xQueueReceive(queue, &message_subscribe, (TickType_t)portMAX_DELAY))
        {
            cJSON *root = cJSON_Parse(message_subscribe);
            if (root != NULL)
            {
                u8Trans_code++;
                char *device_id;
                char *cmd_name;
                char *object_type;

                device_id = cJSON_GetObjectItemCaseSensitive(root, "thing_token")->valuestring;
                cmd_name = cJSON_GetObjectItemCaseSensitive(root, "cmd_name")->valuestring;
                object_type = cJSON_GetObjectItemCaseSensitive(root, "object_type")->valuestring;
                if (device_id != NULL && cmd_name != NULL && object_type != NULL)
                {
                    if (strcmp(device_id, mac_address) == 0 && strcmp(cmd_name, "Bee.conf") == 0)
                    {
                        if (strcmp(object_type, OBJECT_TYPE_TEMP) == 0 && (u8Mqtt_status == MQTT_CONNECTED))
                        {
                            mqtt_vCreate_content_message_json_data(FLAG_TEMPERATURE, u8Temperature);
                        }
                        else if (strcmp(object_type, OBJECT_TYPE_HUM) == 0 && (u8Mqtt_status == MQTT_CONNECTED))
                        {

                            mqtt_vCreate_content_message_json_data(FLAG_HUMIDITY, u8Humidity);
                        }
                    }
                }
                cJSON_Delete(root);
            }
        }
    }
}

void lena_vConfigure_credential()
{
    char command_AT[200] = {};

    // config client Id
    snprintf(command_AT, 200, "AT+UMQTT=0,\"device:029f567e-767a-4250-bd7b-6dfa6191f38b\"\r\n");
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));

    // config IP broker and port
    snprintf(command_AT, 200, "AT+UMQTT=3,\"61.28.238.97\",1993\r\n");
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));

    // config broker user name and password
    snprintf(command_AT, 200, "AT+UMQTT=4,\"VBeeHome\",\"123abcA@!\"\r\n");
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
}

void lena_vConnect_mqtt_broker()
{
    char command_AT[200] = {};
    char receive_AT[200] = {};

    // CGACT
    snprintf(command_AT, 200, "AT+CGACT=1,1\r\n");
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
    uart_read_bytes(EX_UART_NUM, receive_AT, 2, portMAX_DELAY);

    // AT connect
    snprintf(command_AT, 200, "AT+UMQTTC=1\r\n");
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
    uart_read_bytes(EX_UART_NUM, receive_AT, 2, portMAX_DELAY);
}

void lena_vPublish_data_sensor()
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