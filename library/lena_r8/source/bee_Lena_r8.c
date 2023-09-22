/**
 * @file bee_Lena_r8.c
 * @author nguyen__viet_hoang
 * @date 21 September 2023
 * @brief module SIM LENA-R8 with API, init, connect MQTT broker, configure parameters and publish/subscribe data sensor
 */
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>

#include "bee_cJSON.h"
#include "bee_Sht3x.h"
#include "bee_Lena_r8.h"

float f_Sht3x_temp;
float f_Sht3x_humi;

static uint8_t u8Trans_code = 0;
static uint8_t u8Mac_address[6] = {0xb8, 0xd6, 0x1a, 0x6b, 0x2d, 0xe8};
static char message_publish[200];
static char message_publish2[200];
static char mac_address[20];

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
    // snprintf(message_json_publish, 15, "{\"name\":11111}");
    cJSON_Delete(root);

    if (message_json_publish != NULL)
    {
        snprintf(message_publish, 200, "AT+UMQTTC=9,0,0,\"sotatek/living_room\",14\r\n");
        // snprintf(message_publish, 200, "AT+UMQTTC=2,0,0,0,\"sotatek/living_room\",\"hello\"\r\n");
        printf("%s\n", message_publish);
        snprintf(message_publish2, 17, "%s\r\n", "{\"name\":11111}");
    }
}

void lena_vPublish_data_sensor()
{
    snprintf(mac_address, sizeof(mac_address), "%02x%02x%02x%02x%02x%02x", u8Mac_address[0], u8Mac_address[1], u8Mac_address[2], u8Mac_address[3], u8Mac_address[4], u8Mac_address[5]);
    mqtt_vCreate_content_message_json_data(FLAG_TEMPERATURE, f_Sht3x_temp);
    uart_write_bytes(2, message_publish, strlen(message_publish));
    printf("%s\n", message_publish2);
    uart_write_bytes(2, message_publish2, 200);

    // mqtt_vCreate_content_message_json_data(FLAG_HUMIDITY, f_Sht3x_humi);
    // uart_write_bytes(2, message_publish, 200);
}