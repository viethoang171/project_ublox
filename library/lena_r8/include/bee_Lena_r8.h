/**
 * @file bee_Lena_r8.h
 * @author nguyen__viet_hoang
 * @date 21 September 2023
 * @brief module SIM LENA-R8 with API init, connect MQTT broker, publish data sensor
 */
#ifndef __LENA_R8_H__
#define __LENA_R8_H__

#define FLAG_TEMPERATURE 1
#define FLAG_HUMIDITY 0

#define BEE_MQTT_BROKER_URL "61.28.238.97"
#define BEE_BROKER_PORT "1993"
#define BEE_USER_NAME "VBeeHome"
#define BEE_USER_PASSWORD "123abcA@!"

#define OBJECT_TYPE_TEMP "temperature"
#define OBJECT_TYPE_HUM "humidity"

#define BEE_TIME_PUBLISH_DATA_SENSOR 30000

void lena_vConfigure_credential();
void lena_vConnect_mqtt_broker();
void lena_vPublish_data_sensor();
void mqtt_vSubscribe_command_server_task();

#endif