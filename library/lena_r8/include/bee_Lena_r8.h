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

#define BEE_MQTT_BROKER_URL "\"61.28.238.97\""
#define BEE_BROKER_PORT "1993"
#define BEE_USER_NAME "\"VBeeHome\""
#define BEE_USER_PASSWORD "\"123abcA@!\""
#define BEE_MQTT_CLIENT_ID "\"device:029f567e-767a-4250-bd7b-6dfa6191f38b\""

#define OBJECT_TYPE_TEMP "temperature"
#define OBJECT_TYPE_HUM "humidity"

#define BEE_TOPIC_SUBSCRIBE "\"VB/DMP/VBEEON/BEE/SMH/b8d61a6b2de8/command\""
#define BEE_TOPIC_PUBLISH "\"VB/DMP/VBEEON/BEE/SMH/b8d61a6b2de8/telemetry\""

#define BEE_TIME_PUBLISH_DATA_SENSOR 30000

#define BEE_QUEUE_LENGTH 30

#define BEE_LENGTH_AT_COMMAND 200

void mqtt_vLena_r8_start();

#endif

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/