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

void lena_vPublish_data_sensor();
#endif