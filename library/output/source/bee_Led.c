/**
 * @file bee_Led.c
 * @author nguyen__viet_hoang
 * @date 28 September 2023
 * @brief module for output with GPIO, API "create", "set level", "toggle" for others functions
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include <driver/gpio.h>
#include "driver/adc.h"
#include "driver/ledc.h"
#include "bee_Led.h"

static const adc1_channel_t adc_channel = ADC_CHANNEL_4;
static ledc_channel_config_t ledc_channel;

void output_vCreate(gpio_num_t gpio_num)
{
    esp_rom_gpio_pad_select_gpio(gpio_num);
    // Set the GPIO as a push/pull output
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT_OUTPUT);
}

void output_vSetLevel(gpio_num_t gpio_num, int level)
{
    gpio_set_level(gpio_num, level);
}

void output_vToggle(gpio_num_t gpio_num)
{
    int previous_level = gpio_get_level(gpio_num);
    gpio_set_level(gpio_num, 1 - previous_level);
}

static void ledc_init_hardware()
{
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&ledc_timer);
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.duty = 0;
    ledc_channel.gpio_num = LEDC_RED;
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.hpoint = 0;
    ledc_channel.timer_sel = LEDC_TIMER_1;
    ledc_channel_config(&ledc_channel);
}

void ledc_task()
{
    uint8_t mark_led = 0;
    ledc_init_hardware();
    for (;;)
    {
        uint32_t adc_val = 0;
        for (int i = 0; i < SAMPLE_CNT; ++i)
        {
            adc_val += adc1_get_raw(adc_channel);
        }

        adc_val /= SAMPLE_CNT;

        if (adc_val == 0)
        {
            mark_led++;
            mark_led = mark_led % 3;
        }

        switch (mark_led)
        {
        case FLAG_LEDC_RED:
        {
            ledc_channel.channel = LEDC_CHANNEL_0;
            ledc_channel.gpio_num = LEDC_RED;
        }
        break;

        case FLAG_LEDC_GREEN:
        {
            ledc_channel.channel = LEDC_CHANNEL_1;
            ledc_channel.gpio_num = LEDC_GREEN;
        }
        break;

        case FLAG_LEDC_BLUE:
        {
            ledc_channel.channel = LEDC_CHANNEL_2;
            ledc_channel.gpio_num = LEDC_BLUE;
        }
        break;
        default:
            break;
        }

        ledc_channel_config(&ledc_channel);
        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, adc_val);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
