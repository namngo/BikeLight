#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "sdkconfig.h"
#include "esp_log.h"
// #include "led_strip_rmt.h"
#include "led_strip.h"

#define WAKEUP_INPUT_PIN GPIO_NUM_1
#define SLEEP_INPUT_PIN GPIO_NUM_2
#define LED_PIN GPIO_NUM_2
#define RGB_LED_PIN GPIO_NUM_48

static led_strip_handle_t led_strip;


void setup_gpio(gpio_num_t pin_num) {
    // Reset the pin to a known state
    gpio_reset_pin(pin_num);
    
    // Set direction to Input
    gpio_set_direction(pin_num, GPIO_MODE_INPUT);
    
    // Optional: Enable pull-up if your button connects to GND
    gpio_set_pull_mode(pin_num, GPIO_PULLDOWN_ONLY);
}

// void setup_gpio_led(gpio_num_t led_pin_num) {
//     // 1. Reset the pin to a known state
//     gpio_reset_pin(led_pin_num);
    
//     // 2. Set the direction as Output
//     gpio_set_direction(led_pin_num, GPIO_MODE_OUTPUT);
// }


static led_strip_handle_t configure_rgb_led(gpio_num_t rgb_led_pin)
{
    led_strip_handle_t led_strip;
    /* LED strip initialization with the GPIO and pixels number*/

    led_strip_config_t strip_config = {
        .strip_gpio_num = rgb_led_pin,
        .max_leds = 1,
    };
    // led_strip_config_t strip_config;
    // strip_config.strip_gpio_num = rgb_led_pin;
    // strip_config.max_leds = 1;
    // strip_config.color_component_format

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags = {
            .with_dma = false,
        }
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    return led_strip;
}

static void blink_rgb_led(led_strip_handle_t led_strip, uint8_t s_led_state)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void go_sleep(gpio_num_t wakeup_pin) {
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(wakeup_pin, 1));
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(wakeup_pin));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_en(wakeup_pin));

    printf("Going to sleep in 3s. \n");
    // vTaskDelay(3000 / portTICK_PERIOD_MS);
    // printf("Going to sleep in now. \n");

    esp_deep_sleep_start();
}

extern "C" {

void app_main(void)
{
    led_strip = configure_rgb_led(RGB_LED_PIN);
     vTaskDelay(2000 / portTICK_PERIOD_MS);

    setup_gpio(SLEEP_INPUT_PIN);
    int wakeup_level = gpio_get_level(SLEEP_INPUT_PIN);
    if (wakeup_level == 1) {
        printf("wakeup pin is high, not sleeping. \n");
    } else {
        printf("wakeup pin is low, turn off the led and sleeping. \n");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        blink_rgb_led(led_strip, 0);
        go_sleep(WAKEUP_INPUT_PIN);
    }

    while (1) {
        // int level = gpio_get_level(WAKEUP_INPUT_PIN);

        // if (level == 1) {
        //     printf("Pin is HIGH \n");
        // } else {
        //     printf("Pin is LOW \n");
        // }
        // printf("Hello world 1!\n");

        // // 3. Turn the LED ON (High level = 3.3V)
        printf("Turning the LED ON\n");
        blink_rgb_led(led_strip, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // // 4. Turn the LED OFF (Low level = 0V)
        printf("Turning the LED OFF\n");
        
        blink_rgb_led(led_strip, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
}