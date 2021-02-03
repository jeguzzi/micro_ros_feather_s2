#include "esp_log.h"

#include "driver/gpio.h"

#include "blue_led.h"

#define BLUE_LED_PIN 13

static const char* TAG = "BLUE_LED";

void blue_led_init() {
  ESP_LOGI(TAG, "Initializing");
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL<<BLUE_LED_PIN);
  gpio_config(&io_conf);
  ESP_LOGI(TAG, "Initialized");
}

void blue_led_set(bool value) {
  gpio_set_level(BLUE_LED_PIN, value & 1);
}
