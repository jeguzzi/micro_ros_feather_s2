#include "esp_log.h"

#include "driver/gpio.h"

#include "ldo_2.h"

#define LDO_2_ENABLE_PIN 21

static const char* TAG = "LDO_2";

void ldo_2_init() {
  ESP_LOGI(TAG, "Initializing");
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set
  io_conf.pin_bit_mask = (1ULL<<LDO_2_ENABLE_PIN);
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
  ESP_LOGI(TAG, "Initialized");
}

void ldo_2_enable(bool value) {
  gpio_set_level(LDO_2_ENABLE_PIN, value & 1);
  if (value)
    ESP_LOGI(TAG, "Enabled");
  else
    ESP_LOGI(TAG, "Disabled");
}
