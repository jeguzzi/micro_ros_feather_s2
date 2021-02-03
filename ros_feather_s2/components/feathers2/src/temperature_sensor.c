#include "esp_log.h"

#include "driver/temp_sensor.h"

#include "temperature_sensor.h"

static const char* TAG = "TS";

void temperature_sensor_init() {
  ESP_LOGI(TAG, "Initializing");
  temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
  temp_sensor_get_config(&temp_sensor);
  ESP_LOGI(TAG, "Default dac %d, clk_div %d", temp_sensor.dac_offset, temp_sensor.clk_div);
  temp_sensor.dac_offset = TSENS_DAC_DEFAULT;
  temp_sensor_set_config(temp_sensor);
  temp_sensor_start();
  ESP_LOGI(TAG, "Initialized");
}

float temperature_sensor_read() {
  float tsens_out;
  temp_sensor_read_celsius(&tsens_out);
  return tsens_out;
}
