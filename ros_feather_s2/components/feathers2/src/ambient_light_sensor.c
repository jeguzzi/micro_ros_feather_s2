#include "esp_log.h"
#include "esp_adc_cal.h"

#include "driver/adc.h"

#include "ambient_light_sensor.h"

// ALS-PT19-315
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/peripherals/adc.html

// useless
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate

static const char* TAG = "ALS";
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_channel_t channel = ADC_CHANNEL_3;
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "Characterized using Two Point Value");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "Characterized using eFuse Vref");
    } else {
        ESP_LOGI(TAG, "Characterized using Default Vref");
    }
}

void ambient_init()
{
  ESP_LOGI(TAG, "Initializing");
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
      ESP_LOGI(TAG, "eFuse Two Point: Supported");
  } else {
      ESP_LOGI(TAG, "Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.");
  }
  adc1_config_width(width);
  adc1_config_channel_atten(channel, atten);

  // Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
  print_char_val_type(val_type);
  ESP_LOGI(TAG, "Initialized");
}

uint32_t ambient_read() {
  uint32_t adc_reading;
  if (unit == ADC_UNIT_1) {
      adc_reading = adc1_get_raw((adc1_channel_t)channel);
  } else {
      int raw;
      adc2_get_raw((adc2_channel_t)channel, width, &raw);
      adc_reading = raw;
  }
  return esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
}
