idf_component_register(
  SRCS
    "src/ambient_light_sensor.c"
    "src/apa102.c"
    "src/blue_led.c"
    "src/ldo_2.c"
    "src/temperature_sensor.c"
  INCLUDE_DIRS
    "include"
  PRIV_REQUIRES
    "driver"
    "esp_adc_cal"
)
