// Adapted for freertos from origin Aurduino driver:
// https://github.com/simap/PBDriverAdapter
//

#ifndef SERIAL_LED_DRIVER_PRO_H
#define SERIAL_LED_DRIVER_PRO_H

#include "stdint.h"

typedef enum {
  CHANNEL_WS2812 = 1,
  CHANNEL_DRAW_ALL,
  CHANNEL_APA102_DATA,
  CHANNEL_APA102_CLOCK
} channel_type_t;

typedef union {
  struct {
    // color orders, data on the line assumed to be RGB or RGBW
    uint8_t redi : 2, greeni : 2, bluei : 2, whitei : 2;
  } components;
  uint8_t color_orders;
} color_orders_t;

extern const color_orders_t RGB;
extern const color_orders_t BGR;

void pb_init(uint8_t _uart_number, uint8_t tx_pin);
void pb_set_channel(uint8_t channel_id, channel_type_t channel_type,
                    color_orders_t color_orders, uint16_t number_of_pixels,
                    const uint8_t *buffer, uint32_t frequency,
                    uint8_t brightness);
void pb_draw();

#endif /* end of include guard: SERIAL_LED_DRIVER_PRO_H */
