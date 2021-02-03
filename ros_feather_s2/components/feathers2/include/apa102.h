#ifndef APA102_H
#define APA102_H

#include <stdint.h>

void apa102_init();
void apa102_set_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness);
void apa102_set_brightness(uint8_t brightness);
void apa102_set_rgb(uint8_t red, uint8_t green, uint8_t blue);
void apa102_off();

#endif /* end of include guard: APA102_H */
