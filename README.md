# uROS for FeatherS2
A repository for my own ROS2/uROS development related to the FeatherS2, ESP32S2 and uROS.

## Packages

### ROS FEATHER S2

A uROS driver for the naked FeatherS2 that exposes:
- the APA102 RGB LED as `std_msgs/ColorRGBA` subscriber on `apa102`
- the blue LED as a `std_msgs/Bool` subscriber in `blue_led`
- the temperature sensor as `sensor_msgs/Temperature` publisher on `temperature`
- the ALS [ambient light sensor] as a `sensor_msgs/Illuminance` publisher in `illuminance`

### ROS FEATHER WING

A uROS driver for the FeatherS2 + Feather wing 8x4 LED matrix that exposes:
- the LED matrix single color as a `std_msgs/ColorRGBA` subscriber on `color`
The maximal brightness can be through service `set_brightness` of type `led_strip_msgs/SetBrightness`.

### ROS LED DRIVER

A uROS interface to the Serial LED driver pro (https://www.bhencke.com/serial-led-driver-pro) that exposes:
- the LED strips colors as a `led_strip_msgs/LedStrips` subscriber on `led_strips`.

The maximal brightness can be through service `set_brightness` of type `led_strip_msgs/SetBrightness`.


## Caveats

The support for ESP32S2 / FeatherS2 is not complete. Currently, following is missing (from esp-idf and/or uROS):
- multiple subscribers
- console output through USB.
