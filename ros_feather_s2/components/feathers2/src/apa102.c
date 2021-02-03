// Refactored from https://github.com/limitz/esp-apa102

#include "esp_log.h"
#include "esp_system.h"

#include "driver/spi_master.h"

#include "apa102.h"

// every element is scaled from 0 to 0xFF
// static inline uint32_t APA102_RGBL(uint8_t r, uint8_t g, uint8_t b, uint8_t lum) {
//     return (((r & 0xFF)<<24) | ((g & 0xFF)<<16) | ((b & 0xFF)<<8) | ((lum & 0x1F) | 0xE0));
// }

#define APA102_RGBL(r,g,b,lum) ((((r) & 0xFF)<<24) | (((g) & 0xFF)<<16) | (((b) & 0xFF)<<8) | (((lum) & 0x1F)) | 0xE0)

const uint8_t APA102_SPI_HOST = HSPI_HOST;
const uint8_t APA102_DMA_CHANNEL = 2;
const uint8_t APA102_DATA_PIN = 40;
const uint8_t APA102_CLOCK_PIN = 45;
const uint32_t APA102_CLOCK_SPEED = 1000000;  // SPI_MASTER_FREQ_8M;

static const char* TAG = "APA102";
static uint8_t current_red;
static uint8_t current_green;
static uint8_t current_blue;
static uint8_t current_brightness;
static spi_device_handle_t device;
static uint32_t * txbuffer;
static spi_bus_config_t bus_config = {
  .miso_io_num = -1,
  .mosi_io_num = APA102_DATA_PIN,
  .sclk_io_num = APA102_CLOCK_PIN,
  .quadwp_io_num = -1,
  .quadhd_io_num = -1,
  .max_transfer_sz = 96  // 3 packets, each packet 4 bytes RGBL
};
static spi_device_interface_config_t dev_config = {
  .clock_speed_hz = APA102_CLOCK_SPEED,
  .mode = 0,
  .spics_io_num = -1,
  .queue_size = 1
};
static spi_transaction_t transaction = {
  .length = 96
};

void apa102_init() {
  ESP_LOGI(TAG, "Initializing");
  const size_t packet_size = 12;
  txbuffer = heap_caps_malloc(packet_size, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
  txbuffer[0] = 0x0;
  for (int i=0; i<1; i++) txbuffer[1+i] = 0xE0000000;
  txbuffer[2] = 0xFFFFFFFF;
  transaction.tx_buffer = txbuffer;
  spi_bus_initialize(APA102_SPI_HOST, &bus_config, APA102_DMA_CHANNEL);
  spi_bus_add_device(APA102_SPI_HOST, &dev_config, &device);
  ESP_LOGI(TAG, "Initialized");
}

void apa102_set_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) {
  current_red = red;
  current_green = green;
  current_blue = blue;
  current_brightness = brightness & 0x1F;
  txbuffer[1] = APA102_RGBL(red, green, blue, brightness);
  spi_device_queue_trans(device, &transaction, portMAX_DELAY);
  spi_transaction_t* t;
  spi_device_get_trans_result(device, &t, portMAX_DELAY);
}

void apa102_set_brightness(uint8_t brightness) {
  apa102_set_color(current_red, current_green, current_blue, brightness);
}

void apa102_set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
  apa102_set_color(red, green, blue, current_brightness);
}

void apa102_off() {
  apa102_set_color(0, 0, 0, 0);
}
