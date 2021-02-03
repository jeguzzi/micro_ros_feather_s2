#include "string.h"
#include "driver/uart.h"
#include "serial_led_driver_pro.h"

#define BAUD_RATE (2000000L)
#define BUF_SIZE (1024)
#define FRAME_HEADER_MAGIC ("UPXL")

const color_orders_t RGB = {{.redi=0, .greeni=1, .bluei=2, .whitei=3}};
const color_orders_t BGR = {{.redi=2, .greeni=1, .bluei=0, .whitei=3}};

static uint8_t uart_number;

static void write(const uint8_t *buffer, size_t size) {
    uart_write_bytes(uart_number, (const char *) buffer, size);
}

void pb_init(uint8_t _uart_number, uint8_t tx_pin) {
  uart_config_t uart_config = {
      .baud_rate = BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };
  int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
  intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
  uart_number = _uart_number;
  ESP_ERROR_CHECK(uart_driver_install(uart_number, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
  ESP_ERROR_CHECK(uart_param_config(uart_number, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(uart_number, tx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

typedef struct {
    int8_t magic[4];
    uint8_t channel;
    uint8_t recordType;  // set channel ws2812 opts+data, draw all
} pb_frame_header_t;

typedef struct {
    uint8_t numElements; //0 to disable channel, usually 3 (RGB) or 4 (RGBW)
    union {
        struct {
            uint8_t redi :2, greeni :2, bluei :2, whitei :2; //color orders, data on the line assumed to be RGB or RGBW
        };
        uint8_t colorOrders;
    };
    uint16_t pixels;
} pb_ws2812_channel_t;

typedef struct {
    uint32_t frequency;
    union {
        struct {
            uint8_t redi :2, greeni :2, bluei :2; //color orders, data on the line assumed to be RGB
        };
        uint8_t colorOrders;
    };
    uint16_t pixels;
} pb_apa102_data_channel_t;

typedef struct {
    uint32_t frequency;
} pb_apa102_clock_channel_t;


static const uint32_t crc_table[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};


static uint32_t crc_update(uint32_t crc, const void *data, size_t data_len) {
    const unsigned char *d = (const unsigned char *) data;
    unsigned int tbl_idx;

    while (data_len--) {
        tbl_idx = crc ^ *d;
        crc = crc_table[tbl_idx & 0x0f] ^ (crc >> 4);
        tbl_idx = crc ^ (*d >> 4);
        crc = crc_table[tbl_idx & 0x0f] ^ (crc >> 4);
        d++;
    }
    return crc & 0xffffffff;
}

// We consider only RGB inputs
static const uint8_t num_elements = 3;

void pb_set_channel(uint8_t channel_id, channel_type_t channel_type, color_orders_t color_orders,
                    uint16_t number_of_pixels, const uint8_t *buffer,
                    uint32_t frequency, uint8_t brightness) {
    pb_frame_header_t frameHeader;
    memcpy(frameHeader.magic, FRAME_HEADER_MAGIC, 4);

    uint32_t crc = 0xffffffff;
    frameHeader.channel = channel_id;
    frameHeader.recordType = channel_type;
    write((uint8_t *) &frameHeader, sizeof(frameHeader));
    crc = crc_update(crc, &frameHeader, sizeof(frameHeader));

    switch (channel_type) {
        case CHANNEL_WS2812: {
            //write the channel struct
            pb_ws2812_channel_t pb_ws2812_channel_t;
            pb_ws2812_channel_t.numElements = num_elements;
            pb_ws2812_channel_t.pixels = number_of_pixels;
            pb_ws2812_channel_t.colorOrders = color_orders.color_orders;
            write((uint8_t *) &pb_ws2812_channel_t, sizeof(pb_ws2812_channel_t));
            crc = crc_update(crc, &pb_ws2812_channel_t, sizeof(pb_ws2812_channel_t));
            break;
        }
        case CHANNEL_APA102_DATA: {
            pb_apa102_data_channel_t pb_apa102_data_channel_t;
            pb_apa102_data_channel_t.pixels = number_of_pixels;
            pb_apa102_data_channel_t.frequency = frequency;
            pb_apa102_data_channel_t.colorOrders = color_orders.color_orders;
            write((uint8_t *) &pb_apa102_data_channel_t, sizeof(pb_apa102_data_channel_t));
            crc = crc_update(crc, &pb_apa102_data_channel_t, sizeof(pb_apa102_data_channel_t));
            brightness = brightness & 0x1F;
            break;
        }
        case CHANNEL_APA102_CLOCK: {
            pb_apa102_clock_channel_t pb_apa102_clock_channel_t;
            pb_apa102_clock_channel_t.frequency = frequency;
            write((uint8_t *) &pb_apa102_clock_channel_t, sizeof(pb_apa102_clock_channel_t));
            crc = crc_update(crc, &pb_apa102_clock_channel_t, sizeof(pb_apa102_clock_channel_t));
            number_of_pixels = 0;  // make sure we don't send pixel data, even if misconfigured
            break;
        }
        default:
            number_of_pixels = 0; // make sure we don't send pixel data, even if misconfigured
    }

    for (int i = 0; i < number_of_pixels; i++, buffer+=num_elements) {
        crc = crc_update(crc, buffer, num_elements);
        write(buffer, num_elements);
        if(channel_type == CHANNEL_APA102_DATA){
          crc = crc_update(crc, (const uint8_t *)&brightness, 1);
          write((const uint8_t *)&brightness, 1);
        }
    }
    crc = crc ^0xffffffff;
    write((uint8_t *) &crc, 4);
}

void pb_draw() {
    pb_frame_header_t frameHeader;
    memcpy(frameHeader.magic, FRAME_HEADER_MAGIC, 4);
    uint32_t crc = 0xffffffff;
    frameHeader.channel = 0xff;
    frameHeader.recordType = CHANNEL_DRAW_ALL;
    write((uint8_t *) &frameHeader, sizeof(frameHeader));
    crc = crc_update(crc, &frameHeader, sizeof(frameHeader));
    crc = crc ^0xffffffff;
    write((uint8_t *) &crc, 4);
}
