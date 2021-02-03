#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_uros/options.h>
#include <uros_network_interfaces.h>
#include "uxr/client/config.h"

// #include <std_msgs/msg/color_rgba.h>
#include <led_strip_msgs/srv/set_brightness.h>
#include <led_strip_msgs/msg/led_strips.h>

#include "serial_led_driver_pro.h"
#include "ldo_2.h"
#include "apa102.h"
#include "blue_led.h"

// #define TEST_ON_APA102
#define ALIVE_ON_APA102

static const char *TAG = "uROS";

#define NODE_NAME "led_driver_pro"
#define NODE_NS ""
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);apa102_set_color(1, 0, 0, 1); vTaskDelay(1000 / portTICK_PERIOD_MS); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

const uint8_t UART_NUMBER = 0;
const uint8_t UART_IO_TX = 43;
#define FREQUENCY 1000000L
#define MAX_NUMBER_OF_CHANNELS 8
#define MAX_STRIP_LENGTH 1000
#define DEFAULT_BRIGHTNESS 0x1

static rcl_subscription_t subscriber;
static uint8_t brightness[MAX_NUMBER_OF_CHANNELS];
static const led_strip_msgs__msg__LedStrips * last_msg = NULL;

void set_colors(const led_strip_msgs__msg__LedStrips * msg) {
  blue_led_set(1);
  for (size_t i = 0; i < msg->strips.size; i++) {
    const led_strip_msgs__msg__LedStrip * strip_msg = msg->strips.data + i;
    uint8_t channel_id = strip_msg->channel_index;
#ifdef TEST_ON_APA102
  if(channel_id==0 && strip_msg->data.size >= 3) {
    const uint8_t * rgb = (const uint8_t *) strip_msg->data.data;
    apa102_set_color(rgb[0], rgb[1], rgb[2], brightness[channel_id]);
  }
#else
    pb_set_channel(
        channel_id, CHANNEL_APA102_DATA, strip_msg->color_order == 0 ? RGB : BGR,
        strip_msg->data.size, (const uint8_t *) strip_msg->data.data,
        FREQUENCY, brightness[channel_id]);
#endif
  }
#ifndef TEST_ON_APA102
  pb_draw();
#endif
  blue_led_set(0);
}

void subscription_callback(const void * msgin) {
  last_msg = (const led_strip_msgs__msg__LedStrips *)msgin;
  set_colors(last_msg);
}

static void set_brightness(uint8_t channel_mask, float value) {
  uint8_t i_value;
  if (value < 0) {
    i_value = 0x0;
  } else if (value > 1.0) {
    i_value = 0x1F;
  } else {
    i_value = (uint8_t) (31 * value);
  }
  for (size_t i = 0; i < MAX_NUMBER_OF_CHANNELS; i++) {
    if(channel_mask & (1 << i)) {
      brightness[i] = i_value;
    }
  }
}

void set_brightness_service_callback(const void * req, void * res){
  led_strip_msgs__srv__SetBrightness_Request * req_in = (led_strip_msgs__srv__SetBrightness_Request *) req;
  set_brightness(req_in->channel_index_mask, req_in->brightness);
  if(last_msg) {
    set_colors(last_msg);
  }
}

void micro_ros_task(void * arg)
{
  apa102_set_color(0, 0, 32, 1);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  while(rmw_uros_discover_agent(rmw_options) != RCL_RET_OK) {
    usleep(100000);
  }

  // create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  // create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, NODE_NS, &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(led_strip_msgs, msg, LedStrips),
    "led_strips"));

  // We have to allocate the message ourself.
  // TODO(jerome): can we use a convenience function?
  led_strip_msgs__msg__LedStrips msg;
  msg.strips.capacity = 8;
  msg.strips.data = malloc(8 * sizeof(led_strip_msgs__msg__LedStrip));

  for (size_t i = 0; i < MAX_NUMBER_OF_CHANNELS; i++) {
    msg.strips.data[i].data.capacity = MAX_STRIP_LENGTH * 3;
    msg.strips.data[i].data.data = malloc(MAX_STRIP_LENGTH * 3);
  }

  apa102_set_color(0, 32, 32, 1);
  // // create service
  rcl_service_t set_brightness_service;
  RCCHECK(rclc_service_init_default(&set_brightness_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(led_strip_msgs, srv, SetBrightness), "set_brightness"));

  // create executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  led_strip_msgs__srv__SetBrightness_Response res;
  led_strip_msgs__srv__SetBrightness_Request req;
  RCCHECK(rclc_executor_add_service(&executor, &set_brightness_service, &req, &res, set_brightness_service_callback));

  apa102_set_color(0, 32, 0, 1);
#ifdef ALIVE_ON_APA102
  bool on = true;
#endif
  while(1){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    usleep(10000);
#ifdef ALIVE_ON_APA102
    on = !on;
    apa102_set_color(0, on * 32, 0, 0x1);
#endif
  }
  apa102_set_color(0, 32, 32, 1);

  // free resources
  RCCHECK(rcl_service_fini(&set_brightness_service, &node));
  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rcl_node_fini(&node));

  vTaskDelete(NULL);
}


// void alive_task(void * arg)
// {
//   uint8_t on = true;
//   while(1) {
//     on = !on;
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
//     apa102_set_color(0, on * 255, 0, 1);
//   }
// }

void app_main(void)
{
  ldo_2_init();
  ldo_2_enable(true);
  apa102_init();
  blue_led_init();
  set_brightness(0xFF, DEFAULT_BRIGHTNESS);
  pb_init(UART_NUMBER, UART_IO_TX);
#ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif  // UCLIENT_PROFILE_UDP

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
  xTaskCreate(micro_ros_task, "uros_task", 30000, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);

  // xTaskCreate(alive_task, "alive_task", 1000, NULL, 1, NULL);

}
