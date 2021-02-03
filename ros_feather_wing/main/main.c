#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"

#include "driver/rmt.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_uros/options.h>

#include <uros_network_interfaces.h>
#include "uxr/client/config.h"

#include <std_msgs/msg/color_rgba.h>
#include <led_strip_msgs/srv/set_brightness.h>

#include "blue_led.h"
#include "led_strip.h"

static const char *TAG = "FEATHER_WING";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define LED_NUMBER 32
#define RMT_TX_GPIO 38
#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define DEFAULT_BRIGHTNESS 0.1

#define NODE_NAME "feather_wing"
#define NODE_NS "led_0"



static rcl_subscription_t subscriber;
static std_msgs__msg__ColorRGBA msg;

static led_strip_t *strip;
static float current_red;
static float current_green;
static float current_blue;
static float brightness = DEFAULT_BRIGHTNESS;

static void has_set_color() {
  uint8_t red = (uint32_t) (255 * current_red * brightness);
  uint8_t green = (uint32_t) (255 * current_green * brightness);
  uint8_t blue = (uint32_t ) (255 * current_blue * brightness);
  for (size_t i = 0; i < LED_NUMBER; i++) {
    ESP_ERROR_CHECK(strip->set_pixel(strip, i, red, green, blue));
  }
  ESP_ERROR_CHECK(strip->refresh(strip, 100));
}

static void subscription_callback(const void * msgin) {
  const std_msgs__msg__ColorRGBA * _msg = (const std_msgs__msg__ColorRGBA *)msgin;
  current_red = _msg->r;
  current_green = _msg->g;
  current_blue = _msg->b;
  has_set_color();
}

static void brightness_service_callback(const void * req, void * res){
  led_strip_msgs__srv__SetBrightness_Request * req_in = (led_strip_msgs__srv__SetBrightness_Request *) req;
  brightness = req_in->brightness;
  if (brightness < 0) {
    brightness = 0.0;
  } else if (brightness > 1.0) {
    brightness = 1.0;
  }
  has_set_color();
}

void micro_ros_task(void * arg) {
  blue_led_set(1);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

  while(rmw_uros_discover_agent(rmw_options) != RCL_RET_OK){
    ESP_LOGW(TAG, "Micro-ROS agent not found\n");
    usleep(100000);
  }

  // create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, NODE_NS, &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA), "color"));

  // create service
  rcl_service_t brightness_service;
  RCCHECK(rclc_service_init_default(&brightness_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(led_strip_msgs, srv, SetBrightness), "set_brightness"));

  // create executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  led_strip_msgs__srv__SetBrightness_Response res;
  led_strip_msgs__srv__SetBrightness_Request req;
  RCCHECK(rclc_executor_add_service(&executor, &brightness_service, &req, &res, brightness_service_callback));

  blue_led_set(0);
  while(1){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    usleep(10000);
  }

  // free resources
  RCCHECK(rcl_service_fini(&brightness_service, &node));
  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rcl_node_fini(&node));

  vTaskDelete(NULL);
}

void app_main(void) {

  blue_led_init();

  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO, RMT_TX_CHANNEL);
  config.clk_div = 2;

  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

  // initialize ws2812 driver
  led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(LED_NUMBER, (led_strip_dev_t)config.channel);
  strip = led_strip_new_rmt_ws2812(&strip_config);
  if (!strip) {
    ESP_LOGE(TAG, "initialization of WS2812 driver failed");
  }
  ESP_ERROR_CHECK(strip->clear(strip, 100));

#ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif  // UCLIENT_PROFILE_UDP

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
  xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL,
              CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
