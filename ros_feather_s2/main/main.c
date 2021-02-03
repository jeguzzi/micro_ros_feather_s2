#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_uros/options.h>
#include <uros_network_interfaces.h>
#include "uxr/client/config.h"
#include <std_msgs/msg/color_rgba.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/illuminance.h>
#include <sensor_msgs/msg/temperature.h>
#include <led_strip_msgs/srv/set_brightness.h>

#include "apa102.h"
#include "ldo_2.h"
#include "blue_led.h"
#include "ambient_light_sensor.h"
#include "temperature_sensor.h"

// ISSUES
// - cannot use more than one subscriber
// - have no access to console: no usb + wifi/ros, no cable for uart
// - adc1_get_raw((adc1_channel_t)channel) is crashing the program

#define EXPOSE_APA102
// #define EXPOSE_BLUE_LED
#define EXPOSE_TEMPERATURE
// #define EXPOSE_ILLUMINANCE
#ifdef EXPOSE_APA102
#define BRIGHTNESS_SERVICE
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#ifdef EXPOSE_TEMPERATURE
rcl_publisher_t temperature_publisher;
sensor_msgs__msg__Temperature temperature_msg;
#endif
#ifdef EXPOSE_ILLUMINANCE
rcl_publisher_t illuminance_publisher;
sensor_msgs__msg__Illuminance illuminance_msg;
#endif
#ifdef EXPOSE_BLUE_LED
rcl_subscription_t blue_led_subscriber;
std_msgs__msg__Bool blue_led_msg;
#endif
#ifdef EXPOSE_APA102
rcl_subscription_t apa102_subscriber;
std_msgs__msg__ColorRGBA apa102_msg;
#endif

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  if (timer != NULL) {
    // TODO(Jerome): add time stamps
#ifdef EXPOSE_TEMPERATURE
    temperature_msg.temperature = (double) temperature_sensor_read();
    temperature_msg.header.stamp.sec = ts.tv_sec;
    temperature_msg.header.stamp.nanosec = ts.tv_nsec;
    RCSOFTCHECK(rcl_publish(&temperature_publisher, &temperature_msg, NULL));
#endif
    // 2000 Lx/V is a reasonable value from the datasheet.
    // The actual value depends on the kind of light.
#ifdef EXPOSE_ILLUMINANCE
    illuminance_msg.illuminance = (double)(ambient_read() * 2.0);
    illuminance_msg.header.stamp.sec = ts.tv_sec;
    illuminance_msg.header.stamp.nanosec = ts.tv_nsec;
    RCSOFTCHECK(rcl_publish(&illuminance_publisher, &illuminance_msg, NULL));
#endif
  }
}

#ifdef EXPOSE_APA102
static float brightness = -1.0;
void apa102_subscription_callback(const void * msgin)
{
  const std_msgs__msg__ColorRGBA * _msg = (const std_msgs__msg__ColorRGBA *)msgin;
  uint8_t red = (uint8_t) (255 * _msg->r);
  uint8_t green = (uint8_t) (255 * _msg->g);
  uint8_t blue = (uint8_t ) (255 * _msg->b);
  uint8_t l;
  if((brightness >= 0) && (brightness <= 1)) {
    l = (uint8_t ) (31 * brightness);
  }
  else {
    l = (uint8_t ) (31 * _msg->a);
  }

  apa102_set_color(red, green, blue, l);
}
#endif

#ifdef BRIGHTNESS_SERVICE
void brightness_service_callback(const void * req, void * res){
  led_strip_msgs__srv__SetBrightness_Request * req_in = (led_strip_msgs__srv__SetBrightness_Request *) req;
  // led_strip_msgs__srv__SetBrightness_Response * res_in = (led_strip_msgs__srv__SetBrightness_Response *) res;
  brightness = req_in->brightness;
  if(brightness > 1.0) {
    brightness = 1.0;
  }
  if(brightness >= 0) {
    uint8_t l = (uint8_t)(31 * brightness);
    apa102_set_brightness(l);
  }
}
#endif

#ifdef EXPOSE_BLUE_LED
void blue_led_subscription_callback(const void * msgin)
{
  const std_msgs__msg__Bool * _msg = (const std_msgs__msg__Bool *)msgin;
  uint8_t value = _msg->data & 1;
  blue_led_set(value);
}
#endif

void micro_ros_task(void * arg)
{

  blue_led_set(1);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

  while(rmw_uros_discover_agent(rmw_options)) {
    usleep(10000);
  }

  // create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "feathers2", "feathers2", &support));
  unsigned handles = 0;

  // create publishers
#ifdef EXPOSE_TEMPERATURE
  RCCHECK(rclc_publisher_init_default(
    &temperature_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
    "temperature"));
  temperature_msg.header.frame_id.data = "feathers2";
  temperature_msg.header.frame_id.capacity = temperature_msg.header.frame_id.size = strlen(temperature_msg.header.frame_id.data);
  temperature_msg.header.stamp.sec=0;
  temperature_msg.header.stamp.nanosec=0;
  temperature_msg.variance=0.0;
#endif
#ifdef EXPOSE_ILLUMINANCE
  RCCHECK(rclc_publisher_init_default(
    &illuminance_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Illuminance),
    "illuminance"));
  illuminance_msg.header.frame_id.data = "feathers2";
  illuminance_msg.header.frame_id.capacity = illuminance_msg.header.frame_id.size = strlen(illuminance_msg.header.frame_id.data);
  illuminance_msg.header.stamp.sec=0;
  illuminance_msg.header.stamp.nanosec=0;
  illuminance_msg.variance=0.0;
#endif

  // create subscriber
#ifdef EXPOSE_APA102
  RCCHECK(rclc_subscription_init_default(
    &apa102_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA), "apa102"));
    handles++;
#endif
#ifdef EXPOSE_BLUE_LED
  RCCHECK(rclc_subscription_init_default(
    &blue_led_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "blue_led"));
    handles++;
#endif

  // create service
#ifdef BRIGHTNESS_SERVICE
  rcl_service_t brightness_service;
  RCCHECK(rclc_service_init_default(&brightness_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(led_strip_msgs, srv, SetBrightness), "set_brightness"));
  handles++;
#endif

  // create timer,
  rcl_timer_t timer;
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));
  handles++;

  // create executor
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, handles, &allocator));
#ifdef EXPOSE_APA102
  RCCHECK(rclc_executor_add_subscription(&executor, &apa102_subscriber, &apa102_msg, &apa102_subscription_callback, ON_NEW_DATA));
#endif
#ifdef EXPOSE_BLUE_LED
  RCCHECK(rclc_executor_add_subscription(&executor, &blue_led_subscriber, &blue_led_msg, &blue_led_subscription_callback, ON_NEW_DATA));
#endif
#ifdef BRIGHTNESS_SERVICE
  led_strip_msgs__srv__SetBrightness_Response res;
  led_strip_msgs__srv__SetBrightness_Request req;
  RCCHECK(rclc_executor_add_service(&executor, &brightness_service, &req, &res, brightness_service_callback));
#endif
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  blue_led_set(0);
  while(1){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    usleep(10000);
  }

  // free resources
  RCCHECK(rcl_timer_fini(&timer));
#ifdef EXPOSE_TEMPERATURE
  RCCHECK(rcl_publisher_fini(&temperature_publisher, &node));
#endif
#ifdef EXPOSE_ILLUMINANCE
  RCCHECK(rcl_publisher_fini(&illuminance_publisher, &node));
#endif
#ifdef EXPOSE_APA102
  RCCHECK(rcl_subscription_fini(&apa102_subscriber, &node));
#endif
#ifdef EXPOSE_BLUE_LED
  RCCHECK(rcl_subscription_fini(&blue_led_subscriber, &node));
#endif
#ifdef BRIGHTNESS_SERVICE
  RCCHECK(rcl_service_fini(&brightness_service, &node));
#endif
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));

  vTaskDelete(NULL);
}


void app_main(void)
{
  ldo_2_init();
  ldo_2_enable(true);
  blue_led_init();
#ifdef EXPOSE_APA102
  apa102_init();
#endif
#ifdef EXPOSE_TEMPERATURE
  temperature_sensor_init();
#endif
#ifdef EXPOSE_ILLUMINANCE
  ambient_init();
#endif
  vTaskDelay(100 / portTICK_PERIOD_MS);

#ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif  // UCLIENT_PROFILE_UDP

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
  xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL,
              CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
