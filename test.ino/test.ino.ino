#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN_1 2
#define LED_PIN_2 15

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //msg.data = led_state;
    rcl_publish(&publisher, &msg, NULL);
    msg.data++;
  }
}

void setup() {
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);

  set_microros_transports();  
  delay(2000);
  allocator = rcl_get_default_allocator();
  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);
  // Create node
  rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support);
  
  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "led_status");
  
  // Create timer
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(1000),
    timer_callback);

  // Create executor
  //rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  msg.data = false;
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
