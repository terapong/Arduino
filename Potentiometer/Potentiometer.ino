#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_timer_t timer;

#include <std_msgs/msg/int32.h>
std_msgs__msg__Int32 pot_msg;

const int POTENTIOMETER_PIN = 15;  // GPIO34 is ADC1_CH6
const int LED_PIN = 2;
const int PUBLISH_DELAY_MS = 100;  // 10Hz update rate

// Timer callback - reads potentiometer and publishes value
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    // Read potentiometer (0-4095 for ESP32 ADC)
    int t = analogRead(POTENTIOMETER_PIN);
    int x =  t / 16;
    analogWrite(LED_PIN, x);
    pot_msg.data = t;
    
    
    // Publish the value
    rcl_publish(&publisher, &pot_msg, NULL);
  }
}

void setup() {
  pinMode(POTENTIOMETER_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

// Set up Micro-ROS
  set_microros_transports();
  delay(2000);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);
  // Create node
  rclc_node_init_default(&node, "esp32_potentiometer", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "potentiometer_value");

  // Create timer
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(PUBLISH_DELAY_MS),
    timer_callback);

  // Create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
//colcon build --symlink-install
//ros2 topic list
//os2 topic echo /potentiometer_value 
//ros2 run button_tester led_control_gui