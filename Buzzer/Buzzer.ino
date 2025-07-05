#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;



#define BUZZER_PIN 15 
// #define BUZZER_PIN 23

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  
  if (msg->data > 0) {
    // Turn on buzzer with frequency (for passive buzzer)
    tone(BUZZER_PIN, msg->data); // msg->data contains frequency
    // For active buzzer: digitalWrite(BUZZER_PIN, HIGH);
  } else {
    // Turn off buzzer
    noTone(BUZZER_PIN);
    // For active buzzer: digitalWrite(BUZZER_PIN, LOW);
  }
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);

  // Set up Micro-ROS
  set_microros_transports();
  delay(2000);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);
  // Create node
  rclc_node_init_default(&node, "toto_led", "", &support);
  
  // Create subscriber
  rcl_ret_t rc = rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "buzzer_control");
    
  // Create executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rc = rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  
  // Spin executor in loop
  while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    delay(100);
  }
}

void loop() {
  // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  //   delay(100);
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
//ros2 topic list
//ros2 topic echo /buzzer_control
//ros2 topic pub --once /buzzer_control std_msgs/Int32 "data: 1000"
//ros2 run button_tester buzzer_control_gui