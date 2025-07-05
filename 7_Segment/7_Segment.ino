#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include <std_msgs/msg/int32.h>
std_msgs__msg__Int32 msg;

rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

#define LED_PIN_A 4
#define LED_PIN_B 16
#define LED_PIN_C 18
#define LED_PIN_D 5
#define LED_PIN_E 17
#define LED_PIN_F 2
#define LED_PIN_G 15
#define LED_PIN_DP 19

void segment7(int _a, int _b, int _c, int _d, int _e, int _f, int _g, int _dp) {
  digitalWrite(LED_PIN_A, _a);
  digitalWrite(LED_PIN_B, _b);
  digitalWrite(LED_PIN_C, _c);
  digitalWrite(LED_PIN_D, _d);
  digitalWrite(LED_PIN_E, _e);
  digitalWrite(LED_PIN_F, _f);
  digitalWrite(LED_PIN_G, _g);
  digitalWrite(LED_PIN_DP, _dp);
}

void selectValue(int value) {
  if (value == 0) {
    segment7(HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, LOW, LOW);
  } else if (value == 1) {
    segment7(LOW, HIGH, HIGH, LOW, LOW, LOW, LOW, LOW);
  } else if (value == 2) {
    segment7(HIGH, HIGH, LOW, HIGH, HIGH, LOW, HIGH, LOW);
  } else if (value == 3) {
    segment7(HIGH, HIGH, HIGH, HIGH, LOW, LOW, HIGH, LOW);
  } else if (value == 4) {
    segment7(LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH, LOW);
  } else if (value == 5) {
    segment7(HIGH, LOW, HIGH, HIGH, LOW, HIGH, HIGH, LOW);
  } else if (value == 6) {
    segment7(HIGH, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, LOW);
  } else if (value == 7) {
    segment7(HIGH, HIGH, HIGH, LOW, LOW, LOW, LOW, LOW);
  } else if (value == 8) {
    segment7(HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, LOW);
  } else if (value == 9) {
    segment7(HIGH, HIGH, HIGH, HIGH, LOW, HIGH, HIGH, LOW);
  } else if (value == 10) {
    segment7(LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH);
  }
}

void subscription_callback(const void* msgin) {
  const std_msgs__msg__Int32 * msg_toto = (const std_msgs__msg__Int32 *)msgin;
  selectValue(msg_toto->data);
  msg.data = msg_toto->data;
  rcl_publish(&publisher, &msg, NULL); //มันส่งมาแล้วนี่หว่า
}

void setup() {
  pinMode(LED_PIN_A, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  pinMode(LED_PIN_C, OUTPUT);
  pinMode(LED_PIN_D, OUTPUT);
  pinMode(LED_PIN_E, OUTPUT);
  pinMode(LED_PIN_F, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  pinMode(LED_PIN_DP, OUTPUT);

// Set up Micro-ROS
  set_microros_transports();
  delay(2000);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);
  // Create node
  rclc_node_init_default(&node, "otto_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "segment7_state");

  // Create subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "segment7_command");

  // Create executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
//ros2 topic list
//ros2 topic echo /segment7_state
//ros2 topic pub --once /segment7_command std_msgs/msg/Int32 "data: 1"
//ros2 topic pub --once /segment7_command std_msgs/msg/Int32 "data: 2"
//ros2 run otto_robot segment7_control_gui
