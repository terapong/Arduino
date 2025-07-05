#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

class MicroROSLED {
private:
  rcl_node_t node;
  rcl_subscription_t subscriber;
  rclc_executor_t executor;
  rclc_support_t support;
  rcl_allocator_t allocator;
  std_msgs__msg__Bool msg;
  int led_pin;
  bool led_state;

public:
  MicroROSLED(int pin)
    : led_pin(pin), led_state(false) {}

  void initialize() {
    // Initialize LED pin
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);

    // Micro-ROS setup
    set_microros_transports();

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "micro_ros_led_node", "", &support);

    // Create subscriber
    rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "led_control");

    // Create executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &MicroROSLED::subscription_callback, ON_NEW_DATA);  //subscription_callback มันแยู่ ใน class แล้วนี่หว่า this ON_NEW_DATA
  }

  void spin_some() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }

  static void subscription_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    //MicroROSLED *instance = (MicroROSLED *)executor.context;
    //instance->led_state = msg->data;
    //digitalWrite(instance->led_pin, instance->led_state ? HIGH : LOW);
  }
};

// Usage Example

MicroROSLED led(2);  // LED connected to GPIO2

void setup() {
  led.initialize();
}

void loop() {
  led.spin_some();
  delay(10);
}