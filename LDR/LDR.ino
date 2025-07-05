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
rcl_timer_t timer;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

#define LDR_PIN 15  // Analog pin connected to LDR

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }



// Error handling loop
void error_loop() {
  while (1) {
    delay(100);
  }
}

// Timer callback - reads LDR and publishes value
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    // Read LDR value (0-4095 on ESP32)
    int ldr_value = analogRead(LDR_PIN);
    // Fill message
    msg.data = ldr_value;
    // Publish message
    rcl_publish(&publisher, &msg, NULL);
    // Optional: print to serial for debugging
    Serial.printf("LDR Value: %d\n", ldr_value);
  }
}

void setup() {
  pinMode(LDR_PIN, INPUT);

  // Set up Micro-ROS
  set_microros_transports();
  delay(2000);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);
  // Create node
  rclc_node_init_default(&node, "esp32_ldr_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ldr_value");

  //Create timer (publishes every 500ms)
  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  // Optional: send a startup message
  msg.data = -1;
  rcl_publish(&publisher, &msg, NULL);
}

void loop() {
  // Spin the executor to process incoming messages and timer callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
//colcon build --symlink-install
//ros2 topic list
//ros2 topic echo /ldr_value
//ros2 run button_tester ros2 run button_tester ldr_control_gui
