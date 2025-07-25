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
rcl_timer_t timer;

#include <std_msgs/msg/int32.h>
std_msgs__msg__Int32 water_level_msg;

// Water sensor pin (adjust based on your setup)
#define WATER_SENSOR_PIN 15

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

void error_loop() {
  while (1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Read water level from sensor
    int water_level = analogRead(WATER_SENSOR_PIN);

    // Map the value if needed (depends on your sensor)
    // water_level = map(water_level, 0, 4095, 0, 100); // Example for percentage

    water_level_msg.data = water_level;
    RCSOFTCHECK(rcl_publish(&publisher, &water_level_msg, NULL));
  }
}

void setup() {
  pinMode(WATER_SENSOR_PIN, INPUT);

  // Set up Micro-ROS
  set_microros_transports();
  delay(2000);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);
  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_water_sensor", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "water_level"));

  // Create timer
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
//ros2 topic list
//ros2 topic echo /water_level
//ros2 run otto_robot water_sensor_control_gui