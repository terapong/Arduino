#include <micro_ros_arduino.h>
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
std_msgs__msg__Int32 distance_msg;

const int trigPin = 23;
const int echoPin = 22;

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
    // Measure distance
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2;  // Calculate distance in cm

    distance_msg.data = distance;

  // long duration, cm;

  // pinMode(pingPin, OUTPUT);


  // digitalWrite(pingPin, LOW);
  // delayMicroseconds(2);
  // digitalWrite(pingPin, HIGH);
  // delayMicroseconds(5);
  // digitalWrite(pingPin, LOW);
  // pinMode(inPin, INPUT);
  // duration = pulseIn(inPin, HIGH);

  // cm = microsecondsToCentimeters(duration);

    RCSOFTCHECK(rcl_publish(&publisher, &distance_msg, NULL));
  }
}

void setup() {
  // Configure ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Set up Micro-ROS
  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_ultrasonic_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ultrasonic_distance"));

  // Create timer
  const unsigned int timer_timeout = 500;
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
//colcon build --symlink-install
//ros2 topic list
//ros2 topic echo /ultrasonic_distance