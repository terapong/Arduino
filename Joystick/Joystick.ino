#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joy.h>

// Pin definitions
#define JOY_X_PIN 15
#define JOY_Y_PIN 2
#define JOY_BTN_PIN 4
#define EZ_BTN_PIN 16

rcl_publisher_t publisher;
sensor_msgs__msg__Joy joy_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void setup() {
  // Initialize Micro-ROS
  set_microros_transports();

  // Initialize pins
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);
  pinMode(EZ_BTN_PIN, INPUT_PULLUP);

  // Initialize ROS structures
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_joystick_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "joystick_input");

  // Initialize joy message
  joy_msg.axes.data = (float*)malloc(2 * sizeof(float));
  joy_msg.axes.size = 2;
  joy_msg.buttons.data = (int32_t*)malloc(2 * sizeof(int32_t));
  joy_msg.buttons.size = 2;
}

void loop() {
  // Read joystick values (0-4095 for ESP32 ADC)
  int x_raw = analogRead(JOY_X_PIN);
  int y_raw = analogRead(JOY_Y_PIN);

  // Convert to -1.0 to 1.0 range
  joy_msg.axes.data[0] = (x_raw - 2048) / 2048.0;
  joy_msg.axes.data[1] = (y_raw - 2048) / 2048.0;

  // Read buttons (inverted because we're using pullups)
  joy_msg.buttons.data[0] = !digitalRead(JOY_BTN_PIN);
  joy_msg.buttons.data[1] = !digitalRead(EZ_BTN_PIN);

  // Publish message
  rcl_publish(&publisher, &joy_msg, NULL);

  delay(50);  // 20Hz update rate
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
//colcon build --symlink-install
//ros2 topic list
//ros2 topic echo /joystick_input
