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

#include <std_msgs/msg/string.h>
std_msgs__msg__String msg;

#include <RtcDS1302.h>
ThreeWire myWire(2, 15, 4);
RtcDS1302<ThreeWire> Rtc(myWire);

void setup() {
  // Wire.begin();
  Rtc.Begin();
  RtcDateTime now = Rtc.GetDateTime();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  if (compiled < now) {
    Serial.println("RTC time is okay");
  } else {
    Rtc.SetDateTime(compiled);
    Serial.println("RTC time is not okay");
  }

  // Set up Micro-ROS
  set_microros_transports();
  delay(2000);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);
  // Create node
  rclc_node_init_default(&node, "esp32_rtc_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "rtc_time");
}

void loop() {
  static uint32_t last_publish = 0;

  RtcDateTime now = Rtc.GetDateTime();

  if (millis() - last_publish > 1000) {
    // Format time as string
    char time_str[50];
    sprintf(time_str, "%04d-%02d-%02d %02d:%02d:%02d",
            now.Year(), now.Month(), now.Day(),
            now.Hour(), now.Minute(), now.Second());

    msg.data.data = time_str;
    msg.data.size = strlen(time_str);

    rcl_publish(&publisher, &msg, NULL);
    last_publish = millis();
  }

  // Spin Micro-ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

// ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
// ros2 topic echo /rtc_time
// ros2 run otto_robot rtc_control_gui
// CONNECTIONS:
// DS1302 CLK/SCLK --> 15
// DS1302 DAT/IO --> 2
// DS1302 RST/CE --> 4
// DS1302 VCC --> 3.3v - 5v
// DS1302 GND --> GND
