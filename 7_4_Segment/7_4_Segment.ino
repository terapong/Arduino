#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

unsigned long lastTime = 0;
unsigned int digitPos = 1;

unsigned int exitLoop = true;

#define LED_PIN_A 2
#define LED_PIN_B 5
#define LED_PIN_C 3
#define LED_PIN_D 19
#define LED_PIN_E 18
#define LED_PIN_F 4
#define LED_PIN_G 1
#define LED_PIN_DP 21
#define LED_PIN_D1 15
#define LED_PIN_D2 16
#define LED_PIN_D3 17
#define LED_PIN_D4 22

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

void showDigit(int value) {
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

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  //RCLC_UNUSED(last_call_time);
  //if (timer != NULL) {
  //  msg.data = led_state;
  //  rcl_publish(&publisher, &msg, NULL);
  //}
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg_toto = (const std_msgs__msg__Int32 *)msgin;
  unsigned int value = msg_toto->data;

  pinMode(LED_PIN_D1, OUTPUT);
  pinMode(LED_PIN_D2, OUTPUT);
  pinMode(LED_PIN_D3, OUTPUT);
  pinMode(LED_PIN_D4, OUTPUT);

  msg.data = msg_toto->data;
  rcl_publish(&publisher, &msg, NULL);  //มันส่งมาแล้วนี่หว่า

  exitLoop = true;  //รับค่าใหม่ แล้ว loop
  while (exitLoop) {
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 1) {
      lastTime = currentTime;
      digitPos++;
      if (digitPos > 4) {
        digitPos = 1;
      }
      unsigned int digitToShow = (value / (int)(pow(10, 4 - digitPos) + 0.5)) % 10;
      // Publish the LED state
      if (digitPos == 1) {  //bug ก่อนว่าเข้าไหม น่ะ
        showDigit(digitToShow);
        digitalWrite(LED_PIN_D1, LOW);
        digitalWrite(LED_PIN_D2, HIGH);
        digitalWrite(LED_PIN_D3, HIGH);
        digitalWrite(LED_PIN_D4, HIGH);
      } else if (digitPos == 2) {
        showDigit(digitToShow);
        digitalWrite(LED_PIN_D1, HIGH);
        digitalWrite(LED_PIN_D2, LOW);
        digitalWrite(LED_PIN_D3, HIGH);
        digitalWrite(LED_PIN_D4, HIGH);
      } else if (digitPos == 3) {
        showDigit(digitToShow);
        digitalWrite(LED_PIN_D1, HIGH);
        digitalWrite(LED_PIN_D2, HIGH);
        digitalWrite(LED_PIN_D3, LOW);
        digitalWrite(LED_PIN_D4, HIGH);
      } else {
        showDigit(digitToShow);
        digitalWrite(LED_PIN_D1, HIGH);
        digitalWrite(LED_PIN_D2, HIGH);
        digitalWrite(LED_PIN_D3, HIGH);
        digitalWrite(LED_PIN_D4, LOW);
      }
    }
  }
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
  allocator = rcl_get_default_allocator();

  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "toto_node", "", &support);

  // Create int
  rclc_timer_init_default(  //บังคับ ให้ต้องมี
    &timer,
    &support,
    RCL_MS_TO_NS(1000),
    timer_callback);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "segment7_4_state");

  // Create subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "segment7_4_command");

  // Create executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

  rclc_executor_add_timer(&executor, &timer);

  msg.data = false;
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

//ros2 topic echo /segment7_4_state
//ros2 topic pub /segment7_4_command std_msgs/msg/Int32 "data: 1"
//Terapong potisuwan test GIT
//Ying potisuwan test GIT
///tototototototo