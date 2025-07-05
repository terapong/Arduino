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
std_msgs__msg__Int32 msg;

#include <Stepper.h>

// Stepper motor pins
#define IN1 15
#define IN2 2
#define IN3 4
#define IN4 16



// Stepper motor sequence (4-step)
const int stepSequence[4][4] = {
  { 1, 0, 0, 1 },
  { 1, 1, 0, 0 },
  { 0, 1, 1, 0 },
  { 0, 0, 1, 1 }
};

int currentStep = 0;
int targetSteps = 0;
int currentPosition = 0;

// change this to fit the number of steps per revolution for your motor
const int stepsPerRevolution = 2048;  

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 15, 4, 2, 16); ///ต้องแก้ 2 4 เป็น 4 2 ด้วยไหม ต้องแก้ หว่ะ

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Publish current position
    msg.data = currentPosition;
    rcl_publish(&publisher, &msg, NULL);
  }
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  // stepsPerRevolution = msg->data;
  // myStepper.step(stepsPerRevolution);
  targetSteps = msg->data;
  myStepper.step(targetSteps);
}

void setup() {
  // Configure stepper motor pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set up Micro-ROS
  set_microros_transports();
  delay(2000);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);
  // Create node
  rclc_node_init_default(&node, "esp32_stepper_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "stepper_position");

  // Create subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "stepper_target");

  // Create timer
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(100),
    timer_callback);

  // Create executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);
}

void loop() {
  // Execute Micro-ROS tasks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // Move stepper motor if needed
  if (currentPosition < targetSteps) {
    moveStepper(1);  // Move one step forward
    currentPosition++;
  } else if (currentPosition > targetSteps) {
    moveStepper(-1);  // Move one step backward
    currentPosition--;
  }

  delay(1);
}

void moveStepper(int direction) {
  currentStep = (currentStep + direction + 4) % 4;

  digitalWrite(IN1, stepSequence[currentStep][0]);
  digitalWrite(IN2, stepSequence[currentStep][1]);
  digitalWrite(IN3, stepSequence[currentStep][2]);
  digitalWrite(IN4, stepSequence[currentStep][3]);

  delay(2);  // Adjust for motor speed
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
//colcon build --symlink-install
//ros2 topic list
//ros2 topic echo /buzzer_control
//ros2 topic pub --once /buzzer_control std_msgs/Int32 "data: 1000"
//ros2 run button_tester buzzer_control_gui