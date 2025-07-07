#include <Servo.h>
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <math.h>

constexpr double PI = 3.14159265358979323846;


//update these values
const float wheel_base = 1.0;
const float track_width = 0.6;
const float encoderTicksPerRevolution = 900.0;
const float wheelRadius = 0.25;
const float max_robot_linear_speed = 2.0;

const float rx = wheel_base / 2.0;
const float ry = track_width / 2.0;


const int drivePWMPins[4] = {3, 5, 6, 9};
const int driveDirPins[4] = {4, 7, 8, 12};
const int steerPWMPins[4] = {10, 11, A4, A5};
const int steerDirPins[4] = {A0, A1, A2, A3};
const int driveEncoderPinsA[4] = {22, 24, 26, 28};
const int driveEncoderPinsB[4] = {23, 25, 27, 29};
const int steerEncoderPinsA[4] = {30, 32, 34, 36};
const int steerEncoderPinsB[4] = {31, 33, 35, 37};

volatile long driveEncoders[4] = {0};
long lastDriveEncoders[4] = {0};
Servo steerMotors[4];

float vx = 0.0, vy = 0.0, wz = 0.0;
float lastSteerAngle[4] = {0};
float vx_[4] = {0}, vy_[4] = {0};
float speed[4] = {0};
float angle[4] = {0};

float x = 0.0, y = 0.0, theta = 0.0;
float linear_velocity_sum_x = 0.0, linear_velocity_sum_y = 0.0;

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

rcl_publisher_t encoder_pub;
std_msgs__msg__Int32MultiArray encoder_msg;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_twist;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_node_t node;

enum states { WAITING_AGENT, AGENT_CONNECTED, AGENT_DISCONNECTED } micro_ros_state;

void error_loop() {
  while (1) {
    delay(100);
    Serial.print("Micro-ROS agent disconnected. Retrying...");
    set_microros_transports();
    micro_ros_state = WAITING_AGENT;
    break;
  }
}

void twistCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  vx = msg->linear.x;
  vy = msg->linear.y;
  wz = msg->angular.z;
}

unsigned long last_time = 0;
const unsigned long interval = 20;

void driveEncoderISR(int i) {
  if (digitalRead(driveEncoderPinsA[i]) == digitalRead(driveEncoderPinsB[i])) driveEncoders[i]++;
  else driveEncoders[i]--;
}

void driveEncoder0_ISR() { driveEncoderISR(0); }
void driveEncoder1_ISR() { driveEncoderISR(1); }
void driveEncoder2_ISR() { driveEncoderISR(2); }
void driveEncoder3_ISR() { driveEncoderISR(3); }

void steerEncoder0_ISR() {}
void steerEncoder1_ISR() {}
void steerEncoder2_ISR() {}
void steerEncoder3_ISR() {}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup_microros() {
  micro_ros_state = WAITING_AGENT;
  Serial.print("Attempting to connect to Micro-ROS agent");
  set_microros_transports();
  allocator = rcl_get_default_allocator();

  while (micro_ros_state == WAITING_AGENT) {
    if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
      delay(500);
      continue;
    }
    micro_ros_state = AGENT_CONNECTED;
    Serial.println("Agent connected!");

    if (rclc_node_init_default(&node, "robot_node", "", &allocator) != RCL_RET_OK) error_loop();

    if (rclc_subscription_init_default(&subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel") != RCL_RET_OK) error_loop();

    if (rclc_publisher_init_default(&odom_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom") != RCL_RET_OK) error_loop();

    if (rclc_publisher_init_default(&encoder_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/encoder_ticks") != RCL_RET_OK) error_loop();

    encoder_msg.data.capacity = 4;
    encoder_msg.data.size = 4;
    encoder_msg.data.data = (int32_t *) malloc(sizeof(int32_t) * 4);

    rclc_executor_init(&executor, &allocator, 1, &node);
    rclc_executor_add_subscription(&executor, &subscriber, &msg_twist, &twistCallback, ON_NEW_DATA);
  }
}

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 4; i++) {
    pinMode(drivePWMPins[i], OUTPUT);
    pinMode(driveDirPins[i], OUTPUT);
    steerMotors[i].attach(steerPWMPins[i]);
    pinMode(driveEncoderPinsA[i], INPUT_PULLUP);
    pinMode(driveEncoderPinsB[i], INPUT_PULLUP);
    pinMode(steerEncoderPinsA[i], INPUT_PULLUP);
    pinMode(steerEncoderPinsB[i], INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(driveEncoderPinsA[0]), driveEncoder0_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(driveEncoderPinsA[1]), driveEncoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(driveEncoderPinsA[2]), driveEncoder2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(driveEncoderPinsA[3]), driveEncoder3_ISR, CHANGE);

  last_time = millis();
  setup_microros();
}

void loop() {
  if (micro_ros_state == AGENT_CONNECTED) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
      micro_ros_state = AGENT_DISCONNECTED;
      Serial.println("Agent disconnected!");
    }
  } else if (micro_ros_state == AGENT_DISCONNECTED) {
    setup_microros();
  }

  unsigned long current_time = millis();
  if (current_time - last_time >= interval) {
    float dt = (current_time - last_time) / 1000.0;
    last_time = current_time;

    vx_[0] = vx + ry * wz; vy_[0] = vy + rx * wz;
    vx_[1] = vx - ry * wz; vy_[1] = vy + rx * wz;
    vx_[2] = vx - ry * wz; vy_[2] = vy - rx * wz;
    vx_[3] = vx + ry * wz; vy_[3] = vy - rx * wz;

    for (int i = 0; i < 4; i++) {
      speed[i] = sqrt(vx_[i] * vx_[i] + vy_[i] * vy_[i]);
      angle[i] = atan2(vy_[i], vx_[i]) * 180.0 / PI;
      if (angle[i] < 0) angle[i] += 360;

      if (angle[i] - lastSteerAngle[i] >= 90) 
      { angle[i] -= 180; speed[i] = -speed[i]; }
      else if (lastSteerAngle[i] - angle[i] >= 90) 
      { angle[i] += 180; speed[i] = -speed[i]; }

      angle[i] = fmod(angle[i], 360);
      if (angle[i] > 180) angle[i] -= 360;
      else if (angle[i] < -180) angle[i] += 360;

      lastSteerAngle[i] = angle[i];

      float servoAngle = map(angle[i], -90, 90, 0, 180);
      steerMotors[i].write(constrain(servoAngle, 0, 180));

      bool dir = speed[i] >= 0;
      digitalWrite(driveDirPins[i], dir);
      analogWrite(drivePWMPins[i], constrain(abs(speed[i]) / max_robot_linear_speed * 255.0, 0, 255));
    }

    linear_velocity_sum_x = 0;
    linear_velocity_sum_y = 0;

    for (int i = 0; i < 4; i++) {
      long deltaTicks = driveEncoders[i] - lastDriveEncoders[i];
      lastDriveEncoders[i] = driveEncoders[i];
      float driveRotations = (float)deltaTicks / encoderTicksPerRevolution;
      float driveDistance = driveRotations * 2 * PI * wheelRadius;
      float angle_rad = lastSteerAngle[i] * PI / 180.0;
      float vx_local = (driveDistance / dt) * cos(angle_rad);
      float vy_local = (driveDistance / dt) * sin(angle_rad);
      linear_velocity_sum_x += vx_local;
      linear_velocity_sum_y += vy_local;
      encoder_msg.data.data[i] = driveEncoders[i];
    }

    float odom_vx = linear_velocity_sum_x / 4.0;
    float odom_vy = linear_velocity_sum_y / 4.0;
    float odom_dx = (odom_vx * cos(theta) - odom_vy * sin(theta)) * dt;
    float odom_dy = (odom_vx * sin(theta) + odom_vy * cos(theta)) * dt;
    x += odom_dx; y += odom_dy; theta += wz * dt;

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
    odom_msg.twist.twist.linear.x = odom_vx;
    odom_msg.twist.twist.linear.y = odom_vy;
    odom_msg.twist.twist.angular.z = wz;

    rcl_publish(&odom_publisher, &odom_msg, NULL);
    rcl_publish(&encoder_pub, &encoder_msg, NULL);

  }
}
