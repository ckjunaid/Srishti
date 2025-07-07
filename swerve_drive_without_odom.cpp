#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Robot Dimensions (in meters)
const float wheel_base = 1.0;    // Distance between front and rear wheels
const float track_width = 0.6;   // Distance between left and right wheels

// Compute wheel velocities and angles
const float rx = wheel_base / 2.0;
const float ry = track_width / 2.0;

// Motor Pins (adjust as per wiring)
const int driveMotorPins[4] = {3, 5, 6, 9};
const int steerMotorPins[4] = {10, 11, 12, 13};

// Steering servos
Servo steerMotors[4];

// Velocity storage
float vx = 0.0, vy = 0.0, wz = 0.0;

// Last known steering angle (in degrees)
float lastSteerAngle[4] = {0, 0, 0, 0};

// ROS Node Handle
ros::NodeHandle nh;

void twistCallback(const geometry_msgs::Twist& msg) {
  vx = msg.linear.x;
  vy = msg.linear.y;
  wz = msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &twistCallback);

// Timing variables
unsigned long last_time = 0;
const unsigned long interval = 20;  // 20 ms = 50 Hz

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  for (int i = 0; i < 4; i++) {
    pinMode(driveMotorPins[i], OUTPUT);
    steerMotors[i].attach(steerMotorPins[i]);
  }
}

void loop() {
  nh.spinOnce();

  unsigned long current_time = millis();
  if (current_time - last_time >= interval) {
    last_time = current_time;

    float vx_[4];
    float vy_[4];
    float speed[4];
    float angle[4];

    // Order: FR, FL, RL, RR (considering anti-clockwise rotation as positive)
    vx_[0] = vx + ry * wz; vy_[0] = vy + rx * wz; // FR
    vx_[1] = vx - ry * wz; vy_[1] = vy + rx * wz; // FL
    vx_[2] = vx - ry * wz; vy_[2] = vy - rx * wz; // RL
    vx_[3] = vx + ry * wz; vy_[3] = vy - rx * wz; // RR

    for (int i = 0; i < 4; i++) {
      speed[i] = sqrt(vx_[i] * vx_[i] + vy_[i] * vy_[i]);
      angle[i] = atan2(vy_[i], vx_[i]) * 180.0 / PI;
      if (angle[i] < 0) angle[i] += 360;

      // Optimize steering: choose shortest path and reverse speed if needed
      if (angle[i] - lastSteerAngle[i] >= 90) {
        angle[i] -= 180;
        speed[i] = -speed[i];
      } else if (lastSteerAngle[i] - angle[i] >= 90) {
        angle[i] += 180;
        speed[i] = -speed[i];
      }

      angle[i] = fmod(angle[i], 360);
      lastSteerAngle[i] = angle[i];

      float servoAngle = fmod(angle[i], 180);
      steerMotors[i].write(constrain(servoAngle, 0, 180));
      analogWrite(driveMotorPins[i], constrain(abs(speed[i]) * 255.0, 0, 255));
    }
  }
}
