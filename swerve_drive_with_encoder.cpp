#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Robot Dimensions (in meters)
const float wheel_base = 1.0;    // Distance between front and rear wheels
const float track_width = 0.6;   // Distance between left and right wheels
const float rx = wheel_base / 2.0;
const float ry = track_width / 2.0;

const float encoderTicksPerRevolution = 360.0;
const float wheelRadius = 0.25; // 25 cm

// === Pin Definitions ===
const int drivePWMPins[4] = {3, 5, 6, 9};
const int driveDirPins[4] = {4, 7, 8, 12};
const int steerPWMPins[4] = {10, 11, 12, 13};
const int steerDirPins[4] = {A0, A1, A2, A3};

// Encoder pins 
const int driveEncoderPinsA[4] = {22, 24, 26, 28};
const int driveEncoderPinsB[4] = {23, 25, 27, 29};
const int steerEncoderPinsA[4] = {30, 32, 34, 36};
const int steerEncoderPinsB[4] = {31, 33, 35, 37};

// === Global State Variables ===
volatile long driveEncoders[4] = {0, 0, 0, 0};
long lastDriveEncoders[4] = {0, 0, 0, 0};

Servo steerMotors[4];

float vx = 0.0, vy = 0.0, wz = 0.0;
float lastSteerAngle[4] = {0, 0, 0, 0};
float vx_[4] = {0}, vy_[4] = {0};
float speed[4] = {0};
float angle[4] = {0};

float x = 0.0, y = 0.0, theta = 0.0;
float linear_velocity_sum_x = 0.0, linear_velocity_sum_y = 0.0;

ros::NodeHandle nh;

void twistCallback(const geometry_msgs::Twist& msg) {
  vx = msg.linear.x;
  vy = msg.linear.y;
  wz = msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &twistCallback);

unsigned long last_time = 0;
const unsigned long interval = 20;

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(9600);

  for (int i = 0; i < 4; i++) {
    // Drive motor pins
    pinMode(drivePWMPins[i], OUTPUT);
    pinMode(driveDirPins[i], OUTPUT);

    // Steering motor pins
    pinMode(steerPWMPins[i], OUTPUT);
    pinMode(steerDirPins[i], OUTPUT);
    steerMotors[i].attach(steerPWMPins[i]);

    // Encoders
    pinMode(driveEncoderPinsA[i], INPUT);
    pinMode(driveEncoderPinsB[i], INPUT);
    pinMode(steerEncoderPinsA[i], INPUT);
    pinMode(steerEncoderPinsB[i], INPUT);
  }

  last_time = millis();
}

void loop() {
  nh.spinOnce();

  unsigned long current_time = millis();
  if (current_time - last_time >= interval) {
    float dt = (current_time - last_time) / 1000.0;
    last_time = current_time;

    // Compute wheel velocities and steering angles
    vx_[0] = vx + ry * wz; vy_[0] = vy + rx * wz; // FR
    vx_[1] = vx - ry * wz; vy_[1] = vy + rx * wz; // FL
    vx_[2] = vx - ry * wz; vy_[2] = vy - rx * wz; // RL
    vx_[3] = vx + ry * wz; vy_[3] = vy - rx * wz; // RR

    for (int i = 0; i < 4; i++) {
      speed[i] = sqrt(vx_[i] * vx_[i] + vy_[i] * vy_[i]);
      angle[i] = atan2(vy_[i], vx_[i]) * 180.0 / PI;
      if (angle[i] < 0) angle[i] += 360;

      // Optimize steering direction
      if (angle[i] - lastSteerAngle[i] >= 90) {
        angle[i] -= 180;
        speed[i] = -speed[i];
      } else if (lastSteerAngle[i] - angle[i] >= 90) {
        angle[i] += 180;
        speed[i] = -speed[i];
      }

      angle[i] = fmod(angle[i], 360);
      lastSteerAngle[i] = angle[i];

      // Steering servo update
      float servoAngle = fmod(angle[i], 180);
      steerMotors[i].write(constrain(servoAngle, 0, 180));

      // Drive motor control
      bool dir = speed[i] >= 0;
      digitalWrite(driveDirPins[i], dir);
      analogWrite(drivePWMPins[i], constrain(abs(speed[i]) * 255.0, 0, 255));
    }

    // Odometry using encoder
    linear_velocity_sum_x = 0;
    linear_velocity_sum_y = 0;

    for (int i = 0; i < 4; i++) {
      long deltaTicks = driveEncoders[i] - lastDriveEncoders[i];
      lastDriveEncoders[i] = driveEncoders[i];

      float driveRotations = deltaTicks / encoderTicksPerRevolution;
      float driveDistance = driveRotations * 2 * PI * wheelRadius;

      float angle_rad = lastSteerAngle[i] * PI / 180.0;
      float vx_local = (driveDistance / dt) * cos(angle_rad);
      float vy_local = (driveDistance / dt) * sin(angle_rad);

      linear_velocity_sum_x += vx_local;
      linear_velocity_sum_y += vy_local;
    }

    float odom_vx = linear_velocity_sum_x / 4.0;
    float odom_vy = linear_velocity_sum_y / 4.0;

    float odom_dx = (odom_vx * cos(theta) - odom_vy * sin(theta)) * dt;
    float odom_dy = (odom_vx * sin(theta) + odom_vy * cos(theta)) * dt;

    x += odom_dx;
    y += odom_dy;
    theta += wz * dt;

    Serial.print("X: "); Serial.print(x, 4);
    Serial.print(" Y: "); Serial.print(y, 4);
    Serial.print(" Theta: "); Serial.println(theta, 4);
  }
}
