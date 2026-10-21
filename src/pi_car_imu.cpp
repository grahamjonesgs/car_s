/*
                TOP SSD Card

   Func     Colour  GPIO  pin   pin  GPIO       Colour  Func
   IMU Vcc  Brown   3V3   1     2     5v
   IMU SDA          2     3     4     5v   5v to board
   IMU SCL          3     5     6     GND  GND to board
                    4     7     8     14   ENABLE
   IMU GND          GND   9     10    15   RF DIR (A) xxxxxx
                    17    11    12    18   RF STEP (A)
                    27    13    14    GND  LED GND
                    22    15    16    23   LED Red
                    3V3   17    18    24   LED Blue
   M0 SUBSTEP       10    19    20    GND
   M1 SUBSTEP       9     21    22    25
   M2 SUBSTEP       11    23    24    8
                    GND   25    26    7
                    0     27    28    1
   SW YELLOW        5     29    30   GND
   LF DIR (Y)       6     31    32   12   RB STEP (Z)
   LF STEP (Y)      13    33    34   GND
   LB STEP (X)      19    35    36   16   RB DIR (Z) xxxx
   LB DIR (X)       26    37    38   20
                    GND   39    40   21

                  Bottom Camera Port
Normalization values
Ave G -0.0617,0.0115,0.0011  A -0.6567,-1.78,10.47
 */

#include "sensor_msgs/msg/imu.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <pigpiod_if2.h>
#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>

// Pin definition
#define RED_LED_PIN 23
#define BLUE_LED_PIN 24

// Topics
#define TOPIC_IMU "imu/data_raw"

// Defaults for the ROS parameters
#define DEFAULT_RATE_HZ 100.0       // IMU readings per second
#define DEFAULT_CALIBRATION_TIME 2.0 // Seconds of gyro bias calibration at start

// Gyro bias calibration is rejected if the car moves, i.e. if the gyro
// standard deviation on any axis is above this (rad/s)
#define CALIBRATION_MAX_STDDEV 0.01
// If the car moves, calibrate again, up to this many times in all, before
// falling back to the built-in offsets
#define CALIBRATION_ATTEMPTS 5

// Identical readings for this many samples in a row mean the chip has
// stopped converting (e.g. it browned out and woke up asleep): re-initialise
#define FROZEN_SAMPLES 50

// Covariances (diagonals) for the IMU message. Measured at rest the gyro
// variance is about 3e-6 (rad/s)^2; these allow for bias drift and vibration
#define GYRO_VAR 1e-4  // (rad/s)^2
#define ACCEL_VAR 0.05 // (m/s^2)^2

// MPU6050 definitions
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define INT_ENABLE 0x38
#define ACCEL_XOUT_H 0x3B
#define WHO_AM_I 0x75
#define MPU6050_ID 0x68
#define DATA_BYTES 14 // Accel X, Y, Z, temperature, gyro X, Y, Z (big-endian)

// Global variables
rclcpp::TimerBase::SharedPtr imu_timer; // Timer for reading the IMU
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
rclcpp::Node::SharedPtr nh;
int pi;   // Pi ID from Pigpio
int i2ch; // I2C handle

// Normalization values
/*float NGx = 0.0617;
float NGy = -0.0115;
float NGz = 0.0011;
float NAx = 0.6567;
float NAy = 1.78;
float NAz = 9.81-10.47;*/

// Normalization values. The gyro offsets are only used if the start-up
// calibration fails because the car was moving
float NGx = 0.05;
float NGy = 0.01;
float NGz = 0.02;
float NAx = 0.68;
float NAy = 0.41;
float NAz = 9.81 - 10.45;

// Gyro bias calibration state
int calibration_samples_needed = 0;
int calibration_samples = 0; // per attempt
int calibration_attempt = 1;
int calibration_count = 0;
double gyro_sum[3] = {0, 0, 0};
double gyro_sum_sq[3] = {0, 0, 0};

// Set up the MPU6050. Returns false if any write fails
bool MPU6050_Init() {
  // Sample rate = 1 kHz / (1 + 4) = 200 Hz, faster than it is read
  // PWR_MGMT_1 0x01: awake, clocked from the gyro
  // CONFIG 0x03: digital low pass filter at 44 Hz, to cut motor vibration
  // and aliasing
  // GYRO_CONFIG 0: +/-250 deg/s
  // INT_ENABLE 0x01: data ready interrupt
  const int writes[][2] = {{SMPLRT_DIV, 0x04}, {PWR_MGMT_1, 0x01},
                           {CONFIG, 0x03},     {GYRO_CONFIG, 0},
                           {INT_ENABLE, 0x01}};
  for (const auto &w : writes) {
    if (i2c_write_byte_data(pi, i2ch, w[0], w[1]) < 0) {
      return false;
    }
  }
  return true;
}

// Read all the accel and gyro registers in one transfer, so the values all
// come from the same sample and the high and low bytes match
bool read_raw_data(short raw[7]) {
  char buf[DATA_BYTES];
  if (i2c_read_i2c_block_data(pi, i2ch, ACCEL_XOUT_H, buf, DATA_BYTES) !=
      DATA_BYTES) {
    return false;
  }
  for (int i = 0; i < 7; i++) {
    raw[i] = static_cast<short>((static_cast<unsigned char>(buf[2 * i]) << 8) |
                                static_cast<unsigned char>(buf[2 * i + 1]));
  }
  return true;
}

// Add a gyro reading to the start-up bias calibration, and apply the result
// when enough have been collected
void calibrate_gyro(float Gx, float Gy, float Gz) {
  const float g[3] = {Gx, Gy, Gz};
  for (int i = 0; i < 3; i++) {
    gyro_sum[i] += g[i];
    gyro_sum_sq[i] += g[i] * g[i];
  }
  if (++calibration_count < calibration_samples_needed) {
    return;
  }

  double mean[3];
  double max_stddev = 0.0;
  for (int i = 0; i < 3; i++) {
    mean[i] = gyro_sum[i] / calibration_count;
    double var = gyro_sum_sq[i] / calibration_count - mean[i] * mean[i];
    max_stddev = std::max(max_stddev, std::sqrt(std::max(var, 0.0)));
  }
  calibration_samples_needed = 0;

  if (max_stddev > CALIBRATION_MAX_STDDEV) {
    if (calibration_attempt < CALIBRATION_ATTEMPTS) {
      RCLCPP_WARN(nh->get_logger(),
                  "Car moved during gyro calibration (std dev %.4f rad/s), "
                  "trying again (%i of %i), keep it still",
                  max_stddev, calibration_attempt + 1, CALIBRATION_ATTEMPTS);
      calibration_attempt++;
      calibration_count = 0;
      for (int i = 0; i < 3; i++) {
        gyro_sum[i] = gyro_sum_sq[i] = 0.0;
      }
      calibration_samples_needed = calibration_samples;
      return;
    }
    RCLCPP_WARN(nh->get_logger(),
                "Car moved during gyro calibration (std dev %.4f rad/s), "
                "using the built-in offsets %.4f %.4f %.4f",
                max_stddev, NGx, NGy, NGz);
    return;
  }
  NGx = -mean[0];
  NGy = -mean[1];
  NGz = -mean[2];
  RCLCPP_INFO(nh->get_logger(),
              "Gyro calibrated from %i readings, offsets %.4f %.4f %.4f rad/s",
              calibration_count, NGx, NGy, NGz);
}

void imu_timer_callback() {
  // ROS timer callback to publish IMU data
  short raw[7];
  float Ax = 0, Ay = 0, Az = 0;
  float Gx = 0, Gy = 0, Gz = 0;
  sensor_msgs::msg::Imu imu_msg;

  if (!read_raw_data(raw)) {
    RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 2000,
                         "Failed to read IMU");
    return;
  }

  // Real readings always have some noise; exactly the same values for a
  // while mean the chip stopped converting, so don't publish them as data
  static short last_raw[7];
  static int same_count = 0;
  if (std::equal(raw, raw + 7, last_raw)) {
    if (++same_count >= FROZEN_SAMPLES) {
      RCLCPP_ERROR_THROTTLE(nh->get_logger(), *nh->get_clock(), 5000,
                            "IMU readings frozen, re-initialising the MPU6050");
      MPU6050_Init();
      same_count = 0;
    }
    if (same_count >= 3) {
      return;
    }
  } else {
    std::copy(raw, raw + 7, last_raw);
    same_count = 0;
  }

  /* Divide raw value by sensitivity scale factor */
  Ax = 9.81 * raw[0] / 16384.0;
  Ay = 9.81 * raw[1] / 16384.0;
  Az = 9.81 * raw[2] / 16384.0;
  // raw[3] is the temperature
  // 131 LSB per deg/s at +/-250 deg/s, so 7505 LSB per rad/s
  Gx = raw[4] / 7505.0;
  Gy = raw[5] / 7505.0;
  Gz = raw[6] / 7505.0;

  // Don't publish until the gyro bias is known
  if (calibration_samples_needed > 0) {
    calibrate_gyro(Gx, Gy, Gz);
    return;
  }

  // ROS_INFO("\n Gx=%.3f °/s\tGy=%.3f °/s\tGz=%.3f °/s\tAx=%.3f g\tAy=%.3f
  // g\tAz=%.3f g\n",Gx,Gy,Gz,Ax,Ay,Az);
  imu_msg.header.stamp = nh->now();
  imu_msg.header.frame_id = "imu_link";
  imu_msg.orientation_covariance[0] = -1.0; // No orientation estimate
  imu_msg.angular_velocity.x = Gx + NGx;
  imu_msg.angular_velocity.y = Gy + NGy;
  imu_msg.angular_velocity.z = Gz + NGz;
  imu_msg.linear_acceleration.x = Ax + NAx;
  imu_msg.linear_acceleration.y = Ay + NAy;
  imu_msg.linear_acceleration.z = Az + NAz;
  for (int i : {0, 4, 8}) {
    imu_msg.angular_velocity_covariance[i] = GYRO_VAR;
    imu_msg.linear_acceleration_covariance[i] = ACCEL_VAR;
  }

  imu_pub->publish(imu_msg);
}

int main(int argc, char **argv) {
  rclcpp::InitOptions init_options;
  rclcpp::init(argc, argv, init_options);
  nh = rclcpp::Node::make_shared("pi_car_imu");
  RCLCPP_INFO(nh->get_logger(), "Started Pi Car IMU");

  double rate_hz = nh->declare_parameter("rate_hz", DEFAULT_RATE_HZ);
  // 0 skips the calibration and uses the built-in gyro offsets
  double calibration_time =
      nh->declare_parameter("calibration_time", DEFAULT_CALIBRATION_TIME);
  if (rate_hz <= 0.0 || rate_hz > 200.0) {
    RCLCPP_WARN(nh->get_logger(), "rate_hz must be in (0, 200], using %.0f",
                DEFAULT_RATE_HZ);
    rate_hz = DEFAULT_RATE_HZ;
  }
  calibration_samples_needed =
      static_cast<int>(std::max(calibration_time, 0.0) * rate_hz);
  calibration_samples = calibration_samples_needed;

  if ((pi = pigpio_start(NULL, NULL)) < 0) {
    RCLCPP_ERROR(nh->get_logger(), "gpio init failed");
    return 1;
  }
  // Set up MPU6050
  i2ch = i2c_open(pi, 1, 0x68, 0);
  if (i2ch < 0) {
    RCLCPP_ERROR(nh->get_logger(), "Failed to open I2C device");
    pigpio_stop(pi);
    return 1;
  }
  // A missing chip doesn't answer; exit, and the launch restarts the node.
  // Some clone boards report a different ID but work, so only warn then
  int id = i2c_read_byte_data(pi, i2ch, WHO_AM_I);
  if (id < 0) {
    RCLCPP_ERROR(nh->get_logger(), "No MPU6050 answering on I2C bus 1 at 0x68");
    i2c_close(pi, i2ch);
    pigpio_stop(pi);
    return 1;
  }
  if (id != MPU6050_ID) {
    RCLCPP_WARN(nh->get_logger(),
                "Unexpected IMU WHO_AM_I 0x%02X (an MPU6050 reports 0x%02X)",
                id, MPU6050_ID);
  }
  if (!MPU6050_Init()) {
    RCLCPP_ERROR(nh->get_logger(), "Failed to set up the MPU6050");
    i2c_close(pi, i2ch);
    pigpio_stop(pi);
    return 1;
  }
  if (calibration_samples_needed > 0) {
    RCLCPP_INFO(nh->get_logger(),
                "Calibrating gyro for %.1f s, keep the car still",
                calibration_time);
  }

  // Set up ROS. Sensor data QoS (best effort) so late readings are dropped
  // over WiFi rather than resent; robot_localization subscribes this way
  imu_pub = nh->create_publisher<sensor_msgs::msg::Imu>(TOPIC_IMU,
                                                        rclcpp::SensorDataQoS());
  imu_timer = nh->create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz),
                                    std::bind(&imu_timer_callback));

  rclcpp::spin(nh);
  i2c_close(pi, i2ch);
  pigpio_stop(pi); // Stop pigpio
  // Release the global ROS objects before shutting down: destroyed later, as
  // globals at exit, they threw and the node aborted (exit code -6)
  imu_timer.reset();
  imu_pub.reset();
  nh.reset();
  rclcpp::shutdown();
  return 0;
}
