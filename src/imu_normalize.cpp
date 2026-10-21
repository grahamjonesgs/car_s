#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <iostream>
#include <signal.h>
#include <stdlib.h>

// Topics
#define TOPIC_IMU "imu/data_raw"

// Sums in double: in float, the averages drift by about 0.03 m/s^2 after
// 100,000 readings (17 minutes), as big as the corrections being measured
long count = 0;
double Gx = 0.0;
double Gy = 0.0;
double Gz = 0.0;
double Ax = 0.0;
double Ay = 0.0;
double Az = 0.0;

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
  count++;
  // ROS callback on message received
  Gx += imu_msg->angular_velocity.x;
  Gy += imu_msg->angular_velocity.y;
  Gz += imu_msg->angular_velocity.z;
  Ax += imu_msg->linear_acceleration.x;
  Ay += imu_msg->linear_acceleration.y;
  Az += imu_msg->linear_acceleration.z;
  // Once a second at 100 Hz, rather than for every reading
  if (count % 100 == 0) {
    RCLCPP_INFO(rclcpp::get_logger("imu_normalize"),
                "Ave Gyro %0.4f,%0.4f,%0.4f  Acc %0.3f,%0.3f,%0.3f  Readings %li",
                Gx / count, Gy / count, Gz / count, Ax / count, Ay / count,
                Az / count, count);
  }
}

int main(int argc, char **argv) {
  rclcpp::InitOptions init_options;
  rclcpp::init(argc, argv, init_options);
  RCLCPP_INFO(rclcpp::get_logger("imu_normalize"),
              "Started Pi Car IMU normalize");
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("imu_normalize");
  // Best effort to match the car_imu publisher
  auto sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(
      TOPIC_IMU, rclcpp::SensorDataQoS(), imu_callback);

  rclcpp::spin(nh);
}