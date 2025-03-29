/*

 */

 #include "rclcpp/rclcpp.hpp"
 #include "sensor_msgs/msg/imu.hpp"
 #include <iostream>
 #include <stdlib.h>
 #include <signal.h>
 
 // Topics
 #define TOPIC_IMU "imu/data_raw"
 
 int count=0;
 float Gx=0.0;
 float Gy=0.0;
 float Gz=0.0;
 float Ax=0.0;
 float Ay=0.0;
 float Az=0.0;
 
 void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
 {
         count++;
         // ROS callback on message received
         Gx+=imu_msg->angular_velocity.x;
         Gy+=imu_msg->angular_velocity.y;
         Gz+=imu_msg->angular_velocity.z;
         Ax+=imu_msg->linear_acceleration.x;
         Ay+=imu_msg->linear_acceleration.y;
         Az+=imu_msg->linear_acceleration.z;
         RCLCPP_INFO(rclcpp::get_logger("imu_normalize"), "Ave Giro %0.2f,%0.2f,%0.2f  Acc %0.2f,%0.2f,%0.2f  Readings %i", Gx/count, Gy/count, Gz/count, Ax/count, Ay/count, Az/count, count);
         //ROS_INFO("Act G %0.4f,%0.4f,%0.4f  A %0.4f,%0.2f,%0.2f", Gx,Gy,Gz,Ax,Ay,Az);
 }
 
 int main (int argc, char **argv)
 {
        rclcpp::InitOptions init_options;
         rclcpp::init(argc, argv, init_options);
         RCLCPP_INFO(rclcpp::get_logger("imu_normalize"), "Started Pi Car IMU normalize");
         rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("imu_normalize");
         auto sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(TOPIC_IMU, 100, imu_callback);
 
         rclcpp::spin(nh);
 }