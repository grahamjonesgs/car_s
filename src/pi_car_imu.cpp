/*
   Func     Colour  GPIO  pin	pin  GPIO	Colour	Func
   IMU Vcc  Brown   -	    1	  2     -
   IMU SDA          2     3   4     -
   IMU SCL          3     5   6     -
                    4	    7	  8     14
   IMU GND          -	    9	  1     15
                    17    11	12    18  White  RB EN
                    27    13  14	  -   LED GND
                    22	  15	16	  23  LED Red
   ENC VCC          -	    17	18	  24  LED Blue
   ENC L            10	  19	20	  -
   ENC R            9	    21	22	  25
   SW RED           11	  23	24	  8
   ENC GND          -     25	26	  7
                    0	    27	28	  1
   SW YELLOW        5	    29	30	  -
   L CTRL1 Orange	  6	    31	32	  12 Pink   RF EN
   LF EN   Red		  13    33	34	  -
   LB EN   Brown		19    35	36	  16 Brown  R CTRL1
   L CTRL2 Yellow	  26	  37	38	  20 Green  R CTRL2
                    -	    39	40	  21
Normalization values
Ave G -0.0617,0.0115,0.0011  A -0.6567,-1.78,10.47
 */

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include <iostream>
#include <pigpiod_if2.h>
#include <stdlib.h>
#include <signal.h>

// Pin definition
#define RED_LED_PIN 23
#define BLUE_LED_PIN 24

// Topics
#define TOPIC_IMU "imu/data_raw"

// Timers
#define IMU_TIMER 0.005 // Interval between IMU readings

// MPU6050 definitions
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define INT_ENABLE 0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

// Global variables
rclcpp::TimerBase::SharedPtr imu_timer; // Timer for stop motors if no message
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
int pi;   // Pi ID from Pigpio
int i2ch; // I2C handle

// Normalization values
/*float NGx = 0.0617;
float NGy = -0.0115;
float NGz = 0.0011;
float NAx = 0.6567;
float NAy = 1.78;
float NAz = 9.81-10.47;*/

// Normalization values
float NGx = 0.05;
float NGy = 0.01;
float NGz = 0.02;
float NAx = 0.68;
float NAy = 0.41;
float NAz = 9.81 - 10.45;

void MPU6050_Init()
{

    i2c_write_byte_data(pi, i2ch, SMPLRT_DIV, 0x07); /* Write to sample rate register */
    i2c_write_byte_data(pi, i2ch, PWR_MGMT_1, 0x01); /* Write to power management register */
    i2c_write_byte_data(pi, i2ch, CONFIG, 0);        /* Write to Configuration register */
    i2c_write_byte_data(pi, i2ch, GYRO_CONFIG, 0);   /*  sensitivity */
    i2c_write_byte_data(pi, i2ch, INT_ENABLE, 0x01); /*Write to interrupt enable register */
}

short read_raw_data(int addr)
{
    short high_byte, low_byte, value;
    high_byte = i2c_read_byte_data(pi, i2ch, addr);
    low_byte = i2c_read_byte_data(pi, i2ch, addr + 1);
    value = (high_byte << 8) | low_byte;
    return value;
}

void sigintHandler([[maybe_unused]] int sig)
{
    // Shuting down

    rclcpp::shutdown();
}

void imu_timer_callback()
{
    // ROS timer callback to publish IMU data
    float Acc_x, Acc_y, Acc_z;
    float Gyro_x, Gyro_y, Gyro_z;
    float Ax = 0, Ay = 0, Az = 0;
    float Gx = 0, Gy = 0, Gz = 0;
    sensor_msgs::msg::Imu imu_msg;

    Acc_x = read_raw_data(ACCEL_XOUT_H);
    Acc_y = read_raw_data(ACCEL_YOUT_H);
    Acc_z = read_raw_data(ACCEL_ZOUT_H);

    Gyro_x = read_raw_data(GYRO_XOUT_H);
    Gyro_y = read_raw_data(GYRO_YOUT_H);
    Gyro_z = read_raw_data(GYRO_ZOUT_H);

    /* Divide raw value by sensitivity scale factor */
    Ax = 9.81 * Acc_x / 16384.0;
    Ay = 9.81 * Acc_y / 16384.0;
    Az = 9.81 * Acc_z / 16384.0;
    // for sensitivity 0 7505, for full sensitivity 1,876.25
    Gx = Gyro_x / 7505;
    Gy = Gyro_y / 7505;
    Gz = Gyro_z / 7505;

    // ROS_INFO("\n Gx=%.3f °/s\tGy=%.3f °/s\tGz=%.3f °/s\tAx=%.3f g\tAy=%.3f g\tAz=%.3f g\n",Gx,Gy,Gz,Ax,Ay,Az);
    imu_msg.header.stamp = rclcpp::Node::make_shared("node_name")->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.angular_velocity.x = Gx + NGx;
    imu_msg.angular_velocity.y = Gy + NGy;
    imu_msg.angular_velocity.z = Gz + NGz;
    imu_msg.linear_acceleration.x = Ax + NAx; // Facing backwards
    imu_msg.linear_acceleration.y = Ay + NAy;
    imu_msg.linear_acceleration.z = Az + NAz;

    imu_pub->publish(imu_msg);
}

int main(int argc, char **argv)
{
    rclcpp::InitOptions init_options;
    rclcpp::init(argc, argv, init_options);
    auto nh = rclcpp::Node::make_shared("pi_car_imu");
    RCLCPP_INFO(nh->get_logger(), "Started Pi Car IMU");
    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigintHandler);
    signal(SIGKILL, sigintHandler);
    if ((pi = pigpio_start(NULL, NULL)) < 0)
    {
        RCLCPP_INFO(nh->get_logger(), "gpio init failed");
        return 1;
    }
    // Set up MPU6050
    i2ch = i2c_open(pi, 1, 0x68, 0);
    MPU6050_Init();

    // Set up ROS
    imu_timer = nh->create_wall_timer(std::chrono::duration<double>(IMU_TIMER), std::bind(&imu_timer_callback));
    imu_pub = nh->create_publisher<sensor_msgs::msg::Imu>(TOPIC_IMU, 2);

    rclcpp::spin(nh);
}