/*
   Func     Colour  GPIO  pin	pin  GPIO	Colour	Func
   MAG Vcc  Brown   -	    1	  2     -
   MAG SDA          2     3   4     -
   MAG SCL          3     5   6     -
                    4	    7	  8     14
   MAG GND          -	    9	  1     15
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
#include "sensor_msgs/msg/magnetic_field.hpp"

#include <iostream>
#include <pigpiod_if2.h>
#include <stdlib.h>
#include <signal.h>

// Topics
#define TOPIC_MAG "imu/mag"

// Timers
#define MAG_TIMER 0.02 // Interval between MAG readings

// MPU6050 definitions
/* The default I2C address of this chip */
#define QMC5883L_ADDR 0x0D
// #define QMC5883L_ADDR 0x01

/* Register numbers */
#define QMC5883L_X_LSB 0
#define QMC5883L_X_MSB 1
#define QMC5883L_Y_LSB 2
#define QMC5883L_Y_MSB 3
#define QMC5883L_Z_LSB 4
#define QMC5883L_Z_MSB 5
#define QMC5883L_STATUS 6
#define QMC5883L_TEMP_LSB 7
#define QMC5883L_TEMP_MSB 8
#define QMC5883L_CONFIG 9
#define QMC5883L_CONFIG2 10
#define QMC5883L_RESET 11
#define QMC5883L_RESERVED 12
#define QMC5883L_CHIP_ID 13

/* Bit values for the STATUS register */
#define QMC5883L_STATUS_DRDY 1
#define QMC5883L_STATUS_OVL 2
#define QMC5883L_STATUS_DOR 4

/* Oversampling values for the CONFIG register */
#define QMC5883L_CONFIG_OS512 0b00000000
#define QMC5883L_CONFIG_OS256 0b01000000
#define QMC5883L_CONFIG_OS128 0b10000000
#define QMC5883L_CONFIG_OS64 0b11000000

/* Range values for the CONFIG register */
#define QMC5883L_CONFIG_2GAUSS 0b00000000
#define QMC5883L_CONFIG_8GAUSS 0b00010000

/* Rate values for the CONFIG register */
#define QMC5883L_CONFIG_10HZ 0b00000000
#define QMC5883L_CONFIG_50HZ 0b00000100
#define QMC5883L_CONFIG_100HZ 0b00001000
#define QMC5883L_CONFIG_200HZ 0b00001100

/* Mode values for the CONFIG register */
#define QMC5883L_CONFIG_STANDBY 0b00000000
#define QMC5883L_CONFIG_CONT 0b00000001

/* Apparently M_PI isn't available in all environments. */
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

// Global variables
rclcpp::TimerBase::SharedPtr mag_timer;
rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
int pi;   // Pi ID from Pigpio
int i2ch; // I2C handle
rclcpp::Node::SharedPtr nh;

int16_t xhigh, x_low;
int16_t y_high, y_low;
uint8_t mode;
uint8_t rate;
uint8_t range;
uint8_t oversampling;

// Normalisation values
float DigitPermg;

short read_raw_data(int addr)
{
    short high_byte, low_byte, value;
    high_byte = i2c_read_byte_data(pi, i2ch, addr + 1);
    low_byte = i2c_read_byte_data(pi, i2ch, addr);
    value = (high_byte << 8) | low_byte;
    return value;
}

void reconfigure()
{
    i2c_write_byte_data(pi, i2ch, QMC5883L_CONFIG, oversampling | range | rate | mode);
}

void reset()
{
    i2c_write_byte_data(pi, i2ch, QMC5883L_RESET, 0x01);
    reconfigure();
}

void setOversampling(int x)
{
    switch (x)
    {
    case 512:
        oversampling = QMC5883L_CONFIG_OS512;
        break;
    case 256:
        oversampling = QMC5883L_CONFIG_OS256;
        break;
    case 128:
        oversampling = QMC5883L_CONFIG_OS128;
        break;
    case 64:
        oversampling = QMC5883L_CONFIG_OS64;
        break;
    }
    reconfigure();
}

void setRange(int x)
{
    switch (x)
    {
    case 2:
        range = QMC5883L_CONFIG_2GAUSS;
        DigitPermg = 12.0;
        break;
    case 8:
        range = QMC5883L_CONFIG_8GAUSS;
        DigitPermg = 3.0;
        break;
    }
    reconfigure();
}

void setSamplingRate(int x)
{
    switch (x)
    {
    case 10:
        rate = QMC5883L_CONFIG_10HZ;
        break;
    case 50:
        rate = QMC5883L_CONFIG_50HZ;
        break;
    case 100:
        rate = QMC5883L_CONFIG_100HZ;
        break;
    case 200:
        rate = QMC5883L_CONFIG_200HZ;
        break;
    }
    reconfigure();
}

void QMC5883L_init()
{
    /* This assumes the wire library has been initialized. */
    // addr = QMC5883L_ADDR;
    setOversampling(512);
    setRange(2);
    setSamplingRate(100);
    mode = QMC5883L_CONFIG_CONT;
    reset();
}

int readRaw(int16_t *x, int16_t *y, int16_t *z, [[maybe_unused]] int16_t *t)
{
    *x = read_raw_data(QMC5883L_X_LSB);
    *y = read_raw_data(QMC5883L_Y_LSB);
    *z = read_raw_data(QMC5883L_Z_LSB);
    return 1;
}

void resetCalibration()
{
    xhigh = y_high = 0;
    x_low = y_low = 0;
}

int readHeading()
{
    int16_t x, y, z, t;

    if (!readRaw(&x, &y, &z, &t))
        return 0;

    /* Update the observed boundaries of the measurements */

    if (x < x_low)
        x_low = x;
    if (x > xhigh)
        xhigh = x;
    if (y < y_low)
        y_low = y;
    if (y > y_high)
        y_high = y;

    /* Bail out if not enough data is available. */

    if (x_low == xhigh || y_low == y_high)
        return 0;

    /* Recenter the measurement by subtracting the average */

    x -= (xhigh + x_low) / 2;
    y -= (y_high + y_low) / 2;

    /* Rescale the measurement to the range observed. */

    float fx = (float)x / (xhigh - x_low);
    float fy = (float)y / (y_high - y_low);

    int heading = 180.0 * atan2(fy, fx) / M_PI;
    if (heading <= 0)
        heading += 360;

    return heading;
}

void sigintHandler([[maybe_unused]] int sig)
{
    rclcpp::shutdown();
}

void mag_timer_callback()
{
    // ROS timer callback to publish mag data
    int16_t x, y, z, t;
    short status;
    sensor_msgs::msg::MagneticField mag_msg;

    status = i2c_read_byte_data(pi, i2ch, QMC5883L_STATUS);
    if (!status & QMC5883L_STATUS_DRDY)
    {
        return;
    }

    readRaw(&x, &y, &z, &t);
    // ROS_INFO("Raw data %i, %i,%i",x,y,z);
    mag_msg.header.stamp = nh->get_clock()->now();
    mag_msg.header.frame_id = "imu_link";
    mag_msg.magnetic_field.x = (int)(x / DigitPermg);
    mag_msg.magnetic_field.y = (int)(y / DigitPermg);
    mag_msg.magnetic_field.z = (int)(z / DigitPermg);
    mag_pub->publish(mag_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    nh = rclcpp::Node::make_shared("pi_car_mag");
    RCLCPP_INFO(nh->get_logger(), "Started Pi Car mag");

    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigintHandler);
    signal(SIGKILL, sigintHandler);
    if ((pi = pigpio_start(NULL, NULL)) < 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gpio init failed");
        return 1;
    }

    i2ch = i2c_open(pi, 1, QMC5883L_ADDR, 0);
    if (i2ch < 0) {
        RCLCPP_ERROR(nh->get_logger(), "Failed to open I2C device");
        exit(1);  // or handle accordingly
    }
    
    QMC5883L_init();

    // Set up ROS
    auto node = rclcpp::Node::make_shared("pi_car_mag");
    mag_timer = nh->create_wall_timer(
        std::chrono::duration<double>(MAG_TIMER),
        std::bind(&mag_timer_callback));
    mag_pub = nh->create_publisher<sensor_msgs::msg::MagneticField>(TOPIC_MAG, 2);
    rclcpp::spin(nh);
    rclcpp::shutdown();
}