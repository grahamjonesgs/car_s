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

#include "sensor_msgs/msg/magnetic_field.hpp"
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include "i2c_device.hpp"
#include <stdlib.h>

// Topics
#define TOPIC_MAG "imu/mag"

// Timers
#define MAG_TIMER 0.1 // Interval between MAG readings
// Readings in a row with no new data (about 1 s) before the chip is
// configured again, e.g. after it reset into standby. As many again after
// that and the node exits, and the launch respawns it
#define NO_DATA_READINGS 10

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


// Global variables
rclcpp::TimerBase::SharedPtr mag_timer;
rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
// The QMC5883L on I2C bus 1, through the kernel's i2c-dev driver
I2cDevice mag;
#define I2C_BUS 1
rclcpp::Node::SharedPtr nh;
int exit_code = 0;
int no_data_count = 0; // Readings in a row with no new data

uint8_t mode;
uint8_t rate;
uint8_t range;
uint8_t oversampling;

// Normalisation values
float DigitPermg;

// The set-up functions return false if a write fails
bool reconfigure() {
  return mag.write_byte(QMC5883L_CONFIG,
                        oversampling | range | rate | mode);
}

bool reset() {
  return mag.write_byte(QMC5883L_RESET, 0x01) && reconfigure();
}

bool setOversampling(int x) {
  switch (x) {
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
  return reconfigure();
}

bool setRange(int x) {
  switch (x) {
  case 2:
    range = QMC5883L_CONFIG_2GAUSS;
    DigitPermg = 12.0;
    break;
  case 8:
    range = QMC5883L_CONFIG_8GAUSS;
    DigitPermg = 3.0;
    break;
  }
  return reconfigure();
}

bool setSamplingRate(int x) {
  switch (x) {
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
  return reconfigure();
}

bool QMC5883L_init() {
  if (!setOversampling(512) || !setRange(2) || !setSamplingRate(100)) {
    return false;
  }
  mode = QMC5883L_CONFIG_CONT;
  return reset();
}

// Log an error and stop spinning, so main exits with an error and the
// launch respawns the node
void fail(const char *msg) {
  RCLCPP_ERROR(nh->get_logger(), "%s, exiting", msg);
  exit_code = 1;
  rclcpp::shutdown();
}

// Count a reading with no new data. After about 1 s configure the chip
// again; after as long again, exit
void no_data() {
  ++no_data_count;
  if (no_data_count == NO_DATA_READINGS) {
    RCLCPP_ERROR(nh->get_logger(),
                 "No magnetometer data for %.1f s, configuring the QMC5883L "
                 "again", NO_DATA_READINGS * MAG_TIMER);
    if (!reset()) {
      fail("Failed to configure the QMC5883L");
    }
  } else if (no_data_count >= 2 * NO_DATA_READINGS) {
    fail("Still no magnetometer data after configuring the QMC5883L again");
  }
}

// Read X, Y and Z in one transfer, so they all come from the same sample.
// Returns false if the read fails
bool readRaw(int16_t *x, int16_t *y, int16_t *z) {
  uint8_t buf[6];
  if (!mag.read_block(QMC5883L_X_LSB, buf, 6)) {
    return false;
  }
  auto word = [&](int i) {
    return static_cast<int16_t>((buf[i + 1] << 8) | buf[i]);
  };
  *x = word(0);
  *y = word(2);
  *z = word(4);
  return true;
}

void mag_timer_callback() {
  // ROS timer callback to publish mag data
  int16_t x, y, z;
  short status;
  sensor_msgs::msg::MagneticField mag_msg;

  status = mag.read_byte(QMC5883L_STATUS);
  // This chip reports new data with DOR (overrun) rather than DRDY when the
  // previous reading was missed, so accept either
  if (status < 0 ||
      !(status & (QMC5883L_STATUS_DRDY | QMC5883L_STATUS_DOR))) {
    no_data();
    return;
  }

  if (!readRaw(&x, &y, &z)) {
    RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 5000,
                         "Failed to read the magnetometer");
    no_data();
    return;
  }
  // The chip is converting, even if this sample is out of range
  no_data_count = 0;
  // OVL: a field beyond the range (e.g. a magnet or the motors nearby), so
  // the values are clipped. Read anyway to clear the status, but drop it
  if (status & QMC5883L_STATUS_OVL) {
    RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 5000,
                         "Magnetometer overflow, dropping the reading");
    return;
  }
  mag_msg.header.stamp = nh->get_clock()->now();
  mag_msg.header.frame_id = "mag_link";
  // Convert from milligauss to Tesla (1 mG = 1e-7 T)
  mag_msg.magnetic_field.x = x / DigitPermg * 1e-7;
  mag_msg.magnetic_field.y = y / DigitPermg * 1e-7;
  mag_msg.magnetic_field.z = z / DigitPermg * 1e-7;
  mag_pub->publish(mag_msg);
}

// Release the global ROS objects, shut down and return code. Every exit
// goes through here: destroyed later, as globals at exit, they threw or
// crashed (exit codes -6 and -11)
int finish(int code) {
  mag.close();
  mag_timer.reset();
  mag_pub.reset();
  nh.reset();
  rclcpp::shutdown();
  return code;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("pi_car_mag");
  RCLCPP_INFO(nh->get_logger(), "Started Pi Car mag");
  if (!mag.open(I2C_BUS, QMC5883L_ADDR)) {
    RCLCPP_ERROR(nh->get_logger(),
                 "Can't open /dev/i2c-%d: is I2C enabled (dtparam=i2c_arm=on), "
                 "and the user in the i2c group?", I2C_BUS);
    return finish(1);
  }
  // The chip ID register reads 0xFF on a QMC5883L; a failed read means the
  // chip isn't answering
  if (mag.read_byte(QMC5883L_CHIP_ID) < 0) {
    RCLCPP_ERROR(nh->get_logger(), "No QMC5883L answering on I2C bus 1 at 0x%02X",
                 QMC5883L_ADDR);
    return finish(1);
  }

  if (!QMC5883L_init()) {
    RCLCPP_ERROR(nh->get_logger(), "Failed to set up the QMC5883L");
    return finish(1);
  }

  // Set up ROS
  mag_pub = nh->create_publisher<sensor_msgs::msg::MagneticField>(
      TOPIC_MAG, rclcpp::SensorDataQoS());
  mag_timer = nh->create_wall_timer(std::chrono::duration<double>(MAG_TIMER),
                                    std::bind(&mag_timer_callback));
  rclcpp::spin(nh);
  return finish(exit_code);
}