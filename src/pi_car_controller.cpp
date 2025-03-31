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
 */

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <pigpiod_if2.h>
#include <stdlib.h>
#include <signal.h>

// Pin definition
#define LEFT_FRONT_EN_PIN 13
#define LEFT_BACK_EN_PIN 19
#define RIGHT_FRONT_EN_PIN 12
#define RIGHT_BACK_EN_PIN 18
#define LEFT_CONTROL_1_PIN 6
#define LEFT_CONTROL_2_PIN 26
#define RIGHT_CONTROL_1_PIN 16
#define RIGHT_CONTROL_2_PIN 20
#define LEFT_SPEED_PIN 10
#define RIGHT_SPEED_PIN 9
#define SWITCH_RED 5
#define SWITCH_YELLOW 11
#define RED_LED_PIN 23
#define BLUE_LED_PIN 24

// Hardware PWM frequency
#define PWM_FREQ 1600
#define MESSAGE_TIMEOUT 0.5 // Time to stop if no message received
#define SPEED_TIMEOUT 0.5   // Sets frequency of speed check and report
#define LED_FLASH_TIMER 1
#define SPEED_MULTIPLIER 0.0105 // (1/20)*0.21 - divide by number of slots, multiply by circumference
#define MIN_DISTANCE 0.3
#define BACK_EXTRA_DISTANCE 0.13

// Topics
#define TOPIC_CMD_VEL "cmd_vel"
#define TOPIC_LEFT_SPEED "left_speed"
#define TOPIC_RIGHT_SPEED "right_speed"
#define TOPIC_MOTOR "motor_on"
#define TOPIC_SCAN "scan"
#define TOPIC_MSG "msg"

// Global variables
rclcpp::TimerBase::SharedPtr message_timeout; // Timer for stop motors if no message
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_speed_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_speed_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub;
int pi;                    // Pi ID from Pigpio
int left_speed_count = 0;  // Count of edges on encoder
int right_speed_count = 0; // Count of edges on encoder
bool motor_on = false;     // start in safe mode
bool front_block = false;
bool back_block = false;
int current_speed = 0;
std_msgs::msg::String status_msg;
rclcpp::Node::SharedPtr nh;

void sigintHandler([[maybe_unused]] int sig)
{
    // Shuting down, turn off blue LED and all motors and PWM
    gpio_write(pi, BLUE_LED_PIN, PI_LOW);
    gpio_write(pi, LEFT_CONTROL_1_PIN, PI_LOW);
    gpio_write(pi, LEFT_CONTROL_2_PIN, PI_LOW);
    gpio_write(pi, RIGHT_CONTROL_1_PIN, PI_LOW);
    gpio_write(pi, RIGHT_CONTROL_2_PIN, PI_LOW);
    hardware_PWM(pi, LEFT_FRONT_EN_PIN, PWM_FREQ, 0);
    hardware_PWM(pi, LEFT_BACK_EN_PIN, PWM_FREQ, 0);
    hardware_PWM(pi, RIGHT_FRONT_EN_PIN, PWM_FREQ, 0);
    hardware_PWM(pi, RIGHT_BACK_EN_PIN, PWM_FREQ, 0);

    rclcpp::shutdown();
}

float linear_speed(float input_speed)
{
    // Handle car physics to map required speed to PWM output
    float min_speed = 0.3;
    float return_speed;
    if (input_speed == 0)
    {
        return (input_speed);
    }

    return_speed = (input_speed > 0) ? (input_speed + min_speed) : (input_speed - min_speed);
    return_speed = return_speed / (1 + min_speed); // Simple scaling

    if (return_speed > 1)
        return_speed = 1; // Just in case
    if (return_speed < -1)
        return_speed = -1; // Just in case
    return abs(return_speed);
}

void motor_control(float speed, float turn)
{
    // Send output to motors based on speed and turn
    // Positive speed forwards
    // Positive turn clockwise - slower right
    float left_velocity;
    float right_velocity;

    if (speed < -1 || speed > 1)
        return; // Ignore invalid values
    if (turn < -1 || turn > 1)
        return; // Ignore invalid values
    if (speed == 0)
    { // For turn on spot
        left_velocity = turn;
        right_velocity = -turn;
    }
    else
    {
        if (turn < 0)
        {
            left_velocity = speed * (1 - turn);
            right_velocity = speed;
        }
        else
        {
            left_velocity = speed;
            right_velocity = speed * (1 - turn);
        }
    }
    if (left_velocity != 0 || right_velocity != 0)
    {
        RCLCPP_INFO(nh->get_logger(), "left_in=%f, left_map=%f, right_in=%f, right_map=%f", left_velocity, linear_speed(left_velocity), right_velocity, linear_speed(right_velocity));
    }

    hardware_PWM(pi, LEFT_FRONT_EN_PIN, PWM_FREQ, 1e6 * linear_speed(left_velocity));
    hardware_PWM(pi, LEFT_BACK_EN_PIN, PWM_FREQ, 1e6 * linear_speed(left_velocity));
    hardware_PWM(pi, RIGHT_FRONT_EN_PIN, PWM_FREQ, 1e6 * linear_speed(right_velocity));
    hardware_PWM(pi, RIGHT_BACK_EN_PIN, PWM_FREQ, 1e6 * linear_speed(right_velocity));

    if ((speed == 0 && turn == 0) || motor_on == false)
    {
        gpio_write(pi, LEFT_CONTROL_1_PIN, PI_LOW);
        gpio_write(pi, LEFT_CONTROL_2_PIN, PI_LOW);
        gpio_write(pi, RIGHT_CONTROL_1_PIN, PI_LOW);
        gpio_write(pi, RIGHT_CONTROL_2_PIN, PI_LOW);
    }
    else
    {
        (left_velocity < 0) ? gpio_write(pi, LEFT_CONTROL_1_PIN, PI_HIGH) : gpio_write(pi, LEFT_CONTROL_1_PIN, PI_LOW);
        (right_velocity < 0) ? gpio_write(pi, RIGHT_CONTROL_1_PIN, PI_HIGH) : gpio_write(pi, RIGHT_CONTROL_1_PIN, PI_LOW);
        (left_velocity < 0) ? gpio_write(pi, LEFT_CONTROL_2_PIN, PI_LOW) : gpio_write(pi, LEFT_CONTROL_2_PIN, PI_HIGH);
        (right_velocity < 0) ? gpio_write(pi, RIGHT_CONTROL_2_PIN, PI_LOW) : gpio_write(pi, RIGHT_CONTROL_2_PIN, PI_HIGH);
    }
}

void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // ROS callback on message received
    char message[256];
    char segment_status[] = "00000000";

    // Checking for obstacle. Octagon turned 22.5 degrees 0 is back, 4 is front
    // The sides of the front are 3 and 5.
    // The sides of the back are 1 and 7.
    // LIDAR is towards the front by 10cm. So checking is extra for 0<i<90 and 270<i<359
    for (int i = 0; i < 360; i++)
    {
        if (msg->ranges[i] < ((i > 90 && i < 270) ? MIN_DISTANCE : (MIN_DISTANCE + BACK_EXTRA_DISTANCE)))
        {
            // if (msg->ranges[i]<0.3) {
            segment_status[((int)((i + 22) / 45) % 8)] = '1';
        }
    }

    if (segment_status[4] == '1' || (segment_status[3] == '1' && segment_status[5] == '1'))
    {
        front_block = 1;
    }
    else
    {
        front_block = 0;
    }
    if (segment_status[0] == '1' || (segment_status[1] == '1' && segment_status[7] == '1'))
    {
        back_block = 1;
    }
    else
    {
        back_block = 0;
    }

    snprintf(message, sizeof(message), "%s F%i B%i", segment_status, front_block, back_block);
    status_msg.data = message;
    msg_pub->publish(status_msg);

    if (front_block && (current_speed > 0))
        motor_control(0.0, 0.0);
    if (back_block && (current_speed < 0))
        motor_control(0.0, 0.0);
}

void message_timeout_callback()
{
    // ROS callback if not message received to stop motors
    current_speed = 0.0;
    motor_control(0.0, 0.0);
}

void LED_timer_callback()
{
    // ROS callback for LED messages
    if (!motor_on)
    {
        gpio_write(pi, BLUE_LED_PIN, PI_HIGH);
    }
    else
    {
        gpio_write(pi, BLUE_LED_PIN, !gpio_read(pi, BLUE_LED_PIN));
    }
}

static void speed_callback([[maybe_unused]] int pi, [[maybe_unused]] uint32_t gpio, [[maybe_unused]] uint32_t level, [[maybe_unused]] uint32_t tick)
{
    // Pigpio callback on speed encoded pin rising edge
    gpio == LEFT_SPEED_PIN ? left_speed_count++ : right_speed_count++;
}

void speed_timer_callback()
{
    // ROS callback on time to send wheel speeds
    std_msgs::msg::Float32 left_speed;
    std_msgs::msg::Float32 right_speed;
    left_speed.data = left_speed_count * SPEED_MULTIPLIER / SPEED_TIMEOUT;
    right_speed.data = right_speed_count * SPEED_MULTIPLIER / SPEED_TIMEOUT;

    left_speed_pub->publish(left_speed);
    right_speed_pub->publish(right_speed);
    left_speed_count = 0;
    right_speed_count = 0;
}

void velocity_callback(const geometry_msgs::msg::Twist msg)
{
    // ROS callback on message received
    message_timeout->reset(); // Reset the timer to restart the timeout period
    if ((msg.linear.x > 0) && front_block)
        return;
    if ((msg.linear.x < 0) && back_block)
        return;
    current_speed = msg.linear.x;
    motor_control(msg.linear.x, msg.angular.z);
}

void motor_callback(const std_msgs::msg::Bool msg)
{
    // ROS callback on message received
    motor_on = msg.data;
}

int main(int argc, char **argv)
{
    // Set up ROS
    rclcpp::InitOptions init_options;
    rclcpp::init(argc, argv, init_options);

    // Set up pigpio
    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigintHandler);
    signal(SIGKILL, sigintHandler);

    // Start pigpio

    nh = std::make_shared<rclcpp::Node>("pi_car_controller");
    RCLCPP_INFO(nh->get_logger(), "Started Pi Car Controller");

    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigintHandler);
    signal(SIGKILL, sigintHandler);
    if ((pi = pigpio_start(NULL, NULL)) < 0)
    {
        RCLCPP_INFO(nh->get_logger(), "gpio init failed");
        return 1;
    }

    set_mode(pi, LEFT_FRONT_EN_PIN, PI_OUTPUT);
    set_mode(pi, LEFT_BACK_EN_PIN, PI_OUTPUT);
    set_mode(pi, LEFT_CONTROL_1_PIN, PI_OUTPUT);
    set_mode(pi, LEFT_CONTROL_2_PIN, PI_OUTPUT);
    set_mode(pi, RIGHT_FRONT_EN_PIN, PI_OUTPUT);
    set_mode(pi, RIGHT_BACK_EN_PIN, PI_OUTPUT);
    set_mode(pi, RIGHT_CONTROL_1_PIN, PI_OUTPUT);
    set_mode(pi, RIGHT_CONTROL_2_PIN, PI_OUTPUT);
    set_mode(pi, RED_LED_PIN, PI_OUTPUT);
    set_mode(pi, BLUE_LED_PIN, PI_OUTPUT);
    set_mode(pi, LEFT_SPEED_PIN, PI_INPUT);
    set_mode(pi, RIGHT_SPEED_PIN, PI_INPUT);

    gpio_write(pi, BLUE_LED_PIN, PI_HIGH);

    // Set up ROS
    auto sub_velocity = nh->create_subscription<geometry_msgs::msg::Twist>(
        TOPIC_CMD_VEL, 10, velocity_callback);
    auto sub_motor = nh->create_subscription<std_msgs::msg::Bool>(
        TOPIC_MOTOR, 10, motor_callback);
    auto sub_scan = nh->create_subscription<sensor_msgs::msg::LaserScan>(
        TOPIC_SCAN, 1, scan_callback);
    message_timeout = nh->create_wall_timer(std::chrono::duration<double>(MESSAGE_TIMEOUT), message_timeout_callback);
    auto speed_timer = nh->create_wall_timer(std::chrono::duration<double>(SPEED_TIMEOUT), std::bind(&speed_timer_callback));
    auto LED_timer = nh->create_wall_timer(std::chrono::duration<double>(LED_FLASH_TIMER), std::bind(&LED_timer_callback));
    left_speed_pub = nh->create_publisher<std_msgs::msg::Float32>(TOPIC_LEFT_SPEED, 2);
    right_speed_pub = nh->create_publisher<std_msgs::msg::Float32>(TOPIC_RIGHT_SPEED, 2);
    msg_pub = nh->create_publisher<std_msgs::msg::String>(TOPIC_MSG, 2);

    status_msg.data = "Hello, online";
    msg_pub->publish(status_msg);

    callback(pi, LEFT_SPEED_PIN, RISING_EDGE, speed_callback);
    callback(pi, RIGHT_SPEED_PIN, RISING_EDGE, speed_callback);
    rclcpp::spin(nh);
}