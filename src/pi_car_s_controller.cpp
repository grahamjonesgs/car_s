/*
                TOP SSD Card

   Func     Colour  GPIO  pin	pin  GPIO	Colour	Func
   IMU Vcc  Brown   3V3   1	2     5v
   IMU SDA          2     3     4     5v   5v to board
   IMU SCL          3     5     6     GND  GND to board
                    4	  7	8     14   ENABLE
   IMU GND          GND   9	10    15   RF DIR (A) xxxxxx
                    17    11	12    18   RF STEP (A)
                    27    13    14    GND  LED GND
                    22	  15	16    23   LED Red
                    3V3	  17	18    24   LED Blue
   M0 SUBSTEP       10	  19	20    GND
   M1 SUBSTEP       9	  21	22    25
   M2 SUBSTEP       11	  23	24    8
                    GND   25	26    7
                    0	  27	28    1
   SW YELLOW        5	  29	30   GND
   LF DIR (Y)       6	  31    32   12   RB STEP (Z)
   LF STEP (Y)      13    33	34   GND
   LB STEP (X)      19    35	36   16   RB DIR (Z) xxxx
   LB DIR (X)	    26	  37	38   20
                    GND   39	40   21

                  Bottom Camera Port
 */

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <pigpiod_if2.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>

// Pin definition
#define LEFT_FRONT_STEP_PIN 13  // Y Controller
#define LEFT_BACK_STEP_PIN 19   // X Controller
#define RIGHT_FRONT_STEP_PIN 18 // A Controller
#define RIGHT_BACK_STEP_PIN 12  // Z Controller

#define LEFT_FRONT_DIR_PIN 6
#define LEFT_BACK_DIR_PIN 26
#define RIGHT_FRONT_DIR_PIN 15
#define RIGHT_BACK_DIR_PIN 16
#define ENABLE_PIN 14

#define M0_SUBSTEP 10
#define M1_SUBSTEP 9
#define M2_SUBSTEP 11

#define SWITCH_RED 5
#define SWITCH_YELLOW 11
#define RED_LED_PIN 23
#define BLUE_LED_PIN 24

// Hardware PWM frequency and constants
#define MAX_SPEED_FREQ 24000
#define MESSAGE_TIMEOUT 1 // Time to stop if no message received
#define ODOM_TIMEOUT 1    // Sets frequency of speed check and report
#define LED_FLASH_TIMER 1
#define SUB_STEPS 32
#define STEPS_PER_REVOLUTION 200
#define M_PER_REVOLUTION 0.21
#define WHEEL_SEPARATION 0.2337 // 0.25 * 360 / 385
#define ANGLE_DELTA 3

// Max speed at 32 sub steps is 24000 / (32*200) = 7.5 rev/s which is 1.57 m/s
// Total formula is (speed / M_PER_REVOLUTION) * STEPS_PER_REVOLUTION * sub_steps
// Formula for 1 m/s is (1 / M_PER_REVOLUTION) * STEPS_PER_REVOLUTION * sub_steps

// Motor Modes
#define MODE_CONTINUOUS 1
#define MODE_AUTO 2
#define MODE_360 3

// Topics
#define TOPIC_CMD_VEL "cmd_vel"
#define TOPIC_LEFT_SPEED "left_speed"
#define TOPIC_RIGHT_SPEED "right_speed"
#define TOPIC_MOTOR "motor_on"
#define TOPIC_ANGLE "angle"
#define TOPIC_STEPS "motor_steps"
#define TOPIC_FREQ "motor_max_freq"
#define TOPIC_MODE "motor_mode"

// Global variables
rclcpp::TimerBase::SharedPtr message_timeout; // Timer for stop motors if no message
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_speed_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_speed_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub;
std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
rclcpp::Node::SharedPtr nh;
int pi;                   // Pi ID from Pigpio
int left_step_count = 0;  // Count of edges on PWM
int right_step_count = 0; // Count of edges on PWM
bool motor_on = false;    // Start in safe mode
int motor_mode = MODE_AUTO;

double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;

bool left_forwards = true;
bool right_forwards = true;

float last_left_velocity = 0;
float last_right_velocity = 0;

int sub_steps = SUB_STEPS;
int max_speed_freq = MAX_SPEED_FREQ;
float target_angle;
float current_angle;
float angle_delta = ANGLE_DELTA; // Allowable angle deviance for defined turn
bool turn_started = false;       // Check if the turn has started to avoid immediate stop

void sigintHandler([[maybe_unused]] int sig)
{
        // Shuting down, turn off blue LED and all motors and PWM
        gpio_write(pi, ENABLE_PIN, PI_HIGH);
        rclcpp::shutdown();
}

void set_substep(int steps)
{
        int bin_steps;
        bool M0;
        bool M1;
        bool M2;

        bin_steps = (steps == 0) ? 0 : (log((double)steps) / log(2.0));
        RCLCPP_INFO(nh->get_logger(), "Substeps set to %i", steps);

        M0 = bin_steps & 1;
        M1 = bin_steps & 2;
        M2 = bin_steps & 4;

        gpio_write(pi, M0_SUBSTEP, (M0 == 1) ? PI_HIGH : PI_LOW);
        gpio_write(pi, M1_SUBSTEP, (M1 == 1) ? PI_HIGH : PI_LOW);
        gpio_write(pi, M2_SUBSTEP, (M2 == 1) ? PI_HIGH : PI_LOW);

        sub_steps = steps;
}

void motor_set_target(float speed, float turn)
{
        // Send output to motors based on speed and turn
        // Positive speed forwards
        // Positive turn clockwise - slower right
        float left_velocity;
        float right_velocity;
        RCLCPP_INFO(nh->get_logger(), "In motor control, speed %.2f, turn %.2f", speed, turn);

        /*
        
        NOTE Removed code to check for invalid values, as trying abs speed values for ROS2 in m/s

        Need to add code to check from max velocity and set max speed based on measurements
        
        if (speed < -1 || speed > 1)
                return; // Ignore invalid values */
        if (turn < -1 || turn > 1)
                return; // Ignore invalid values
        if (speed == 0)
        { // For turn on spot
                left_velocity = turn;
                right_velocity = -turn;
        }
        else
        {
                if (motor_mode == MODE_360)
                {
                        target_angle = current_angle;
                        turn_started = false;
                }
                if (turn < 0)
                {
                        left_velocity = speed * (1 - turn);
                        right_velocity = speed * (1 + turn);
                }
                else
                {
                        left_velocity = speed * (1 - turn);
                        right_velocity = speed * (1 + turn);
                }
        }
        RCLCPP_INFO(nh->get_logger(), "Left v %.2f, right speed %.2f", left_velocity, right_velocity);
        if ((speed == 0 && turn == 0))
        {
                hardware_PWM(pi, LEFT_FRONT_STEP_PIN, 0, 0);
                hardware_PWM(pi, LEFT_BACK_STEP_PIN, 0, 0);
                hardware_PWM(pi, RIGHT_FRONT_STEP_PIN, 0, 0);
                hardware_PWM(pi, RIGHT_BACK_STEP_PIN, 0, 0);
        }
        else
        {

                if (last_left_velocity != left_velocity)
                {
                        gpio_write(pi, LEFT_FRONT_DIR_PIN, (left_velocity < 0) ? PI_LOW : PI_HIGH);
                        gpio_write(pi, LEFT_BACK_DIR_PIN, (left_velocity < 0) ? PI_LOW : PI_HIGH);
                        hardware_PWM(pi, LEFT_FRONT_STEP_PIN, STEPS_PER_REVOLUTION * sub_steps * abs(left_velocity) / M_PER_REVOLUTION, 1e6 * 0.5);
                        hardware_PWM(pi, LEFT_BACK_STEP_PIN, STEPS_PER_REVOLUTION * sub_steps * abs(left_velocity) / M_PER_REVOLUTION, 1e6 * 0.5);
                }
                if (last_right_velocity != right_velocity)
                {
                        gpio_write(pi, RIGHT_FRONT_DIR_PIN, (right_velocity > 0) ? PI_LOW : PI_HIGH);
                        gpio_write(pi, RIGHT_BACK_DIR_PIN, (right_velocity > 0) ? PI_LOW : PI_HIGH);
                        hardware_PWM(pi, RIGHT_FRONT_STEP_PIN, STEPS_PER_REVOLUTION * sub_steps * abs(right_velocity) / M_PER_REVOLUTION, 1e6 * 0.5);
                        hardware_PWM(pi, RIGHT_BACK_STEP_PIN, STEPS_PER_REVOLUTION * sub_steps * abs(right_velocity) / M_PER_REVOLUTION, 1e6 * 0.5);
                }
        }
        left_forwards = (left_velocity >= 0) ? 1 : 0;
        right_forwards = (right_velocity >= 0) ? 1 : 0;
        last_left_velocity = left_velocity;
        last_right_velocity = right_velocity;
}

void message_timeout_callback()
{
        // ROS callback if not message received to stop motors
        if ((last_left_velocity != 0 || last_right_velocity != 0) && motor_mode == MODE_AUTO)
        {
                motor_set_target(0.0, 0.0);
                message_timeout->reset(); // reset no message timeout
                RCLCPP_INFO(nh->get_logger(), "Timeout");
        }
}

void LED_timer_callback()
{
        // ROS callback for LED messages
        if (!motor_on)
        {
                // Switch to disable
                gpio_write(pi, BLUE_LED_PIN, PI_HIGH);
        }
        else
        {
                // Switch to enable
                gpio_write(pi, BLUE_LED_PIN, !gpio_read(pi, BLUE_LED_PIN));
        }
}

static void left_speed_callback([[maybe_unused]] int pi, [[maybe_unused]] uint32_t gpio, [[maybe_unused]] uint32_t level, [[maybe_unused]] uint32_t tick)
{
        // Pigpio callback on speed encoded pin rising edge
        (left_forwards) ? left_step_count++ : left_step_count--;
}

static void right_speed_callback([[maybe_unused]] int pi, [[maybe_unused]] uint32_t gpio, [[maybe_unused]] uint32_t level, [[maybe_unused]] uint32_t tick)
{
        // Pigpio callback on speed encoded pin rising edge
        (right_forwards) ? right_step_count++ : right_step_count--;
}

auto createQuaternionMsgFromYaw(double yaw)
{
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
}

void odom_timer_callback()
{
        // ROS callback on time to send wheel speeds and odom
        std_msgs::msg::Float32 left_speed;
        std_msgs::msg::Float32 right_speed;
        std_msgs::msg::Float32 angle;
        float left_delta;
        float right_delta;
        float x_delta;
        float y_delta;
        float theta_delta;

        left_delta = left_step_count * M_PER_REVOLUTION / (sub_steps * STEPS_PER_REVOLUTION);
        right_delta = right_step_count * M_PER_REVOLUTION / (sub_steps * STEPS_PER_REVOLUTION);

        left_step_count = 0;
        right_step_count = 0;

        left_speed.data = left_delta / ODOM_TIMEOUT;
        right_speed.data = right_delta / ODOM_TIMEOUT;
        theta_delta = (right_delta - left_delta) / WHEEL_SEPARATION;

        theta += theta_delta;
        if (theta > M_PI)
        {
                theta -= 2 * M_PI;
        }
        if (theta < -M_PI)
        {
                theta += 2 * M_PI;
        }
        angle.data = 180 * theta / M_PI; // Send angle in degrees
        current_angle = angle.data;
        if ((abs(current_angle - target_angle) > angle_delta) && (turn_started == false))
        {
                turn_started = true;
                RCLCPP_INFO(nh->get_logger(), "Turn started set to true");
        }

        if (turn_started && abs(current_angle - target_angle) < angle_delta && motor_mode == MODE_360 && (last_left_velocity != 0 || last_right_velocity != 0))
        {
                RCLCPP_INFO(nh->get_logger(), "Target angle reached");
                motor_set_target(0.0, 0.0);
        }

        x_delta = cos(theta) * (left_speed.data + right_speed.data) / 2; // approx for small angle delta
        y_delta = sin(theta) * (left_speed.data + right_speed.data) / 2; // approx for small angle delta
        x_pos += x_delta;
        y_pos += y_delta;

        geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(theta);

        // first, we'll publish the transform over tf
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = nh->now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x_pos;
        odom_trans.transform.translation.y = y_pos;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster->sendTransform(odom_trans);

        // next, we'll publish the odometry message over ROS
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = nh->now();
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x_pos;
        odom.pose.pose.position.y = y_pos;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = x_delta;
        odom.twist.twist.linear.y = y_delta;
        odom.twist.twist.angular.z = theta_delta;

        left_speed_pub->publish(left_speed);
        right_speed_pub->publish(right_speed);
        angle_pub->publish(angle);
        odom_pub->publish(odom);
}

void velocity_callback(const geometry_msgs::msg::Twist &msg)
{
        // ROS callback on message received
        message_timeout->reset(); // reset no message timeout
        motor_set_target(msg.linear.x, msg.angular.z);
}

void steps_callback(const std_msgs::msg::Int32 &msg)
{
        // ROS callback on message received
        RCLCPP_INFO(nh->get_logger(), "Setting to %i steps", msg.data);
        set_substep(msg.data);
}

void freq_callback(const std_msgs::msg::Int32 &msg)
{
        // ROS callback on message received
        RCLCPP_INFO(nh->get_logger(), "Setting freq to %iHz", msg.data);
        max_speed_freq = msg.data;
}

void motor_mode_callback(const std_msgs::msg::Int32 &msg)
{
        // ROS callback on message received
        switch (msg.data)
        {
        case MODE_AUTO:
                motor_mode = msg.data;
                RCLCPP_INFO(nh->get_logger(), "Set to auto mode");
                break;
        case MODE_CONTINUOUS:
                motor_mode = msg.data;
                RCLCPP_INFO(nh->get_logger(), "Set to continuous mode");
                break;
        case MODE_360:
                motor_mode = msg.data;
                RCLCPP_INFO(nh->get_logger(), "Set to 360 mode");
                break;
        default:
                RCLCPP_INFO(nh->get_logger(), "Invalid mode received %i", msg.data);
                break;
        }
}

void motor_callback(const std_msgs::msg::Bool &msg)
{
        // ROS callback on message received
        motor_on = msg.data;
        if (msg.data)
        {
                RCLCPP_INFO(nh->get_logger(), "Set motor on");
                gpio_write(pi, ENABLE_PIN, PI_LOW); // Enable drive
        }
        else
        {
                RCLCPP_INFO(nh->get_logger(), "Set motor off");
                gpio_write(pi, ENABLE_PIN, PI_HIGH); // Disable drive
        }
}

int main(int argc, char **argv)
{
        rclcpp::InitOptions init_options;
        rclcpp::init(argc, argv, init_options);
        nh = rclcpp::Node::make_shared("pi_car_s_controller");
        RCLCPP_INFO(nh->get_logger(), "Started Pi Car Stepper Controller");

        signal(SIGINT, sigintHandler);
        signal(SIGTERM, sigintHandler);
        signal(SIGKILL, sigintHandler);
        if ((pi = pigpio_start(NULL, NULL)) < 0)
        {
                RCLCPP_ERROR(nh->get_logger(), "gpio init failed");
                return 1;
        }

        set_mode(pi, LEFT_FRONT_STEP_PIN, PI_OUTPUT);
        set_mode(pi, LEFT_BACK_STEP_PIN, PI_OUTPUT);
        set_mode(pi, RIGHT_FRONT_STEP_PIN, PI_OUTPUT);
        set_mode(pi, RIGHT_BACK_STEP_PIN, PI_OUTPUT);
        set_mode(pi, LEFT_FRONT_DIR_PIN, PI_OUTPUT);
        set_mode(pi, LEFT_BACK_DIR_PIN, PI_OUTPUT);
        set_mode(pi, RIGHT_FRONT_DIR_PIN, PI_OUTPUT);
        set_mode(pi, RIGHT_BACK_DIR_PIN, PI_OUTPUT);
        set_mode(pi, ENABLE_PIN, PI_OUTPUT);

        set_mode(pi, RED_LED_PIN, PI_OUTPUT);
        set_mode(pi, BLUE_LED_PIN, PI_OUTPUT);
        set_mode(pi, M0_SUBSTEP, PI_OUTPUT);
        set_mode(pi, M1_SUBSTEP, PI_OUTPUT);
        set_mode(pi, M2_SUBSTEP, PI_OUTPUT);

        gpio_write(pi, BLUE_LED_PIN, PI_HIGH);

        // Set up ROS
        auto sub_velocity = nh->create_subscription<geometry_msgs::msg::Twist>(
            TOPIC_CMD_VEL, 2, velocity_callback);
        auto sub_motor = nh->create_subscription<std_msgs::msg::Bool>(
            TOPIC_MOTOR, 1, motor_callback);
        auto sub_motor_mode = nh->create_subscription<std_msgs::msg::Int32>(
            TOPIC_MODE, 1, motor_mode_callback);
        auto sub_set_steps = nh->create_subscription<std_msgs::msg::Int32>(
            TOPIC_STEPS, 1, steps_callback);
        auto sub_set_freq = nh->create_subscription<std_msgs::msg::Int32>(
            TOPIC_FREQ, 1, freq_callback);
        message_timeout = nh->create_wall_timer(std::chrono::duration<double>(MESSAGE_TIMEOUT), std::bind(&message_timeout_callback));
        auto odom_timer = nh->create_wall_timer(std::chrono::duration<double>(ODOM_TIMEOUT), [=]()
                                                { odom_timer_callback(); });
        auto LED_timer = nh->create_wall_timer(std::chrono::duration<double>(LED_FLASH_TIMER), std::bind(&LED_timer_callback));
        odom_pub = nh->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
        left_speed_pub = nh->create_publisher<std_msgs::msg::Float32>(TOPIC_LEFT_SPEED, 2);
        right_speed_pub = nh->create_publisher<std_msgs::msg::Float32>(TOPIC_RIGHT_SPEED, 2);
        angle_pub = nh->create_publisher<std_msgs::msg::Float32>(TOPIC_ANGLE, 2);
        odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);

        callback(pi, LEFT_FRONT_STEP_PIN, RISING_EDGE, left_speed_callback);
        callback(pi, RIGHT_FRONT_STEP_PIN, RISING_EDGE, right_speed_callback);
        gpio_write(pi, ENABLE_PIN, PI_HIGH); // Disable drive
        set_substep(sub_steps);

        rclcpp::spin(nh);

        // Shutdown all motor functions
        gpio_write(pi, ENABLE_PIN, PI_HIGH); // Disable drive
        hardware_PWM(pi, LEFT_FRONT_STEP_PIN, 0, 0);
        hardware_PWM(pi, LEFT_BACK_STEP_PIN, 0, 0);
        hardware_PWM(pi, RIGHT_FRONT_STEP_PIN, 0, 0);
        hardware_PWM(pi, RIGHT_BACK_STEP_PIN, 0, 0);
        gpio_write(pi, BLUE_LED_PIN, PI_HIGH); // Disable blue LED
        gpio_write(pi, RED_LED_PIN, PI_LOW);   // Disable red LED
        pigpio_stop(pi);                       // Stop pigpio
}
