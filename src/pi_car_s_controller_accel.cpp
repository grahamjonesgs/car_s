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
 */

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <pigpiod_if2.h>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

// Pin definitions
const int LEFT_FRONT_STEP_PIN = 13;  // Y Controller
const int LEFT_BACK_STEP_PIN = 19;   // X Controller
const int RIGHT_FRONT_STEP_PIN = 18; // A Controller
const int RIGHT_BACK_STEP_PIN = 12;  // Z Controller

const int LEFT_FRONT_DIR_PIN = 6;
const int LEFT_BACK_DIR_PIN = 26;
const int RIGHT_FRONT_DIR_PIN = 15;
const int RIGHT_BACK_DIR_PIN = 16;
const int ENABLE_PIN = 14;

const int M0_SUBSTEP = 10;
const int M1_SUBSTEP = 9;
const int M2_SUBSTEP = 11;

const int RED_LED_PIN = 23;
const int BLUE_LED_PIN = 24;

// Hardware PWM frequency and constants
const double ODOM_TIMEOUT = 0.05;   // Sets frequency of speed check and report
const double LED_FLASH_TIMER = 1.0;
const int SUB_STEPS = 32;
const int STEPS_PER_REVOLUTION = 200;
const double ANGLE_DELTA = 3.0;

// Defaults for the ROS parameters
const double DEFAULT_CMD_VEL_TIMEOUT = 0.3; // Time to stop if no message received
const double DEFAULT_M_PER_REVOLUTION = 0.21;
const double DEFAULT_WHEEL_SEPARATION = 0.2337;

// Odometry covariances (diagonals). Wheel yaw is unreliable on a skid steer,
// so its rate variance is large and the EKF takes yaw rate from the gyro
const double ODOM_POSE_XY_VAR = 0.01;
const double ODOM_POSE_YAW_VAR = 0.1;
const double ODOM_TWIST_VX_VAR = 0.01;
const double ODOM_TWIST_VY_VAR = 0.01;
const double ODOM_TWIST_VYAW_VAR = 0.5;
const double ODOM_UNUSED_VAR = 1e6; // z, roll and pitch, which are not measured

// Motor Modes
enum class MotorMode { CONTINUOUS = 1, AUTO = 2, TURN_360 = 3 };

// Topics
const std::string TOPIC_CMD_VEL = "cmd_vel";
const std::string TOPIC_LEFT_SPEED = "left_speed";
const std::string TOPIC_RIGHT_SPEED = "right_speed";
const std::string TOPIC_MOTOR = "motor_on";
const std::string TOPIC_ANGLE = "angle";
const std::string TOPIC_STEPS = "motor_steps";
const std::string TOPIC_FREQ = "motor_max_freq";
const std::string TOPIC_MODE = "motor_mode";
const std::string TOPIC_ACCEL = "motor_accel";

// PiCarController class
// This class controls the Pi Car using pigpio and ROS2
// It subscribes to velocity commands and publishes odometry and speed
// information
class PiCarController : public rclcpp::Node {
public:
  PiCarController()
      : Node("pi_car_s_controller_accel"), pi_(-1), left_step_count_(0),
        right_step_count_(0), motor_on_(false), motor_mode_(MotorMode::AUTO),
        x_pos_(0.0), y_pos_(0.0), theta_(0.0), left_forwards_(true),
        right_forwards_(true), last_left_target_velocity_(0.0),
        last_right_target_velocity_(0.0), current_left_velocity_(0.0),
        current_right_velocity_(0.0), acceleration_rate_(1.0),
        sub_steps_(SUB_STEPS), target_angle_(0.0), current_angle_(0.0),
        angle_delta_(ANGLE_DELTA), turn_started_(false) {
    RCLCPP_INFO(
        this->get_logger(),
        "Started Pi Car Stepper Controller with Acceleration (C++ Class)");

    // Parameters. All can be changed at run time with ros2 param set, so
    // the odometry can be calibrated without rebuilding, and the TF can be
    // turned off when an EKF publishes odom -> base_link instead
    publish_odom_tf_ = this->declare_parameter("publish_odom_tf", true);
    wheel_separation_ =
        this->declare_parameter("wheel_separation", DEFAULT_WHEEL_SEPARATION);
    m_per_revolution_ =
        this->declare_parameter("m_per_revolution", DEFAULT_M_PER_REVOLUTION);
    cmd_vel_timeout_ =
        this->declare_parameter("cmd_vel_timeout", DEFAULT_CMD_VEL_TIMEOUT);
    if (wheel_separation_ <= 0.0 || m_per_revolution_ <= 0.0 ||
        cmd_vel_timeout_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(),
                   "wheel_separation, m_per_revolution and cmd_vel_timeout "
                   "must be positive, using defaults");
      wheel_separation_ = DEFAULT_WHEEL_SEPARATION;
      m_per_revolution_ = DEFAULT_M_PER_REVOLUTION;
      cmd_vel_timeout_ = DEFAULT_CMD_VEL_TIMEOUT;
    }
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PiCarController::parameters_callback, this,
                  std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
                "wheel_separation %.4f m, m_per_revolution %.4f m, "
                "cmd_vel_timeout %.2f s, publish_odom_tf %s",
                wheel_separation_, m_per_revolution_, cmd_vel_timeout_,
                publish_odom_tf_ ? "true" : "false");

    // Initialize pigpio
    if ((pi_ = pigpio_start(nullptr, nullptr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "gpio init failed");
      rclcpp::shutdown();
      return;
    }

    // Set GPIO modes
    gpio_write(pi_, ENABLE_PIN, PI_HIGH); // Disable drive initially
    set_mode(pi_, LEFT_FRONT_STEP_PIN, PI_OUTPUT);
    set_mode(pi_, LEFT_BACK_STEP_PIN, PI_OUTPUT);
    set_mode(pi_, RIGHT_FRONT_STEP_PIN, PI_OUTPUT);
    set_mode(pi_, RIGHT_BACK_STEP_PIN, PI_OUTPUT);
    set_mode(pi_, LEFT_FRONT_DIR_PIN, PI_OUTPUT);
    set_mode(pi_, LEFT_BACK_DIR_PIN, PI_OUTPUT);
    set_mode(pi_, RIGHT_FRONT_DIR_PIN, PI_OUTPUT);
    set_mode(pi_, RIGHT_BACK_DIR_PIN, PI_OUTPUT);
    set_mode(pi_, ENABLE_PIN, PI_OUTPUT);
    set_mode(pi_, RED_LED_PIN, PI_OUTPUT);
    set_mode(pi_, BLUE_LED_PIN, PI_OUTPUT);
    set_mode(pi_, M0_SUBSTEP, PI_OUTPUT);
    set_mode(pi_, M1_SUBSTEP, PI_OUTPUT);
    set_mode(pi_, M2_SUBSTEP, PI_OUTPUT);

    gpio_write(pi_, BLUE_LED_PIN, PI_HIGH); // Initial blue LED state

    // ROS subscriptions
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
        TOPIC_CMD_VEL, 2,
        std::bind(&PiCarController::velocity_callback, this,
                  std::placeholders::_1));
    sub_motor_ = this->create_subscription<std_msgs::msg::Bool>(
        TOPIC_MOTOR, 1,
        std::bind(&PiCarController::motor_enable_callback, this,
                  std::placeholders::_1));
    sub_motor_mode_ = this->create_subscription<std_msgs::msg::Int32>(
        TOPIC_MODE, 1,
        std::bind(&PiCarController::motor_mode_callback, this,
                  std::placeholders::_1));
    sub_set_steps_ = this->create_subscription<std_msgs::msg::Int32>(
        TOPIC_STEPS, 1,
        std::bind(&PiCarController::steps_callback, this,
                  std::placeholders::_1));
    sub_motor_accel_ = this->create_subscription<std_msgs::msg::Float32>(
        TOPIC_ACCEL, 1,
        std::bind(&PiCarController::motor_accel_callback, this,
                  std::placeholders::_1));

    // ROS publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    left_speed_pub_ =
        this->create_publisher<std_msgs::msg::Float32>(TOPIC_LEFT_SPEED, 2);
    right_speed_pub_ =
        this->create_publisher<std_msgs::msg::Float32>(TOPIC_RIGHT_SPEED, 2);
    angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(TOPIC_ANGLE, 2);

    // TF broadcaster
    odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Timers
    create_message_timeout_timer();
    odom_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(ODOM_TIMEOUT),
        std::bind(&PiCarController::odom_timer_callback, this));
    control_timer_ = this->create_wall_timer(
        10ms, // 100Hz control loop
        std::bind(&PiCarController::control_loop, this));
    led_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(LED_FLASH_TIMER),
        std::bind(&PiCarController::led_timer_callback, this));

    // Pigpio callbacks for speed encoders
    callback_ex(pi_, LEFT_FRONT_STEP_PIN, RISING_EDGE,
                left_speed_callback_static, this);
    callback_ex(pi_, RIGHT_FRONT_STEP_PIN, RISING_EDGE,
                right_speed_callback_static, this);

    // Set initial values
    gpio_write(pi_, ENABLE_PIN, PI_HIGH); // Disable drive initially
    set_substep(sub_steps_);
    last_control_time_ = this->now();
  }

  // Destructor
  // It stops the motors, disables the drive, and stops pigpio
  ~PiCarController() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Pi Car Controller");
    gpio_write(pi_, ENABLE_PIN, PI_HIGH); // Disable drive
    hardware_PWM(pi_, LEFT_FRONT_STEP_PIN, 0, 0);
    hardware_PWM(pi_, LEFT_BACK_STEP_PIN, 0, 0);
    hardware_PWM(pi_, RIGHT_FRONT_STEP_PIN, 0, 0);
    hardware_PWM(pi_, RIGHT_BACK_STEP_PIN, 0, 0);
    gpio_write(pi_, BLUE_LED_PIN, PI_HIGH); // Disable blue LED
    gpio_write(pi_, RED_LED_PIN, PI_LOW);   // Disable red LED
    pigpio_stop(pi_);                       // Stop pigpio
  }

private:
  int pi_;
  rclcpp::TimerBase::SharedPtr message_timeout_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr led_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_velocity_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_motor_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_motor_mode_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_set_steps_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_set_freq_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_motor_accel_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;

  // ROS parameters
  bool publish_odom_tf_;
  double wheel_separation_;
  double m_per_revolution_;
  double cmd_vel_timeout_;

  // Step counts and directions are shared with the pigpio callback thread
  std::atomic<int> left_step_count_;
  std::atomic<int> right_step_count_;
  bool motor_on_;
  MotorMode motor_mode_;
  double x_pos_;
  double y_pos_;
  double theta_;
  std::atomic<bool> left_forwards_;
  std::atomic<bool> right_forwards_;
  float last_left_target_velocity_;
  float last_right_target_velocity_;
  float current_left_velocity_;
  float current_right_velocity_;
  float acceleration_rate_;
  int sub_steps_;
  float target_angle_;
  float current_angle_;
  float angle_delta_;
  bool turn_started_;
  rclcpp::Time last_control_time_;

  // (Re)create the timer that stops the motors when cmd_vel stops arriving
  void create_message_timeout_timer() {
    if (message_timeout_timer_) {
      message_timeout_timer_->cancel();
    }
    message_timeout_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(cmd_vel_timeout_),
        std::bind(&PiCarController::message_timeout_callback, this));
  }

  // Validate and apply parameter changes made at run time
  rcl_interfaces::msg::SetParametersResult
  parameters_callback(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Check everything first, so a rejected set changes nothing
    for (const auto &param : params) {
      const std::string &name = param.get_name();
      if ((name == "wheel_separation" || name == "m_per_revolution" ||
           name == "cmd_vel_timeout") &&
          param.as_double() <= 0.0) {
        result.successful = false;
        result.reason = name + " must be positive";
        return result;
      }
    }

    for (const auto &param : params) {
      const std::string &name = param.get_name();
      if (name == "publish_odom_tf") {
        publish_odom_tf_ = param.as_bool();
      } else if (name == "wheel_separation") {
        wheel_separation_ = param.as_double();
      } else if (name == "m_per_revolution") {
        m_per_revolution_ = param.as_double();
      } else if (name == "cmd_vel_timeout") {
        cmd_vel_timeout_ = param.as_double();
        create_message_timeout_timer();
      } else {
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "Parameter %s set to %s", name.c_str(),
                  param.value_to_string().c_str());
    }
    return result;
  }

  // Set the number of substeps for the motor
  void set_substep(int steps) {
    int bin_steps;
    bool m0;
    bool m1;
    bool m2;

    bin_steps = (steps == 0)
                    ? 0
                    : static_cast<int>(std::log2(static_cast<double>(steps)));
    RCLCPP_INFO(this->get_logger(), "Substeps set to %i", steps);

    m0 = bin_steps & 1;
    m1 = bin_steps & 2;
    m2 = bin_steps & 4;

    gpio_write(pi_, M0_SUBSTEP, (m0 == 1) ? PI_HIGH : PI_LOW);
    gpio_write(pi_, M1_SUBSTEP, (m1 == 1) ? PI_HIGH : PI_LOW);
    gpio_write(pi_, M2_SUBSTEP, (m2 == 1) ? PI_HIGH : PI_LOW);

    sub_steps_ = steps;
  }

  // Apply acceleration to the current velocity
  void apply_acceleration(float target_velocity, float &current_velocity,
                          double time_diff) {
    if (acceleration_rate_ == 0.0) {
      current_velocity = target_velocity; // Instant change
      return;
    } else {

      if (current_velocity != target_velocity) {
        if (target_velocity > current_velocity) {
          current_velocity += acceleration_rate_ * time_diff;
          if (current_velocity > target_velocity) {
            current_velocity = target_velocity;
          }
        } else {
          current_velocity -= acceleration_rate_ * time_diff;
          if (current_velocity < target_velocity) {
            current_velocity = target_velocity;
          }
        }
      }
    }
  }

  // Set the left motor PWM based on the target velocities
  void set_motor_pwm_left(float left_vel) {
    uint frequency =
        static_cast<unsigned int>(STEPS_PER_REVOLUTION * sub_steps_ *
                                  std::abs(left_vel) / m_per_revolution_);
    RCLCPP_DEBUG(this->get_logger(), "Left motor vel: %f, freq: %i", left_vel,
                 frequency);
    // Speeds too small for a 1 Hz step rate (e.g. at the end of a ramp) stop
    if (frequency == 0) {
      hardware_PWM(pi_, LEFT_FRONT_STEP_PIN, 0, 0);
      hardware_PWM(pi_, LEFT_BACK_STEP_PIN, 0, 0);
    } else {
      gpio_write(pi_, LEFT_FRONT_DIR_PIN, (left_vel < 0) ? PI_LOW : PI_HIGH);
      gpio_write(pi_, LEFT_BACK_DIR_PIN, (left_vel < 0) ? PI_LOW : PI_HIGH);
      hardware_PWM(pi_, LEFT_FRONT_STEP_PIN, frequency,
                   500000); // 0.5 duty cycle
      hardware_PWM(pi_, LEFT_BACK_STEP_PIN, frequency,
                   500000); // 0.5 duty cycle
    }
  }

  // Set the right motor PWM based on the target velocities
  void set_motor_pwm_right(float right_vel) {
    uint frequency =
        static_cast<unsigned int>(STEPS_PER_REVOLUTION * sub_steps_ *
                                  std::abs(right_vel) / m_per_revolution_);
    RCLCPP_DEBUG(this->get_logger(), "Right motor vel: %f, freq: %i", right_vel,
                 frequency);
    // Speeds too small for a 1 Hz step rate (e.g. at the end of a ramp) stop
    if (frequency == 0) {
      hardware_PWM(pi_, RIGHT_FRONT_STEP_PIN, 0, 0);
      hardware_PWM(pi_, RIGHT_BACK_STEP_PIN, 0, 0);
    } else {
      gpio_write(pi_, RIGHT_FRONT_DIR_PIN, (right_vel > 0) ? PI_LOW : PI_HIGH);
      gpio_write(pi_, RIGHT_BACK_DIR_PIN, (right_vel > 0) ? PI_LOW : PI_HIGH);
      hardware_PWM(pi_, RIGHT_FRONT_STEP_PIN, frequency,
                   500000); // 0.5 duty cycle
      hardware_PWM(pi_, RIGHT_BACK_STEP_PIN, frequency,
                   500000); // 0.5 duty cycle
    }
  }

  // Set the target velocity for the motors
  // speed is in m/s and turn_rate in rad/s, positive turns anticlockwise
  // (REP-103), using differential drive kinematics
  void motor_set_target(float speed, float turn_rate) {
    if (speed != 0.0 && motor_mode_ == MotorMode::TURN_360) {
      target_angle_ = current_angle_;
      turn_started_ = false;
    }

    float wheel_offset = turn_rate * wheel_separation_ / 2.0;
    last_left_target_velocity_ = speed - wheel_offset;
    last_right_target_velocity_ = speed + wheel_offset;
  }

  // Control loop to set the motor PWM based on the target velocities
  // It applies acceleration to the target velocities and sets the PWM
  // accordingly
  void control_loop() {
    rclcpp::Time current_time = this->now();
    double time_diff = (current_time - last_control_time_).seconds();
    float old_current_left_velocity = current_left_velocity_;
    float old_current_right_velocity = current_right_velocity_;

    // Apply acceleration to the current velocities
    apply_acceleration(last_left_target_velocity_, current_left_velocity_,
                       time_diff);
    apply_acceleration(last_right_target_velocity_, current_right_velocity_,
                       time_diff);

    // Only set PWM if the target velocity has changed
    if (old_current_left_velocity != current_left_velocity_) {
      set_motor_pwm_left(current_left_velocity_);
    }

    if (old_current_right_velocity != current_right_velocity_) {
      set_motor_pwm_right(current_right_velocity_);
    }

    // Update the direction based on the current velocity
    left_forwards_ = (current_left_velocity_ >= 0.0);
    right_forwards_ = (current_right_velocity_ >= 0.0);

    last_control_time_ = current_time;
  }

  // Callback for message timeout
  // If no message is received within the timeout period, stop the motors
  void message_timeout_callback() {
    // ROS callback if no message received to stop motors
    if ((last_left_target_velocity_ != 0.0 ||
         last_right_target_velocity_ != 0.0) &&
        motor_mode_ == MotorMode::AUTO) {

      motor_set_target(0.0, 0.0); // Set target velocities to zero
    }
  }

  // Callback for LED timer
  // It toggles the blue LED based on the motor state
  void led_timer_callback() {
    // ROS callback for LED messages
    if (!motor_on_) {
      // Switch to disable
      gpio_write(pi_, BLUE_LED_PIN, PI_HIGH);
    } else {
      // Switch to enable (toggle)
      gpio_write(pi_, BLUE_LED_PIN, !gpio_read(pi_, BLUE_LED_PIN));
    }
  }

  static void left_speed_callback_static(int pi, uint32_t gpio, uint32_t level,
                                         uint32_t tick, void *userdata) {
    auto controller = static_cast<PiCarController *>(userdata);
    controller->left_speed_callback(pi, gpio, level, tick);
  }

  void left_speed_callback([[maybe_unused]] int pi,
                           [[maybe_unused]] uint32_t gpio,
                           [[maybe_unused]] uint32_t level,
                           [[maybe_unused]] uint32_t tick) {
    // Pigpio callback on speed encoded pin rising edge
    (left_forwards_) ? left_step_count_++ : left_step_count_--;
  }

  static void right_speed_callback_static(int pi, uint32_t gpio, uint32_t level,
                                          uint32_t tick, void *userdata) {
    auto controller = static_cast<PiCarController *>(userdata);
    controller->right_speed_callback(pi, gpio, level, tick);
  }

  void right_speed_callback([[maybe_unused]] int pi,
                            [[maybe_unused]] uint32_t gpio,
                            [[maybe_unused]] uint32_t level,
                            [[maybe_unused]] uint32_t tick) {
    // Pigpio callback on speed encoded pin rising edge
    (right_forwards_) ? right_step_count_++ : right_step_count_--;
  }

  // Callback for motor acceleration
  // It sets the acceleration rate for the motors
  void motor_accel_callback(const std_msgs::msg::Float32 &msg) {
    acceleration_rate_ = msg.data;
    RCLCPP_INFO(this->get_logger(), "Setting acceleration rate to %.2f",
                acceleration_rate_);
  }

  // Create a quaternion message from yaw angle
  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  // Callback for odometry timer
  // It calculates the odometry based on the wheel speeds and publishes the
  // odometry message It also checks if the target angle has been reached and
  // stops the motors if necessary
  void odom_timer_callback() {
    // ROS callback on time to send wheel speeds and odom
    std_msgs::msg::Float32 left_speed_msg;
    std_msgs::msg::Float32 right_speed_msg;
    std_msgs::msg::Float32 angle_msg;
    double left_delta;
    double right_delta;
    double x_delta;
    double y_delta;
    double theta_delta;
    double angle_error;

    // Read and reset atomically so no steps are lost to the callback thread
    left_delta = static_cast<double>(left_step_count_.exchange(0)) *
                 m_per_revolution_ / (sub_steps_ * STEPS_PER_REVOLUTION);
    right_delta = static_cast<double>(right_step_count_.exchange(0)) *
                  m_per_revolution_ / (sub_steps_ * STEPS_PER_REVOLUTION);

    left_speed_msg.data = static_cast<float>(left_delta / ODOM_TIMEOUT);
    right_speed_msg.data = static_cast<float>(right_delta / ODOM_TIMEOUT);
    theta_delta = (right_delta - left_delta) / wheel_separation_;

    theta_ += theta_delta;
    if (theta_ > M_PI) {
      theta_ -= 2.0 * M_PI;
    }
    if (theta_ < -M_PI) {
      theta_ += 2.0 * M_PI;
    }
    angle_msg.data =
        static_cast<float>(180.0 * theta_ / M_PI); // Send angle in degrees
    current_angle_ = angle_msg.data;
    // Wrap the error to +/-180 so it is correct across the +/-180 boundary
    angle_error = std::abs(std::remainder(current_angle_ - target_angle_, 360.0));
    if ((angle_error > angle_delta_) && (!turn_started_)) {
      turn_started_ = true;
      RCLCPP_INFO(this->get_logger(), "Turn started set to true");
    }

    if (turn_started_ && angle_error < angle_delta_ &&
        motor_mode_ == MotorMode::TURN_360 &&
        (last_left_target_velocity_ != 0.0 ||
         last_right_target_velocity_ != 0.0)) {
      RCLCPP_INFO(this->get_logger(), "Target angle reached");
      motor_set_target(0.0, 0.0); // Set target velocities to zero to stop
    }

    x_delta = cos(theta_) * (left_speed_msg.data + right_speed_msg.data) / 2.0 *
              ODOM_TIMEOUT; // Approx. for small angle delta
    y_delta = sin(theta_) * (left_speed_msg.data + right_speed_msg.data) / 2.0 *
              ODOM_TIMEOUT; // Approx. for small angle delta
    x_pos_ += x_delta;
    y_pos_ += y_delta;

    geometry_msgs::msg::Quaternion odom_quat =
        createQuaternionMsgFromYaw(theta_);

    rclcpp::Time stamp = this->now();

    // First, publish the transform over tf, unless an EKF publishes it
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    if (publish_odom_tf_) {
      odom_broadcaster_->sendTransform(odom_trans);
    }

    // Next, publish the odometry message over ROS
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // Set the velocity, which is in the child (base_link) frame
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x =
        (left_speed_msg.data + right_speed_msg.data) / 2.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = theta_delta / ODOM_TIMEOUT;

    // Covariances, row-major 6x6 over (x, y, z, roll, pitch, yaw)
    odom.pose.covariance[0] = ODOM_POSE_XY_VAR;
    odom.pose.covariance[7] = ODOM_POSE_XY_VAR;
    odom.pose.covariance[14] = ODOM_UNUSED_VAR;
    odom.pose.covariance[21] = ODOM_UNUSED_VAR;
    odom.pose.covariance[28] = ODOM_UNUSED_VAR;
    odom.pose.covariance[35] = ODOM_POSE_YAW_VAR;
    odom.twist.covariance[0] = ODOM_TWIST_VX_VAR;
    odom.twist.covariance[7] = ODOM_TWIST_VY_VAR;
    odom.twist.covariance[14] = ODOM_UNUSED_VAR;
    odom.twist.covariance[21] = ODOM_UNUSED_VAR;
    odom.twist.covariance[28] = ODOM_UNUSED_VAR;
    odom.twist.covariance[35] = ODOM_TWIST_VYAW_VAR;

    left_speed_pub_->publish(left_speed_msg);
    right_speed_pub_->publish(right_speed_msg);
    angle_pub_->publish(angle_msg);
    odom_pub_->publish(odom);
  }

  // Callback for velocity command
  // It sets the target velocities for the motors based on the received message
  void velocity_callback(const geometry_msgs::msg::Twist &msg) {
    // ROS callback on message received
    // The timer will automatically reset if it's periodic

    // Ignore commands while the drivers are disabled, so the car does not
    // jump to a stored speed when they are enabled
    if (!motor_on_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Ignoring cmd_vel, motors are off");
      return;
    }

    // reset the message timeout timer
    message_timeout_timer_->reset();
    motor_set_target(msg.linear.x, msg.angular.z);
  }

  // Callback for motor steps
  // It sets the number of steps for the motor driver
  void steps_callback(const std_msgs::msg::Int32 &msg) {
    // ROS callback on message received
    // Driver supports 1, 2, 4, 8, 16 or 32 microsteps
    if (msg.data < 1 || msg.data > 32 || (msg.data & (msg.data - 1)) != 0) {
      RCLCPP_WARN(this->get_logger(), "Invalid substeps received %i",
                  msg.data);
      return;
    }
    set_substep(msg.data);
  }

  void motor_mode_callback(const std_msgs::msg::Int32 &msg) {
    // ROS callback on message received
    switch (msg.data) {
    case static_cast<int>(MotorMode::AUTO):
      motor_mode_ = MotorMode::AUTO;
      RCLCPP_INFO(this->get_logger(), "Set to auto mode");
      break;
    case static_cast<int>(MotorMode::CONTINUOUS):
      motor_mode_ = MotorMode::CONTINUOUS;
      RCLCPP_INFO(this->get_logger(), "Set to continuous mode");
      break;
    case static_cast<int>(MotorMode::TURN_360):
      motor_mode_ = MotorMode::TURN_360;
      RCLCPP_INFO(this->get_logger(), "Set to 360 mode");
      break;
    default:
      RCLCPP_INFO(this->get_logger(), "Invalid mode received %i", msg.data);
      break;
    }
  }

  void motor_enable_callback(const std_msgs::msg::Bool &msg) {
    // ROS callback on message received
    motor_on_ = msg.data;
    if (msg.data) {
      RCLCPP_INFO(this->get_logger(), "Set motor on");
      gpio_write(pi_, ENABLE_PIN, PI_LOW); // Enable drive
    } else {
      RCLCPP_INFO(this->get_logger(), "Set motor off");
      gpio_write(pi_, ENABLE_PIN, PI_HIGH); // Disable drive
      // Stop immediately, with no ramp down, so nothing is left to resume
      // and odometry does not count steps the wheels never made
      motor_set_target(0.0, 0.0);
      current_left_velocity_ = 0.0;
      current_right_velocity_ = 0.0;
      set_motor_pwm_left(0.0);
      set_motor_pwm_right(0.0);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PiCarController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}