
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"
#include <chrono>
#include <cmath>
#include <errno.h>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

// Arrow keys arrive as escape sequences (ESC [ A); readKey() turns them into
// these codes, outside the range of single bytes
#define KEYCODE_U_ARROW 0x101 // Forwards
#define KEYCODE_D_ARROW 0x102 // Backwards
#define KEYCODE_R_ARROW 0x103 // Right
#define KEYCODE_L_ARROW 0x104 // Left
#define KEY_NONE 0            // Nothing, or a key that does nothing
#define KEY_QUIT -1           // Quit, end of input, or an error
#define KEYCODE_Q 0x71  // Quit
#define KEYCODE_F 0x66  // Faster
#define KEYCODE_S 0x73  // Slower
#define KEYCODE_M 0x6D  // Motor control on off
#define KEYCODE_0 0x30  // Set to single steps
#define KEYCODE_1 0x31  // Set to 2 steps
#define KEYCODE_2 0x32  // Set to 4 steps
#define KEYCODE_3 0x33  // Set to 8 steps
#define KEYCODE_4 0x34  // Set to 16 steps
#define KEYCODE_5 0x35  // Set to 32 steps
#define KEYCODE_8 0x38  // Turn left
#define KEYCODE_9 0x39  // Turn right
#define KEYCODE_SP 0x20 // Stop
#define KEYCODE_Z 0x7A  // Increase acceleration
#define KEYCODE_X 0x78  // Decrease acceleration
#define KEYCODE_C 0x63  // Set acceleration to 0.5
#define KEYCODE_V 0x76  // Set acceleration to 1.0
#define KEYCODE_L 0x6C  // Lidar motor on off

// Effective wheel separation in m, used to convert wheel speed to turn rate in
// rad/s. Matches the controller's calibrated default
#define WHEEL_SEPARATION 0.262
// Top speed for f and the velocity parameter, in m/s. The controller's
// default max_wheel_speed: the steppers stall above about this
#define MAX_VELOCITY 0.5
#define DEFAULT_VELOCITY 0.2
// The lidar counts as on if a scan arrived within this long, in s
#define SCAN_RECENT 1.0
std_msgs::msg::Bool motor;
std_msgs::msg::Int32 motor_steps;
std_msgs::msg::Float32 motor_accel;

class TeleopCmd {
public:
  TeleopCmd();
  void keyLoop();
  void stop();

private:
  int readKey();
  bool lidarOn();
  bool moving; // the last command sent was not a stop
  std::shared_ptr<rclcpp::Node> nh;
  // Spun from the key loop, between key reads
  rclcpp::executors::SingleThreadedExecutor executor_;
  double linear, angular, velocity, acceleration;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor_steps_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_accel_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr lidar_start_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr lidar_stop_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motor_state_sub_;
  rclcpp::GenericSubscription::SharedPtr scan_sub_;
  bool motor_state_known; // a /motor_state message has arrived
  bool motor_state;       // the drivers' real enable state, from the controller
  bool scan_seen;
  std::chrono::steady_clock::time_point last_scan;
};

TeleopCmd::TeleopCmd()
    : moving(false), linear(0), angular(0), velocity(DEFAULT_VELOCITY),
      acceleration(0.5), motor_state_known(false), motor_state(false),
      scan_seen(false) {
  nh = rclcpp::Node::make_shared("teleop_cmd");
  nh->declare_parameter("velocity", velocity);
  nh->get_parameter("velocity", velocity);
  if (!std::isfinite(velocity) || velocity <= 0.0 || velocity > MAX_VELOCITY) {
    RCLCPP_ERROR(nh->get_logger(),
                 "velocity must be above 0 and at most %.2f m/s, using %.2f",
                 MAX_VELOCITY, DEFAULT_VELOCITY);
    velocity = DEFAULT_VELOCITY;
  }

  twist_pub_ = nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  motor_pub_ = nh->create_publisher<std_msgs::msg::Bool>("/motor_on", 1);
  motor_steps_pub_ =
      nh->create_publisher<std_msgs::msg::Int32>("/motor_steps", 1);
  motor_accel_pub_ =
      nh->create_publisher<std_msgs::msg::Float32>("/motor_accel", 1);
  lidar_start_client_ =
      nh->create_client<std_srvs::srv::Empty>("/start_motor");
  lidar_stop_client_ = nh->create_client<std_srvs::srv::Empty>("/stop_motor");

  // The controller keeps the latest real motor state here, so m can toggle it
  motor_state_sub_ = nh->create_subscription<std_msgs::msg::Bool>(
      "/motor_state", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool &msg) {
        motor_state = msg.data;
        motor_state_known = true;
      });
  // Only the arrival time matters, so the scan isn't deserialised
  scan_sub_ = nh->create_generic_subscription(
      "/scan", "sensor_msgs/msg/LaserScan", rclcpp::SensorDataQoS(),
      [this](std::shared_ptr<const rclcpp::SerializedMessage>) {
        last_scan = std::chrono::steady_clock::now();
        scan_seen = true;
      });
  executor_.add_node(nh);
}

// The lidar is spinning if scans are arriving
bool TeleopCmd::lidarOn() {
  return scan_seen && std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - last_scan)
                              .count() < SCAN_RECENT;
}

int kfd = 0;
struct termios cooked, raw;

// Set by SIGINT, SIGTERM, SIGHUP and SIGQUIT; the key loop then stops the car
// and restores the terminal, which isn't safe to do inside a signal handler
static volatile sig_atomic_t quit_requested = 0;
static void on_quit_signal(int) { quit_requested = 1; }

int main(int argc, char **argv) {
  // Our own signal handling, so Ctrl-C stops the car before exiting
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  if (!isatty(kfd) || tcgetattr(kfd, &cooked) < 0) {
    fprintf(stderr, "key_cmd needs a terminal to read keys from\n");
    rclcpp::shutdown();
    return 1;
  }
  struct sigaction sa = {};
  sa.sa_handler = on_quit_signal; // no SA_RESTART, so poll() wakes up
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
  sigaction(SIGHUP, &sa, nullptr);
  sigaction(SIGQUIT, &sa, nullptr); // Ctrl-\, which raw mode leaves on
  // A suspended key_cmd couldn't stop the car, so Ctrl-Z does nothing
  signal(SIGTSTP, SIG_IGN);

  // After an error too, send the stop and restore the terminal
  std::string error;
  try {
    TeleopCmd teleop_cmd;
    try {
      teleop_cmd.keyLoop();
    } catch (const std::exception &e) {
      error = e.what();
    }
    teleop_cmd.stop();
  } catch (const std::exception &e) {
    if (error.empty()) {
      error = e.what();
    }
  }
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  if (!error.empty()) {
    fprintf(stderr, "key_cmd: %s\n", error.c_str());
    return 1;
  }
  return 0;
}

// Wait up to timeout_ms for one byte. Returns 1 if read, 0 if none came, or
// -1 on quit, end of input or an error (never repeat the last key)
static int readByte(unsigned char &b, int timeout_ms) {
  if (quit_requested) {
    return -1;
  }
  struct pollfd pfd = {kfd, POLLIN, 0};
  int r = poll(&pfd, 1, timeout_ms);
  if (r < 0) {
    return (errno == EINTR && !quit_requested) ? 0 : -1;
  }
  if (r == 0) {
    return 0;
  }
  return (read(kfd, &b, 1) == 1) ? 1 : -1;
}

// The next key: a byte, an arrow code, KEY_NONE or KEY_QUIT. Other escape
// sequences (F-keys, PgUp, Insert, Ctrl-arrows) are read whole and ignored,
// so their last byte isn't taken as a key
int TeleopCmd::readKey() {
  unsigned char b;
  int r = readByte(b, 100);
  if (r <= 0) {
    return r < 0 ? KEY_QUIT : KEY_NONE;
  }
  if (b != 0x1B) {
    return b;
  }
  // ESC [ or ESC O, optional parameter bytes, then a final byte
  if ((r = readByte(b, 30)) <= 0) {
    return r < 0 ? KEY_QUIT : KEY_NONE; // Esc on its own
  }
  if (b != '[' && b != 'O') {
    return KEY_NONE;
  }
  bool parameters = false;
  for (;;) {
    if ((r = readByte(b, 30)) <= 0) {
      return r < 0 ? KEY_QUIT : KEY_NONE;
    }
    if (b >= 0x40 && b <= 0x7E) {
      break;
    }
    parameters = true;
  }
  if (parameters) {
    return KEY_NONE;
  }
  switch (b) {
  case 'A':
    return KEYCODE_U_ARROW;
  case 'B':
    return KEYCODE_D_ARROW;
  case 'C':
    return KEYCODE_R_ARROW;
  case 'D':
    return KEYCODE_L_ARROW;
  default:
    return KEY_NONE;
  }
}

// Stop the car if this program last told it to move
void TeleopCmd::stop() {
  if (!moving) {
    return;
  }
  geometry_msgs::msg::Twist zero;
  for (int i = 0; i < 3; ++i) {
    twist_pub_->publish(zero);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  moving = false;
  puts("Stopped");
}

void TeleopCmd::keyLoop() {
  int c;
  bool dirty = false;

  // get the console in raw mode
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the car. q or Ctrl-C to quit.");

  for (;;) {
    executor_.spin_some();
    // get the next event from the keyboard
    c = readKey();
    if (c == KEY_QUIT || c == KEYCODE_Q) {
      break;
    }
    if (c == KEY_NONE) {
      continue;
    }

    linear = angular = 0;
    RCLCPP_DEBUG(nh->get_logger(), "value: 0x%02X\n", c);

    switch (c) {
    case KEYCODE_SP:
      RCLCPP_DEBUG(nh->get_logger(), "STOP");
      dirty = true;
      break;
    case KEYCODE_8:
      RCLCPP_DEBUG(nh->get_logger(), "TURN LEFT");
      linear = 1.0;
      angular = 0.25;
      dirty = true;
      break;
    case KEYCODE_9:
      RCLCPP_DEBUG(nh->get_logger(), "TURN RIGHT");
      linear = 1.0;
      angular = -0.25;
      dirty = true;
      break;
    case KEYCODE_L_ARROW:
      RCLCPP_DEBUG(nh->get_logger(), "LEFT");
      angular = 1.0;
      dirty = true;
      break;
    case KEYCODE_R_ARROW:
      RCLCPP_DEBUG(nh->get_logger(), "RIGHT");
      angular = -1.0;
      dirty = true;
      break;
    case KEYCODE_U_ARROW:
      RCLCPP_DEBUG(nh->get_logger(), "UP");
      linear = 1.0;
      dirty = true;
      break;
    case KEYCODE_D_ARROW:
      RCLCPP_DEBUG(nh->get_logger(), "DOWN");
      linear = -1.0;
      dirty = true;
      break;
    case KEYCODE_F:
      velocity += 0.1;
      linear = 1.0;
      dirty = true;
      if (velocity > MAX_VELOCITY)
        velocity = MAX_VELOCITY;
      RCLCPP_INFO(nh->get_logger(), "Velocity is %0.2f", velocity);
      break;
    case KEYCODE_S:
      velocity -= 0.1;
      if (velocity < 0.0)
        velocity = 0.0;
      dirty = true;
      RCLCPP_INFO(nh->get_logger(), "Velocity is %0.2f", velocity);
      break;
    case KEYCODE_0:
      motor_steps.data = 1;
      motor_steps_pub_->publish(motor_steps);
      RCLCPP_INFO(nh->get_logger(), "Set to single steps");
      break;
    case KEYCODE_1:
      motor_steps.data = 2;
      motor_steps_pub_->publish(motor_steps);
      RCLCPP_INFO(nh->get_logger(), "Set to 2 steps");
      break;
    case KEYCODE_2:
      motor_steps.data = 4;
      motor_steps_pub_->publish(motor_steps);
      RCLCPP_INFO(nh->get_logger(), "Set to 4 steps");
      break;
    case KEYCODE_3:
      motor_steps.data = 8;
      motor_steps_pub_->publish(motor_steps);
      RCLCPP_INFO(nh->get_logger(), "Set to 8 steps");
      break;
    case KEYCODE_4:
      motor_steps.data = 16;
      motor_steps_pub_->publish(motor_steps);
      RCLCPP_INFO(nh->get_logger(), "Set to 16 steps");
      break;
    case KEYCODE_5:
      motor_steps.data = 32;
      motor_steps_pub_->publish(motor_steps);
      RCLCPP_INFO(nh->get_logger(), "Set to 32 steps");
      break;
    case KEYCODE_M:
      RCLCPP_DEBUG(nh->get_logger(), "MOTOR");

      // Toggle the real state. Not knowing it, stop: that's the safe guess
      if (!motor_state_known) {
        motor.data = false;
        RCLCPP_WARN(nh->get_logger(),
                    "Motor state unknown (nothing on /motor_state), motor off");
      } else {
        motor.data = !motor_state;
        motor_state = motor.data; // until the controller confirms it
        RCLCPP_INFO(nh->get_logger(), motor.data ? "Motor on" : "Motor off");
      }
      motor_pub_->publish(motor);

      break;
    case KEYCODE_Z:
      RCLCPP_DEBUG(nh->get_logger(), "ACCELERATION UP");
      acceleration += 0.1;
      if (acceleration > 2.0)
        acceleration = 2.0;
      motor_accel.data = acceleration;
      motor_accel_pub_->publish(motor_accel);
      RCLCPP_INFO(nh->get_logger(), "Acceleration is %0.1f", acceleration);
      break;
    case KEYCODE_X:
      RCLCPP_DEBUG(nh->get_logger(), "ACCELERATION DOWN");
      acceleration -= 0.1;
      if (acceleration < 0.1)
        acceleration = 0.1;
      motor_accel.data = acceleration;
      motor_accel_pub_->publish(motor_accel);
      RCLCPP_INFO(nh->get_logger(), "Acceleration is %0.1f", acceleration);
      break;
    case KEYCODE_C:
      RCLCPP_DEBUG(nh->get_logger(), "ACCELERATION 0");
      acceleration = 0.0;
      motor_accel.data = acceleration;
      motor_accel_pub_->publish(motor_accel);
      RCLCPP_INFO(nh->get_logger(), "Acceleration is off");
      break;
    case KEYCODE_V:
      RCLCPP_DEBUG(nh->get_logger(), "ACCELERATION 1.0");
      acceleration = 1.0;
      motor_accel.data = acceleration;
      motor_accel_pub_->publish(motor_accel);
      RCLCPP_INFO(nh->get_logger(), "Acceleration is %0.1f", acceleration);
      break;
    case KEYCODE_L: {
      RCLCPP_DEBUG(nh->get_logger(), "LIDAR");
      bool lidar_on = lidarOn();
      auto client = lidar_on ? lidar_stop_client_ : lidar_start_client_;
      if (!client->service_is_ready()) {
        RCLCPP_WARN(nh->get_logger(), "Lidar service not available");
        break;
      }
      // Send without waiting for the empty response
      client->async_send_request(
          std::make_shared<std_srvs::srv::Empty::Request>());
      client->prune_pending_requests();
      RCLCPP_INFO(nh->get_logger(), lidar_on ? "Lidar off" : "Lidar on");
      break;
    }
    }

    geometry_msgs::msg::Twist twist;
    // angular is the wheel speed difference as a fraction of velocity, sent
    // as a turn rate in rad/s (positive is anticlockwise)
    twist.angular.z = velocity * angular / (WHEEL_SEPARATION / 2.0);
    twist.linear.x = velocity * linear;
    if (dirty == true) {
      twist_pub_->publish(twist);
      moving = (twist.linear.x != 0.0 || twist.angular.z != 0.0);
      dirty = false;
    }
  }

  return;
}