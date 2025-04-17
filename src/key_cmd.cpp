
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43  // Right
#define KEYCODE_L 0x44  // Left
#define KEYCODE_U 0x41  // Forwards
#define KEYCODE_D 0x42  // Backwards
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
std_msgs::msg::Bool motor;
std_msgs::msg::Int32 motor_steps;
std_msgs::msg::Float32 motor_accel;

class TeleopCmd
{
public:
    TeleopCmd();
    void keyLoop();

private:
    std::shared_ptr<rclcpp::Node> nh;
    double linear, angular, velocity, acceleration;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor_steps_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_accel_pub_;
};

TeleopCmd::TeleopCmd() : linear(0),
                         angular(0),
                         velocity(0.2),
                         acceleration(0.5)
{
    nh = rclcpp::Node::make_shared("teleop_cmd");
    nh->declare_parameter("velocity", velocity);
    nh->get_parameter("velocity", velocity);

    twist_pub_ = nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    motor_pub_ = nh->create_publisher<std_msgs::msg::Bool>("/motor_on", 1);
    motor_steps_pub_ = nh->create_publisher<std_msgs::msg::Int32>("/motor_steps", 1);
    motor_accel_pub_ = nh->create_publisher<std_msgs::msg::Float32>("/motor_accel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    TeleopCmd teleop_cmd;
    signal(SIGINT, quit);
    teleop_cmd.keyLoop();
    return (0);
}

void TeleopCmd::keyLoop()
{
    char c;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the car.");

    for (;;)
    {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        linear = angular = 0;
        RCLCPP_DEBUG(nh->get_logger(), "value: 0x%02X\n", c);

        switch (c)
        {
        case KEYCODE_SP:
            RCLCPP_DEBUG(nh->get_logger(), "STOP");
            dirty = true;
            break;
        case KEYCODE_8:
            RCLCPP_DEBUG(nh->get_logger(), "TURN LEFT");
            linear = 1.0;
            angular = -0.5;
            dirty = true;
            break;
        case KEYCODE_9:
            RCLCPP_DEBUG(nh->get_logger(), "TURN RIGHT");
            linear = 1.0;
            angular = 0.5;
            dirty = true;
            break;
        case KEYCODE_L:
            RCLCPP_DEBUG(nh->get_logger(), "LEFT");
            angular = 1.0;
            dirty = true;
            break;
        case KEYCODE_R:
            RCLCPP_DEBUG(nh->get_logger(), "RIGHT");
            angular = -1.0;
            dirty = true;
            break;
        case KEYCODE_U:
            RCLCPP_DEBUG(nh->get_logger(), "UP");
            linear = 1.0;
            dirty = true;
            break;
        case KEYCODE_D:
            RCLCPP_DEBUG(nh->get_logger(), "DOWN");
            linear = -1.0;
            dirty = true;
            break;
        case KEYCODE_F:
            RCLCPP_DEBUG(nh->get_logger(), "FASTER");
            velocity += 0.1;
            /*
            Removed the following line to allow velocity to go above 1.0 for testing
            if (velocity > 1.0)
                velocity = 1.0;*/
            RCLCPP_INFO(nh->get_logger(), "Velocity is %0.1f", velocity);
            break;
        case KEYCODE_S:
            RCLCPP_DEBUG(nh->get_logger(), "SLOWER");
            velocity -= 0.1;
            if (velocity < 0.0)
                velocity = 0.0;
            RCLCPP_INFO(nh->get_logger(), "Velocity is %0.1f", velocity);
            break;
        case KEYCODE_0:
            motor_steps.data = 0;
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

            if (motor.data == 0)
            {
                motor.data = 1;
                RCLCPP_INFO(nh->get_logger(), "Motor on");
            }
            else
            {
                motor.data = 0;
                RCLCPP_INFO(nh->get_logger(), "Motor off");
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
            RCLCPP_DEBUG(nh->get_logger(), "ACCELERATION 0.5");
            acceleration = 0.5;
            motor_accel.data = acceleration;
            motor_accel_pub_->publish(motor_accel);
            RCLCPP_INFO(nh->get_logger(), "Acceleration is %0.1f", acceleration);
            break;
        case KEYCODE_V:
            RCLCPP_DEBUG(nh->get_logger(), "ACCELERATION 1.0");
            acceleration = 1.0;
            motor_accel.data = acceleration;
            motor_accel_pub_->publish(motor_accel);
            RCLCPP_INFO(nh->get_logger(), "Acceleration is %0.1f", acceleration);
            break;
        }

        geometry_msgs::msg::Twist twist;
        twist.angular.z = velocity * angular;
        twist.linear.x = velocity * linear;
        if (dirty == true)
        {
            twist_pub_->publish(twist);
            dirty = false;
        }
    }

    return;
}