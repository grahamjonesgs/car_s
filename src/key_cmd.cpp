
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_F 0x66
#define KEYCODE_S 0x73
#define KEYCODE_M 0x6D
#define KEYCODE_0 0x30
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39
#define KEYCODE_SP 0x20
std_msgs::msg::Bool motor;
std_msgs::msg::Int32 motor_steps;

class TeleopCmd
{
public:
    TeleopCmd();
    void keyLoop();

private:
    std::shared_ptr<rclcpp::Node> nh;
    double linear, angular, scale;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor_steps_pub_;
};

TeleopCmd::TeleopCmd() : linear(0),
                         angular(0),
                         scale(0.2)
{
    nh = rclcpp::Node::make_shared("teleop_cmd");
    nh->declare_parameter("scale", scale);
    nh->get_parameter("scale", scale);

    twist_pub_ = nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    motor_pub_ = nh->create_publisher<std_msgs::msg::Bool>("/motor_on", 1);
    motor_steps_pub_ = nh->create_publisher<std_msgs::msg::Int32>("/motor_steps", 1);
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
            scale += 0.1;
            /*
            Removed the following line to allow scale to go above 1.0 for testing
            if (scale > 1.0)
                scale = 1.0;*/
            RCLCPP_INFO(nh->get_logger(), "Scale is %0.1f", scale);
            break;
        case KEYCODE_S:
            RCLCPP_DEBUG(nh->get_logger(), "SLOWER");
            scale -= 0.1;
            if (scale < 0.0)
                scale = 0.0;
            RCLCPP_INFO(nh->get_logger(), "Scale is %0.1f", scale);
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
        }

        geometry_msgs::msg::Twist twist;
        twist.angular.z = scale * angular;
        twist.linear.x = scale * linear;
        if (dirty == true)
        {
            twist_pub_->publish(twist);
            dirty = false;
        }
    }

    return;
}