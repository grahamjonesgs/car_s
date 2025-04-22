#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <pigpiod_if2.h>
#include <rclcpp/rclcpp.hpp>
#define LED_PIN 13 // change pin number here
int pi;
rclcpp::Node::SharedPtr nh;

void blink_callback(const std_msgs::msg::Bool msg) {
  if (msg.data == 1) {
    // set_PWM_duty cycle(pi,LED_PIN, 250);
    hardware_PWM(pi, LED_PIN, 800, 1e6);
    RCLCPP_INFO(nh->get_logger(), "LED ON");
  }
  if (msg.data == 0) {
    // set_PWM_duty cycle(pi,LED_PIN, 200);
    hardware_PWM(pi, LED_PIN, 800, 1e6 * 0.25);
    RCLCPP_INFO(nh->get_logger(), "LED OFF");
  }
}

void velocity_callback(const geometry_msgs::msg::Twist msg) {
  float speed;
  // float turn;
  speed = msg.linear.x;
  // turn=msg.angular.z;
  hardware_PWM(pi, LED_PIN, 800, 1e6 * speed);
  RCLCPP_INFO(nh->get_logger(), "Float control");
  /*if(msg->data == 1) {
          //set_PWM_duty cycle(pi,LED_PIN, 250);
          hardware_PWM(pi,LED_PIN,800,1e6);
          ROS_INFO("LED ON");
  }
  if(msg->data == 0) {
          //set_PWM_duty cycle(pi,LED_PIN, 200);
          hardware_PWM(pi,LED_PIN,800,1e6*0.25);
          ROS_INFO("LED OFF");
  }*/
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("pwm_led_node");
  RCLCPP_INFO(nh->get_logger(), "Started pwm LED Node");

  if ((pi = pigpio_start(NULL, NULL)) < 0) {
    RCLCPP_INFO(nh->get_logger(), "gpio init failed");
    return 1;
  } else {
    RCLCPP_INFO(nh->get_logger(), "gpio init ok");
  }

  set_mode(pi, LED_PIN, PI_OUTPUT);
  // hardware_PWM(pi,LED_PIN, )

  RCLCPP_INFO(nh->get_logger(), "GPIO has been set as OUTPUT.");

  auto sub_flash = nh->create_subscription<std_msgs::msg::Bool>("led_pwm", 10,
                                                                blink_callback);
  auto sub_velocity = nh->create_subscription<geometry_msgs::msg::Twist>(
      "velocity", 10, velocity_callback);
  rclcpp::spin(nh);
}