#include "pigpiod_if2.h"
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#define RED_LED_PIN 23
#define BLUE_LED_PIN 24
#define SW_RED 11
#define SW_YELLOW 5

int pi;
int command_sent = 0;
rclcpp::Node::SharedPtr nh;

void delay(int number_of_nano_seconds) {
  int milli_seconds = 1000 * number_of_nano_seconds;
  clock_t start_time = clock();
  while (clock() < start_time + milli_seconds)
    ;
}

void switch_callback([[maybe_unused]] int pi, [[maybe_unused]] uint32_t gpio,
                     [[maybe_unused]] uint32_t level,
                     [[maybe_unused]] uint32_t tick) {
  int red = gpio_read(pi, SW_RED);
  int yellow = gpio_read(pi, SW_YELLOW);

  if (command_sent == 1) {
    return; // Once we are rebooting or shutting down do nothing more
  }

  if (yellow == PI_LOW && red == PI_LOW) {
    RCLCPP_INFO(nh->get_logger(), "Pi shutting down");
    printf("Shutting down\n");
    command_sent = 1;
    for (int i = 0; i < 10; i++) {
      gpio_write(pi, RED_LED_PIN, PI_LOW);
      usleep(1000 * 200);
      gpio_write(pi, RED_LED_PIN, PI_HIGH);
      usleep(1000 * 200);
    }
    system("shutdown -h now");
    return;
  }

  if (yellow == PI_LOW) {
    RCLCPP_INFO(nh->get_logger(), "Pi rebooting");
    printf("Rebooting\n");
    for (int i = 0; i < 5; i++) {
      gpio_write(pi, RED_LED_PIN, PI_LOW);
      usleep(1000 * 500);
      gpio_write(pi, RED_LED_PIN, PI_HIGH);
      usleep(1000 * 500);
    }
    system("shutdown -r now");
    command_sent = 1;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("pi_power_control");

  pi = pigpio_start(NULL, NULL);
  if (pi < 0) {
    RCLCPP_ERROR(nh->get_logger(), "Failed to initialize pigpio: %d", pi);
    rclcpp::shutdown();
    return 1;
  } else {
    RCLCPP_INFO(nh->get_logger(), "pigpio initialized successfully");
  }

  set_mode(pi, RED_LED_PIN, PI_OUTPUT);
  set_mode(pi, BLUE_LED_PIN, PI_OUTPUT);
  set_mode(pi, SW_RED, PI_INPUT);
  set_mode(pi, SW_YELLOW, PI_INPUT);

  gpio_write(pi, RED_LED_PIN, PI_HIGH);
  gpio_write(pi, BLUE_LED_PIN, PI_LOW);
  set_pull_up_down(pi, SW_RED, PI_PUD_UP);
  set_pull_up_down(pi, SW_YELLOW, PI_PUD_UP);

  callback(pi, SW_RED, FALLING_EDGE, switch_callback);
  callback(pi, SW_YELLOW, FALLING_EDGE, switch_callback);

  rclcpp::spin(nh);

  pigpio_stop(pi);
  rclcpp::shutdown();
  return 0;
}