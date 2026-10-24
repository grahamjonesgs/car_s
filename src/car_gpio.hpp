// GPIO and step pulses for the car controller, through the kernel's own
// drivers.
//
// Step pulses come from the pwm-bcm2835 driver through /sys/class/pwm.
// overlays/car-pwm.dts puts PWM channel 0 on GPIO 12 and 18 (the right
// wheels) and channel 1 on GPIO 13 and 19 (the left), and
// tools/car_pwm_setup.sh (run as root by the car_s service) loads it,
// exports both channels and hands their files to the car's user. The other
// pins are outputs through libgpiod 2 (/dev/gpiochip0), in one line request.
//
// The kernel can't report 10,000 edges a second to a program, so the steps
// are worked out from the rate set: hardware PWM produces it exactly, so
// steps = rate x time, summed over every change.
#pragma once

#include <chrono>
#include <string>

#include <gpiod.h>
#include <rclcpp/rclcpp.hpp>

enum class Side { LEFT = 0, RIGHT = 1 };

// The pins to use as outputs, and the drivers' ENABLE pin among them
struct CarPins {
  int outputs[16];
  int n_outputs;
  int enable; // high = drivers off; set high first
};

class CarGpio {
public:
  // Throws std::runtime_error with the reason if the GPIO or PWM can't be
  // opened
  CarGpio(const CarPins &pins, rclcpp::Logger logger);
  ~CarGpio();
  CarGpio(const CarGpio &) = delete;
  CarGpio &operator=(const CarGpio &) = delete;

  // Set an output pin (BCM GPIO number) high or low
  void write(int gpio, bool high);

  // Step pulses on both of a side's STEP pins at freq Hz, 0 to stop.
  // forwards says which way the steps count; the DIR pins are set separately.
  // A failed PWM write leaves the step count at what the hardware still
  // does; after MAX_PWM_FAILURES in a row on a side it disables the drivers,
  // stops the pulses and throws std::runtime_error, so the node exits and
  // systemd restarts it
  void set_step_rate(Side side, unsigned freq, bool forwards);

  // Signed steps on a side since the last call
  double take_steps(Side side);

private:
  using Clock = std::chrono::steady_clock;

  struct Channel {
    int period_fd = -1, duty_fd = -1, enable_fd = -1;
    long period = 0; // ns
    long duty = 0;   // ns
    bool enabled = false;
    bool forwards = true;
    double rate = 0.0;  // steps/s now
    double steps = 0.0; // signed, since the last take_steps
    int failures = 0;   // set_step_rate calls that failed in a row
    Clock::time_point last;
  };

  void accumulate(Channel &c);
  // Disable the drivers, stop the pulses and close everything opened
  void release();
  // libgpiod: claim the output pins (ENABLE high, the rest low), set one,
  // release them all
  void open_outputs();
  bool set_output(int gpio, bool high);
  void close_outputs();
  static int open_attr(const std::string &path);
  bool write_attr(int fd, long value);
  static int idx(Side side) { return static_cast<int>(side); }

  CarPins pins_;
  rclcpp::Logger logger_;
  gpiod_chip *chip_ = nullptr;
  gpiod_line_request *request_ = nullptr;
  Channel ch_[2];
  static constexpr int MAX_PWM_FAILURES = 3;
  bool pwm_failed_ = false; // gave up: the node is exiting
};
