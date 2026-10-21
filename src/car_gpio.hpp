// GPIO and step-pulse access for the car controller, with two backends:
//
// - pigpio: through the pigpiod daemon (hardware_PWM, gpio_write), counting
//   the step pulses with pigpio callbacks. Its PWM channel 0 (the right
//   wheels) fails above about 6 kHz on this car.
// - linux: the kernel's own drivers. Step pulses from the pwm-bcm2835 driver
//   through /sys/class/pwm (overlays/car-pwm.dts, set up by
//   tools/car_pwm_setup.sh), other pins through libgpiod. The kernel can't
//   count 10,000 edges a second for a program, so the steps are worked out
//   from the rate actually set, which hardware PWM produces exactly.
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

enum class Side { LEFT = 0, RIGHT = 1 };

class CarGpio {
public:
  virtual ~CarGpio() = default;

  // Set an output pin (BCM GPIO number) high or low
  virtual void write(int gpio, bool high) = 0;

  // Step pulses on both of a side's STEP pins at freq Hz, 0 to stop.
  // forwards says which way the steps count; the DIR pins are set separately
  virtual void set_step_rate(Side side, unsigned freq, bool forwards) = 0;

  // Signed steps on a side since the last call
  virtual double take_steps(Side side) = 0;

  virtual std::string name() const = 0;
};

// The pins the backends need as outputs, and the STEP pins of each side
struct CarPins {
  int left_step[2];
  int right_step[2];
  int outputs[16];
  int n_outputs;
  int enable; // high = drivers off; set high first
};

// Throws std::runtime_error with the reason if the backend can't start
std::unique_ptr<CarGpio> make_car_gpio(const std::string &backend,
                                       const CarPins &pins,
                                       rclcpp::Logger logger);
