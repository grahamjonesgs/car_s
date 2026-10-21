// linux backend for CarGpio: the kernel's own drivers, no pigpiod.
//
// Step pulses come from the pwm-bcm2835 driver through /sys/class/pwm.
// overlays/car-pwm.dts puts PWM channel 0 on GPIO 12 and 18 (the right
// wheels) and channel 1 on GPIO 13 and 19 (the left), and
// tools/car_pwm_setup.sh (run as root by the car_s service) loads it,
// exports both channels and hands their files to the car's user. The other
// pins are outputs through libgpiod (/dev/gpiochip0).
//
// The kernel can't report 10,000 edges a second to a program, so the steps
// are worked out from the rate set: hardware PWM produces it exactly, so
// steps = rate x time, summed over every change.
#include "car_gpio.hpp"

#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <stdexcept>
#include <unistd.h>
#include <vector>

#include <gpiod.h>

namespace {

const char *PWM_DIR = "/sys/class/pwm/pwmchip0";
const char *CONSUMER = "car_s";

class LinuxGpio : public CarGpio {
public:
  LinuxGpio(const CarPins &pins, rclcpp::Logger logger) : pins_(pins), logger_(logger) {
    chip_ = gpiod_chip_open_by_name("gpiochip0");
    if (!chip_) {
      throw std::runtime_error("can't open /dev/gpiochip0: is the user in dialout?");
    }
    // ENABLE first and high, so the drivers stay off while the rest is set up
    request_output(pins_.enable, 1);
    for (int i = 0; i < pins_.n_outputs; ++i) {
      if (pins_.outputs[i] != pins_.enable) {
        request_output(pins_.outputs[i], 0);
      }
    }
    for (Side s : {Side::LEFT, Side::RIGHT}) {
      Channel &c = ch_[idx(s)];
      // Channel 0 drives the right wheels, channel 1 the left
      std::string dir = std::string(PWM_DIR) + "/pwm" + (s == Side::RIGHT ? "0" : "1");
      c.period_fd = open_attr(dir + "/period");
      c.duty_fd = open_attr(dir + "/duty_cycle");
      c.enable_fd = open_attr(dir + "/enable");
      write_attr(c.enable_fd, 0);
      c.last = Clock::now();
    }
  }

  ~LinuxGpio() override {
    write(pins_.enable, true);
    for (Channel &c : ch_) {
      if (c.enable_fd >= 0) {
        write_attr(c.enable_fd, 0);
      }
      for (int fd : {c.period_fd, c.duty_fd, c.enable_fd}) {
        if (fd >= 0) {
          close(fd);
        }
      }
    }
    for (auto &l : lines_) {
      gpiod_line_release(l.second);
    }
    if (chip_) {
      gpiod_chip_close(chip_);
    }
  }

  void write(int gpio, bool high) override {
    for (auto &l : lines_) {
      if (l.first == gpio) {
        gpiod_line_set_value(l.second, high ? 1 : 0);
        return;
      }
    }
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000,
                          "GPIO %d isn't set up as an output", gpio);
  }

  void set_step_rate(Side side, unsigned freq, bool forwards) override {
    Channel &c = ch_[idx(side)];
    accumulate(c);
    c.forwards = forwards;
    if (freq == 0) {
      if (c.enabled) {
        write_attr(c.enable_fd, 0);
        c.enabled = false;
      }
      c.rate = 0.0;
      return;
    }
    long period = std::lround(1e9 / freq);
    if (period != c.period) {
      // The duty cycle must never exceed the period, so the order depends
      // on whether the period grows or shrinks
      long duty = period / 2;
      if (period > c.period) {
        write_attr(c.period_fd, period);
        write_attr(c.duty_fd, duty);
      } else {
        write_attr(c.duty_fd, duty);
        write_attr(c.period_fd, period);
      }
      c.period = period;
    }
    if (!c.enabled) {
      write_attr(c.enable_fd, 1);
      c.enabled = true;
    }
    c.rate = 1e9 / c.period; // what the hardware really produces
  }

  double take_steps(Side side) override {
    Channel &c = ch_[idx(side)];
    accumulate(c);
    double steps = c.steps;
    c.steps = 0.0;
    return steps;
  }

  std::string name() const override { return "linux"; }

private:
  using Clock = std::chrono::steady_clock;

  struct Channel {
    int period_fd = -1, duty_fd = -1, enable_fd = -1;
    long period = 0; // ns
    bool enabled = false;
    bool forwards = true;
    double rate = 0.0;  // steps/s now
    double steps = 0.0; // signed, since the last take_steps
    Clock::time_point last;
  };

  // Add the steps made at the current rate since the last change
  void accumulate(Channel &c) {
    auto now = Clock::now();
    double dt = std::chrono::duration<double>(now - c.last).count();
    c.last = now;
    c.steps += (c.forwards ? 1.0 : -1.0) * c.rate * dt;
  }

  void request_output(int gpio, int initial) {
    gpiod_line *line = gpiod_chip_get_line(chip_, gpio);
    if (!line || gpiod_line_request_output(line, CONSUMER, initial) < 0) {
      throw std::runtime_error("can't use GPIO " + std::to_string(gpio) +
                               " as an output (is another program using it?)");
    }
    lines_.push_back({gpio, line});
  }

  static int open_attr(const std::string &path) {
    int fd = open(path.c_str(), O_WRONLY);
    if (fd < 0) {
      throw std::runtime_error("can't open " + path +
                               ": run tools/car_pwm_setup.sh (as root) first");
    }
    return fd;
  }

  void write_attr(int fd, long value) {
    std::string s = std::to_string(value);
    if (pwrite(fd, s.c_str(), s.size(), 0) < 0) {
      RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000,
                            "PWM write of %ld failed", value);
    }
  }

  static int idx(Side side) { return static_cast<int>(side); }

  CarPins pins_;
  rclcpp::Logger logger_;
  gpiod_chip *chip_ = nullptr;
  std::vector<std::pair<int, gpiod_line *>> lines_;
  Channel ch_[2];
};

} // namespace

std::unique_ptr<CarGpio> make_linux_gpio(const CarPins &pins, rclcpp::Logger logger) {
  return std::make_unique<LinuxGpio>(pins, logger);
}
