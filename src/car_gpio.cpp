// CarGpio: the car's GPIO and step pulses through the kernel (see
// car_gpio.hpp)
#include "car_gpio.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <unistd.h>

namespace {
const char *PWM_DIR = "/sys/class/pwm/pwmchip0";
const char *CONSUMER = "car_s";
} // namespace

CarGpio::CarGpio(const CarPins &pins, rclcpp::Logger logger)
    : pins_(pins), logger_(logger) {
  // The destructor doesn't run if this throws, so let go of what was opened
  try {
    open_outputs();
    for (Side s : {Side::LEFT, Side::RIGHT}) {
      Channel &c = ch_[idx(s)];
      // Channel 0 drives the right wheels, channel 1 the left
      std::string dir = std::string(PWM_DIR) + "/pwm" + (s == Side::RIGHT ? "0" : "1");
      c.period_fd = open_attr(dir + "/period");
      c.duty_fd = open_attr(dir + "/duty_cycle");
      c.enable_fd = open_attr(dir + "/enable");
      // Off, in case an earlier run left it on. A freshly exported channel is
      // already off, and refuses this until it has a period, so ignore the
      // result
      std::string off = "0";
      (void)!pwrite(c.enable_fd, off.c_str(), off.size(), 0);
      // Duty 0 fits any period, so it matches the cached state and the
      // first period write can't be refused for an old, longer duty
      (void)!pwrite(c.duty_fd, off.c_str(), off.size(), 0);
      c.last = Clock::now();
    }
  } catch (...) {
    release();
    throw;
  }
}

CarGpio::~CarGpio() { release(); }

void CarGpio::release() {
  if (request_) {
    write(pins_.enable, true);
  }
  for (Channel &c : ch_) {
    if (c.enable_fd >= 0) {
      // A channel that never had a period refuses this, harmlessly
      std::string off = "0";
      (void)!pwrite(c.enable_fd, off.c_str(), off.size(), 0);
    }
    for (int *fd : {&c.period_fd, &c.duty_fd, &c.enable_fd}) {
      if (*fd >= 0) {
        close(*fd);
        *fd = -1;
      }
    }
  }
  close_outputs();
}

void CarGpio::write(int gpio, bool high) {
  if (!set_output(gpio, high)) {
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000,
                          "GPIO %d isn't set up as an output", gpio);
  }
}

void CarGpio::set_step_rate(Side side, unsigned freq, bool forwards) {
  Channel &c = ch_[idx(side)];
  accumulate(c);
  c.forwards = forwards;
  // The cached state only changes when the hardware took the write, so the
  // steps counted are the ones it really makes
  bool ok = true;
  if (freq == 0) {
    if (c.enabled) {
      ok = write_attr(c.enable_fd, 0);
      c.enabled = !ok;
    }
  } else {
    long period = std::lround(1e9 / freq);
    long duty = period / 2;
    // The duty cycle must never exceed the period, so the order depends on
    // whether the period grows or shrinks. A write that failed is tried
    // again on the next call, as the cache still differs
    if (period > c.period) {
      ok = write_attr(c.period_fd, period);
      if (ok) {
        c.period = period;
      }
    }
    if (ok && duty != c.duty) {
      ok = write_attr(c.duty_fd, duty);
      if (ok) {
        c.duty = duty;
      }
    }
    if (ok && period != c.period) {
      ok = write_attr(c.period_fd, period);
      if (ok) {
        c.period = period;
      }
    }
    if (ok && !c.enabled) {
      ok = write_attr(c.enable_fd, 1);
      c.enabled = ok;
    }
  }
  c.rate = c.enabled ? 1e9 / c.period : 0.0; // what the hardware really produces

  if (ok) {
    c.failures = 0;
  } else if (++c.failures >= MAX_PWM_FAILURES && !pwm_failed_) {
    // Only once, so stopping on the way out doesn't throw again
    pwm_failed_ = true;
    write(pins_.enable, true);
    for (Channel &ch : ch_) {
      std::string off = "0";
      (void)!pwrite(ch.enable_fd, off.c_str(), off.size(), 0);
    }
    throw std::runtime_error("PWM writes keep failing, drivers disabled");
  }
}

double CarGpio::take_steps(Side side) {
  Channel &c = ch_[idx(side)];
  accumulate(c);
  double steps = c.steps;
  c.steps = 0.0;
  return steps;
}

// Add the steps made at the current rate since the last change
void CarGpio::accumulate(Channel &c) {
  auto now = Clock::now();
  double dt = std::chrono::duration<double>(now - c.last).count();
  c.last = now;
  c.steps += (c.forwards ? 1.0 : -1.0) * c.rate * dt;
}

// libgpiod: one request for all the output pins

void CarGpio::open_outputs() {
  chip_ = gpiod_chip_open("/dev/gpiochip0");
  if (!chip_) {
    throw std::runtime_error("can't open /dev/gpiochip0: is the user in dialout?");
  }
  gpiod_line_settings *high = gpiod_line_settings_new();
  gpiod_line_settings *low = gpiod_line_settings_new();
  gpiod_line_config *lines = gpiod_line_config_new();
  gpiod_request_config *req = gpiod_request_config_new();
  bool ok = high && low && lines && req;
  if (ok) {
    gpiod_line_settings_set_direction(high, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(high, GPIOD_LINE_VALUE_ACTIVE);
    gpiod_line_settings_set_direction(low, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(low, GPIOD_LINE_VALUE_INACTIVE);
    // ENABLE high, so the drivers stay off; everything else low
    for (int i = 0; i < pins_.n_outputs && ok; ++i) {
      unsigned int offset = pins_.outputs[i];
      ok = gpiod_line_config_add_line_settings(
               lines, &offset, 1, pins_.outputs[i] == pins_.enable ? high : low) == 0;
    }
    gpiod_request_config_set_consumer(req, CONSUMER);
    if (ok) {
      request_ = gpiod_chip_request_lines(chip_, req, lines);
    }
  }
  gpiod_request_config_free(req);
  gpiod_line_config_free(lines);
  gpiod_line_settings_free(low);
  gpiod_line_settings_free(high);
  if (!request_) {
    throw std::runtime_error("can't use the GPIO pins as outputs "
                             "(is another program using them? gpioinfo shows)");
  }
}

bool CarGpio::set_output(int gpio, bool high) {
  for (int i = 0; i < pins_.n_outputs; ++i) {
    if (pins_.outputs[i] == gpio) {
      return gpiod_line_request_set_value(
                 request_, gpio, high ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE) == 0;
    }
  }
  return false;
}

void CarGpio::close_outputs() {
  if (request_) {
    gpiod_line_request_release(request_);
    request_ = nullptr;
  }
  if (chip_) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
  }
}


int CarGpio::open_attr(const std::string &path) {
  int fd = open(path.c_str(), O_WRONLY);
  if (fd < 0) {
    throw std::runtime_error("can't open " + path +
                             ": run tools/car_pwm_setup.sh (as root) first");
  }
  return fd;
}

bool CarGpio::write_attr(int fd, long value) {
  std::string s = std::to_string(value);
  if (pwrite(fd, s.c_str(), s.size(), 0) < 0) {
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000,
                          "PWM write of %ld failed: %s", value, strerror(errno));
    return false;
  }
  return true;
}
