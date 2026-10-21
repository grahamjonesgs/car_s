// pigpio backend for CarGpio: the controller's original way of driving the
// hardware, through the pigpiod daemon
#include "car_gpio.hpp"

#include <atomic>
#include <stdexcept>
#include <thread>

#include <pigpiod_if2.h>

namespace {

class PigpioGpio : public CarGpio {
public:
  PigpioGpio(const CarPins &pins, rclcpp::Logger logger) : pins_(pins) {
    // pigpiod can still be starting at boot, so retry for a while
    for (int attempt = 0; (pi_ = pigpio_start(nullptr, nullptr)) < 0; ++attempt) {
      if (attempt >= 10) {
        throw std::runtime_error("gpio init failed: is pigpiod running?");
      }
      RCLCPP_WARN(logger, "gpio init failed, retrying");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    // pigpiod keeps PWM running after a client dies, so clear anything a
    // crashed earlier run left behind before doing anything else
    gpio_write(pi_, pins_.enable, PI_HIGH);
    for (Side s : {Side::LEFT, Side::RIGHT}) {
      for (int pin : step_pins(s)) {
        hardware_PWM(pi_, pin, 0, 0);
        set_mode(pi_, pin, PI_OUTPUT);
      }
    }
    for (int i = 0; i < pins_.n_outputs; ++i) {
      set_mode(pi_, pins_.outputs[i], PI_OUTPUT);
    }
    gpio_write(pi_, pins_.enable, PI_HIGH);
    // Count the steps on one STEP pin per side (both pins of a side carry
    // the same PWM channel)
    callback_ex(pi_, pins_.left_step[0], RISING_EDGE, &PigpioGpio::on_edge, &count_[0]);
    callback_ex(pi_, pins_.right_step[0], RISING_EDGE, &PigpioGpio::on_edge, &count_[1]);
  }

  ~PigpioGpio() override {
    gpio_write(pi_, pins_.enable, PI_HIGH);
    for (Side s : {Side::LEFT, Side::RIGHT}) {
      for (int pin : step_pins(s)) {
        hardware_PWM(pi_, pin, 0, 0);
      }
    }
    pigpio_stop(pi_);
  }

  void write(int gpio, bool high) override {
    gpio_write(pi_, gpio, high ? PI_HIGH : PI_LOW);
  }

  void set_step_rate(Side side, unsigned freq, bool forwards) override {
    count_[idx(side)].forwards = forwards;
    for (int pin : step_pins(side)) {
      hardware_PWM(pi_, pin, freq, freq ? 500000 : 0); // 0.5 duty cycle
    }
  }

  double take_steps(Side side) override {
    return static_cast<double>(count_[idx(side)].steps.exchange(0));
  }

  std::string name() const override { return "pigpio"; }

private:
  // Shared with the pigpio callback thread
  struct Counter {
    std::atomic<int> steps{0};
    std::atomic<bool> forwards{true};
  };

  static void on_edge(int, unsigned, unsigned, uint32_t, void *userdata) {
    auto *c = static_cast<Counter *>(userdata);
    c->forwards ? c->steps++ : c->steps--;
  }

  static int idx(Side side) { return static_cast<int>(side); }
  // A side's two STEP pins, to loop over
  struct Pins {
    const int *p;
    const int *begin() const { return p; }
    const int *end() const { return p + 2; }
  };
  Pins step_pins(Side side) const {
    return {side == Side::LEFT ? pins_.left_step : pins_.right_step};
  }

  CarPins pins_;
  int pi_ = -1;
  Counter count_[2];
};

} // namespace

std::unique_ptr<CarGpio> make_pigpio_gpio(const CarPins &pins, rclcpp::Logger logger) {
  return std::make_unique<PigpioGpio>(pins, logger);
}
