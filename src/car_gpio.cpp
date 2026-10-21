// Picks the CarGpio backend; each is only built where its library is
// installed (see CMakeLists.txt)
#include "car_gpio.hpp"

#include <stdexcept>

#ifdef HAVE_PIGPIO
std::unique_ptr<CarGpio> make_pigpio_gpio(const CarPins &pins, rclcpp::Logger logger);
#endif
#ifdef HAVE_GPIOD
std::unique_ptr<CarGpio> make_linux_gpio(const CarPins &pins, rclcpp::Logger logger);
#endif

std::unique_ptr<CarGpio> make_car_gpio(const std::string &backend,
                                       const CarPins &pins,
                                       rclcpp::Logger logger) {
#ifdef HAVE_PIGPIO
  if (backend == "pigpio") {
    return make_pigpio_gpio(pins, logger);
  }
#endif
#ifdef HAVE_GPIOD
  if (backend == "linux") {
    return make_linux_gpio(pins, logger);
  }
#endif
  throw std::runtime_error("gpio_backend \"" + backend +
                           "\" isn't available in this build (pigpio or linux)");
}
