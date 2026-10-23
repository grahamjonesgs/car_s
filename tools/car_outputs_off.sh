#!/bin/bash
# Stop the step pulses and disable the stepper drivers. The kernel PWM keeps
# running after the controller dies, so robot_car.launch.py runs this
# straight away when the controller exits, and the car_s service runs it
# again as root (ExecStopPost) however the service ended. It works as the
# car's user too: car_pwm_setup.sh gives that user the PWM files.
P=/sys/class/pwm/pwmchip0
for c in 0 1; do
  [ -e $P/pwm$c/enable ] && echo 0 2>/dev/null > $P/pwm$c/enable
done
# ENABLE (GPIO 14) high = drivers off: drive it high, then leave it as an
# input. The driver board's pull-up keeps it high from then on, whatever
# the pin was last set to
# (gpioset holds the pin until it exits; -t0 makes it exit at once, and
# gpioget makes the pin an input)
gpioset -c gpiochip0 -t0 14=1 2>/dev/null
gpioget -c gpiochip0 14 >/dev/null 2>&1
exit 0
