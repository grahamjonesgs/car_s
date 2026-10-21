#!/bin/bash
# Stop the step pulses and disable the stepper drivers, however the car_s
# service ended (the controller can't do it after a crash: the PWM, kernel or
# pigpio, keeps running without it). Run straight away by robot_car.launch.py
# when the controller exits, and again as root by the service's ExecStopPost
# (with CAR_S_GPIO_BACKEND from /etc/default/car_s). It works as the car's
# user too: car_pwm_setup.sh gives it the kernel PWM files.
#
# For the linux backend it doesn't use pigpio's PWM commands: those
# reprogram the PWM clock behind the kernel's back, and the kernel then sets
# every later step rate wrong.
BACKEND=${1:-${CAR_S_GPIO_BACKEND:-pigpio}}
P=/sys/class/pwm/pwmchip0
for c in 0 1; do
  [ -e $P/pwm$c/enable ] && echo 0 2>/dev/null > $P/pwm$c/enable
done
if [ "$BACKEND" = linux ]; then
  # ENABLE (GPIO 14) high = drivers off. A plain pin write, not PWM
  /usr/local/bin/pigs w 14 1 2>/dev/null || gpioset gpiochip0 14=1
else
  /usr/local/bin/pigs w 14 1 hp 12 0 0 hp 13 0 0 hp 18 0 0 hp 19 0 0
fi
exit 0
