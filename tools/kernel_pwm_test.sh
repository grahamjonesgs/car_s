#!/bin/bash
# Kernel PWM bench test: drives the four step pins through the kernel
# pwm-bcm2835 driver directly (no controller) at 3-10 kHz and counts the
# pulses with pulse_count.py. Stops car_s for the test and restarts it after.
# Motor battery off. Run on the Pi: sudo tools/kernel_pwm_test.sh
D=$(dirname "$(readlink -f "$0")")
P=/sys/class/pwm/pwmchip0
trap 'systemctl start car_s && echo "car_s started"' EXIT
systemctl stop car_s; sleep 1
"$D/car_pwm_setup.sh" || exit 1
for f in 3175 5500 6000 6349 9524; do
  per=$(( 1000000000 / f ))
  for c in 0 1; do
    echo 0 > $P/pwm$c/enable
    # The duty cycle can't exceed the period, and can't be set before one is
    [ "$(cat $P/pwm$c/period)" -gt 0 ] && echo 0 > $P/pwm$c/duty_cycle
    echo $per > $P/pwm$c/period; echo $(( per / 2 )) > $P/pwm$c/duty_cycle; echo 1 > $P/pwm$c/enable
  done
  sleep 0.5
  echo "== kernel PWM at $f Hz (period $per ns):"
  python3 "$D/pulse_count.py" 1
done
for c in 0 1; do echo 0 > $P/pwm$c/enable; done
