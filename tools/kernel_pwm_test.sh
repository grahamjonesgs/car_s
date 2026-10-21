#!/bin/bash
# Kernel PWM bench test: drives the four step pins through the kernel
# pwm-bcm2835 driver (overlays/car-pwm.dts, loaded at run time) and counts
# the pulses with pigpio. Stops car_s for the test and restarts it after.
# Motor battery off. Run on the Pi: sudo tools/kernel_pwm_test.sh
D=$(dirname "$(readlink -f "$0")")
P=/sys/class/pwm/pwmchip0
restore() {
  for c in 0 1; do [ -d $P/pwm$c ] && { echo 0 > $P/pwm$c/enable; echo $c > $P/unexport; }; done 2>/dev/null
  dtoverlay -r car-pwm 2>/dev/null && echo "overlay removed"
  systemctl start car_s && echo "car_s started"
}
trap restore EXIT
systemctl stop car_s; sleep 1
echo "ENABLE pin after stopping car_s: $(pigs r 14) (1 = drivers off)"
dtc -q -@ -I dts -O dtb -o /tmp/car-pwm.dtbo $D/../overlays/car-pwm.dts || exit 1
dtoverlay -d /tmp car-pwm || exit 1
sleep 1
[ -d $P ] || { echo "no pwmchip0: the kernel PWM didn't start"; dmesg | tail -5; exit 1; }
echo "pwmchip0 has $(cat $P/npwm) channels; pin modes 12:$(pigs mg 12) 13:$(pigs mg 13) 18:$(pigs mg 18) 19:$(pigs mg 19) (4 = Alt0, 2 = Alt5)"
for c in 0 1; do echo $c > $P/export; done; sleep 0.5
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
  sudo -u "${SUDO_USER:-graham}" python3 $D/pulse_count.py | sed "s/, mode.*//"
done
