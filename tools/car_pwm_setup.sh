#!/bin/bash
# Set up the kernel PWM for the controller's linux GPIO backend, or take it
# down for the pigpio backend. Run as root; the car_s service runs it before
# starting (ExecStartPre=+), with CAR_S_GPIO_BACKEND from /etc/default/car_s.
#
#   sudo tools/car_pwm_setup.sh linux    # load overlays/car-pwm.dts, export both
#                                        # channels, give them to the car's user
#   sudo tools/car_pwm_setup.sh pigpio   # remove the overlay (the default)
#
# The two must never both drive the PWM, so pigpio gets it back only with
# the overlay removed.
set -e
BACKEND=${1:-${CAR_S_GPIO_BACKEND:-pigpio}}
CAR_USER=${CAR_S_USER:-graham}
D=$(dirname "$(readlink -f "$0")")
P=/sys/class/pwm/pwmchip0

loaded() { dtoverlay -l 2>/dev/null | grep -q car-pwm; }
take_down() {
  for c in 0 1; do
    if [ -d $P/pwm$c ]; then echo 0 > $P/pwm$c/enable 2>/dev/null || true; echo $c > $P/unexport; fi
  done
  dtoverlay -r car-pwm
}

if [ "$BACKEND" != linux ]; then
  if loaded; then
    take_down
    # Removing the kernel driver stops the PWM clock, which pigpiod only sets
    # up when it starts
    echo "car-pwm: overlay removed. Restart pigpiod before pigpio's PWM works again:"
    echo "  sudo systemctl restart pigpiod car_s"
  fi
  exit 0
fi

# Load it fresh every time: that also sets the pin functions again, in case
# something (pigs, pigpio) changed them since
if loaded; then take_down; fi
dtc -q -@ -I dts -O dtb -o /run/car-pwm.dtbo "$D/../overlays/car-pwm.dts"
dtoverlay -d /run car-pwm
for i in $(seq 50); do [ -d $P ] && break; sleep 0.1; done
[ -d $P ] || { echo "car-pwm: $P didn't appear"; exit 1; }
for c in 0 1; do [ -d $P/pwm$c ] || echo $c > $P/export; done
for i in $(seq 50); do [ -e $P/pwm1/enable ] && break; sleep 0.1; done
for c in 0 1; do
  # A newly exported channel is off (and refuses enable writes until it has
  # a period)
  for a in period duty_cycle enable; do chown "$CAR_USER" $P/pwm$c/$a; done
done
echo "car-pwm: kernel PWM ready, pwm0 on GPIO 12 + 18 (right), pwm1 on GPIO 13 + 19 (left), for $CAR_USER"
