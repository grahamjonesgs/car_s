#!/bin/bash
# Set up the kernel PWM for the car controller: load overlays/car-pwm.dts,
# export both PWM channels and give their files to the car's user. Run as
# root; the car_s service runs it before starting (ExecStartPre=+).
#
#   sudo tools/car_pwm_setup.sh          # set up (again)
#   sudo tools/car_pwm_setup.sh remove   # take it down
set -e
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

if [ "$1" = remove ]; then
  if loaded; then take_down; echo "car-pwm: overlay removed"; fi
  exit 0
fi

# Load it fresh every time, which also sets the pin functions again
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
