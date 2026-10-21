"""Count rising edges on the four step pins for 2 s with pigpio (which only
reads the pins), and print pulses per second. Used by kernel_pwm_test.sh."""
import time, pigpio
pi = pigpio.pi()
cbs = {g: pi.callback(g, pigpio.RISING_EDGE) for g in (12, 13, 18, 19)}
time.sleep(2.0)
names = {13: 'left front', 19: 'left back', 18: 'right front', 12: 'right back'}
for g, cb in cbs.items():
    print(f'GPIO {g:2d} ({names[g]:11s}): {cb.tally()/2.0:8.0f} pulses/s, mode {pi.get_mode(g)}')
    cb.cancel()
pi.stop()
