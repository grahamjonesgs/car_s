"""Count the step pulses on the four STEP pins, for checking what the motors
are really sent (odometry only knows the rate that was set).

    sudo python3 tools/pulse_count.py          # 2 s
    sudo python3 tools/pulse_count.py 1.0      # 1 s

Reads the GPIO level register directly through /dev/gpiomem (no daemon) as
fast as Python can, and counts rising edges. That's about a million samples a
second on a Pi 4, enough for step rates to about 50 kHz; it prints its sample
rate so you can tell. Only reads, so it's safe alongside the controller.
"""
import mmap
import os
import struct
import sys
import time

PINS = {13: 'left front', 19: 'left back', 18: 'right front', 12: 'right back'}
GPLEV0 = 0x34  # GPIO level register, pins 0-31


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    try:
        fd = os.open('/dev/gpiomem', os.O_RDONLY | os.O_SYNC)
    except OSError as e:
        sys.exit(f"Can't open /dev/gpiomem ({e}): run it with sudo (on Ubuntu "
                 "only root can read /dev/gpiomem)")
    mem = mmap.mmap(fd, 4096, mmap.MAP_SHARED, mmap.PROT_READ)
    unpack = struct.Struct('<I').unpack_from
    mask = sum(1 << p for p in PINS)
    edges = {p: 0 for p in PINS}
    last = unpack(mem, GPLEV0)[0] & mask
    samples = 0
    end = time.perf_counter() + duration
    while True:
        # A batch between clock checks keeps the loop fast
        for _ in range(1000):
            level = unpack(mem, GPLEV0)[0] & mask
            rising = level & ~last
            if rising:
                for p in PINS:
                    if rising >> p & 1:
                        edges[p] += 1
            last = level
        samples += 1000
        if time.perf_counter() >= end:
            break
    mem.close()
    os.close(fd)
    print(f'{samples / duration / 1e6:.2f} M samples/s')
    for p, name in sorted(PINS.items()):
        print(f'GPIO {p:2d} ({name:11s}): {edges[p] / duration:8.0f} pulses/s')


if __name__ == '__main__':
    main()
