"""Grab some lidar scans and print the closest return in each 15 degree
sector (0 = straight ahead, positive = anticlockwise), plus any returns
closer than 0.3 m, to spot the lidar seeing the car itself. Start the lidar
motor first (ros2 service call /start_motor std_srvs/srv/Empty).

    python3 tools/scan_check.py
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SECTOR_DEG = 15
CLOSE_M = 0.3


def angle_deg(scan, i):
    return math.degrees(scan.angle_min + i * scan.angle_increment)


def main():
    rclpy.init()
    node = Node('scan_check')
    scans = []
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    start = time.time()
    while time.time() - start < 10 and len(scans) < 20:
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f'{len(scans)} scans')
    if not scans:
        print('No scans: is the lidar motor running?')
        return
    s = scans[-1]
    print(f'frame {s.header.frame_id}, {len(s.ranges)} points, range_min {s.range_min:.2f} m')

    closest = {}
    for scan in scans:
        for i, r in enumerate(scan.ranges):
            if math.isfinite(r) and r > 0:
                k = int(((angle_deg(scan, i) + 180) % 360) // SECTOR_DEG) * SECTOR_DEG - 180
                closest[k] = min(closest.get(k, math.inf), r)
    print('Closest return per sector (deg: m):')
    print(' '.join(f'{k}:{v:.2f}' for k, v in sorted(closest.items())))

    close = [(round(angle_deg(s, i)), round(r, 3))
             for i, r in enumerate(s.ranges) if 0 < r < CLOSE_M]
    print(f'Returns under {CLOSE_M} m in the last scan (deg, m): {close or "none"}')


if __name__ == '__main__':
    main()
