"""Grab some lidar scans and print the closest return in each 15 degree
sector of the car's base_link frame (0 = straight ahead, positive =
anticlockwise), plus any returns closer than 0.3 m, to spot the lidar seeing
the car itself. Angles are turned into base_link with the laser_link TF (the
lidar is mounted facing backwards). Start the lidar motor first, if it isn't
running (ros2 service call /start_motor std_srvs/srv/Empty).

    python3 tools/scan_check.py
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

SECTOR_DEG = 15
CLOSE_M = 0.3


def angle_deg(scan, i, yaw):
    """Angle of reading i in base_link, -180 to 180 degrees."""
    a = math.degrees(scan.angle_min + i * scan.angle_increment + yaw)
    return (a + 180) % 360 - 180


def main():
    rclpy.init()
    node = Node('scan_check')
    scans = []
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    start = time.time()
    while time.time() - start < 10 and len(scans) < 20:
        rclpy.spin_once(node, timeout_sec=0.2)

    yaw = 0.0
    try:
        q = tf_buffer.lookup_transform('base_link', 'laser_link', rclpy.time.Time()).transform.rotation
        yaw = 2 * math.atan2(q.z, q.w)
    except Exception as e:  # noqa: BLE001
        print(f'No base_link -> laser_link TF ({e}), using raw lidar angles')
    print(f'laser_link yaw in base_link: {math.degrees(yaw):.0f} deg')

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
                k = int((angle_deg(scan, i, yaw) + 180) // SECTOR_DEG) * SECTOR_DEG - 180
                closest[k] = min(closest.get(k, math.inf), r)
    print('Closest return per sector (deg: m):')
    print(' '.join(f'{k}:{v:.2f}' for k, v in sorted(closest.items())))

    close = [(round(angle_deg(s, i, yaw)), round(r, 3))
             for i, r in enumerate(s.ranges) if 0 < r < CLOSE_M]
    print(f'Returns under {CLOSE_M} m in the last scan (deg, m): {close or "none"}')


if __name__ == '__main__':
    main()
