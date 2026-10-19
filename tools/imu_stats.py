"""Record /imu/data_raw for a while and print the real publish rate (from
the message timestamps, so WiFi drops don't hide a slow Pi) and the mean and
variance of each axis. Keep the car still while it runs.

    python3 tools/imu_stats.py [seconds]
"""
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

AXES = ['gyro x', 'gyro y', 'gyro z', 'accel x', 'accel y', 'accel z']


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    node = Node('imu_stats')
    rows, stamps = [], []

    def callback(m):
        rows.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                     m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
        stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

    node.create_subscription(Imu, '/imu/data_raw', callback, qos_profile_sensor_data)
    start = time.time()
    while time.time() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.1)

    if len(stamps) < 2:
        print('No IMU messages received')
        return
    span = stamps[-1] - stamps[0]
    print(f'{len(rows)} messages over {span:.1f} s of timestamps: '
          f'{(len(stamps) - 1) / span:.1f} Hz published')
    for i, name in enumerate(AXES):
        col = [r[i] for r in rows]
        print(f'{name:8s} mean {statistics.mean(col):+.4f}  variance {statistics.pvariance(col):.2e}')


if __name__ == '__main__':
    main()
