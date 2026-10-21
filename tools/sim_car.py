"""Simulated car for testing the PC stack without the real car.

Stands in for the Pi nodes: integrates cmd_vel, and publishes /odom (and the
odom -> base_link TF unless publish_odom_tf is false), /imu/data_raw and a
/scan of a 4 x 3 m room with a 0.4 m box in it. Offers the lidar
start_motor/stop_motor services, and prints the true pose once a second
("TRUTH x y yaw_deg") so EKF, SLAM and AMCL can be checked against it.

It is a node called car_controller, like the real one, so it must run on its
own ROS domain: it refuses to start on domain 0. Use tools/sim_test.sh,
which sets that up.
"""
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

WALLS = [((-1, -1), (3, -1)), ((3, -1), (3, 2)), ((3, 2), (-1, 2)), ((-1, 2), (-1, -1)),
         ((1.0, 0.3), (1.4, 0.3)), ((1.4, 0.3), (1.4, 0.7)), ((1.4, 0.7), (1.0, 0.7)), ((1.0, 0.7), (1.0, 0.3))]


def ray(px, py, a):
    dx, dy = math.cos(a), math.sin(a)
    best = float('inf')
    for (x1, y1), (x2, y2) in WALLS:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        t = ((x1 - px) * ey - (y1 - py) * ex) / den
        u = ((x1 - px) * dy - (y1 - py) * dx) / den
        if t > 0 and 0 <= u <= 1:
            best = min(best, t)
    return best


class FakeCar(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.declare_parameter('publish_odom_tf', True)
        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self.last_cmd = self.get_clock().now()
        # Like the real controller: motors off until motor_on, and cmd_vel
        # ignored while they're off, so a missing enable shows up here too
        self.motor_on = False
        self.create_subscription(Twist, 'cmd_vel', self.cmd, 2)
        self.create_subscription(Bool, 'motor_on', self.motor, 1)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.create_service(Empty, 'start_motor', lambda r, s: s)
        self.create_service(Empty, 'stop_motor', lambda r, s: s)
        self.create_timer(0.05, self.odom)
        self.create_timer(0.01, self.imu)
        self.create_timer(1 / 7.0, self.scan)
        self.create_timer(1.0, lambda: print(f'TRUTH {self.x:.3f} {self.y:.3f} {math.degrees(self.th):.1f}', flush=True))

    def motor(self, m):
        self.motor_on = m.data
        if not m.data:
            self.v = self.w = 0.0
        print(f'motor_on {m.data}', flush=True)

    def cmd(self, m):
        if not self.motor_on:
            return
        self.v, self.w = m.linear.x, m.angular.z
        self.last_cmd = self.get_clock().now()

    def odom(self):
        if (self.get_clock().now() - self.last_cmd).nanoseconds > 0.3e9:
            self.v = self.w = 0.0
        dt = 0.05
        self.th += self.w * dt
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        now = self.get_clock().now().to_msg()
        q = (0.0, 0.0, math.sin(self.th / 2), math.cos(self.th / 2))
        if self.get_parameter('publish_odom_tf').value:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
            self.tf.sendTransform(t)
        o = Odometry()
        o.header.stamp = now
        o.header.frame_id, o.child_frame_id = 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = q[2], q[3]
        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = self.w
        for i, v in zip((0, 7, 14, 21, 28, 35), (0.01, 0.01, 1e6, 1e6, 1e6, 0.1)):
            o.pose.covariance[i] = v
        for i, v in zip((0, 7, 14, 21, 28, 35), (0.01, 0.01, 1e6, 1e6, 1e6, 0.5)):
            o.twist.covariance[i] = v
        self.odom_pub.publish(o)

    def imu(self):
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'imu_link'
        m.orientation_covariance[0] = -1.0
        m.angular_velocity.z = self.w
        m.linear_acceleration.z = 9.81
        for i in (0, 4, 8):
            m.angular_velocity_covariance[i] = 1e-4
            m.linear_acceleration_covariance[i] = 0.05
        self.imu_pub.publish(m)

    def scan(self):
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'laser_link'
        s.angle_min, s.angle_max = -math.pi, math.pi - 2 * math.pi / 360
        s.angle_increment = 2 * math.pi / 360
        s.range_min, s.range_max = 0.15, 12.0
        s.scan_time = 1 / 7.0
        # laser_link is 0.05 m ahead of base_link, facing backwards like the
        # real lidar (laser_joint in the URDF)
        lx = self.x + 0.05 * math.cos(self.th)
        ly = self.y + 0.05 * math.sin(self.th)
        s.ranges = [ray(lx, ly, self.th + math.pi + s.angle_min + i * s.angle_increment) for i in range(360)]
        self.scan_pub.publish(s)


try:
    domain = int(os.environ.get('ROS_DOMAIN_ID') or 0)
except ValueError:
    sys.exit(f'Unreadable ROS_DOMAIN_ID {os.environ.get("ROS_DOMAIN_ID")!r}')
if domain == 0:
    sys.exit('Refusing to run on ROS domain 0, where the real car is. '
             'Use tools/sim_test.sh, or set ROS_DOMAIN_ID and '
             'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST.')
rclpy.init()
rclpy.spin(FakeCar())
