"""Simulated car for testing the PC stack without the real car.

Stands in for the Pi nodes: integrates cmd_vel, and publishes /odom (and the
odom -> base_link TF unless publish_odom_tf is false), /imu/data_raw and a
/scan of a 4 x 3 m room with a 0.4 m box in it. Offers the lidar
start_motor/stop_motor services, and prints the true pose once a second
("TRUTH x y yaw_deg") so EKF, SLAM and AMCL can be checked against it.

Like the real car, /odom counts what was commanded, and the parameters fake
what the real odometry can't see (for testing motion_check):

  ros2 param set /car_controller slip 1.0      # stuck: wheels turn, car doesn't
  ros2 param set /car_controller slip 0.5      # half the commanded movement
  ros2 param set /car_controller push_vx 0.1   # pushed forwards at 0.1 m/s
  ros2 param set /car_controller hidden_leg "0.8,-0.2"  # a chair leg at
                                  # (0.8, -0.2) the scans don't show, which
                                  # blocks the car ("" removes it)

The gyro and the scans follow the true movement.

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
        self.declare_parameter('slip', 0.0)
        self.declare_parameter('push_vx', 0.0)
        self.declare_parameter('hidden_leg', '')
        self.x = self.y = self.th = 0.0        # true pose
        self.ox = self.oy = self.oth = 0.0     # odometry: what was commanded
        self.v = self.w = 0.0
        self.true_w = 0.0                      # true turn rate, for the gyro
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
        self.oth += self.w * dt
        self.ox += self.v * math.cos(self.oth) * dt
        self.oy += self.v * math.sin(self.oth) * dt
        keep = 1.0 - min(max(self.get_parameter('slip').value, 0.0), 1.0)
        v = self.v * keep + self.get_parameter('push_vx').value
        th = self.th + self.w * keep * dt
        x = self.x + v * math.cos(th) * dt
        y = self.y + v * math.sin(th) * dt
        if self.hits_leg(x, y, th):
            self.true_w = 0.0
        else:
            self.true_w = (th - self.th) / dt
            self.x, self.y, self.th = x, y, th
        now = self.get_clock().now().to_msg()
        q = (0.0, 0.0, math.sin(self.oth / 2), math.cos(self.oth / 2))
        if self.get_parameter('publish_odom_tf').value:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
            t.transform.translation.x, t.transform.translation.y = self.ox, self.oy
            t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
            self.tf.sendTransform(t)
        o = Odometry()
        o.header.stamp = now
        o.header.frame_id, o.child_frame_id = 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.ox, self.oy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = q[2], q[3]
        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = self.w
        for i, v in zip((0, 7, 14, 21, 28, 35), (0.01, 0.01, 1e6, 1e6, 1e6, 0.1)):
            o.pose.covariance[i] = v
        for i, v in zip((0, 7, 14, 21, 28, 35), (0.01, 0.01, 1e6, 1e6, 1e6, 0.5)):
            o.twist.covariance[i] = v
        self.odom_pub.publish(o)

    def hits_leg(self, x, y, th):
        leg = self.get_parameter('hidden_leg').value
        if not leg:
            return False
        lx, ly = (float(v) for v in leg.split(','))
        # The leg in base_link, against the footprint (config/nav2.yaml)
        dx, dy = lx - x, ly - y
        bx = math.cos(th) * dx + math.sin(th) * dy
        by = -math.sin(th) * dx + math.cos(th) * dy
        r = 0.015
        return -0.17 - r < bx < 0.12 + r and abs(by) < 0.11 + r

    def imu(self):
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'imu_link'
        m.orientation_covariance[0] = -1.0
        m.angular_velocity.z = self.true_w
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
