"""Calibrate the odometry on the floor, against the gyro and the lidar.

/odom only counts the step pulses sent to the motors, so it says how far the
car was told to go. This drives the car and compares that with the gyro
(real rotation) and the lidar (real distance to the walls in front and
behind), and prints the corrected wheel_separation or m_per_revolution.

    python3 tools/odom_cal.py check          # clearance all round, no driving
    python3 tools/odom_cal.py forward 0.5    # drive 0.5 m forward at 0.1 m/s
    python3 tools/odom_cal.py spin 2         # spin 2 turns anticlockwise at 0.5 rad/s

Needs the car running and the lidar spinning (l in key_cmd), and nothing
else driving it (not pc_bringup with Nav2). It enables the motors, and
always stops and disables them at the end. Measure the forward distance with
a tape too: the lidar is only as good as the walls it sees.

Apply a result with ros2 param set /car_controller <name> <value>, check it
by running the test again, then make it the default in
src/pi_car_s_controller_accel.cpp.
"""
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Int32
from tf2_ros import Buffer, TransformListener

MIN_CLEAR = 0.40          # m, all round, for any test
FORWARD_SPEED = 0.1       # m/s
SPIN_RATE = 0.5           # rad/s


class OdomCal(Node):
    def __init__(self):
        super().__init__('odom_cal')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.motor = self.create_publisher(Bool, '/motor_on', 10)
        self.mode = self.create_publisher(Int32, '/motor_mode', 10)
        self.odom = None
        self.scan = None
        self.gyro_yaw = 0.0
        self.last_imu = None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)

    def on_odom(self, msg):
        self.odom = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu is not None:
            self.gyro_yaw += msg.angular_velocity.z * (t - self.last_imu)
        self.last_imu = t

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def laser_yaw(self):
        q = self.tf.lookup_transform('base_link', 'laser_link', rclpy.time.Time()).transform.rotation
        return 2 * math.atan2(q.z, q.w)

    def ranges_towards(self, scan, centre_deg, half_deg, yaw):
        """Valid ranges within half_deg of centre_deg in base_link."""
        out = []
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max and math.isfinite(r)):
                continue
            a = math.degrees(scan.angle_min + i * scan.angle_increment + yaw)
            if abs((a - centre_deg + 180) % 360 - 180) <= half_deg:
                out.append(r)
        return out

    def clearance(self, yaw):
        out = {}
        for name, c in [('front', 0), ('left', 90), ('back', 180), ('right', -90)]:
            v = self.ranges_towards(self.scan, c, 30, yaw)
            out[name] = min(v) if v else float('nan')
        return out

    def front_back(self, yaw, n=10):
        """Median range straight ahead and straight behind over n scans."""
        f, b = [], []
        for _ in range(n):
            cur = self.scan
            while self.scan is cur:
                rclpy.spin_once(self, timeout_sec=0.05)
            f += self.ranges_towards(self.scan, 0, 3, yaw)
            b += self.ranges_towards(self.scan, 180, 3, yaw)
        return statistics.median(f), statistics.median(b)

    def pose(self):
        p = self.odom.pose.pose
        return p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w)

    def drive(self, vx, wz, sec, on_step=None):
        tw = Twist()
        tw.linear.x = vx
        tw.angular.z = wz
        end = time.time() + sec
        while time.time() < end:
            self.cmd.publish(tw)
            self.spin_for(0.05)
            if on_step:
                on_step()
        self.stop()

    def stop(self):
        for _ in range(5):
            self.cmd.publish(Twist())
            self.spin_for(0.05)

    def set_motors(self, on):
        for _ in range(5):
            self.motor.publish(Bool(data=on))
            self.spin_for(0.1)

    def controller_param(self, name):
        cli = self.create_client(GetParameters, '/car_controller/get_parameters')
        if not cli.wait_for_service(timeout_sec=5):
            return None
        fut = cli.call_async(GetParameters.Request(names=[name]))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        if not fut.result() or not fut.result().values:
            return None
        return fut.result().values[0].double_value


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('check', 'forward', 'spin'):
        print(__doc__)
        sys.exit(1)
    test = sys.argv[1]
    amount = float(sys.argv[2]) if len(sys.argv) > 2 else (0.5 if test == 'forward' else 2.0)

    rclpy.init()
    n = OdomCal()
    t0 = time.time()
    while (n.scan is None or n.odom is None) and time.time() - t0 < 20:
        n.spin_for(0.2)
    if n.scan is None or n.odom is None:
        sys.exit(f'No /scan or /odom (scan {n.scan is not None}, odom {n.odom is not None}): '
                 'is the car running and the lidar spinning?')
    n.spin_for(1.0)
    yaw = n.laser_yaw()
    c = n.clearance(yaw)
    print('Clearance (nearest within 30 deg), m:', {k: round(v, 2) for k, v in c.items()})
    if test == 'check':
        return
    if not all(v > MIN_CLEAR for v in c.values()):
        sys.exit(f'Need {MIN_CLEAR} m clear all round, not driving')
    if test == 'forward' and c['front'] < amount + 0.4:
        sys.exit(f'Need {amount + 0.4:.1f} m clear in front, not driving')

    try:
        n.mode.publish(Int32(data=2))   # auto mode: stops if commands stop
        n.set_motors(True)
        n.spin_for(1.0)
        if test == 'forward':
            current = n.controller_param('m_per_revolution')
            f0, b0 = n.front_back(yaw)
            o0 = n.pose()
            n.gyro_yaw = 0.0
            n.drive(FORWARD_SPEED, 0.0, amount / FORWARD_SPEED)
            n.spin_for(1.0)
            f1, b1 = n.front_back(yaw)
            o1 = n.pose()
            moved = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
            lidar = ((f0 - f1) + (b1 - b0)) / 2
            print(f'Odometry: {moved:.3f} m. Lidar: front {f0 - f1:+.3f} m, back {b1 - b0:+.3f} m, '
                  f'mean {lidar:.3f} m. Gyro: {math.degrees(n.gyro_yaw):+.1f} deg')
            if current:
                print(f'm_per_revolution {current:.4f} -> {current * lidar / moved:.4f} from the lidar. '
                      f'From a tape measure: {current:.4f} x tape / {moved:.3f}')
        else:
            current = n.controller_param('wheel_separation')
            odom_yaw = 0.0
            last = [n.pose()[2]]
            n.gyro_yaw = 0.0

            def track():
                nonlocal odom_yaw
                y = n.pose()[2]
                odom_yaw += wrap(y - last[0])
                last[0] = y
            n.drive(0.0, SPIN_RATE, amount * 2 * math.pi / SPIN_RATE, on_step=track)
            n.spin_for(1.0)
            track()
            ratio = odom_yaw / n.gyro_yaw
            print(f'Odometry: {math.degrees(odom_yaw):+.1f} deg. Gyro: {math.degrees(n.gyro_yaw):+.1f} deg. '
                  f'Ratio {ratio:.3f}')
            if current:
                print(f'wheel_separation {current:.4f} -> {current * ratio:.4f}')
    finally:
        n.stop()
        n.set_motors(False)
        print('Motors stopped and disabled')


if __name__ == '__main__':
    main()
