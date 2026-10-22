#!/usr/bin/env python3
"""Notice when the car is stuck, slipping or being pushed.

Runs on the PC, started by pc_bringup.launch.py. The wheel odometry counts
the steps sent to the motors, so it carries on when the wheels spin without
moving the car (stuck, slipping, lifted) and misses the car being pushed.
This compares it, over the last second, with two measurements of what the
car really did:

- the lidar: the latest scan matched against one from about a second
  earlier (point-to-line ICP), giving the real movement
- the gyro: the real rotation

and publishes on /motion_check (std_msgs/String):

  idle      nothing commanded, nothing measured
  ok        moving about as commanded
  slipping  moving, but less than commanded (the ratio follows)
  stuck     commanded to move, but hardly moving
  pushed    moving without being commanded
  unknown   the scans can't tell, e.g. a featureless corridor, or no
            scans yet

A state is only reported once it has lasted hold_s, so one bad match can't
trigger it. Changes are logged too (a warning for stuck, slipping and
pushed). The measured motion is published on /motion_check/twist
(geometry_msgs/TwistStamped, in base_link) for plotting against /odom.

It only reports: it doesn't stop the car or change Nav2's plans.
"""
import math
from collections import deque

import numpy as np
import rclpy
import rclpy.executors
import rclpy.time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

MAX_POINTS = 240       # scan points used for matching, after thinning
ICP_ITERATIONS = 20
MATCH_DIST = 0.25      # m: farther pairs are ignored
MIN_INLIERS = 60       # fewer matched points: the match isn't trusted
MAX_RMS = 0.04         # m: a worse fit isn't trusted
MIN_CONSTRAINT = 0.15  # weakest/strongest direction of the scan's walls
NORMAL_GAP = 0.15      # m: neighbours farther apart don't give a normal
HISTORY_S = 3.0


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def at_time(history, t):
    """The (t, value...) entry in history closest to time t, or None."""
    if not history:
        return None
    return min(history, key=lambda e: abs(e[0] - t))


def normals(points):
    """Unit normals of a scan's points, from their neighbours in scan order.
    Points without close neighbours on both sides get NaN"""
    prev, nxt = np.roll(points, 1, axis=0), np.roll(points, -1, axis=0)
    ok = ((np.linalg.norm(points - prev, axis=1) < NORMAL_GAP)
          & (np.linalg.norm(nxt - points, axis=1) < NORMAL_GAP))
    d = nxt - prev
    n = np.stack([-d[:, 1], d[:, 0]], axis=1)
    length = np.linalg.norm(n, axis=1)
    ok &= length > 1e-6
    n[ok] /= length[ok, None]
    n[~ok] = np.nan
    return n


def icp(cur, old, old_n, guess):
    """Find (x, y, yaw) moving cur's points onto old's (point-to-line), from
    guess. Returns (pose, rms, inliers, constraint), or None"""
    keep = ~np.isnan(old_n[:, 0])
    old, old_n = old[keep], old_n[keep]
    if len(old) < MIN_INLIERS or len(cur) < MIN_INLIERS:
        return None
    x, y, th = guess
    for _ in range(ICP_ITERATIONS):
        c, s = math.cos(th), math.sin(th)
        p = cur @ np.array([[c, s], [-s, c]]) + (x, y)
        d2 = ((p[:, None, :] - old[None, :, :]) ** 2).sum(axis=2)
        j = d2.argmin(axis=1)
        close = d2[np.arange(len(p)), j] < MATCH_DIST ** 2
        if close.sum() < MIN_INLIERS:
            return None
        p, q, n = p[close], old[j[close]], old_n[j[close]]
        r = ((p - q) * n).sum(axis=1)
        # d(residual)/d(x, y, yaw), the yaw turning about base_link
        jac = np.stack([n[:, 0], n[:, 1],
                        n[:, 0] * -(p[:, 1] - y) + n[:, 1] * (p[:, 0] - x)], axis=1)
        h = jac.T @ jac
        try:
            step = -np.linalg.solve(h, jac.T @ r)
        except np.linalg.LinAlgError:
            return None
        x, y, th = x + step[0], y + step[1], th + step[2]
        if np.abs(step).max() < 1e-4:
            break
    # How well the walls pin down the movement in its weakest direction: a
    # corridor constrains across it, not along it
    ev = np.linalg.eigvalsh(h[:2, :2])
    constraint = ev[0] / ev[1] if ev[1] > 0 else 0.0
    rms = math.sqrt(float((r ** 2).mean()))
    return (x, y, wrap(th)), rms, int(close.sum()), constraint


class MotionCheck(Node):
    def __init__(self):
        super().__init__('motion_check')
        self.window = self.declare_parameter('window_s', 1.0).value
        self.hold = self.declare_parameter('hold_s', 1.5).value
        # Below these, per window, the car isn't commanded to move
        self.min_dist = self.declare_parameter('min_distance', 0.04).value
        self.min_turn = self.declare_parameter('min_turn', 0.25).value
        # Measured/commanded below these: stuck, then slipping
        self.stuck_ratio = self.declare_parameter('stuck_ratio', 0.3).value
        self.slip_ratio = self.declare_parameter('slip_ratio', 0.7).value
        # Uncommanded movement above these: pushed
        self.push_dist = self.declare_parameter('push_distance', 0.06).value
        self.push_turn = self.declare_parameter('push_turn', 0.2).value

        self.odom = deque()   # (t, x, y, yaw) from the wheel steps
        self.gyro = deque()   # (t, integrated yaw)
        self.scans = deque()  # (t, points, normals) in base_link
        self.laser = None     # base_link <- laser_link (x, y, yaw)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.state = 'unknown'
        self.candidate = None
        self.since = None
        self.detail = ''
        self.state_pub = self.create_publisher(String, '/motion_check', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/motion_check/twist', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self.publish_state)
        self.get_logger().info('Comparing the wheel odometry with the lidar and gyro')

    @staticmethod
    def trim(history, t):
        while history and history[0][0] < t - HISTORY_S:
            history.popleft()

    def on_odom(self, m):
        t = stamp_s(m.header.stamp)
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
        self.trim(self.odom, t)

    def on_imu(self, m):
        t = stamp_s(m.header.stamp)
        if self.gyro:
            t0, yaw = self.gyro[-1]
            dt = t - t0
            if 0 < dt < 0.5:
                yaw += m.angular_velocity.z * dt
        else:
            yaw = 0.0
        self.gyro.append((t, yaw))
        self.trim(self.gyro, t)

    def laser_pose(self):
        if self.laser is None:
            try:
                tr = self.tf.lookup_transform('base_link', 'laser_link', rclpy.time.Time())
            except Exception:
                return None
            q = tr.transform.rotation
            self.laser = (tr.transform.translation.x, tr.transform.translation.y,
                          math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        return self.laser

    def on_scan(self, m):
        laser = self.laser_pose()
        if laser is None:
            return
        t = stamp_s(m.header.stamp)
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + m.angle_increment * np.arange(len(r)) + laser[2]
        ok = np.isfinite(r) & (r > max(m.range_min, 0.2)) & (r < min(m.range_max, 8.0))
        if ok.sum() < MIN_INLIERS:
            return
        step = max(1, int(ok.sum()) // MAX_POINTS)
        r, a = r[ok][::step], a[ok][::step]
        pts = np.stack([laser[0] + r * np.cos(a), laser[1] + r * np.sin(a)], axis=1)
        self.scans.append((t, pts, normals(pts)))
        self.trim(self.scans, t)
        self.check(t)

    def check(self, t):
        old = at_time(self.scans, t - self.window)
        o0, o1 = at_time(self.odom, old[0]), at_time(self.odom, t)
        g0, g1 = at_time(self.gyro, old[0]), at_time(self.gyro, t)
        span = t - old[0]
        if (span < 0.5 * self.window or None in (o0, o1, g0, g1)
                or abs(o1[0] - t) > 0.2 or abs(g1[0] - t) > 0.2):
            return self.update('unknown', 'waiting for odometry, gyro and scans')

        # What the wheels did, in the old base_link
        c, s = math.cos(o0[3]), math.sin(o0[3])
        dx, dy = o1[1] - o0[1], o1[2] - o0[2]
        cmd = (c * dx + s * dy, -s * dx + c * dy, wrap(o1[3] - o0[3]))
        cmd_dist = math.hypot(cmd[0], cmd[1])
        turn = g1[1] - g0[1]

        # What the scans say, tried from the wheels' guess and from standing
        # still (it converges better from whichever is nearer the truth)
        cur = self.scans[-1]
        best = None
        for guess in ((cmd[0], cmd[1], turn), (0.0, 0.0, turn)):
            res = icp(cur[1], old[1], old[2], guess)
            if res and (best is None or res[1] < best[1]):
                best = res
        meas_dist = None
        if best and best[1] < MAX_RMS and best[3] > MIN_CONSTRAINT:
            (mx, my, myaw), rms = best[0], best[1]
            meas_dist = math.hypot(mx, my)
            tw = TwistStamped()
            tw.header.stamp = rclpy.time.Time(seconds=t).to_msg()
            tw.header.frame_id = 'base_link'
            tw.twist.linear.x, tw.twist.linear.y = mx / span, my / span
            tw.twist.angular.z = myaw / span
            self.twist_pub.publish(tw)

        moving = cmd_dist > self.min_dist
        turning = abs(cmd[2]) > self.min_turn
        if moving or turning:
            ratios = []
            if moving and meas_dist is not None:
                ratios.append(meas_dist / cmd_dist)
            if turning:
                ratios.append(abs(turn) / abs(cmd[2]))
            if not ratios:
                return self.update('unknown', 'the scans can\'t measure the movement here')
            ratio = min(ratios)
            detail = (f'moved {meas_dist if meas_dist is not None else float("nan"):.2f} m of '
                      f'{cmd_dist:.2f} m, turned {math.degrees(turn):.0f} of '
                      f'{math.degrees(cmd[2]):.0f} deg in {span:.1f} s')
            if ratio < self.stuck_ratio:
                return self.update('stuck', detail)
            if ratio < self.slip_ratio:
                return self.update('slipping', f'{100 * ratio:.0f}%: {detail}')
            return self.update('ok', detail)
        if abs(turn) > self.push_turn or (meas_dist is not None and meas_dist > self.push_dist):
            return self.update('pushed', f'moved {meas_dist or 0.0:.2f} m, turned '
                                         f'{math.degrees(turn):.0f} deg without a command')
        if meas_dist is None:
            return self.update('unknown', 'the scans can\'t measure the movement here')
        return self.update('idle', '')

    def update(self, label, detail):
        now = self.get_clock().now().nanoseconds * 1e-9
        if label != self.candidate:
            self.candidate, self.since = label, now
        self.detail = detail
        if label != self.state and now - self.since >= self.hold:
            self.state = label
            text = f'Motion: {label}' + (f' ({detail})' if detail else '')
            # rclpy wants one severity per logging call site
            if label in ('stuck', 'slipping', 'pushed'):
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
            self.publish_state()

    def publish_state(self):
        self.state_pub.publish(String(data=self.state))


def main():
    rclpy.init()
    node = MotionCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
