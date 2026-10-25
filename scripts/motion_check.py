#!/usr/bin/env python3
"""Notice when the car is stuck, slipping or being pushed.

Runs on the car, started by launch/car_local.launch.py, next to the data it
uses, so WiFi delays don't reach it. The wheel odometry counts
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
trigger it. Stuck, the clearest case, is also checked over a shorter window
(stuck_window_s, held for stuck_hold_s), so it is reported in about a second
instead of window_s + hold_s. Changes are logged too (a warning for stuck, slipping and
pushed). The measured motion is published on /motion_check/twist
(geometry_msgs/TwistStamped, in base_link) for plotting against /odom.

Bumps: when the car gets stuck driving forwards or backwards (the distance
short, not just the turn), something the lidar can't see is probably in the
way, such as a low chair leg. It marks a row of points just beyond the
footprint's front or rear edge, in the map frame, and publishes the new
marks on /motion_check/new_bumps (PointCloud2). bump_store on the PC
(scripts/bump_store.py) keeps them, beside the saved map, and publishes them
all for the costmaps.

A stuck car's odometry carries on counting, so the EKF's pose drifts on past
the obstacle, and Nav2 then finds the car's own footprint in collision. So
when it marks a bump it also sets the EKF (on /set_pose) back to where the
car stopped, and keeps it there while the car stays stuck (Nav2 carries on
pushing for a while), if reset_ekf is true. "Where it stopped" is its map
pose from before the drift, turned into odom with SLAM's map -> odom as it is
now: SLAM corrects part of the drift by itself, and resetting odom to its old
value as well would correct that part twice.
"""
import math
from collections import deque

import numpy as np
import rclpy
import rclpy.executors
import rclpy.time
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener

MAX_POINTS = 240       # scan points used for matching, after thinning
ICP_ITERATIONS = 20
MATCH_DIST = 0.25      # m: farther pairs are ignored
MIN_INLIERS = 60       # fewer matched points: the match isn't trusted
MAX_RMS = 0.04         # m: a worse fit isn't trusted
MIN_CONSTRAINT = 0.15  # weakest/strongest direction of the scan's walls
NORMAL_GAP = 0.15      # m: neighbours farther apart don't give a normal
HISTORY_S = 3.0
BUMP_MARGIN = 0.08     # m beyond the footprint edge: over a costmap cell (0.05 m),
                       # or the car would start in collision with its own mark
BUMP_SPACING = 0.05    # m between marked points along the edge
BUMP_HEIGHT = 0.05     # m: z of the points, within the costmap's heights


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
    # Squared distances as |p|^2 + |q|^2 - 2 p.q in float32: one matrix
    # product, about 4x faster on the Pi than differencing every pair
    old32 = old.astype(np.float32)
    old_sq = (old32 ** 2).sum(axis=1)
    x, y, th = guess
    for _ in range(ICP_ITERATIONS):
        c, s = math.cos(th), math.sin(th)
        p = cur @ np.array([[c, s], [-s, c]]) + (x, y)
        p32 = p.astype(np.float32)
        d2 = (p32 ** 2).sum(axis=1)[:, None] + old_sq[None, :] - 2 * (p32 @ old32.T)
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
        # The quick stuck check: over this window, only when the wheels
        # commanded at least stuck_min_distance in it
        self.stuck_window = self.declare_parameter('stuck_window_s', 0.5).value
        self.stuck_hold = self.declare_parameter('stuck_hold_s', 0.5).value
        self.stuck_min_dist = self.declare_parameter('stuck_min_distance', 0.05).value
        # Below these, per window, the car isn't commanded to move
        self.min_dist = self.declare_parameter('min_distance', 0.04).value
        self.min_turn = self.declare_parameter('min_turn', 0.25).value
        # Measured/commanded below these: stuck, then slipping
        self.stuck_ratio = self.declare_parameter('stuck_ratio', 0.3).value
        self.slip_ratio = self.declare_parameter('slip_ratio', 0.7).value
        # Uncommanded movement above these: pushed
        self.push_dist = self.declare_parameter('push_distance', 0.06).value
        self.push_turn = self.declare_parameter('push_turn', 0.2).value
        # The footprint's edges in base_link (config/nav2.yaml)
        self.front = self.declare_parameter('footprint_front', 0.12).value
        self.rear = self.declare_parameter('footprint_rear', -0.17).value
        self.half_width = self.declare_parameter('footprint_half_width', 0.11).value
        self.mark_bumps = self.declare_parameter('mark_bumps', True).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.reset_ekf = self.declare_parameter('reset_ekf', True).value
        self.cmd_sign = 0.0   # direction of a translation found blocked, else 0
        # (clock time, x, y, yaw) of base_link in map_frame: a stuck car's
        # odometry carries on counting, so its pose drifts on past the
        # obstacle; the bump is marked, and the EKF reset, from before
        self.map_poses = deque()
        self.pin = None       # map pose the EKF is held at while stuck
        self.pin_sign = 0.0   # the direction it got stuck in
        self.cmd_vx = 0.0     # the wheels' speed now (from /odom)
        self.gyro_wz = 0.0    # the turn rate now

        self.odom = deque()   # (t, x, y, yaw) from the wheel steps
        self.gyro = deque()   # (t, integrated yaw)
        self.scans = deque()  # (t, points, normals) in base_link
        self.laser = None     # base_link <- laser_link (x, y, yaw)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.state = 'unknown'
        self.candidate = None
        self.since = None
        self.candidate_window = None  # window of the check that started it
        self.detail = ''
        self.state_pub = self.create_publisher(String, '/motion_check', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/motion_check/twist', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.new_bumps_pub = self.create_publisher(PointCloud2, '/motion_check/new_bumps', 10)
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/set_pose', 10)
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
        self.cmd_vx = m.twist.twist.linear.x
        self.trim(self.odom, t)

    def on_imu(self, m):
        t = stamp_s(m.header.stamp)
        self.gyro_wz = m.angular_velocity.z
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
        # Evenly spread over the scan (a whole-number step kept all of the
        # A1's 360 points)
        keep = np.linspace(0, int(ok.sum()) - 1, min(int(ok.sum()), MAX_POINTS)).astype(int)
        r, a = r[ok][keep], a[ok][keep]
        pts = np.stack([laser[0] + r * np.cos(a), laser[1] + r * np.sin(a)], axis=1)
        self.scans.append((t, pts, normals(pts)))
        self.trim(self.scans, t)
        self.check(t)

    def record_pose(self, frame, history):
        try:
            tr = self.tf.lookup_transform(frame, 'base_link', rclpy.time.Time())
        except Exception:
            return
        q = tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        now = self.get_clock().now().nanoseconds * 1e-9
        history.append((now, tr.transform.translation.x, tr.transform.translation.y, yaw))
        self.trim(history, now)

    def measure(self, t, window, min_cmd_dist=0.0):
        """Wheel and real movement over about window seconds up to t:
        (span, cmd, cmd_dist, turn, meas_dist), meas_dist None if the scans
        can't tell (or, to save the scan match, the wheels drove under
        min_cmd_dist), or None if the data isn't there yet"""
        old = at_time(self.scans, t - window)
        o0, o1 = at_time(self.odom, old[0]), at_time(self.odom, t)
        g0, g1 = at_time(self.gyro, old[0]), at_time(self.gyro, t)
        span = t - old[0]
        if (span < 0.5 * window or None in (o0, o1, g0, g1)
                or abs(o0[0] - old[0]) > 0.2 or abs(g0[0] - old[0]) > 0.2
                or abs(o1[0] - t) > 0.2 or abs(g1[0] - t) > 0.2):
            return None

        # What the wheels did, in the old base_link
        c, s = math.cos(o0[3]), math.sin(o0[3])
        dx, dy = o1[1] - o0[1], o1[2] - o0[2]
        cmd = (c * dx + s * dy, -s * dx + c * dy, wrap(o1[3] - o0[3]))
        cmd_dist = math.hypot(cmd[0], cmd[1])
        turn = g1[1] - g0[1]
        if cmd_dist < min_cmd_dist:
            return span, cmd, cmd_dist, turn, None

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
            meas_dist = math.hypot(best[0][0], best[0][1])
            if window == self.window:
                (mx, my, myaw) = best[0]
                tw = TwistStamped()
                tw.header.stamp = rclpy.time.Time(seconds=t).to_msg()
                tw.header.frame_id = 'base_link'
                tw.twist.linear.x, tw.twist.linear.y = mx / span, my / span
                tw.twist.angular.z = myaw / span
                self.twist_pub.publish(tw)
        return span, cmd, cmd_dist, turn, meas_dist

    def check(self, t):
        self.record_pose(self.map_frame, self.map_poses)

        # The quick stuck check first: driven a clear distance, hardly moved
        quick = self.measure(t, self.stuck_window, self.stuck_min_dist)
        if quick is not None:
            span, cmd, cmd_dist, turn, meas_dist = quick
            if (cmd_dist > self.stuck_min_dist and meas_dist is not None
                    and meas_dist / cmd_dist < self.stuck_ratio):
                self.cmd_sign = math.copysign(1.0, cmd[0])
                return self.update('stuck', f'moved {meas_dist:.2f} m of {cmd_dist:.2f} m '
                                            f'in {span:.1f} s', self.stuck_hold, self.stuck_window)

        slow = self.measure(t, self.window)
        if slow is None:
            return self.update('unknown', 'waiting for odometry, gyro and scans')
        span, cmd, cmd_dist, turn, meas_dist = slow
        moving = cmd_dist > self.min_dist
        turning = abs(cmd[2]) > self.min_turn
        self.cmd_sign = 0.0
        if moving or turning:
            ratios = []
            if moving and meas_dist is not None:
                ratios.append(meas_dist / cmd_dist)
                # Only a short distance is something in the way ahead or
                # behind: a skid steer's arc often turns short on carpet
                if ratios[-1] < self.stuck_ratio:
                    self.cmd_sign = math.copysign(1.0, cmd[0])
            if turning:
                # Signed, so a turn the wrong way counts as none
                ratios.append(turn / cmd[2])
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

    def update(self, label, detail, hold=None, window=None):
        """Report label once it has lasted hold (default hold_s). window is
        the window of the check that gave it (default window_s)"""
        now = self.get_clock().now().nanoseconds * 1e-9
        hold = self.hold if hold is None else hold
        if self.pin is not None:
            # Hold it only while the wheels still drive into the obstacle and
            # the car isn't turning: the stuck verdict covers the last window,
            # so it lags a car that is already backing away
            pushing = (self.cmd_vx * self.pin_sign > 0.01 and abs(self.gyro_wz) < 0.1)
            if label == 'stuck' and self.cmd_sign == self.pin_sign and pushing:
                self.set_ekf_pose(self.pin, log=False)
            else:
                self.pin = None
        if label != self.candidate:
            self.candidate, self.since = label, now
            self.candidate_window = self.window if window is None else window
        self.detail = detail
        if label != self.state and now - self.since >= hold:
            self.state = label
            text = f'Motion: {label}' + (f' ({detail})' if detail else '')
            # rclpy wants one severity per logging call site
            if label in ('stuck', 'slipping', 'pushed'):
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
            self.publish_state()
            if label == 'stuck' and self.mark_bumps:
                self.mark_bump()

    def mark_bump(self):
        """Mark a row of points just beyond the edge the car was driving
        towards (not for a stuck turn, or a turn that fell short while the
        distance didn't: which corner caught is unknown)"""
        if self.cmd_sign == 0.0:
            return
        # Where it was when it stopped moving: the start of the first window
        # that showed it stuck
        pose = at_time(self.map_poses, self.since - self.candidate_window)
        if pose is None:
            self.get_logger().warning(f'No {self.map_frame} -> base_link pose to mark the bump')
            return
        _, px, py, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        x = (self.front + BUMP_MARGIN) if self.cmd_sign > 0 else (self.rear - BUMP_MARGIN)
        n = int(2 * self.half_width / BUMP_SPACING) + 1
        new = []
        for i in range(n):
            y = -self.half_width + i * 2 * self.half_width / (n - 1)
            new.append([round(px + c * x - s * y, 3), round(py + s * x + c * y, 3)])
        header = Header(frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())
        self.new_bumps_pub.publish(point_cloud2.create_cloud_xyz32(
            header, [(p[0], p[1], BUMP_HEIGHT) for p in new]))
        if self.reset_ekf:
            # Held there while it stays stuck
            self.pin = pose
            self.pin_sign = self.cmd_sign
            if self.pin is not None:
                self.set_ekf_pose(self.pin)
        mx = sum(p[0] for p in new) / n
        my = sum(p[1] for p in new) / n
        self.get_logger().warning(
            f'Marked something the lidar can\'t see {"ahead" if self.cmd_sign > 0 else "behind"} '
            f'at ({mx:.2f}, {my:.2f}) in {self.map_frame}; Nav2 will plan around it')

    def set_ekf_pose(self, pose, log=True):
        """Put the EKF where base_link is at pose (time, x, y, yaw) in map_frame"""
        if self.count_subscribers('/set_pose') == 0:
            return
        try:
            tr = self.tf.lookup_transform(self.odom_frame, self.map_frame, rclpy.time.Time())
        except Exception:
            return
        q = tr.transform.rotation
        t_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(t_yaw), math.sin(t_yaw)
        _, mx, my, myaw = pose
        x = tr.transform.translation.x + c * mx - s * my
        y = tr.transform.translation.y + s * mx + c * my
        yaw = wrap(myaw + t_yaw)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.odom_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        for i in (0, 7, 35):
            m.pose.covariance[i] = 1e-3
        for i in (14, 21, 28):
            m.pose.covariance[i] = 1e6
        self.set_pose_pub.publish(m)
        if log:
            self.get_logger().info(f'Holding the EKF where the car stopped, '
                                   f'({x:.2f}, {y:.2f}) in {self.odom_frame}, while it is stuck')

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
