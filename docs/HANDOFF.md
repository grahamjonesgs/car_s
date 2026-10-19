# Handoff: localisation, SLAM and obstacle avoidance

Written 2026-10-19 on the Pi (`pi-car`) for the Claude Code session on the
remote control PC. Read [README.md](../README.md) first for the hardware,
topics and how to run things. This file covers where things stand and what
needs doing to get sensor fusion, SLAM and Nav2 working.

## Goal

The owner has never got the IMU, lidar and wheel odometry working together
well enough for good tracking, obstacle avoidance or SLAM. The aim is:

1. Fused odometry (wheel odometry + gyro) that tracks well, especially in
   rotation.
2. SLAM with `slam_toolbox` to build a map.
3. Nav2 navigation with obstacle avoidance from the lidar.

## Split of work between the machines

| Pi (`pi-car`) | PC |
|---|---|
| `car_s_controller_accel`, `car_imu`, `car_mag`, `rplidar_composition`, `robot_state_publisher` (all started by `ros2 launch car_s robot_car.launch.py`) | `robot_localization` EKF, `slam_toolbox`, Nav2, RViz, `key_cmd` / teleop |

Keep the heavy processing on the PC. The Pi is a Pi 4 with 8 GB of RAM, on
WiFi.

## Current state (verified on the Pi today)

- ROS 2 Jazzy on Ubuntu 24.04 on both machines. The Pi uses the default RMW
  (`rmw_fastrtps_cpp`), and `ROS_DOMAIN_ID` is unset (0).
- The Pi is `pi-car`, 192.168.68.78/22 on WiFi. Its clock is NTP-synced
  (`systemd-timesyncd`). Make sure the PC is NTP-synced too, or TF lookups
  across machines will fail or extrapolate.
- `pigpiod` runs as a systemd service on the Pi. The lidar is on
  `/dev/ttyUSB0`, and the user is in `dialout`.
- Tested working:
  - `/scan`: RPLIDAR A1 at about 7 Hz, 360 points, frame `laser_link`, ranges seen 0.18–4.45 m, Express mode, 12 m maximum.
  - `/imu/data_raw`: MPU6050 at 200 Hz, frame `imu_link`, at rest about (−0.47, 0.32, 9.70) m/s². It has no orientation, so `orientation_covariance[0] = -1`.
  - `/imu/mag`: QMC5883L at 10 Hz, in Tesla, uncalibrated. Its field magnitude is about 108 µT, roughly twice Earth's field, so it's distorted by the motors. **Don't use it** indoors near the steppers.
  - `/odom` and the TF `odom` → `base_link` from the controller at 20 Hz.
- Fixed today (see `git log`):
  - `cmd_vel` is now standard: `linear.x` in m/s and `angular.z` in rad/s, positive anticlockwise (REP-103), using differential-drive kinematics. Previously `angular.z` was a ratio limited to ±1, with inconsistent signs.
  - The `odom` twist is now in `base_link` (`linear.x` = forward speed, `y` = 0).
  - The step counters are atomic.
  - The `cmd_vel` timeout safety bug is fixed.
  - `cmd_vel` is ignored while the motors are disabled (`motor_on` false).
  - The IMU node no longer creates a node for every reading, which was flooding DDS discovery over WiFi.
  - The magnetometer data-ready check and units are fixed.
  - GPIO clashes are removed.
- **Not yet verified on the real car:** that up-arrow drives forward and
  left-arrow turns anticlockwise. If it's mirrored, fix it in the controller
  (the `DIR` pin logic), not in the URDF or Nav2.

## Why fusion and SLAM have struggled (likely causes, in order)

1. **Skid steer.** The car has 4 fixed wheels and no steering, so it skids
   whenever it turns. Wheel odometry is good for distance but poor for
   rotation, and the effective track width when spinning is larger than the
   physical 0.2337 m. Yaw from the wheels drifts badly, which breaks
   `slam_toolbox` scan matching. **Fix:** take yaw rate from the gyro, not
   the wheels (see the EKF configuration below), and calibrate the effective
   wheel separation.
2. **Duplicate TF.** The controller always publishes the TF `odom` →
   `base_link`. If you also run a `robot_localization` EKF with
   `publish_tf: true`, `base_link` gets two parents and everything breaks.
   The controller needs a parameter to turn its TF off (see Pi-side changes).
3. **Zero covariances.** Neither `/odom` nor `/imu/data_raw` sets any
   covariance (all zeros), so `robot_localization` treats every measurement
   as perfect. Set realistic covariances in the Pi nodes.
4. **The Madgwick filter.** If run with defaults it publishes its own TF
   `odom` → `imu_link`, which clashes with the URDF (it needs
   `publish_tf:=false`). For this robot it isn't needed: the EKF can use the
   gyro rate directly.
5. **The magnetometer.** It's distorted by the steppers and uncalibrated, so
   leave it out of the fusion.
6. **IMU and lidar mounting not checked.** The URDF has both mounted level
   and facing forward (`rpy="0 0 0"`), but an old comment in `pi_car_imu.cpp`
   says the IMU is "facing backwards". Check both (see below).

## What to do

### 1. PC setup

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-teleop-twist-keyboard
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone https://github.com/grahamjonesgs/car_s.git   # or git pull if already cloned
cd ~/ros_ws && colcon build --symlink-install --packages-select car_s
```

On the PC only `key_cmd` and `imu_normalize` build, because pigpio isn't
installed there. That's expected. Check that `ros2 topic list` shows the
Pi's topics while the car is running. If discovery over WiFi is unreliable,
consider a Fast DDS discovery server, or Cyclone DDS with unicast peers on
both machines.

Put new configuration and launch files for the PC (EKF, SLAM, Nav2 parameters)
in the `car_s` repo, in a new folder such as `config/` plus a PC-side launch
file, so they're version controlled. CMake already installs `launch/` and
`urdf/`; add `config` to the `install(DIRECTORY ...)` line.

### 2. Pi-side code changes (make them in the repo, then pull and build on the Pi)

All in `src/pi_car_s_controller_accel.cpp` and `src/pi_car_imu.cpp`:

- [ ] Add a parameter `publish_odom_tf` (default true) to the controller,
  and skip `odom_broadcaster_->sendTransform` when it's false. Set it to
  false in `launch/robot_car.launch.py` when the EKF is in use.
- [ ] Set covariances on `/odom`: twist `vx` about 0.01, `vyaw` large
  (about 0.5, as wheel yaw is unreliable), and pose diagonals non-zero.
- [ ] Set `angular_velocity_covariance` and `linear_acceleration_covariance`
  on `/imu/data_raw` (gyro about 0.0005 (rad/s)², accel about 0.05 (m/s²)²
  on the diagonals as a starting point).
- [ ] Make `WHEEL_SEPARATION` (and ideally `M_PER_REVOLUTION`) ROS
  parameters, so they can be calibrated without rebuilding.
- [ ] Optional: reduce the IMU to 100 Hz to cut WiFi traffic (`IMU_TIMER`).

The owner builds on the Pi with:
```bash
cd ~/ros_ws/src/car_s && git pull
cd ~/ros_ws && MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2 --packages-select car_s
```

### 3. Check sensor mounting (with the owner, on the real car)

- **IMU:** at rest, `linear_acceleration.z` should be about +9.8, and it is.
  Tip the nose down: `x` should go positive if the IMU's X axis faces
  forward. Spin the car anticlockwise by hand: `angular_velocity.z` should be
  positive. If the axes are wrong, fix the `imu_joint` rotation in
  `urdf/robot_car.urdf.xml` (for example `rpy="0 0 3.14159"` if it faces
  backwards).
- **Lidar:** put an object about 30 cm directly in front of the car. In RViz
  (fixed frame `base_link`) it should appear on +X. If not, rotate
  `laser_joint` in the URDF.
- **Recalibrate the IMU offsets** if needed. The procedure is in the README.

### 4. Calibrate the odometry

- **Straight line:** drive 2 m by tape measure and compare `/odom`. Adjust
  `M_PER_REVOLUTION` (currently 0.21) if it's off.
- **Rotation:** spin on the spot exactly 5 turns and compare `/angle` (or
  integrate gyro Z). Scale the effective `WHEEL_SEPARATION` up until the wheel
  yaw matches. Skid steer usually needs 1.3–2× the physical track.

### 5. EKF (`robot_localization`) on the PC

A starting point (`config/ekf.yaml`):

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    odom0: /odom
    # Only forward velocity from the wheels (x, y, z, roll, pitch, yaw,
    # vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az)
    odom0_config: [false, false, false, false, false, false,
                   true,  true,  false, false, false, false,
                   false, false, false]
    imu0: /imu/data_raw
    # Only yaw rate from the gyro
    imu0_config: [false, false, false, false, false, false,
                  false, false, false, false, false, true,
                  false, false, false]
    imu0_remove_gravitational_acceleration: true
```

Set `publish_odom_tf: false` on the controller when this runs. Test by
spinning in place and driving a square, and compare the result in RViz.

### 6. SLAM (`slam_toolbox`) on the PC

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=<repo>/config/slam.yaml
```

Copy the default `mapper_params_online_async.yaml` into the repo, and set
`base_frame: base_link`, `odom_frame: odom`, `scan_topic: /scan`, and
`max_laser_range: 8.0` (the A1 is reliable to about 8 m indoors). Drive slowly
(0.1–0.15 m/s, gentle turns). Save the map with `ros2 run nav2_map_server
map_saver_cli -f <name>`.

### 7. Nav2 on the PC

- **Twist type:** the controller subscribes to `geometry_msgs/Twist` (not
  stamped) on `/cmd_vel`. Make sure Nav2 publishes unstamped Twist
  (`enable_stamped_cmd_vel: false` where present).
- **Robot model:** use a differential-drive robot model with
  `robot_radius` of about 0.15 m (the body is about 0.2 × 0.15 m; measure it).
- **Speed limits:** start with a maximum of about 0.2 m/s linear and
  1.0 rad/s angular. Set the acceleration limits in the velocity smoother at
  or below the controller's `motor_accel` (default 1.0 m/s²), so the
  steppers don't slip.
- **Mode and enable:** the controller must be in auto mode (default, mode 2).
  It stops 0.1 s after the last `cmd_vel`, which is fine at Nav2's default
  20 Hz. Send `motor_on` true before navigating, as the controller ignores
  `cmd_vel` while disabled.
- **Costmaps:** the lidar feeds the local and global costmaps for obstacle
  avoidance. Its minimum range is about 0.15 m, and objects lower than the
  lidar's height are invisible to it.

## Cautions

- **Only one controller at a time.** Never start `car_s_controller_accel` (or
  any node using pigpio) while another is running on the Pi: they fight over
  the GPIO, and starting or stopping one disables the other's motors. Check
  with `ros2 node list` first.
- **Test new motion code with the wheels off the ground.**
- **Pi builds:** use `MAKEFLAGS=-j2 ... --parallel-workers 2`.
- **Don't change these on the Pi:** `/boot/firmware/config.txt` must keep
  `dtparam=audio=off` (audio uses the motor PWM hardware), and `pigpiod`
  must keep `-n 127.0.0.1`.

## Useful checks

```bash
ros2 run tf2_tools view_frames            # expect map -> odom -> base_link -> {imu_link, laser_link, wheels}
ros2 topic hz /scan /odom /imu/data_raw
ros2 topic echo /odom --field twist.twist
ros2 run tf2_ros tf2_echo odom base_link
```
