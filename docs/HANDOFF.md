# Handoff: localisation, SLAM and obstacle avoidance

First written 2026-10-19 on the Pi (`pi-car`) for a Claude Code session on
the PC; updated the same day by that session for the next one, which runs on
the **Beelink** (see "The PC is now the Beelink"). Read
[README.md](../README.md) first for the hardware, topics and how to run
things. Sections from "Goal" onwards are the original plan, annotated with
what was done.

## Start here (state at the end of 2026-10-19)

**Working and checked on the real car** (battery disconnected, so the car
can't move; checked from the old PC VM):

- The car runs all of today's code (repo at `78ff675` or later on the Pi).
- `car_imu` calibrates the gyro at start-up. At boot it logged offsets of
  0.0349, 0.0188, 0.0504 rad/s, and afterwards the bias at rest was
  0.0000–0.0001 rad/s on every axis (it had been −0.030 on Z).
- IMU at 87 Hz (target 100, was about 70), gyro noise about 4e-7 (rad/s)²
  thanks to the new 44 Hz low pass filter. `/odom` at 20 Hz. Both now carry
  covariances.
- The new controller parameters are live, and the wheel TFs are published.
- An EKF-only run of `pc_bringup.launch.py` against the car: the controller
  TF switched off and back on correctly over WiFi, no warnings, and the
  fused heading drifted 0.13° in 40 s while stationary.
- The lidar scans normally. Close returns behind the car earlier turned out
  to be the room: the car had been moved since this morning and the pattern
  changed. Recheck with `tools/scan_check.py` once it's on the floor in open
  space.

**Not yet done** (all need the battery connected and someone at the car):

1. Check that up-arrow in `key_cmd` drives forward and left-arrow turns
   anticlockwise, wheels off the ground first. If mirrored, fix the `DIR`
   pin logic in the controller.
2. Check the sensor mounting (section 3) and calibrate the odometry
   (section 4). `wheel_separation` and `m_per_revolution` can be changed live
   with `ros2 param set /car_controller ...`.
3. Map a room: `ros2 launch car_s pc_bringup.launch.py` plus `key_cmd`, then
   save it with `ros2 run nav2_map_server map_saver_cli -f ~/maps/room`.
4. Nav2 with clicked goals (`nav2:=true motors_on:=true`). Nav2 hasn't driven
   the real car yet.
5. Optional: autonomous exploration with `explore_lite` (m-explore-ros2,
   built from source; not set up).

## The PC is now the Beelink

| | |
|---|---|
| Host | `Ubuntu-Beelink`, 192.168.68.31, wired Ethernet |
| Hardware | AMD Ryzen AI 9 HX 470 (24 threads), Radeon 890M, 27 GB RAM. Disk was 83% full (40 GB free) before ROS went on |
| OS | Ubuntu 26.04 (the owner doesn't want to downgrade) |
| ROS | **Jazzy inside a distrobox container called `jazzy`** (Ubuntu 24.04), to match the car. Don't mix ROS releases between the machines |
| Workspace | `~/ros2_ws`, repo at `~/ros2_ws/src/car_s`. The container shares the host's home folder, so the same files are seen from both |

**ROS only exists inside the container.** A shell on the host (including a
VS Code terminal and Claude Code's own shell) has no `ros2`. Use the wrapper
[tools/jazzy](../tools/jazzy), which runs a command in the container with
ROS and `~/ros2_ws` sourced, in the current directory:

```bash
cd ~/ros2_ws/src/car_s
tools/jazzy ros2 topic list
tools/jazzy ros2 param get /car_controller publish_odom_tf
cd ~/ros2_ws && src/car_s/tools/jazzy colcon build --symlink-install --packages-select car_s
tools/jazzy          # no command: an interactive shell in the container
```

Don't use `distrobox enter jazzy -- bash -lc '...'`: Ubuntu's `.bashrc`
returns early in non-interactive shells, so ROS isn't sourced. `~/.bashrc` is
shared by the host and the container, and only sources ROS when
`/opt/ros/jazzy/setup.bash` exists, i.e. inside the container (the original
is saved as `~/.bashrc.before-ros`). The container uses the host's network,
so the car sees it like any other machine, and the host's display and GPU, so
RViz opens as a normal window.

Checked from the Beelink on 2026-10-19: all five car nodes visible, IMU
received at a full 100 Hz (the VM over its virtual network got 87 Hz),
`/odom` at 20 Hz with 2.4 ms jitter (28 ms on the VM), and an EKF-only
`pc_bringup` ran at the full 30 Hz and switched the car's TF off. Interactive
Ctrl-C through the container hasn't been tried yet: after the first real run,
check `ros2 param get /car_controller publish_odom_tf` is back to `true`.

**Firewall:** the Beelink runs `ufw`, which denies incoming traffic by
default and would block ROS. There's a rule allowing all UDP from the car
(192.168.68.78) only. A 7400–7600 range wasn't enough: Fast DDS also uses
unicast ports in the 7000s, and with those blocked, topics worked but
`ros2 node list` and parameter calls didn't. Any other machine that needs to
talk ROS to the Beelink needs its own rule, e.g.
`sudo ufw allow proto udp from <ip>`.

**SSH to the car:** `ssh graham@192.168.68.78` works from the Beelink with
its key (`~/.ssh/id_ed25519`), and from the old VM with its own. On the Pi the workspace is `~/ros_ws` (not `ros2_ws`), the car runs as
the systemd service `car_s` (logs with `journalctl -u car_s`), and builds
need `MAKEFLAGS=-j2 ... --parallel-workers 2`. Never start car nodes by hand
while the service is running.

**The old PC** (`Ubuntu-ROS`, 192.168.68.82, a 4-core Parallels VM) has the
same setup. Don't run the PC stack on both machines at once: two EKFs or two
SLAM nodes fight over the car's TF.

**Nav2 on the Beelink:** `config/nav2.yaml` had been cut down for the
4-core VM (MPPI `batch_size` 1000 and `time_steps` 40). It's now back to the
defaults, 2000 and 56. Checked with `tools/sim_test.sh nav2:=true`: goals
around the box and back succeeded with no missed control loops. Keep
`default_server_timeout: 100`. **Note:** `config/nav2.yaml` is now too heavy
for the old VM. If Nav2 ever runs there again, cut these back.

## Testing without the car

`tools/sim_test.sh` runs the PC stack against a simulated car
(`tools/sim_car.py`) on ROS domain 42, limited to localhost, so the real car
is never touched. It prints the simulated car's true pose once a second, for
comparison with the EKF, SLAM and AMCL. `tools/imu_stats.py` and
`tools/scan_check.py` check the real IMU and lidar.

Pitfalls from testing:

- **Duplicate stacks:** a leftover EKF, SLAM or Nav2 from an earlier run
  causes TF fights and SLAM drift that looks like a config problem. Check
  `ros2 node list` for duplicates before believing a bad result.
- **Stopping launches from scripts:** background jobs in non-interactive
  shells start with SIGINT ignored, so `kill -INT` or `timeout -s INT` don't
  stop `ros2 launch`, and SIGTERM kills it without cleanup (leaving its nodes
  running, and the controller TF switched off). Use Ctrl-C in a terminal,
  or reset SIGINT before exec'ing the launch.
- **`pkill -f` matches its own shell** when the pattern appears anywhere in
  the command line, killing the script that ran it.
- slam_toolbox's `online_async_launch.py` defaults to `use_sim_time:=true`;
  `pc_bringup.launch.py` overrides it.
- If `pc_bringup` is killed without cleanup, turn the car's TF back on with
  `ros2 param set /car_controller publish_odom_tf true` (or restart the
  `car_s` service on the Pi).

## History: the first PC session (2026-10-19)

Done on the old PC VM:

- **NTP was off on the PC** and its clock was 0.68 s behind the Pi. Turned
  on; it now shows synchronized.
- Stopped tracking `build/`, `install/` and `log/` in git.
- Pi-side code changes from section 2 (commit "Prepare Pi nodes for EKF
  fusion"), now running on the car.
- PC configs and launch files: `config/ekf.yaml`, `config/slam.yaml`,
  `config/nav2.yaml`, `launch/pc_bringup.launch.py`, `launch/rviz.launch.py`
  and `rviz/car_s.rviz` (see the README's "SLAM and navigation" section).
- Tested against the simulated car: the EKF matched ground truth, SLAM drift
  stayed under 5 cm, and Nav2 reached goals around an obstacle. With AMCL on
  a saved map, the first goal succeeded (AMCL within about 10 cm of ground
  truth), but the return goal didn't finish within the test's 90 s limit (it
  got within 0.4 m). This tests the configuration and wiring only, not the
  real car's sensors or slip.

Found while checking the live car:

- **The IMU only produced about 70 Hz, not 200 Hz**: the 12 single-byte I2C
  reads per sample through `pigpiod` were too slow. It now uses one block
  read at 100 Hz.
- **The gyro Z bias had drifted to −0.030 rad/s (−1.7°/s)**, which would
  make yaw drift about 100° a minute. Now calibrated at start-up.
- The URDF wheel joints were `continuous` with nothing publishing
  `joint_states`, so the wheel TFs were missing. They're now `fixed`.

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

## State as found on the Pi (morning of 2026-10-19)

- ROS 2 Jazzy on Ubuntu 24.04 on both machines. The Pi uses the default RMW
  (`rmw_fastrtps_cpp`), and `ROS_DOMAIN_ID` is unset (0).
- The Pi is `pi-car`, 192.168.68.78/22 on WiFi. Its clock is NTP-synced
  (`systemd-timesyncd`). Make sure the PC is NTP-synced too, or TF lookups
  across machines will fail or extrapolate.
- `pigpiod` and `car_s` (which runs `robot_car.launch.py`) are systemd
  services on the Pi, so the car's nodes come up at boot. After pulling and
  building new Pi code, run `sudo systemctl restart car_s`. Stop it
  (`sudo systemctl stop car_s`) before running car nodes by hand.
- The lidar is on `/dev/ttyUSB0`, and the user is in `dialout`.
- **The lidar motor is stopped by default** after launch, so `/scan` is
  silent until it's started with `ros2 service call /start_motor
  std_srvs/srv/Empty` (or `l` in `key_cmd`). Start it before running SLAM or
  Nav2, or have the PC launch files call `/start_motor`. The alternative is to
  launch with `lidar_on:=true`. The driver also has an `auto_standby`
  parameter (spin only while `/scan` has subscribers), but in that mode it
  ignores the start and stop services.
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

- [x] Add a parameter `publish_odom_tf` (default true) to the controller,
  and skip `odom_broadcaster_->sendTransform` when it's false. Set it to
  false in `launch/robot_car.launch.py` when the EKF is in use.
  *Done differently: it can be changed at run time, and
  `pc_bringup.launch.py` sets it to false while the EKF runs and back to
  true when it stops. `robot_car.launch.py` also has a `publish_odom_tf`
  argument.*
- [x] Set covariances on `/odom`: twist `vx` about 0.01, `vyaw` large
  (about 0.5, as wheel yaw is unreliable), and pose diagonals non-zero.
- [x] Set `angular_velocity_covariance` and `linear_acceleration_covariance`
  on `/imu/data_raw` (gyro about 0.0005 (rad/s)², accel about 0.05 (m/s²)²
  on the diagonals as a starting point).
- [x] Make `WHEEL_SEPARATION` (and ideally `M_PER_REVOLUTION`) ROS
  parameters, so they can be calibrated without rebuilding. *Done:
  `wheel_separation` and `m_per_revolution`, settable at run time with
  `ros2 param set /car_controller ...`. Also added `cmd_vel_timeout`
  (default raised from 0.1 to 0.3 s, for WiFi jitter).*
- [x] Optional: reduce the IMU to 100 Hz to cut WiFi traffic (`IMU_TIMER`).

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
  integrate gyro Z). Scale the effective `wheel_separation` up until the wheel
  yaw matches. Skid steer usually needs 1.3–2× the physical track. Both can
  be tried live with `ros2 param set /car_controller wheel_separation 0.35`
  (and `m_per_revolution`), then made permanent by changing the defaults in
  the controller source.

### 5. EKF (`robot_localization`) on the PC

*Done: [config/ekf.yaml](../config/ekf.yaml), started by
`launch/pc_bringup.launch.py`. The starting point was:*

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

*Done: [config/slam.yaml](../config/slam.yaml). Note that slam_toolbox's
`online_async_launch.py` defaults to `use_sim_time:=true`, which
`pc_bringup.launch.py` overrides.* Copy the default
`mapper_params_online_async.yaml` into the repo, and set
`base_frame: base_link`, `odom_frame: odom`, `scan_topic: /scan`, and
`max_laser_range: 8.0` (the A1 is reliable to about 8 m indoors). Drive slowly
(0.1–0.15 m/s, gentle turns). Save the map with `ros2 run nav2_map_server
map_saver_cli -f <name>`.

### 7. Nav2 on the PC

*Done: [config/nav2.yaml](../config/nav2.yaml), run composed in one process
(separate processes overloaded the 4-core PC). The MPPI controller is lighter
than the default (1,000 samples, a 2 s horizon), and the BT navigator's
`default_server_timeout` is raised from 20 to 100 ms, which stopped goals
aborting under load.*

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
