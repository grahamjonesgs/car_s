# car_s

ROS 2 (Lyrical, or Jazzy) software for a Raspberry Pi 4 robot car with four stepper motors,
an RPLIDAR A1, an MPU6050 IMU (accelerometer + gyro) and a QMC5883L magnetometer.

The Pi runs the hardware nodes. A PC on the same network does the heavy work:
sensor fusion (EKF), mapping (SLAM), navigation (Nav2) and autonomous
exploration, plus RViz and keyboard driving with `key_cmd`.

## Contents

- [Status](#status)
- [Hardware](#hardware)
- [Machines and network](#machines-and-network)
- [Nodes and topics](#nodes-and-topics)
- [How localisation works](#how-localisation-works)
- [Setting up the Pi from scratch](#setting-up-the-pi-from-scratch)
- [Setting up the PC](#setting-up-the-pc)
- [Running the car](#running-the-car)
- [Mapping, navigation and exploration](#mapping-navigation-and-exploration)
- [Calibration](#calibration)
- [Testing without the car](#testing-without-the-car)
- [Troubleshooting](#troubleshooting)
- [Cautions](#cautions)

## Status

As of 2026-10-20, all of this has been checked on the real car:

- Driving with `key_cmd`. The directions are right: forward is forward, and a
  positive turn is anticlockwise.
- Odometry calibrated on the floor (see [Calibration](#calibration)). The
  distance comes from a tape measure, and hasn't been rechecked since.
  Rotation was rechecked: within 0.2% of the gyro.
- EKF fusion of wheel speed and gyro yaw rate, and SLAM maps with straight,
  single walls.
- Nav2 to goals clicked in RViz, on a saved map with AMCL.
- Autonomous exploration (`explore:=true`): mapped about 42 m² of a large,
  cluttered room by itself before it was stopped, and on 2026-10-21 a second
  room (saved as `~/maps/cave`).

Next steps, in order:

- **The car no longer uses pigpio** (since 2026-10-22): the controller drives
  the hardware through the kernel's PWM driver and libgpiod, and the IMU and
  magnetometer use the kernel's I²C driver (see
  [How the Pi drives the hardware](#how-the-pi-drives-the-hardware)). This
  also removes the one dependency that was built from source and tied to the
  Pi 4, ahead of upgrading Ubuntu and ROS on the Pi. pigpio's PWM had failed
  on the right wheels above about 0.19 m/s, so the car used to swerve left
  above that speed. Tested with the wheels off the ground on 2026-10-22:
  every wheel's direction, both sides equal up to 0.3 m/s, microstep
  changes, Ctrl-C in `key_cmd`, and the drivers disabled within 49 ms of a
  `kill -9` on the controller; then odometry recalibrated on the floor and a
  first exploration run. The I²C move hasn't run on the car yet.

Still to look at:

- **Glass doors.** In the first exploration run on the kernel PWM, the car
  got stuck at a glass door: the lidar sees through glass from a distance,
  so the room beyond is mapped as open and exploration sends the car there,
  but close up the glass reflects and blocks it. A strip of tape at lidar
  height (10–15 cm) makes a door visible; the MaxBotix sonars on the parts
  list would see glass.
- New ROS command-line tools started on the Pi over SSH couldn't see any
  nodes on 2026-10-21 (even `--no-daemon`), while the Beelink saw them all.
  Run checks from the PC until that's understood.
- Exploration in cluttered rooms. In the second room, the car ended up in a
  gap between furniture where Nav2 found no path out ("no valid path found"
  from every goal), and in a later run drove its nose to within a few
  centimetres of a chair leg. Since then, none of it yet tried on the real
  car:
  - the measured rectangular footprint (narrower at the sides than the
    0.17 m circle that boxed the car in)
  - a wider, gentler local inflation for steering (0.40 m)
  - the SLAM map in the local costmap, so chair legs aren't wiped between
    lidar scans
  Chair legs are the hardest case: thin, and seen by only a few lidar points.
- The lidar's height above the floor hasn't been measured (the URDF guesses
  0.13 m). Nav2's planner logs "Inflation layer either not found or
  inflation is not set sufficiently for optimized non-circular collision
  checking" at start-up, although the 0.25 m inflation exceeds the
  footprint's 0.20 m corner radius. It seems harmless; exploration works.
- While planning long routes into unknown space, the Nav2 controller
  sometimes ran at about 13 Hz instead of 20 Hz. It still drove fine.
- The magnetometer is unused (see [How localisation works](#how-localisation-works)).
- On 2026-10-22 the Pi moved to a fresh Ubuntu 26.04 card with ROS 2
  Lyrical, and the PC to Lyrical too (see
  [ROS 2 Lyrical on the PC](#ros-2-lyrical-on-the-pc)). Checked on the Pi with
  the motor battery off: step pulses exact (0.3 m/s gives 9523 pulses/s on
  all four STEP pins, 1.0 rad/s gives 4158), ENABLE and the direction pins
  right, the crash stop disables the drivers 23 ms after a `kill -9`, IMU
  100 Hz, magnetometer 10 Hz, lidar 7.4 Hz. With the wheels off the ground
  and the motors powered, forward, backward and both turns were right. On
  2026-10-23 it drove on the floor (0.4 m forward and back, and turns),
  which looked right. Not yet mapped or navigated on the real car with
  Lyrical. The code still
  builds with Jazzy and libgpiod 1 (Ubuntu 24.04). Jazzy and Lyrical don't
  reliably talk to each other, so the Pi and the PC have to match.

## Hardware

| Part | Details |
|---|---|
| Computer | Raspberry Pi 4 Model B (8 GB), Ubuntu 26.04, ROS 2 Lyrical (Ubuntu 24.04 and Jazzy until 2026-10-22) |
| Motors | 4 steppers, 200 steps/rev, step/dir drivers with M0/M1/M2 microstep pins (up to 1/32) |
| Wheels | 4 fixed 65 mm wheels, skid steer, about 0.204 m travel per revolution. Axles about 0.13 m apart, wheel centres about 0.19 m apart side to side. The controller uses calibrated values (see [Calibration](#calibration)) |
| Size | About 0.29 m long and 0.22 m wide over the wheels. Measured from base_link (midway between the axles): front 0.12 m ahead, rear 0.17 m behind, as the rear deck overhangs. Spinning on the spot, the rear corners sweep a 0.20 m radius |
| Lidar | RPLIDAR A1 on USB, `/dev/ttyUSB0`, 115200 baud, about 7 Hz, 360 points. Spin axis 0.05 m ahead of base_link, on the centre line. **Mounted facing backwards** (its motor end to the rear): its 0° points to the rear, so the URDF turns `laser_link` by 180° |
| IMU | MPU6050 on I2C bus 1, address `0x68`. Mounted level and facing forward: Z up, positive Z rotation is anticlockwise |
| Magnetometer | QMC5883L on I2C bus 1, address `0x0D` |
| LEDs | Red on GPIO 23, blue on GPIO 24 (blue flashes while the motors are enabled) |

Step pulses are generated by the Pi's hardware PWM, so the two motors on each
side share one PWM channel: left on PWM1 (GPIO 13 and 19), right on PWM0
(GPIO 18 and 12). For that reason onboard audio must be disabled (it uses the
same PWM hardware), and nothing else may use GPIO 12, 13, 18 or 19.

### How the Pi drives the hardware

Everything goes through the kernel's own drivers; there's no daemon:

| What | How | Code |
|---|---|---|
| Step pulses | The kernel's `pwm-bcm2835` driver, through `/sys/class/pwm`. [overlays/car-pwm.dts](overlays/car-pwm.dts) puts PWM channel 0 on GPIO 12 and 18 (right wheels) and channel 1 on GPIO 13 and 19 (left) | [src/car_gpio.cpp](src/car_gpio.cpp) |
| DIR, ENABLE, microstep and LED pins | libgpiod (`/dev/gpiochip0`) | [src/car_gpio.cpp](src/car_gpio.cpp) |
| IMU and magnetometer | The kernel's `i2c-dev` driver (`/dev/i2c-1`) | [src/i2c_device.hpp](src/i2c_device.hpp) |

[tools/car_pwm_setup.sh](tools/car_pwm_setup.sh), run as root by the `car_s`
service before it starts, loads the overlay fresh at run time (no
`config.txt` change), exports both PWM channels and gives their files to the
car's user. `/dev/gpiochip0` belongs to the `dialout` group, and `/dev/i2c-1`
to `i2c` on Ubuntu 26.04 (`dialout` on 24.04).

The kernel can't report 10,000 edges a second to a program, so odometry is
worked out from the step rate actually set, multiplied by time: hardware PWM
produces it exactly. To check what the motors really get,
[tools/pulse_count.py](tools/pulse_count.py) counts the pulses on all four
STEP pins by reading the GPIO level register through `/dev/gpiomem`, and
[tools/kernel_pwm_test.sh](tools/kernel_pwm_test.sh) drives the kernel PWM
at 3–10 kHz without the controller and counts them.

Until 2026-10-22 all of this went through the `pigpiod` daemon. pigpio is
unmaintained, built from source, and doesn't support the Pi 5, and its PWM
channel 0 (the right wheels) failed above about 6 kHz (0.19 m/s at 32
microsteps) and ran 3–6% short from 4 kHz. If it's still installed, keep its
daemon off: pigpio's PWM reprograms the PWM clock behind the kernel's back,
after which the kernel sets every step rate wrong.

The lidar only sees obstacles at its own height, about 10 cm above the floor,
and nothing closer than 15 cm.

### Pinout

Looking at the Pi with the SD card at the top:

| Function | GPIO | Pin | Pin | GPIO | Function |
|---|---|---|---|---|---|
| IMU VCC | 3V3 | 1 | 2 | 5V | 5V to driver board |
| IMU SDA | 2 | 3 | 4 | 5V | |
| IMU SCL | 3 | 5 | 6 | GND | GND to driver board |
| | 4 | 7 | 8 | 14 | Driver ENABLE (low = on) |
| IMU GND | GND | 9 | 10 | 15 | Right front DIR (A) |
| | 17 | 11 | 12 | 18 | Right front STEP (A) |
| | 27 | 13 | 14 | GND | LED GND |
| | 22 | 15 | 16 | 23 | Red LED |
| | 3V3 | 17 | 18 | 24 | Blue LED |
| M0 microstep | 10 | 19 | 20 | GND | |
| M1 microstep | 9 | 21 | 22 | 25 | |
| M2 microstep | 11 | 23 | 24 | 8 | |
| | GND | 25 | 26 | 7 | |
| | 0 | 27 | 28 | 1 | |
| | 5 | 29 | 30 | GND | |
| Left front DIR (Y) | 6 | 31 | 32 | 12 | Right back STEP (Z) |
| Left front STEP (Y) | 13 | 33 | 34 | GND | |
| Left back STEP (X) | 19 | 35 | 36 | 16 | Right back DIR (Z) |
| Left back DIR (X) | 26 | 37 | 38 | 20 | |
| | GND | 39 | 40 | 21 | |

The magnetometer shares the IMU's I2C bus (SDA/SCL).

## Machines and network

| Machine | Address | Role |
|---|---|---|
| Pi, `pi-car` | 192.168.68.78, WiFi | Hardware nodes, run at boot by the `car_s` service. Workspace `~/ros_ws` |
| PC, `Ubuntu-Beelink` | 192.168.68.31, wired | EKF, SLAM, Nav2, exploration, RViz, `key_cmd`. AMD Ryzen AI 9 HX 470 (24 threads), Ubuntu 26.04, ROS in a container (see [Setting up the PC](#setting-up-the-pc)). Workspace `~/ros2_ws` |
| Old PC, `Ubuntu-ROS` | 192.168.68.82 | A 4-core VM with the same setup. Now too slow for the Nav2 settings |

- Both machines use ROS 2 Lyrical (the PC through
  [tools/lyrical](tools/lyrical)) and **Cyclone DDS**
  (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, set in `tools/lyrical` and
  `systemd/car_s.service`), with `ROS_DOMAIN_ID` unset (0). Keep the same ROS
  release and RMW on every machine: a node on another RMW doesn't see the
  car at all. With Lyrical's default, Fast DDS, a process sometimes never
  saw another's services: explore_lite and the CLI couldn't reach Nav2
  (2026-10-23, on the real car and in about half the simulator runs). Five
  simulator explorations in a row then worked on Cyclone. A plain shell on
  the Pi gets it from `~/.bashrc` (see [4. Workspace](#4-workspace)).
- **Run the PC stack on one machine only.** Two EKFs or two SLAM nodes fight
  over the car's TF.
- **SSH to the car:** `ssh graham@192.168.68.78` (key login from the
  Beelink and the old VM).
- **Clocks:** both machines must be NTP-synced (`timedatectl` shows
  `System clock synchronized: yes`), or TF lookups between them fail. After a
  cold boot, the Pi's clock can run tens of milliseconds fast until
  `systemd-timesyncd` catches up. `sudo systemctl restart systemd-timesyncd`
  on the Pi speeds this up.
- **Firewall:** the Beelink runs `ufw`, which denies incoming traffic by
  default. It has a rule allowing all UDP from the car:
  `sudo ufw allow proto udp from 192.168.68.78`. A port range isn't enough:
  Fast DDS also uses unicast ports in the 7000s, and with those blocked,
  topics work but `ros2 node list` and parameter calls don't. Any other
  machine that needs to talk ROS to the Beelink needs its own rule.

  It also needs one for its own address,
  `sudo ufw allow proto udp from 192.168.68.31`. Cyclone DDS (unlike Fast
  DDS, which used shared memory) finds the nodes on the same machine by
  multicast, which loops back with the Beelink's own address as the source.
  Without the rule, the firewall dropped it: the Beelink's own nodes didn't
  find each other (explore_lite couldn't reach Nav2, the launch couldn't
  configure slam_toolbox), while everything to and from the car worked.
  `sudo dmesg | grep "UFW BLOCK"` shows such drops. The simulator wasn't
  affected: on `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` it uses the loopback
  interface, which ufw always allows.

## Nodes and topics

| Executable | Runs on | What it does |
|---|---|---|
| `car_s_controller_accel` | Pi | Drives the steppers from `cmd_vel` with an acceleration ramp, publishes odometry |
| `car_imu` | Pi | Reads the MPU6050 at 100 Hz, publishes `imu/data_raw` |
| `car_mag` | Pi | Reads the QMC5883L at 10 Hz, publishes `imu/mag` |
| `rplidar_composition` | Pi | The lidar driver (`rplidar_ros`), publishes `scan` |
| `robot_state_publisher` | Pi | Publishes the URDF's fixed frames |
| `car_power` | Pi | `/car/shutdown` and `/car/reboot` services, to turn the Pi off cleanly |
| `key_cmd` | PC | Keyboard teleop |
| `imu_normalize` | Either | Prints running averages of `imu/data_raw`, for calibration |
| `car_guard` | PC | Started by `pc_bringup.launch.py`: keeps the car's TF off for the EKF, the lidar spinning, and enables the motors, again after a car restart |

The controller ramps each wheel's speed towards its target at `motor_accel`
m/s² (default 1.0) rather than jumping straight to it, so the steppers don't
slip or stall when a high speed is commanded.

### Controller topics

Subscribed:

| Topic | Type | Meaning |
|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | `linear.x` in m/s, `angular.z` in rad/s (positive = anticlockwise). Unstamped |
| `motor_on` | `std_msgs/Bool` | Enables or disables the stepper drivers. **The motors stay disabled until this is set true**, and `cmd_vel` is ignored while they're off |
| `motor_mode` | `std_msgs/Int32` | 1 = continuous (keep going until told otherwise), 2 = auto (stop `cmd_vel_timeout`, 0.3 s, after the last `cmd_vel`, the default), 3 = turn 360 (stop after one full spin) |
| `motor_steps` | `std_msgs/Int32` | Microsteps: 1, 2, 4, 8, 16 or 32 (default 32) |
| `motor_accel` | `std_msgs/Float32` | Acceleration ramp in m/s², 0 = no ramp |

Published: `odom` (`nav_msgs/Odometry`, with covariances), TF `odom` →
`base_link`, `left_speed` and `right_speed` (m/s), `angle` (heading in
degrees).

Parameters (all can be changed while it runs, with
`ros2 param set /car_controller <name> <value>`):

| Parameter | Default | Meaning |
|---|---|---|
| `publish_odom_tf` | `true` | Publish the TF `odom` → `base_link`. `pc_bringup.launch.py` turns it off while its EKF publishes that TF instead, and back on when it stops |
| `wheel_separation` | `0.262` | Effective distance between left and right wheels (m), calibrated against the gyro. Skid steer needs more than the physical 0.19 m |
| `m_per_revolution` | `0.2016` | Distance travelled per wheel revolution (m), calibrated by tape measure (nominal 0.21) |
| `cmd_vel_timeout` | `0.3` | In auto mode, stop this long (s) after the last `cmd_vel`. Long enough for WiFi jitter |
| `max_wheel_speed` | `0.5` | Cap on each wheel's speed (m/s). Above it, both wheels are scaled down together, keeping the turn. The steppers stall above about this, while odometry would still count the commanded steps |

Safety limits that aren't parameters:

- In continuous and turn-360 modes, the car still stops if no `cmd_vel`
  arrives for 15 s, e.g. when WiFi drops.
- A timeout stop decelerates at 1 m/s² or more, whatever `motor_accel` is.
- `cmd_vel` with NaN or infinite values stops the car; `motor_accel` must be
  0 (no ramp) to 5 m/s².
- Changing microsteps while moving keeps the speed: the step rate is set
  again, and steps already counted are converted at the old setting.

Values set with `ros2 param set` are lost when the node restarts. To keep
them, change the defaults in
[src/pi_car_s_controller_accel.cpp](src/pi_car_s_controller_accel.cpp)
(and `WHEEL_SEPARATION` in [src/key_cmd.cpp](src/key_cmd.cpp)).

### Frames

`odom` → `base_link` comes from the controller, or from the EKF on the PC
while `pc_bringup.launch.py` runs. `robot_state_publisher` adds
`base_link` → `imu_link`, `laser_link` (turned 180°) and the wheel links
from [urdf/robot_car.urdf.xml](urdf/robot_car.urdf.xml). The wheel joints are
`fixed`, because nothing publishes `joint_states`. With SLAM or AMCL running,
they add `map` → `odom`.

### IMU

`car_imu` calibrates the gyro bias for its first 2 seconds, so **keep the
car still while it starts** (including at boot, and whenever the `car_s`
service restarts). If it moves during that time, the node logs a warning
and tries again, up to 5 times, then uses the built-in offsets. The
`rate_hz` (default 100) and `calibration_time` (default 2.0 s, 0 to skip)
parameters change this. A 44 Hz low-pass filter is set in the MPU6050,
which keeps the gyro noise around 4e-7 (rad/s)².

At start-up it checks the chip answers (WHO_AM_I), and exits if not, so
the launch restarts it. If the readings freeze (all seven values identical
for 0.5 s, e.g. after a brown-out puts the chip to sleep), it stops
publishing them and sets the chip up again.

## How localisation works

The car is a **skid steer**: four fixed wheels and no steering, so it skids
sideways whenever it turns. **Its odometry is also open loop**: there are no
wheel encoders, and `/odom` comes from the step pulses the controller sends
to the motors (the rate set, multiplied by time). So `/odom` reports what was
commanded, not what the car did:

- Pushing the car by hand doesn't move `/odom`, while spinning the wheels
  off the ground does.
- Skids, stalls and missed steps go unnoticed.
- The distance is good, but rotation from the wheels is poor.

The localisation is built around that:

- **EKF** ([config/ekf.yaml](config/ekf.yaml), `robot_localization` on the
  PC): takes only the forward and sideways speed from `/odom`, and the yaw
  rate from the gyro, so it never uses the wheels' idea of rotation. It
  publishes `odom` → `base_link` at 30 Hz.
- **SLAM** ([config/slam.yaml](config/slam.yaml), `slam_toolbox`): scan
  matching corrects the drift and publishes `map` → `odom`. The lidar range
  is capped at 8 m, about what the A1 manages reliably indoors.
- **AMCL** on a saved map does the same job as SLAM, without changing the map.
- **Not used:** the magnetometer. It reads about 108 µT, roughly twice the
  Earth's field, because of the steppers, and it's uncalibrated. A Madgwick
  orientation filter isn't needed either, since the EKF uses the gyro rate
  directly.

## Setting up the Pi from scratch

### 1. Operating system

Install Ubuntu 26.04 for Raspberry Pi (24.04 works too, with ROS 2 Jazzy),
then set these in `/boot/firmware/config.txt` and reboot:

```
dtparam=audio=off
dtparam=i2c_arm=on
enable_uart=0
```

Audio must be off because it uses the PWM hardware that drives the motors.
Leave SPI off too (no `dtparam=spi=on`): SPI0's pins, GPIO 9, 10 and 11, are
the stepper drivers' microstep lines M0–M2, and anything opening the SPI
device would toggle them mid-drive. (Turned off on the car on 2026-10-21; an
SPI sensor would need the microstep lines moving first.) The UART must be
off because its TX pin, GPIO 14, is the stepper drivers' ENABLE. Ubuntu
26.04's image turns the UART on, along with audio and SPI.

Add your user to the `dialout` and `i2c` groups, for the lidar's serial
port, the GPIO and the I²C bus without `sudo` (log out and back in
afterwards; on Ubuntu 24.04 `dialout` covers I²C too):

```bash
sudo usermod -aG dialout,i2c $USER
```

### 2. ROS 2 Lyrical

Follow the [ROS 2 Lyrical Ubuntu install guide](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html),
installing `ros-lyrical-ros-base` (no desktop is needed on the Pi), then the
build tools. (On Ubuntu 24.04, use Jazzy: `jazzy` in place of `lyrical`
throughout.)

```bash
sudo apt install build-essential python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

### 3. GPIO, PWM and I²C tools

The controller needs libgpiod (version 1 on Ubuntu 24.04, 2 on 26.04: it
builds against either), and the PWM set-up compiles a device-tree overlay:

```bash
sudo apt install libgpiod-dev gpiod device-tree-compiler i2c-tools
```

The `dialout` and `i2c` groups (step 1) give access to `/dev/gpiochip0` and
`/dev/i2c-1`. (`/dev/gpiomem`, which `tools/pulse_count.py` reads, is
root-only on Ubuntu, so that tool needs `sudo`.) `i2cdetect -y 1` should show the IMU at `68` and the
magnetometer at `0d`.

### 4. Workspace

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone https://github.com/grahamjonesgs/car_s.git
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# Fix for the lidar driver (see patches/README.md)
git -C rplidar_ros apply ~/ros_ws/src/car_s/patches/rplidar_ros.patch

cd ~/ros_ws
source /opt/ros/lyrical/setup.bash
rosdep install -i --from-paths src --rosdistro lyrical -y
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2
```

The limited parallelism stops the Pi running out of memory; the first build
takes a while. `rplidar_ros` is built from source rather than installed
from apt, where Lyrical has no package for it. It's Slamtec's own driver
(the car used the older allenh1 fork until 2026-10-22). (The car used to
build `imu_tools` for a Madgwick filter too; nothing uses it now.) The
patch was made against Slamtec's `ros2` branch at commit `24cc9b6`.

Add these to the end of `~/.bashrc`:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Cyclone DDS (`sudo apt install ros-lyrical-rmw-cyclonedds-cpp`, also on the
PC) is what every machine uses; see [Machines and network](#machines-and-network).

### Updating

```bash
cd ~/ros_ws/src
git -C car_s pull
cd ~/ros_ws && MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2 --packages-select car_s
sudo systemctl restart car_s     # keep the car still while the gyro calibrates
```

`rplidar_ros` carries a local patch, so pull it only on purpose, then
reapply the patch if needed. After an `apt upgrade` that updates ROS
packages, rebuild the whole workspace.

### 5. Start the car at boot

[systemd/car_s.service](systemd/car_s.service) runs
`ros2 launch car_s robot_car.launch.py` at boot, once WiFi has an address, and after giving NTP up to 30 s to set the clock (the Pi
has no clock battery). Edit `User`, `HOME` and the paths if your user isn't
`graham`.

Before starting, the service runs `tools/car_pwm_setup.sh` as root to set up
the kernel PWM (see [How the Pi drives the hardware](#how-the-pi-drives-the-hardware)).

The kernel PWM keeps running after the controller dies, so if
the controller crashes, [launch/robot_car.launch.py](launch/robot_car.launch.py)
runs `car_outputs_off` ([tools/car_outputs_off.sh](tools/car_outputs_off.sh))
straight away: it stops the step pulses and disables the drivers (43 ms
after a `kill -9` in testing). Then the whole launch stops, the service's
`ExecStopPost` runs the same script again as a backstop, and the service
restarts after 5 s (`Restart=always`, since `ros2 launch` exits 0 even then). The lidar, IMU and magnetometer nodes restart by
themselves if they die. The IMU recalibrates the gyro when it restarts.

After changing the service file, copy it again and run
`sudo systemctl daemon-reload`.

```bash
sudo cp ~/ros_ws/src/car_s/systemd/car_s.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now car_s
```

The motors stay disabled until `motor_on` is sent, so it's safe to leave
running.

| Task | Command |
|---|---|
| Watch the output | `journalctl -u car_s -f` |
| Stop it, to run things by hand | `sudo systemctl stop car_s` |
| Start it again | `sudo systemctl start car_s` |
| Restart it after a rebuild | `sudo systemctl restart car_s` |
| Stop it starting at boot | `sudo systemctl disable car_s` |

Stop the service before running the launch file or any car node by hand.
Two controllers on the Pi fight over the GPIO pins.

## Setting up the PC

The PC needs the same ROS release as the car: Lyrical since 2026-10-22,
installed natively on the Beelink (Ubuntu 26.04) and run through
[tools/lyrical](tools/lyrical) (see
[ROS 2 Lyrical on the PC](#ros-2-lyrical-on-the-pc)). Every `ros2` command in
this README works as `tools/lyrical ros2 ...` from the repo folder.

Jazzy, which the car ran before, doesn't support Ubuntu 26.04, so on the
Beelink it lives in a distrobox container called `jazzy` (Ubuntu 24.04),
kept as a fallback for a car on Jazzy. On a PC with Ubuntu 24.04, install
Jazzy natively and skip the container part.

### The container (Beelink)

```bash
sudo apt install distrobox podman
distrobox create --name jazzy --image docker.io/library/ubuntu:24.04
distrobox enter jazzy
# Inside the container: install ROS 2 Jazzy from the install guide, then
sudo apt install ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization ros-jazzy-slam-toolbox ros-jazzy-tf2-tools \
  ros-jazzy-xacro ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions python3-rosdep
```

The container shares the host's home folder, network, display and GPU. So
`~/ros2_ws` is the same on both sides, the car sees ROS traffic as coming
from the Beelink itself, and RViz opens as a normal window.

`~/.bashrc` is shared too, so it only sources ROS where ROS exists, i.e.
inside the container. Put this at the end (the Beelink's original is saved
as `~/.bashrc.before-ros`):

```bash
# ROS 2 Jazzy lives in the "jazzy" distrobox container. This file is shared
# by the host and the container, so only source ROS where it exists.
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
  [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
fi
```

**From a host shell (including a VS Code terminal), use
[tools/jazzy](tools/jazzy).** It runs a command in the container, with ROS
and `~/ros2_ws` sourced, in the current folder. Every `ros2` command in this
README can be run that way from the repo folder:

```bash
cd ~/ros2_ws/src/car_s
tools/jazzy ros2 topic list
tools/jazzy ros2 launch car_s pc_bringup.launch.py
tools/jazzy                  # no command: an interactive shell in the container
```

Don't use `distrobox enter jazzy -- bash -lc '...'`: Ubuntu's `.bashrc`
returns early in non-interactive shells, so ROS isn't sourced.

### Workspace

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/grahamjonesgs/car_s.git
git clone https://github.com/robo-friends/m-explore-ros2.git
git -C m-explore-ros2 apply ~/ros2_ws/src/car_s/patches/m-explore-ros2.patch

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash      # inside the container
colcon build --symlink-install --packages-select car_s explore_lite_msgs explore_lite
```

Without libgpiod on the PC, `car_s` doesn't build the controller there.
`key_cmd` and `imu_normalize` are what the PC needs, plus the launch files and
configs; `car_imu` and `car_mag` build too but are only useful on the Pi.
`explore_lite` isn't in apt for Jazzy. The patch was made against
`m-explore-ros2` commit `326cf8a`, and fixes three problems found on the real
car (see [patches/README.md](patches/README.md)).

Then add the firewall rule for the car (see
[Machines and network](#machines-and-network)), and check that
`ros2 topic list` shows the car's topics while it runs.

### ROS 2 Lyrical on the PC

ROS 2 Lyrical supports Ubuntu 26.04, so on the Beelink it's installed
natively, beside the Jazzy container (from the ROS install guide, with the
same packages as above but `ros-lyrical-*`, plus `libgpiod-dev gpiod`). Its
build is kept apart from the Jazzy one, in `build_lyrical` and
`install_lyrical`, and [tools/lyrical](tools/lyrical) runs a command with it
sourced, as `tools/jazzy` does for the container:

```bash
cd ~/ros2_ws
src/car_s/tools/lyrical colcon --log-base log_lyrical build --symlink-install \
    --build-base build_lyrical --install-base install_lyrical \
    --packages-select car_s explore_lite_msgs explore_lite
cd src/car_s
tools/lyrical tools/sim_test.sh explore:=true motors_on:=true
```

Use the one that matches the car: the two versions don't reliably talk to
each other. Nav2 1.5 (Lyrical) changed a few things, which the launch file
and `config/nav2.yaml` handle so that one copy works with both:

- Its `navigation_launch.py` no longer starts a lifecycle manager, so
  `pc_bringup.launch.py` starts one (`lifecycle_manager_nav2`) when it
  doesn't.
- MPPI's motion model is a plugin (`DiffDrive.plugin`), and path pruning
  moved to the controller server (`PathHandler`). Jazzy ignores both.
- `bt_navigator` refuses the old `error_code_names`, so it's left at each
  version's default, and the new `following_server` needs unstamped
  `cmd_vel` like the other servers.
- Lyrical's MPPI got stuck with `vx_std: 0.1`. With Nav2's 0.2, and the goal
  heading only counted within the 0.25 m goal tolerance, both versions
  reach all the test goals in the simulator. Lyrical was still sometimes
  slow to finish beside a goal, where only the spin recovery got it out.
- Lyrical's MPPI checks its chosen trajectory before using it, by default
  rejecting it if the car's centre comes within the inscribed radius of
  anything in the next 2 s. In tight spaces that rejected safe trajectories
  and the car stalled ("Optimizer fail to compute path"), on the real car
  too. It now checks the real footprint, 1 s ahead (`TrajectoryValidator`).
- The local costmap's inflation is now 0.30 m with a cost scaling of 5.0
  (was 0.40 m and 3.0): with the wider one, a narrow but passable gap cost
  so much that MPPI chose to stand still. Both versions pass the simulator
  goal and bump tests with it.

## Running the car

### On the Pi

With the `car_s` service enabled, the full robot is already running after
boot, and you can go straight to the PC. Otherwise, start it by hand:

```bash
ros2 launch car_s robot_car.launch.py     # motors, IMU, magnetometer, lidar, robot_state_publisher
ros2 launch car_s robot_car.launch.py lidar:=false   # the same without the lidar
```

### Shutting down the Pi

Turn the Pi off cleanly from any machine before switching off the battery:

```bash
ros2 service call /car/shutdown std_srvs/srv/Trigger
ros2 service call /car/reboot std_srvs/srv/Trigger
```

`car_power` ([scripts/car_power.py](scripts/car_power.py)) first checks
that `sudo` will run the command, and refuses (the Pi keeps running) if
not. Then it disables the motors, replies, keeps them disabled, and 2 s
later runs `sudo systemctl poweroff` (or `reboot`), so systemd stops the
`car_s` service first. Wait for the Pi's green activity LED to stop flashing
before cutting the power.

It needs passwordless `sudo` for those two commands only. If the Pi user
doesn't already have passwordless `sudo` for everything, add a rule:

```bash
echo "$USER ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff, /usr/bin/systemctl reboot" | sudo tee /etc/sudoers.d/car_power
sudo chmod 440 /etc/sudoers.d/car_power
sudo visudo -c        # checks the syntax
```

### Lidar motor

With `robot_car.launch.py`, the lidar motor is **stopped by default**: it
spins up briefly while the node starts, then stops. No `/scan` messages are
published while it's stopped. `pc_bringup.launch.py` starts it, and stops it
again when it exits. Otherwise, start and stop it with `l` in `key_cmd`, or:

```bash
ros2 service call /start_motor std_srvs/srv/Empty
ros2 service call /stop_motor std_srvs/srv/Empty
```

To leave it spinning from start-up, launch with
`ros2 launch car_s robot_car.launch.py lidar_on:=true`.

### Driving from the PC

```bash
ros2 run car_s key_cmd
```

Press `m` first to enable the motors. Lift the wheels off the ground the first
time after any change to the motion code.

| Key | Action |
|---|---|
| `m` | Motors on / off |
| ↑ / ↓ | Forward / backward |
| ← / → | Turn on the spot anticlockwise / clockwise |
| `8` / `9` | Drive forward turning left / right |
| Space | Stop |
| `f` / `s` | Speed up / slow down by 0.1 m/s (`f` also drives forward). Starts at 0.2 m/s |
| `z` / `x` | Acceleration ramp up / down by 0.1 m/s² |
| `v` / `c` | Acceleration ramp to 1.0 / off |
| `0`–`5` | Microsteps 1, 2, 4, 8, 16, 32 |
| `l` | Lidar motor on / off (starts off) |
| `q` or Ctrl-C | Quit. If it last sent a move, it sends a stop first |

In the default auto mode, the car stops 0.3 s after the last command, and
`key_cmd` sends one command per key press, so **hold the key down** to keep
moving. To keep moving after a single press, switch to continuous mode:

```bash
ros2 topic pub --once /motor_mode std_msgs/msg/Int32 "{data: 1}"
```

Other useful commands:

```bash
ros2 topic pub --times 3 /motor_on std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /motor_accel std_msgs/msg/Float32 "{data: 0.5}"
ros2 topic echo /odom --field pose.pose.position
ros2 topic hz /scan /odom /imu/data_raw
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames        # expect map -> odom -> base_link -> imu_link, laser_link, wheels
```

### RViz

```bash
ros2 launch car_s rviz.launch.py fixed_frame:=odom
```

This loads [rviz/car_s.rviz](rviz/car_s.rviz): the robot model, TF, lidar
scan, EKF odometry, the map and Nav2's costmaps and paths, and the
**2D Pose Estimate** and **Nav2 Goal** tools. Use `fixed_frame:=odom` when
nothing publishes a map (just driving with `key_cmd`). Leave the default
(`map`) when SLAM or AMCL is running, but then nothing appears until the
`map` frame exists. `pc_bringup.launch.py` opens RViz itself, with the right
frame.

## Mapping, navigation and exploration

[launch/pc_bringup.launch.py](launch/pc_bringup.launch.py) runs everything on
the PC: the EKF, `slam_toolbox` or AMCL, Nav2, `explore_lite` and RViz, with
the parameters in [config/](config/). Its `car_guard` node
([scripts/car_guard.py](scripts/car_guard.py)) also:

- turns off the controller's TF while the EKF runs, and checks it stays off
- starts the lidar motor, and again whenever scans stop for 5 s
- with `motors_on:=true`, enables the motors once the controller is listening

If the car restarts during a session (a crash, `/car/reboot`), `car_guard`
sets up the TF and lidar again, but leaves the motors off and logs an error:
enable them with `m` in `key_cmd` when it's safe.

When you stop it with Ctrl-C, it disables the motors (if it enabled them),
gives the controller its TF back and stops the lidar.

| Task | Command |
|---|---|
| Build a map, driving with `key_cmd` | `ros2 launch car_s pc_bringup.launch.py` |
| Save the map (before stopping the launch) | `ros2 run nav2_map_server map_saver_cli -f ~/maps/room` |
| Save the SLAM model too, to extend the map later | `ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/maps/room'}"` |
| Carry on mapping from a saved SLAM model | `ros2 launch car_s pc_bringup.launch.py slam_map:=$HOME/maps/room` (add `explore:=true motors_on:=true` to explore) |
| Navigate on a saved map | `ros2 launch car_s pc_bringup.launch.py slam:=false nav2:=true motors_on:=true map:=$HOME/maps/room.yaml` |
| Map and navigate at the same time | `ros2 launch car_s pc_bringup.launch.py nav2:=true motors_on:=true` |
| Explore and map by itself | `ros2 launch car_s pc_bringup.launch.py explore:=true motors_on:=true` |
| EKF only, no SLAM or RViz | `ros2 launch car_s pc_bringup.launch.py slam:=false rviz:=false` |

Maps are saved outside the repo, in `~/maps` on the PC.

### Mapping by hand

Drive slowly (0.1–0.15 m/s) with gentle turns, and pause after turns. Go back
over places you've already been, and finish where you started, so SLAM can
close loops. If walls appear doubled or smeared, slow down and drive back
over that area.

### Continuing a saved map

`map_saver_cli` saves the map image (`.pgm` + `.yaml`), which is what AMCL
navigates on. To add to a map later, also save slam_toolbox's own model with
`serialize_map` (`.posegraph` + `.data`), as in the table. Then
`slam_map:=$HOME/maps/<name>` loads it and carries on mapping, with no
extension on the name. Save both again at the end, and the image afterwards
matches the model.

slam_toolbox assumes the car starts where the saved mapping started, facing
the same way. If it's elsewhere, give its pose on the saved map with
`slam_map_pose:="x y yaw"` (metres and radians, in the map frame). Hover over
the spot in RViz to read the coordinates.

### Navigating

With a saved map, AMCL assumes the car starts at the map's origin (where
mapping started), facing the same way. Anywhere else, set its pose with
**2D Pose Estimate** in RViz first. Until AMCL has a pose there's no `map`
frame, so the planner can't start and goals are rejected. Then click
**Nav2 Goal**s. Keep `key_cmd` open: `m` stops the motors.

The Nav2 settings ([config/nav2.yaml](config/nav2.yaml)) are adapted for this
car:

- **Base frame:** `base_link`, with odometry from the EKF's
  `/odometry/filtered`.
- **Speed:** unstamped `cmd_vel`, up to 0.2 m/s and 1.0 rad/s, with gentle
  acceleration so the steppers don't skip.
- **Controller:** MPPI, with the default 2000 samples × 56 steps. That's too
  heavy for the old 4-core VM, which needed 1000 × 40.
- **Planner:** Smac 2D. NavFn often failed to plan into unknown space, which
  is where exploration sends the car.
- **Costmaps:** the car's measured rectangle as the footprint (0.12 m ahead,
  0.17 m behind, 0.11 m each side), which MPPI checks as it steers
  (`consider_footprint: true`): it's narrower than the old 0.17 m circle at
  the sides, but the rear corners reach further. The global costmap, which the
  planner uses, has 0.25 m inflation: with 0.35 m, the margins round
  furniture legs merged and trapped the car. The local costmap, which the
  controller steers by, has a wider, slower-fading 0.40 m margin, so the car
  keeps further from chair legs as it passes them without the planner losing
  the gaps. In the simulator, the closest approach to the box went from
  0.36 m to 0.44 m.
- **Timeouts:** longer transform tolerances for WiFi, and
  `default_server_timeout: 100` so goals don't abort under load. The
  velocity smoother stops repeating the last command after 0.3 s
  (`velocity_timeout`), so a stalled Nav2 can't hide behind it from the
  car's own 0.3 s timeout.
- Nav2 runs composed in one process.
- **The map subscription depends on the mode,** and `pc_bringup` sets it.
  slam_toolbox publishes `/map` as volatile, and a transient-local subscriber
  never receives it, which left the global costmap empty. `map_server`
  publishes once, as transient local, so it needs a transient-local
  subscriber.

### Exploring

`explore:=true` runs `explore_lite`, with its settings in
[config/explore.yaml](config/explore.yaml). It sends Nav2 goals to the edges
of the SLAM map (the frontiers) until none are left, then drives back to the
start. Each goal appears as "Sending goal (x, y)" in the launch terminal.
When it says "No frontiers found" or "All frontiers traversed", save the map.

- Clear low things (cables, shoes) off the floor, and keep the car away from
  stairs. Watch it, and stop it with `m` in `key_cmd` or Ctrl-C.
- A goal that takes more than 90 s is dropped, and so is one Nav2 aborts
  (Nav2 aborts when the car makes no progress).
- If it stops early with "All frontiers traversed" because goals failed (for
  example with the motors off), restart the launch. It doesn't retry dropped
  goals.

### Stuck, slipping or pushed

The wheel odometry counts the steps sent to the motors, so it can't tell
when the wheels turn without moving the car (stuck on something, slipping,
lifted off the floor) or when the car is pushed.
[scripts/motion_check.py](scripts/motion_check.py), started by
`pc_bringup.launch.py` (`motion_check:=false` turns it off), compares it
over the last second with what the car really did: the latest lidar scan
matched against one from a second earlier, and the gyro. It publishes the
result on `/motion_check`:

| State | Meaning |
|---|---|
| `idle` | Nothing commanded, nothing measured |
| `ok` | Moving about as commanded |
| `slipping` | Moving, but under 70% of what was commanded |
| `stuck` | Commanded to move, but under 30% of it |
| `pushed` | Moving without a command |
| `unknown` | The scans can't tell, e.g. a featureless corridor |

A state is only reported once it has lasted 1.5 s. Changes are logged in the
launch terminal ("Motion: stuck (moved 0.00 m of 0.15 m ...)"), as warnings
for stuck, slipping and pushed. The measured movement is on
`/motion_check/twist`, to plot against `/odom`.

Checked on the real car on 2026-10-23 with the wheels off the ground: driving
(0.01 m measured of 0.14 m commanded) and turning both showed `stuck`, and
standing still `idle`. Turning it by hand with the motors off showed
`pushed` for each quarter-turn (27° and 44° measured), and `idle` between
them. Driving on the floor (forward and back 0.4 m, turns of about 85°)
showed `ok` with the lidar and gyro agreeing with the odometry to about
2 cm and 1°, and no false alarms. A state is reported about 2.5 s after it
starts (1 s of movement, then the 1.5 s hold).

```bash
tools/lyrical ros2 topic echo /motion_check
```

### Low obstacles the lidar can't see

The lidar sees one horizontal slice, so a low chair leg is invisible to it,
and the car drives into it. When motion_check finds the car `stuck` while
driving forwards or backwards, it treats that as an obstacle the lidar
can't see:

1. It marks a row of points 8 cm beyond the footprint's front (or rear)
   edge, across its width, at the map pose from before the stuck: the
   odometry carries on counting while the wheels push, so the pose drifts
   on past the obstacle.
2. It holds the EKF at where the car stopped (on `/set_pose`), while the
   wheels still push and the car isn't turning. Otherwise Nav2 finds the
   car's own footprint inside the mark and gives up. The pose it holds is
   the map pose from before the drift, turned into odom with SLAM's
   map -> odom as it is then: SLAM corrects part of the drift itself.
3. The marks go to `bump_layer` in both costmaps (config/nav2.yaml), which
   only motion_check's `/motion_check/bumps` feeds and the lidar never
   clears, so Nav2 plans round them. They are republished every second, so a
   costmap clear doesn't lose them.

The log shows "Marked something the lidar can't see ahead at (x, y)". The
marks are kept beside the map in use, `<map>.bumps.yaml` (for `map:=` or
`slam_map:=`), and loaded with it next time. A new map keeps them in
`~/.ros/car_s_bumps.yaml`: copy that to `~/maps/<name>.bumps.yaml` when
saving the map as `<name>`. To forget them all (a false "stuck", say on a
ramp):

```bash
tools/lyrical ros2 service call /motion_check/clear_bumps std_srvs/srv/Empty
```

A mark is only made after about 2.5 s of pushing, and it is a row at the
car's edge, not the obstacle's shape. Stuck while turning on the spot
marks nothing, as which corner caught is unknown.

Checked in the simulator (a chair leg the simulated lidar doesn't see,
`hidden_leg`, see [Testing without the car](#testing-without-the-car)): the
car hits it, marks it within 5 cm, backs off, goes round it to the goal, and
comes back round it. Not yet tried on the real car.

## Calibration

### Sensor mounting

Checked on 2026-10-20:

- **IMU:** at rest, `linear_acceleration.z` is about +9.8. Turning the car
  anticlockwise gives a positive `angular_velocity.z`.
- **Lidar:** put an object about 30 cm in front of the car and run
  `python3 tools/scan_check.py`. It should appear at 0°, and a wall on the
  car's left at +90°. The sensor is mounted facing backwards, so `laser_joint`
  in the URDF has a yaw of π, and [tools/sim_car.py](tools/sim_car.py)
  matches it.
- If the motors ever drive mirrored, fix the `DIR` pin logic in the
  controller, not the URDF or Nav2.

### Odometry

`/odom` only counts commanded steps, so calibrate on the floor, against real
motion. [tools/odom_cal.py](tools/odom_cal.py) does the driving and the
sums:

```bash
# car running, lidar spinning (l in key_cmd), nothing else driving it
python3 tools/odom_cal.py check          # clearances, no driving
python3 tools/odom_cal.py forward 0.5    # also measure with a tape
python3 tools/odom_cal.py spin 2         # gyro gives the real angle
ros2 param set /car_controller m_per_revolution <new>
ros2 param set /car_controller wheel_separation <new>
```

Run each test again to confirm, then make the values the defaults in the
controller source (and `key_cmd`).

| Test (2026-10-20, this floor) | Result | New value |
|---|---|---|
| 0.5 m forward at 0.1 m/s | Tape 0.48 m, lidar 0.47 m | `m_per_revolution` 0.21 → 0.2016 |
| 2 turns at 0.5 rad/s | Gyro 636° for 720° commanded | `wheel_separation` 0.2337 → 0.265. Recheck: 719.5° commanded, 720.7° gyro |

Rechecked on 2026-10-22 after moving to the kernel PWM, as pigpio's
right-hand pulses had run about 1% short even at low rates, so the car now
turns a little more for the same command:

| Test (2026-10-22, another floor) | Result | New value |
|---|---|---|
| 0.5 m forward at 0.1 m/s | Odometry 0.505 m, lidar 0.508 m | `m_per_revolution` unchanged, 0.2016 |
| 2 turns at 0.5 rad/s | Gyro 731.9° for 721.3° from odometry | `wheel_separation` 0.265 → 0.2612. Recheck: 720.2° odometry, 717.9° gyro; default 0.262 |

At other speeds or on other floors, the slip may differ.

### IMU offsets

The gyro offsets are measured automatically at start-up (see [IMU](#imu)).
The accelerometer offsets, and the gyro fallback offsets, are hard-coded as
`NGx`…`NAz` near the top of [src/pi_car_imu.cpp](src/pi_car_imu.cpp) and
added to every reading. To recalibrate them:

1. Put the car on a level surface and keep it still.
2. Run `car_imu` with `-p calibration_time:=0.0` (so the built-in gyro
   offsets are used) and `ros2 run car_s imu_normalize`, and wait for the
   averages to settle (a few thousand readings).
3. The averages should be 0 for all gyro axes and accel X/Y, and 9.81 for
   accel Z. For each value, adjust the offset by the error: for example if
   gyro X averages 0.02, subtract 0.02 from `NGx`; if accel Z averages 9.75,
   add 0.06 to `NAz`.
4. Rebuild and repeat until the averages are close.

[tools/imu_stats.py](tools/imu_stats.py) prints the IMU's rate, bias and
noise.

## Testing without the car

[tools/sim_test.sh](tools/sim_test.sh) runs `pc_bringup.launch.py` against a
simulated car ([tools/sim_car.py](tools/sim_car.py)) in a 4 × 3 m room with a
box in it. It uses ROS domain 42, limited to localhost, so the real car is
never involved. Arguments pass through to `pc_bringup.launch.py`:

```bash
tools/sim_test.sh                                    # EKF + SLAM + RViz
tools/sim_test.sh nav2:=true motors_on:=true         # send goals with Nav2 Goal
tools/sim_test.sh explore:=true motors_on:=true      # explores the room in about 40 s
```

The simulated car can fake what the real odometry misses, for testing
`motion_check`: `ros2 param set /car_controller slip 1.0` makes it stuck
(0.5 is half the commanded movement), and `push_vx 0.1` pushes it forwards
at 0.1 m/s. `/odom` still reports the commanded movement, as on the real car.
`hidden_leg "0.8,-0.05"` puts a chair leg there that blocks the car but
isn't in the scans, for testing the bump marking (`""` removes it).

Like the real controller, the simulated car ignores `cmd_vel` until
`motor_on` is true, so Nav2 and exploration need `motors_on:=true` here too.

It prints the simulated car's true pose once a second ("TRUTH x y yaw"), for
comparison with the EKF, SLAM and AMCL. It tests the configuration and
wiring, not the real car's sensors or slip.

Other tools:

| Tool | Use |
|---|---|
| [tools/rosout_watch.py](tools/rosout_watch.py) | Print every node's log messages, e.g. Nav2 and `explore_lite`, from any terminal |
| [tools/scan_check.py](tools/scan_check.py) | Closest lidar return in each direction around the car |
| [tools/imu_stats.py](tools/imu_stats.py) | IMU rate, bias and noise |
| [tools/odom_cal.py](tools/odom_cal.py) | Odometry calibration on the floor |
| [tools/pulse_count.py](tools/pulse_count.py) | Step pulses per second on the four STEP pins, on the Pi (`sudo`) |
| [tools/kernel_pwm_test.sh](tools/kernel_pwm_test.sh) | The kernel PWM on its own at 3–10 kHz, on the Pi (stops `car_s` while it runs) |

## Troubleshooting

| Problem | Fix |
|---|---|
| Controller exits with "can't open /sys/class/pwm/…" | The kernel PWM isn't set up: `sudo tools/car_pwm_setup.sh` (the service does it at start), check with `sudo dtoverlay -l` |
| Controller exits with "can't open /dev/gpiochip0" or "can't use GPIO n as an output" | The user isn't in `dialout`, or another program holds the pin: `gpioinfo gpiochip0` shows which |
| Step pulses at wildly wrong rates (`tools/pulse_count.py`) | Something changed the PWM clock behind the kernel's back, such as pigpio's PWM: reboot |
| Motors don't move, and `journalctl -u car_s` says "Ignoring cmd_vel, motors are off" | Send `motor_on` true (`m` in `key_cmd`). In auto mode, hold the key down |
| No `/scan`, and the lidar node logs "cannot bind to the specified serial port" | The lidar isn't on USB. Check `lsusb` for the CP210x and `ls /dev/ttyUSB*`, then the cable (one failed). Then `sudo systemctl restart car_s` |
| No `/scan`, the lidar spins, and `rplidar_composition` uses 100% CPU | The driver has wedged, and service calls to it hang. It used to happen when `/start_motor` was called while it was already running; the patch stops that. Restart `car_s` |
| `car_imu` or `car_mag`: "Can't open /dev/i2c-1" or "No MPU6050/QMC5883L answering" | Check that `dtparam=i2c_arm=on` is set, the user is in `i2c` (`dialout` on Ubuntu 24.04), and the wiring: `i2cdetect -y 1` should show `68` and `0d` |
| The PC can't see the car's topics, or sees topics but not nodes or parameters | Same network and `ROS_DOMAIN_ID` on both, and the Beelink's firewall rule for the car (see [Machines and network](#machines-and-network)) |
| "Lookup would require extrapolation into the future" over and over | The clocks disagree: check `timedatectl` on both machines |
| RViz shows no lidar points with the fixed frame `map` | There's no `map` frame yet: with AMCL, set **2D Pose Estimate** |
| Nav2 goals do nothing, and RViz logs "Goal was rejected by server" | Nav2 isn't fully started (`ros2 lifecycle get /bt_navigator`). With AMCL, it needs a pose first |
| The car's TF is off after the PC stack died | `ros2 param set /car_controller publish_odom_tf true`, or restart `car_s` |
| Odd TF, drifting maps or doubled robots | Something from an earlier run is still running. Check `ros2 node list` for duplicates, and for the simulator, look for leftovers on domain 42 (see below) |
| Build killed on the Pi | Out of memory, use the `MAKEFLAGS=-j2 ... --parallel-workers 2` build command |

### Pitfalls when scripting

- **Signalling a background command from a script:** `cmd &` in a script
  starts it with SIGINT ignored, and if `cmd` is a shell function, `$!` is a
  subshell, not the program. Start it as `python3 -c 'import signal,os,sys;
  signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1],
  sys.argv[1:])' <program> <args> &` and signal `$!`.
- **Stale Fast DDS shared memory.** Processes killed hard leave files in
  `/dev/shm`, and once there were over a hundred, nodes on the same machine
  stopped finding each other (explore_lite waited for Nav2 for ever).
  `fastdds shm clean` (with ROS sourced) removes the unused ones;
  `tools/sim_test.sh` runs it at start.
- **A long-running `ros2` daemon** for the simulator's domain, left by CLI
  calls during many test runs, went with explore_lite failing to reach Nav2
  more often. Stop it (`pgrep -af "ros-domain-id 42"`, then `kill`); it's
  only a cache, and the next `ros2` command starts a new one.
- **`.warn()` in rclpy nodes:** Lyrical removed it; use `.warning()`, which
  Jazzy has too. One logging call can't switch severity between calls
  either.
- **Leftover processes.** A simulator `robot_state_publisher` left over from
  an interrupted run kept publishing an old URDF's TF on domain 42 a day
  later. Find leftovers with `pgrep -af robot_state_publisher`, and check
  their domain with `tr '\0' '\n' < /proc/<pid>/environ | grep ROS_DOMAIN_ID`.
- **`timeout tools/jazzy ...`** only kills the `distrobox` wrapper, not the
  ROS command inside the container. A timed-out `ros2 service call` stays
  waiting, and can fire much later when the service reappears.
- **Background jobs** in non-interactive shells start with SIGINT ignored, so
  `kill -INT` doesn't stop `ros2 launch`. SIGTERM kills it without cleanup,
  leaving its nodes running and the car's TF off. Reset SIGINT before
  exec'ing the launch. To run one in the background through `tools/jazzy`,
  keep it in the foreground of that command (`setsid --wait`, not `&`):
  background processes die when the `distrobox enter` session ends.
- **`pkill -f` and `pgrep -f` match their own shell** when the pattern
  appears anywhere in the command line.
- **`ros2 topic pub --once`** from a process that exits straight away can
  lose the message over WiFi. Use `--times 3`.
- slam_toolbox's `online_async_launch.py` defaults to `use_sim_time:=true`;
  `pc_bringup.launch.py` overrides it.

## Cautions

- **Only one controller at a time.** Never start `car_s_controller_accel`
  while another is running on the Pi: the second can't claim the pins and
  exits, and anything else driving the PWM upsets the first. Check with
  `ros2 node list` first.
- **Test new motion code with the wheels off the ground.**
- **Don't change these on the Pi:** `/boot/firmware/config.txt` must keep
  `dtparam=audio=off` (audio uses the motor PWM hardware), and SPI off (its
  pins are the microstep lines).
- **Pi builds:** use `MAKEFLAGS=-j2 ... --parallel-workers 2`.
