# Local patches to third-party packages

These packages are cloned and built from source next to `car_s`, with small
fixes that aren't upstream. Apply a patch after cloning with
`git apply <patch>` in the package's directory, then rebuild it.

| Patch | Package | Built on | Base commit | Fixes |
|---|---|---|---|---|
| [rplidar_ros.patch](rplidar_ros.patch) | [Slamtec/rplidar_ros](https://github.com/Slamtec/rplidar_ros), branch `ros2` (`~/ros_ws/src/rplidar_ros`) | Pi | `24cc9b6` (2.1.4 and later fixes, SDK 2.1) | `/start_motor` while the lidar is already running restarts the scan mid-stream, which wedged the driver. Now ignored. Also builds with ROS 2 Lyrical (no `ament_target_dependencies`, C++17). (Until 2026-10-22 the car used the [allenh1/rplidar_ros](https://github.com/allenh1/rplidar_ros) fork, 2.1.0, with the same fix) |
| [m-explore-ros2.patch](m-explore-ros2.patch) | [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) (`~/ros2_ws/src/m-explore-ros2`) | Beelink | `326cf8a` | See below |

The `explore_lite` changes:

- **Blacklist a frontier once its goal succeeds.** Otherwise a frontier that
  is still there after reaching it (a blind spot within Nav2's goal
  tolerance) is chosen again for ever, about 10 times a second.
- **Don't aim at a centroid that is next to the robot.** In open space seen
  from one spot, the frontier is a ring round the robot and its centroid is
  the robot itself. Then aim at the closest frontier cell at least 0.6 m away.
- **Keep the current goal** until Nav2 finishes or aborts it (or after
  3 × `progress_timeout`), instead of switching to the best-scoring frontier
  on every replan, which made the car dither between distant goals.
- Log each goal sent at INFO.
- **Exit if Nav2's action server can't be reached within 20 s**, so the
  launch file restarts it (`respawn` in `pc_bringup.launch.py`). On ROS 2
  Lyrical, in the simulator, about one start in three never saw the whole
  server, even with a new client, while a new process connected at once.
