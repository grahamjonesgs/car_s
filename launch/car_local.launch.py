# What runs on the car beside the hardware nodes, so that it doesn't wait on
# WiFi: the EKF, the collision monitor in front of the controller, and
# motion_check. Included by robot_car.launch.py, and by tools/sim_test.sh
# for the simulated car.
#
# All of them only use the car's own data (/odom, /imu/data_raw, /scan and
# its TF); Nav2, SLAM and RViz stay on the PC.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    car_s_dir = get_package_share_directory('car_s')
    ekf = LaunchConfiguration('ekf')
    safety_stop = LaunchConfiguration('safety_stop')
    motion_check = LaunchConfiguration('motion_check')

    args = [
        DeclareLaunchArgument(
            'ekf', default_value='true',
            description='Fuse wheel odometry and gyro with robot_localization, '
                        'which then publishes odom -> base_link'),
        DeclareLaunchArgument(
            'safety_stop', default_value='true',
            description='Run the collision monitor between Nav2 '
                        '(cmd_vel_smoothed) and the controller (cmd_vel)'),
        DeclareLaunchArgument(
            'motion_check', default_value='true',
            description='Compare the wheel odometry with the lidar and gyro, and '
                        'report on /motion_check when the car is stuck, slipping '
                        'or pushed'),
    ]

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(car_s_dir, 'config', 'ekf.yaml')],
        condition=IfCondition(ekf),
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='car_collision_monitor',
        output='screen',
        parameters=[os.path.join(car_s_dir, 'config', 'collision_monitor.yaml')],
        condition=IfCondition(safety_stop),
    )

    # The collision monitor is a lifecycle node
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_car',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['car_collision_monitor']}],
        condition=IfCondition(safety_stop),
    )

    # The wheel odometry counts commanded steps; this notices when the car
    # isn't doing what they say (scripts/motion_check.py). The bumps it marks
    # are kept by bump_store on the PC, beside the map
    motion_check_node = Node(
        package='car_s',
        executable='motion_check',
        name='motion_check',
        output='screen',
        condition=IfCondition(motion_check),
    )

    return LaunchDescription(args + [
        ekf_node,
        collision_monitor,
        lifecycle_manager,
        motion_check_node,
    ])
