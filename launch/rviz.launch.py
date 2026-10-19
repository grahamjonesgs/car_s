# RViz for the car, with the car_s layout (rviz/car_s.rviz).
#
#   With SLAM or AMCL running:  ros2 launch car_s rviz.launch.py
#   Without a map (teleop/EKF): ros2 launch car_s rviz.launch.py fixed_frame:=odom
#
# pc_bringup.launch.py includes this, so it is only needed on its own when
# RViz is wanted without the rest of the PC stack.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    car_s_dir = get_package_share_directory('car_s')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fixed_frame', default_value='map',
            description='RViz fixed frame: map when SLAM or AMCL is running, '
                        'otherwise odom'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(car_s_dir, 'rviz', 'car_s.rviz'),
            description='RViz config file to load'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', LaunchConfiguration('rviz_config'),
                       '-f', LaunchConfiguration('fixed_frame')],
            parameters=[{'use_sim_time': False}],
        ),
    ])
