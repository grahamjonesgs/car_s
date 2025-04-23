from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare arguments
    declare_robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        default_value='robot_car',
        description='Name of the robot.'
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation time if true.'
    )

    # Get URDF file path
    package_name = 'my_robot_description'  # Replace with your package name
    urdf_path = os.path.join(
        FindPackageShare(package=package_name).find(package_name),
        'urdf',
        'robot_car.urdf.xml'  # Make sure this matches your URDF file name
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command(['xacro ', urdf_path, ' robot_name:=', LaunchConfiguration('robot_name')])
        }],
    )

    # Laser scanner node (RPLIDAR A1).  You might need to adjust the topic name and package.
    laser_scanner_node = Node(
        package='rplidar_ros',  # Or whatever package provides the RPLIDAR node
        executable='rplidar_node',
        name='laser_scanner',
        output='screen',
        parameters=[{
            'topic_name': '/scan',  # Adjust if necessary
            'frame_id': 'laser_link', # Important to match the URDF link name
            'angle_compensate': True,
        }],
    )

    #  Include your controller node, assuming it's in your package.
    picar_controller_node = Node(
        package='car_s',  # Replace with your package name
        executable='car_s_controller_accel', # Replace with your executable name
        name='car_controller',
        output='screen',
    )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(declare_robot_name_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(laser_scanner_node)
    ld.add_action(picar_controller_node)
    return ld