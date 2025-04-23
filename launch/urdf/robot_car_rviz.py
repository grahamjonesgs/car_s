from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare the argument for the RViz configuration file
    declare_rviz_config_file_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=os.path.join(
            FindPackageShare(package='car_s').find('car_s'), # Replace
            'rviz',
            'robot_car.rviz'
        ),
        description='Path to the RViz configuration file'
    )

    # Launch the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Create the launch description and add the nodes
    ld = LaunchDescription()
    ld.add_action(declare_rviz_config_file_arg)
    ld.add_action(rviz_node)
    return ld