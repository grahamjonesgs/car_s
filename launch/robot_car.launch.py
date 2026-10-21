from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, Shutdown)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    declare_lidar_arg = DeclareLaunchArgument(
        name='lidar',
        default_value='true',
        description='Run the lidar driver. false runs the car without it.'
    )

    declare_lidar_on_arg = DeclareLaunchArgument(
        name='lidar_on',
        default_value='false',
        description='Leave the lidar motor spinning at start-up. When false it is '
                    'stopped, and can be started with the /start_motor service.'
    )

    declare_publish_odom_tf_arg = DeclareLaunchArgument(
        name='publish_odom_tf',
        default_value='true',
        description='Publish the odom -> base_link TF from wheel odometry. The '
                    'PC launch turns it off at run time while its EKF runs.'
    )

    # Get URDF file path
    package_name = 'car_s'  # Replace with your package name
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
            # As a string: otherwise launch reads it as YAML, and a colon
            # followed by a space anywhere in the URDF breaks it
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_path, ' robot_name:=', LaunchConfiguration('robot_name')]),
                value_type=str)
        }],
    )

    # Laser scanner node (RPLIDAR A1).  You might need to adjust the topic name and package.
    laser_scanner_node = Node(
        package='rplidar_ros',  # Or whatever package provides the RPLIDAR node
        executable='rplidar_composition',
        name='laser_scanner',
        output='screen',
        parameters=[{
            'topic_name': '/scan',  # Adjust if necessary
            'frame_id': 'laser_link', # Important to match the URDF link name
            'angle_compensate': True,
        }],
        # Restart it if it dies (USB glitch); it comes back with its motor on
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('lidar')),
    )

    # The lidar node always spins up its motor, so stop it once the service is
    # available (ros2 service call waits for it) unless lidar_on is true
    stop_lidar_motor = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/stop_motor', 'std_srvs/srv/Empty'],
        output='log',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('lidar'), "'.lower() == 'true' and '",
            LaunchConfiguration('lidar_on'), "'.lower() != 'true'"])),
    )

    #  Include your controller node, assuming it's in your package.
    picar_controller_node = Node(
        package='car_s',  # Replace with your package name
        executable='car_s_controller_accel', # Replace with your executable name
        name='car_controller',
        output='screen',
        parameters=[{
            'publish_odom_tf': ParameterValue(
                LaunchConfiguration('publish_odom_tf'), value_type=bool),
        }],
        # If the controller dies, end the whole launch, so the car_s service
        # stops the step pulses (ExecStopPost) and restarts everything
        on_exit=Shutdown(reason='car_controller exited'),
    )

    picar_imu_node = Node(
        package='car_s',  # Replace with your package name
        executable='car_imu', # Replace with your executable name
        name='car_imu',
        output='screen',
        # It recalibrates the gyro on a restart, so best done standing still
        respawn=True,
        respawn_delay=2.0,
    )

    picar_mag_node = Node(
        package='car_s',  # Replace with your package name
        executable='car_mag', # Replace with your executable name
        name='car_mag',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # /car/shutdown and /car/reboot services
    car_power_node = Node(
        package='car_s',
        executable='car_power',
        name='car_power',
        output='screen',
    )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(declare_robot_name_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_lidar_arg)
    ld.add_action(declare_lidar_on_arg)
    ld.add_action(declare_publish_odom_tf_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(laser_scanner_node) # Add the laser scanner node.
    ld.add_action(stop_lidar_motor)
    ld.add_action(picar_controller_node)
    ld.add_action(picar_imu_node)
    ld.add_action(car_power_node)
    ld.add_action(picar_mag_node)
    return ld
