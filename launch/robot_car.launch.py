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
from ament_index_python.packages import get_package_prefix

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

    declare_scan_mode_arg = DeclareLaunchArgument(
        name='scan_mode',
        default_value='Boost',
        description='Lidar scan mode: Boost (8 kHz, about 1080 points per scan) '
                    'or Express (4 kHz, about 720). Both turn at 7.5 Hz.'
    )

    declare_ekf_arg = DeclareLaunchArgument(
        name='ekf',
        default_value='true',
        description='Run the EKF on the car (launch/car_local.launch.py), which '
                    'publishes odom -> base_link. When false, the controller '
                    'publishes that TF from the wheel odometry instead.'
    )

    declare_safety_stop_arg = DeclareLaunchArgument(
        name='safety_stop',
        default_value='true',
        description='Run the collision monitor between Nav2 and the controller.'
    )

    declare_motion_check_arg = DeclareLaunchArgument(
        name='motion_check',
        default_value='true',
        description='Run motion_check (stuck, slipping or pushed).'
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
            'scan_mode': LaunchConfiguration('scan_mode'),
            # The A1 talks at 115200 baud; Slamtec's driver defaults to 1000000
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
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
            # Only one of the two may publish odom -> base_link
            'publish_odom_tf': ParameterValue(PythonExpression(
                ["'", LaunchConfiguration('ekf'), "'.lower() != 'true'"]), value_type=bool),
        }],
        # If the controller dies, the kernel PWM keeps running without it: stop the
        # step pulses and disable the drivers at once, then end the whole
        # launch, so the car_s service (whose ExecStopPost does the same
        # again) restarts everything
        on_exit=[
            ExecuteProcess(
                cmd=[os.path.join(get_package_prefix(package_name), 'lib',
                                  package_name, 'car_outputs_off')],
                output='screen'),
            Shutdown(reason='car_controller exited'),
        ],
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

    # The EKF, collision monitor and motion_check: on the car, so they don't
    # wait on WiFi
    car_local = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare(package=package_name).find(package_name),
            'launch', 'car_local.launch.py')),
        launch_arguments={
            'ekf': LaunchConfiguration('ekf'),
            'safety_stop': LaunchConfiguration('safety_stop'),
            'motion_check': LaunchConfiguration('motion_check'),
        }.items(),
    )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(declare_robot_name_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_lidar_arg)
    ld.add_action(declare_lidar_on_arg)
    ld.add_action(declare_scan_mode_arg)
    ld.add_action(declare_ekf_arg)
    ld.add_action(declare_safety_stop_arg)
    ld.add_action(declare_motion_check_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(laser_scanner_node) # Add the laser scanner node.
    ld.add_action(stop_lidar_motor)
    ld.add_action(picar_controller_node)
    ld.add_action(picar_imu_node)
    ld.add_action(car_power_node)
    ld.add_action(picar_mag_node)
    ld.add_action(car_local)
    return ld
