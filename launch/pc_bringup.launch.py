# PC side of the car: EKF, SLAM, Nav2 and RViz. The Pi runs
# robot_car.launch.py (the controller, sensors and robot_state_publisher).
#
#   Mapping:             ros2 launch car_s pc_bringup.launch.py
#   Mapping + Nav2:      ros2 launch car_s pc_bringup.launch.py nav2:=true
#   Nav2 on a saved map: ros2 launch car_s pc_bringup.launch.py slam:=false \
#                          nav2:=true map:=$HOME/maps/room.yaml
#   Explore and map by itself: ros2 launch car_s pc_bringup.launch.py \
#                          explore:=true motors_on:=true
#   Carry on mapping from a saved SLAM model (cave.posegraph + cave.data),
#   with the car where that mapping started:
#                        ros2 launch car_s pc_bringup.launch.py \
#                          slam_map:=$HOME/maps/cave
#
# While the EKF runs it publishes odom -> base_link, so this turns off the
# controller's own TF (publish_odom_tf) and turns it back on at shutdown.

import os
import subprocess
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

CONTROLLER_NODE = '/car_controller'

# Discovery of the Pi's nodes over WiFi can take a few seconds, so retry
RETRY = 'for i in $(seq 30); do {cmd} && exit 0; sleep 1; done; exit 1'


def restore_on_shutdown(context):
    """Give the controller back its TF, and stop the lidar, when this exits."""
    commands = []
    if LaunchConfiguration('ekf').perform(context).lower() == 'true':
        commands.append(['ros2', 'param', 'set', CONTROLLER_NODE,
                         'publish_odom_tf', 'true'])
    if LaunchConfiguration('start_lidar').perform(context).lower() == 'true':
        commands.append(['ros2', 'service', 'call', '/stop_motor',
                         'std_srvs/srv/Empty'])
    for cmd in commands:
        print(f'[pc_bringup] Running: {" ".join(cmd)}', flush=True)
        try:
            r = subprocess.run(cmd, timeout=10, check=False,
                               capture_output=True, text=True)
            print(f'[pc_bringup] Exit code {r.returncode}', flush=True)
        except subprocess.TimeoutExpired:
            print(f'[pc_bringup] Timed out running: {" ".join(cmd)}', flush=True)
    return []


def slam_params_file(context):
    """slam.yaml, plus loading a saved pose graph when slam_map is set."""
    path = os.path.join(get_package_share_directory('car_s'), 'config', 'slam.yaml')
    saved = os.path.expanduser(LaunchConfiguration('slam_map').perform(context))
    if not saved:
        return path
    saved = saved.removesuffix('.posegraph')
    if not os.path.exists(saved + '.posegraph'):
        raise RuntimeError(f'slam_map: {saved}.posegraph not found')
    with open(path) as f:
        params = yaml.safe_load(f)
    p = params['slam_toolbox']['ros__parameters']
    p['map_file_name'] = saved
    pose = LaunchConfiguration('slam_map_pose').perform(context).split()
    if pose:
        # Where the car is now on the saved map: x, y (m), yaw (rad)
        p['map_start_pose'] = [float(v) for v in pose]
        p['map_start_at_dock'] = False
    else:
        # Where the saved mapping started
        p['map_start_at_dock'] = True
    out = tempfile.NamedTemporaryFile('w', prefix='car_s_slam_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, out)
    out.close()
    print(f'[pc_bringup] SLAM continues from {saved} '
          f'({"at " + " ".join(pose) if pose else "where it started"})', flush=True)
    return out.name


def start_slam(context):
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_file(context),
            # Its default is true, which would wait for a simulator clock
            'use_sim_time': 'false',
        }.items(),
    )]


def generate_launch_description():
    car_s_dir = get_package_share_directory('car_s')
    nav2_dir = get_package_share_directory('nav2_bringup')

    ekf = LaunchConfiguration('ekf')
    slam = LaunchConfiguration('slam')
    nav2 = LaunchConfiguration('nav2')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')
    start_lidar = LaunchConfiguration('start_lidar')
    motors_on = LaunchConfiguration('motors_on')
    explore = LaunchConfiguration('explore')

    args = [
        DeclareLaunchArgument(
            'ekf', default_value='true',
            description='Fuse wheel odometry and gyro with robot_localization'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='Build a map with slam_toolbox'),
        DeclareLaunchArgument(
            'nav2', default_value='false',
            description='Run Nav2 navigation'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Map yaml to localise on with AMCL. Use with slam:=false'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz'),
        DeclareLaunchArgument(
            'start_lidar', default_value='true',
            description='Start the lidar motor (it is stopped after the Pi boots)'),
        DeclareLaunchArgument(
            'motors_on', default_value='false',
            description='Enable the stepper drivers, needed for Nav2 to drive. '
                        'Otherwise press m in key_cmd'),
        DeclareLaunchArgument(
            'explore', default_value='false',
            description='Explore with explore_lite, which sends Nav2 goals to '
                        'the edges of the SLAM map (turns Nav2 on)'),
        DeclareLaunchArgument(
            'slam_map', default_value='',
            description='Saved SLAM model to carry on mapping from, without '
                        'extension (saved with /slam_toolbox/serialize_map)'),
        DeclareLaunchArgument(
            'slam_map_pose', default_value='',
            description='With slam_map: the car\'s pose on the saved map, '
                        '"x y yaw" in m and rad. Default: where that mapping started'),
    ]

    # Exploration drives with Nav2
    nav2_on = PythonExpression(
        ["'", nav2, "'.lower() == 'true' or '", explore, "'.lower() == 'true'"])

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(car_s_dir, 'config', 'ekf.yaml'),
                    {'use_sim_time': False}],
        condition=IfCondition(ekf),
    )

    controller_tf_off = ExecuteProcess(
        cmd=['bash', '-c', RETRY.format(
            cmd=f'ros2 param set {CONTROLLER_NODE} publish_odom_tf false')],
        output='screen',
        condition=IfCondition(ekf),
    )

    lidar_on = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/start_motor', 'std_srvs/srv/Empty'],
        output='log',
        condition=IfCondition(start_lidar),
    )

    # Publish a few times: a single message from a process that exits
    # straight away can be lost over WiFi
    enable_motors = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-w', '1', '--times', '3', '--rate', '2',
             '/motor_on', 'std_msgs/msg/Bool', '{data: true}'],
        output='log',
        condition=IfCondition(motors_on),
    )

    slam_launch = OpaqueFunction(function=start_slam, condition=IfCondition(slam))

    # slam_toolbox publishes /map as volatile (republished every
    # map_update_interval), and a transient local subscriber never receives
    # it. map_server publishes once as transient local, so needs it
    nav2_params = RewrittenYaml(
        source_file=os.path.join(car_s_dir, 'config', 'nav2.yaml'),
        param_rewrites={'map_subscribe_transient_local': PythonExpression(
            ["'False' if '", slam, "'.lower() == 'true' else 'True'"])},
        convert_types=True)
    run_nav2 = PythonExpression(
        ["'", nav2_on, "' == 'True' or '", map_yaml, "' != ''"])

    # Nav2 runs composed in one process, which uses far less CPU than a
    # process per server
    nav2_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[nav2_params, {'autostart': True, 'use_sim_time': False}],
        output='screen',
        condition=IfCondition(run_nav2),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items(),
        condition=IfCondition(PythonExpression(["'", map_yaml, "' != ''"])),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items(),
        condition=IfCondition(nav2_on),
    )

    # explore_lite is only built on the PC, so it is looked up only when used
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[os.path.join(car_s_dir, 'config', 'explore.yaml')],
        condition=IfCondition(explore),
    )

    explore_needs_slam = LogInfo(
        msg='WARNING: explore:=true needs SLAM, but slam is false',
        condition=IfCondition(PythonExpression(
            ["'", explore, "'.lower() == 'true' and '", slam, "'.lower() != 'true'"])),
    )

    both_maps_warning = LogInfo(
        msg='WARNING: slam is on and a map is given, so both slam_toolbox and '
            'AMCL publish map -> odom. Use slam:=false with map:=...',
        condition=IfCondition(PythonExpression(
            ["'", slam, "'.lower() == 'true' and '", map_yaml, "' != ''"])),
    )

    # The map frame only exists with SLAM or AMCL running
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_s_dir, 'launch', 'rviz.launch.py')),
        launch_arguments={
            'fixed_frame': PythonExpression(
                ["'map' if '", slam, "'.lower() == 'true' or '", map_yaml,
                 "' != '' else 'odom'"]),
        }.items(),
        condition=IfCondition(rviz),
    )

    on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=restore_on_shutdown)]))

    return LaunchDescription(args + [
        both_maps_warning,
        controller_tf_off,
        ekf_node,
        lidar_on,
        enable_motors,
        slam_launch,
        nav2_container,
        localization_launch,
        navigation_launch,
        explore_needs_slam,
        explore_node,
        rviz_launch,
        on_shutdown,
    ])
