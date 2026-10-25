# PC side of the car: SLAM, Nav2 and RViz. The Pi runs robot_car.launch.py:
# the controller, sensors and robot_state_publisher, and next to them the
# EKF, the collision monitor and motion_check (launch/car_local.launch.py).
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
# car_guard (scripts/car_guard.py) keeps the lidar running and enables the
# motors (once: not again after a car restart). At shutdown the lidar is
# stopped and the motors disabled.

import importlib.util
import os
import subprocess

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

def restore_on_shutdown(context):
    """Stop the lidar, and disable the motors if this enabled them, when it
    exits."""
    commands = []
    if LaunchConfiguration('motors_on').perform(context).lower() == 'true':
        # First, so the car can't act on a last command from Nav2
        commands.append(['ros2', 'topic', 'pub', '-w', '1', '--times', '3',
                         '--rate', '10', '/motor_on', 'std_msgs/msg/Bool',
                         '{data: false}'])
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
    # One file, overwritten each time, rather than a new temporary file per run
    out_path = os.path.expanduser('~/.ros/car_s_slam_map.yaml')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w') as out:
        yaml.safe_dump(params, out)
    print(f'[pc_bringup] SLAM continues from {saved} '
          f'({"at " + " ".join(pose) if pose else "where it started"})', flush=True)
    return out_path


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


def start_lifecycle_manager(context, nav2_dir, nav2_on, map_yaml):
    # Nav2 1.5's navigation_launch.py and localization_launch.py don't start
    # a lifecycle manager: they list their nodes in get_lifecycle_nodes(),
    # and bringup_launch.py starts one manager for them all, as this does
    nodes = []
    if map_yaml.perform(context) != '':
        nodes += ['map_server', 'amcl']
    if nav2_on.perform(context) == 'True':
        spec = importlib.util.spec_from_file_location(
            'nav2_navigation_launch',
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        # The collision monitor runs on the car (car_collision_monitor), so
        # Nav2's own is left unconfigured, which leaves it doing nothing
        nodes += [n for n in module.get_lifecycle_nodes(context)
                  if n != 'collision_monitor']
    if not nodes:
        return []
    return [LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_nav2',
            parameters=[{'autostart': True, 'node_names': nodes,
                         'use_sim_time': False}],
        )],
    )]


def start_bump_store(context):
    # Bumps (low obstacles motion_check on the car marks) belong to a map
    # frame: keep them beside the saved map in use, and load them with it. A
    # new map starts with none, in ~/.ros/car_s_bumps.yaml (copy it beside
    # the map when saving it)
    slam_map = os.path.expanduser(
        LaunchConfiguration('slam_map').perform(context)).removesuffix('.posegraph')
    map_yaml = os.path.expanduser(LaunchConfiguration('map').perform(context))
    if slam_map:
        bumps_file, load = slam_map + '.bumps.yaml', True
    elif map_yaml:
        bumps_file, load = os.path.splitext(map_yaml)[0] + '.bumps.yaml', True
    else:
        bumps_file, load = '~/.ros/car_s_bumps.yaml', False
    return [Node(
        package='car_s',
        executable='bump_store',
        name='bump_store',
        output='screen',
        parameters=[{'bumps_file': bumps_file, 'load_bumps': load}],
    )]


def generate_launch_description():
    car_s_dir = get_package_share_directory('car_s')
    nav2_dir = get_package_share_directory('nav2_bringup')

    slam = LaunchConfiguration('slam')
    nav2 = LaunchConfiguration('nav2')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')
    start_lidar = LaunchConfiguration('start_lidar')
    motors_on = LaunchConfiguration('motors_on')
    explore = LaunchConfiguration('explore')

    args = [
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

    # Keeps the lidar spinning and enables the motors (not again after a car
    # restart)
    car_guard = Node(
        package='car_s',
        executable='car_guard',
        name='car_guard',
        output='screen',
        parameters=[{
            'start_lidar': ParameterValue(start_lidar, value_type=bool),
            'motors_on': ParameterValue(motors_on, value_type=bool),
        }],
    )

    slam_launch = OpaqueFunction(function=start_slam, condition=IfCondition(slam))

    # slam_toolbox (2.10, Lyrical) and map_server both publish /map as
    # transient local, which a subscriber of either durability receives.
    # map_server publishes only once, so its subscribers must be transient
    # local to get the map; with SLAM, volatile is enough
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

    lifecycle_manager = OpaqueFunction(
        function=start_lifecycle_manager,
        args=[nav2_dir, nav2_on, map_yaml],
        condition=IfCondition(run_nav2))

    # Keeps the bumps motion_check marks on the car (scripts/bump_store.py)
    bump_store_node = OpaqueFunction(function=start_bump_store)

    # explore_lite is only built on the PC, so it is looked up only when used
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[os.path.join(car_s_dir, 'config', 'explore.yaml')],
        # It exits if it can't reach Nav2 within 20 s (the patch, for a
        # discovery problem seen on Lyrical), and a new process connects
        respawn=True,
        respawn_delay=2.0,
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
        car_guard,
        bump_store_node,
        slam_launch,
        nav2_container,
        localization_launch,
        navigation_launch,
        lifecycle_manager,
        explore_needs_slam,
        explore_node,
        rviz_launch,
        on_shutdown,
    ])
