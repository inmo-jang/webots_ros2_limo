#!/usr/bin/env python

# Launch Webots AgileX LIMO driver.
#
# Structure mirrors the official Cyberbotics example
# `webots_ros2/webots_ros2_turtlebot/launch/robot_launch.py`:
#   * `slam:=true`  -> includes turtlebot3_cartographer (cartographer SLAM + RViz)
#   * `nav:=true`   -> includes turtlebot3_navigation2 (Nav2/AMCL + RViz) with a saved map
# Both are optional and only activate when the corresponding package is installed.

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_limo')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    nav2_map = LaunchConfiguration(
        'map', default=os.path.join(package_dir, 'resource', 'limo_example_map.yaml'))

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    robot_description_path = os.path.join(package_dir, 'resource', 'limo.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yaml')
    mappings = [
        ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
        ('/diffdrive_controller/odom', '/odom'),
    ]
    limo_driver = WebotsController(
        robot_name='LIMO',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # Optional stacks. Both launch files live in this package and each brings
    # its own RViz, so these shortcut arguments run exactly what the README
    # asks you to type in a second terminal.
    navigation_nodes = []
    if 'nav2_bringup' in get_packages_with_prefixes():
        limo_navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                package_dir, 'launch', 'nav2_launch.py')),
            launch_arguments=[
                ('map', nav2_map),
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_nav))
        navigation_nodes.append(limo_navigation)

    if 'cartographer_ros' in get_packages_with_prefixes():
        limo_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                package_dir, 'launch', 'cartographer_launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_slam))
        navigation_nodes.append(limo_slam)

    # Wait for the simulation to be ready to start navigation nodes.
    #
    # NOTE: starting the simulator and the SLAM/Nav stack from this single
    # command loses a startup race in roughly 1 launch in 5 — Nav2's
    # global_costmap can latch the robot at the map origin and never recover.
    # The upstream webots_ros2_turtlebot example behaves the same way, so the
    # wiring here is left identical to it. The README documents starting the
    # two stacks in separate terminals, which avoids the race entirely.
    waiting_nodes = WaitForControllerConnection(
        target_driver=limo_driver,
        nodes_to_start=navigation_nodes + ros_control_spawners
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='limo_world.wbt',
            description='Choose one of the world files from `webots_ros2_limo/worlds`'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Also start cartographer SLAM + RViz (cartographer_launch.py)'
        ),
        DeclareLaunchArgument(
            'nav',
            default_value='false',
            description='Also start Nav2/AMCL + RViz (nav2_launch.py)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(package_dir, 'resource', 'limo_example_map.yaml'),
            description='Map yaml used when nav:=true'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Webots /clock as ROS 2 time'
        ),
        webots,
        webots._supervisor,

        robot_state_publisher,
        footprint_publisher,

        limo_driver,
        waiting_nodes,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ])
