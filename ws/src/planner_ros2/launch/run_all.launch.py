#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # -------------------------
    # Top-level launch arguments
    # -------------------------
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation'
    )

    # Expose random_map_generator topics at top level (optional but handy)
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/global_map',
        description='Topic name for global map (forwarded to random_map_generator)'
    )
    polygon_topic_arg = DeclareLaunchArgument(
        'polygon_topic',
        default_value='/global_map_polygon',
        description='Topic name for global map polygon (forwarded to random_map_generator)'
    )

    # -------------------------
    # Package paths & params
    # -------------------------
    planner_ros2_dir = get_package_share_directory('planner_ros2')
    grid_map_params     = os.path.join(planner_ros2_dir, 'params', 'grid_map.yaml')
    hybrid_astar_params = os.path.join(planner_ros2_dir, 'params', 'hybrid_astar.yaml')
    optimizer_params    = os.path.join(planner_ros2_dir, 'params', 'optimizer.yaml')
    trailer_params      = os.path.join(planner_ros2_dir, 'params', 'trailer.yaml')
    controller_params   = os.path.join(planner_ros2_dir, 'params', 'controller.yaml')

    # -------------------------
    # Nodes
    # -------------------------
    simulator_node = Node(
        package='planner_ros2',
        executable='simulator_node',
        name='simulator_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim')),
        remappings=[
            ('~/cmd', '/trailer_cmd'),
            ('~/odom', '/trailer_odom'),
            ('~/sensor_odom', '/sensor_odom'),
        ],
        parameters=[
            grid_map_params,
            hybrid_astar_params,
            optimizer_params,
            trailer_params,
            controller_params,
        ]
    )

    planner_node = Node(
        package='planner_ros2',
        executable='planner_node',
        name='planner_node',
        output='screen',
        remappings=[
            ('~/odom', '/trailer_odom'),
        ],
        parameters=[
            grid_map_params,
            hybrid_astar_params,
            optimizer_params,
            trailer_params,
            controller_params,
        ]
    )

    mpc_node = Node(
        package='planner_ros2',
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim')),
        remappings=[
            ('~/cmd', '/trailer_cmd'),
            ('~/odom', '/trailer_odom'),
            ('~/arc_traj', '/arc_trailer_traj'),
        ],
        parameters=[
            grid_map_params,
            hybrid_astar_params,
            optimizer_params,
            trailer_params,
            controller_params,
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(planner_ros2_dir, 'rviz', 'default.rviz')]
    )

    # -------------------------
    # Include: random_map_generator launch
    # -------------------------
    random_map_generator_dir = get_package_share_directory('random_map_generator')
    random_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(random_map_generator_dir, 'launch', 'test.launch.py')
        ),
        # Forward top-level args to the included launch
        launch_arguments={
            'map_topic': LaunchConfiguration('map_topic'),
            'polygon_topic': LaunchConfiguration('polygon_topic'),
        }.items()
    )

    # -------------------------
    # LaunchDescription
    # -------------------------
    return LaunchDescription([
        use_sim_arg,
        map_topic_arg,
        polygon_topic_arg,
        random_map_launch,   # <-- now actually included
        simulator_node,
        mpc_node,
        planner_node,
        rviz_node,
    ])
