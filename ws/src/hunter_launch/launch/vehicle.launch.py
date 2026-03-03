#!/usr/bin/env python3
"""
Physical vehicle launch file (without trailer)
Replaces Gazebo with lidarslam for localization and canbridge for control
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ========================
    # Launch Arguments
    # ========================
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )
    use_sim_time_cfg = LaunchConfiguration("use_sim_time")

    ros_domain_id_arg = DeclareLaunchArgument(
        "ros_domain_id",
        default_value="25",
        description="ROS 2 domain id",
    )

    # Planning mode
    use_planning = DeclareLaunchArgument(
        "use_planning",
        default_value="true",
        description="Enable planning and control stack",
    )

    # Sensor toggles (passed to sensors.launch.py)
    enable_can = DeclareLaunchArgument("enable_can", default_value="true")
    enable_lidar = DeclareLaunchArgument("enable_lidar", default_value="true")
    enable_imu = DeclareLaunchArgument("enable_imu", default_value="true")
    enable_gps = DeclareLaunchArgument("enable_gps", default_value="true")
    enable_rviz = DeclareLaunchArgument("enable_rviz", default_value="true")
    enable_joy = DeclareLaunchArgument("enable_joy", default_value="false")
    enable_camera = DeclareLaunchArgument("enable_camera", default_value="false")

    # CAN config
    can_if = DeclareLaunchArgument("can_if", default_value="can0")
    can_bitrate = DeclareLaunchArgument("can_bitrate", default_value="500000")

    # ========================
    # Environment Setup
    # ========================
    set_domain_env = SetEnvironmentVariable(
        name="ROS_DOMAIN_ID",
        value=LaunchConfiguration("ros_domain_id"),
    )

    set_casadipath = SetEnvironmentVariable(
        name="CASADIPATH", value="/opt/casadi-3.6.5/lib"
    )

    set_ld = SetEnvironmentVariable(
        name="LD_LIBRARY_PATH",
        value=[
            os.environ.get("LD_LIBRARY_PATH", ""),
            ":/opt/casadi-3.6.5/lib:/opt/qpOASES/lib",
        ],
    )

    # ========================
    # Robot Description
    # ========================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("hunter_description"),
            "description",
            "robot.urdf.xacro"  # No trailer
        ]),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time_cfg}],
    )

    # ========================
    # Sensors (CAN, Lidar, IMU, GPS, etc.)
    # ========================
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hunter_launch"),
                "launch",
                "sensors.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "enable_can": LaunchConfiguration("enable_can"),
            "enable_lidar": LaunchConfiguration("enable_lidar"),
            "enable_imu": LaunchConfiguration("enable_imu"),
            "enable_gps": LaunchConfiguration("enable_gps"),
            "enable_rviz": "false",  # We'll launch rviz separately
            "enable_joy": LaunchConfiguration("enable_joy"),
            "enable_camera": LaunchConfiguration("enable_camera"),
            "can_if": LaunchConfiguration("can_if"),
            "can_bitrate": LaunchConfiguration("can_bitrate"),
        }.items(),
    )

    # ========================
    # Localization (lidarslam)
    # ========================
    lidarslam_params_file = os.path.join(
        get_package_share_directory("lidarslam"),
        "param",
        "lidarslam.yaml"
    )

    # Scanmatcher: Lidar odometry
    scanmatcher_node = Node(
        package="scanmatcher",
        executable="scanmatcher_node",
        name="scanmatcher_node",
        output="screen",
        parameters=[lidarslam_params_file, {"use_sim_time": use_sim_time_cfg}],
        remappings=[("/input_cloud", "/rslidar_points")],
    )

    # Graph-based SLAM: Loop closure
    graph_based_slam_node = Node(
        package="graph_based_slam",
        executable="graph_based_slam_node",
        name="graph_based_slam_node",
        output="screen",
        parameters=[lidarslam_params_file, {"use_sim_time": use_sim_time_cfg}],
    )

    # Static TF: base_link -> rsLidar (lidar frame)
    tf_base_to_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_lidar",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "rsLidar"],
        output="screen",
    )

    # ========================
    # Odometry Conversion
    # ========================
    # Convert /current_pose (PoseStamped) to /odom (Odometry)
    # This node needs to be created or we can use pose_to_tf + a custom converter
    # For now, we'll create a simple converter node
    pose_to_odom_node = Node(
        package="planning",
        executable="pose_to_odom",  # You may need to create this executable
        name="pose_to_odom_converter",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_cfg}],
        remappings=[
            ("/current_pose", "/current_pose"),
            ("/odom", "/odom"),
        ],
    )

    # Static TF: world -> map (if needed)
    static_world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_world_to_map",
        arguments=["0", "0", "0", "0", "0", "0", "1", "world", "map"],
        output="screen",
    )

    # Note: map -> odom -> base_link is handled by lidarslam
    # lidarslam publishes map -> base_link, so we need odom in between
    # Actually, lidarslam typically publishes map -> odom, and we need odom -> base_link from odometry

    # ========================
    # Map Publishing
    # ========================
    default_pcd_map_path = os.path.join(
        get_package_share_directory("hunter_launch"), "maps", "map.pcd"
    )

    pcd_static = Node(
        package="planning",
        executable="static_pcd_publisher",
        name="static_pcd_publisher",
        arguments=[default_pcd_map_path, "map"],
        output="screen",
    )

    pcd_filtered = Node(
        package="planning",
        executable="pcd_filter",
        name="pcd_filter_node",
        parameters=[{
            "input_topic": "/pcd_map",
            "output_topic": "/global_map",
            "z_min": 0.05,
            "z_max": 2.0,
        }],
        output="screen",
    )

    occupancy_grid = Node(
        package="planning",
        executable="pcd_to_occupancy_grid",
        name="pcd_to_occupancy_grid",
        output="screen",
        parameters=[{
            "cloud_topic": "/pcd_map",
            "frame_id": "map",
            "resolution": 0.05,
            "size_x": 80.0,
            "size_y": 80.0,
            "origin_x": -40.0,
            "origin_y": -40.0,
            "z_min": 0.05,
            "z_max": 1.8,
            "downsample_voxel": 0.05,
            "min_hits_per_cell": 2,
            "inflate_radius": 0.20,
            "occupied_value": 100,
            "free_value": 0,
            "unknown_value": -1,
        }],
    )

    # ========================
    # Planning & Control
    # ========================
    vehicle_params = os.path.join(
        get_package_share_directory("hunter_launch"),
        "config",
        "vehicle_params.yaml",
    )

    footprint = Node(
        package="planning",
        executable="footprint_publisher",
        name="footprint_publisher",
        output="screen",
        parameters=[vehicle_params],
        condition=IfCondition(LaunchConfiguration("use_planning")),
    )

    planner_ros2_dir = get_package_share_directory("planner_ros2")
    grid_map_params = os.path.join(planner_ros2_dir, "params", "grid_map.yaml")
    hybrid_astar_params = os.path.join(planner_ros2_dir, "params", "hybrid_astar.yaml")
    optimizer_params = os.path.join(planner_ros2_dir, "params", "optimizer.yaml")
    trailer_params = os.path.join(planner_ros2_dir, "params", "trailer.yaml")
    controller_params = os.path.join(planner_ros2_dir, "params", "controller.yaml")

    planner_node = Node(
        package="planner_ros2",
        executable="planner_node",
        name="planner_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_planning")),
        remappings=[
            ("~/odom", "/odom"),  # From pose_to_odom converter
        ],
        parameters=[
            grid_map_params,
            hybrid_astar_params,
            optimizer_params,
            trailer_params,
            controller_params,
        ],
    )

    mpc_node = Node(
        package="planner_ros2",
        executable="mpc_node",
        name="mpc_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_planning")),
        remappings=[
            ("~/cmd", "/ackermann_cmd"),  # Output to vehicle control
            ("~/odom", "/odom"),
            ("~/arc_traj", "/arc_traj"),
        ],
        parameters=[
            grid_map_params,
            hybrid_astar_params,
            optimizer_params,
            trailer_params,
            controller_params,
        ],
    )

    # ========================
    # Control Bridge
    # ========================
    # Convert AckermannDrive to Joy messages for canbridge
    ackermann_to_can_bridge = Node(
        package="planning",
        executable="ackermann_to_can",
        name="ackermann_to_can_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_planning")),
        parameters=[{
            "ackermann_topic": "/ackermann_cmd",
            "joy_topic": "/joy",
            "wheelbase": 0.65,
            "max_speed": 0.5,  # m/s (safety limit)
            "max_steering_angle": 0.576,  # radians
        }],
    )

    # ========================
    # Visualization
    # ========================
    hunter_description_path = get_package_share_directory("hunter_description")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(hunter_description_path, "rviz", "robot_view.rviz")],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_cfg}],
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        additional_env={"XDG_RUNTIME_DIR": "/tmp/xdg-runtime-rviz"},
    )

    # ========================
    # Launch Description
    # ========================
    return LaunchDescription([
        # Arguments
        use_sim_time,
        ros_domain_id_arg,
        use_planning,
        enable_can,
        enable_lidar,
        enable_imu,
        enable_gps,
        enable_rviz,
        enable_joy,
        enable_camera,
        can_if,
        can_bitrate,

        # Environment
        set_domain_env,
        set_casadipath,
        set_ld,

        # Robot Description
        node_robot_state_publisher,

        # Sensors (includes CAN/canbridge)
        sensors,

        # Localization
        scanmatcher_node,
        graph_based_slam_node,
        tf_base_to_lidar,

        # Odometry
        pose_to_odom_node,
        static_world_to_map,

        # Map
        pcd_static,
        pcd_filtered,
        occupancy_grid,

        # Planning & Control
        footprint,
        planner_node,
        mpc_node,
        ackermann_to_can_bridge,

        # Visualization
        rviz,
    ])
