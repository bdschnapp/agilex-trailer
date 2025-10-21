import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')

    # --- Gazebo ---
    gazebo_params_file = os.path.join(get_package_share_directory("hunter_launch"), 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # --- Robot description ---
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("hunter_description"), "description", "robot.urdf.xacro"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=None)}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time_cfg}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'hunter'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_cfg}],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_ackermann_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'ackermann_like_controller'],
        output='screen'
    )

    hunter_description_path = os.path.join(get_package_share_directory('hunter_description'))
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(hunter_description_path, 'rviz/robot_view.rviz')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_cfg}],
        additional_env={"XDG_RUNTIME_DIR": "/tmp/xdg-runtime-rviz"},  # silence the runtime dir warning
    )

    # --- PCD Map Publisher (PointCloud2) ---
    default_pcd_map_path = os.path.join(get_package_share_directory('hunter_launch'), 'maps', 'map.pcd')
    pcd_static = Node(
        package='planning',
        executable='static_pcd_publisher',
        name='static_pcd_publisher',
        arguments=[default_pcd_map_path, 'map'],
        output='screen'
    )

    # --- Occupancy Grid Publisher ---
    occupancy_grid = Node(
            package='planning',
            executable='pcd_to_occupancy_grid',
            name='pcd_to_occupancy_grid',
            output='screen',
            parameters=[{
                'cloud_topic': '/pcd_map',
                'frame_id': 'map',
                'resolution': 0.05,
                'size_x': 80.0, 'size_y': 80.0,
                'origin_x': -40.0, 'origin_y': -40.0,
                'z_min': 0.05, 'z_max': 1.8,
                'downsample_voxel': 0.05,
                'min_hits_per_cell': 2,
                'inflate_radius': 0.20,
                'occupied_value': 100,
                'free_value': 0,
                'unknown_value': -1
            }]
        )

    # --- Static TF: world -> map ---
    static_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map'],
        output='screen'
    )
    # --- Static TF: map -> odom ---
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        output='screen'
    )

    # --- /odom msg -> (odom -> base_link) TF ---
    odom_to_tf = Node(
        package='planning',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen'
    )

    vehicle_params = os.path.join(
        get_package_share_directory('hunter_launch'),
        'config',
        'vehicle_params.yaml'
    )
    footprint = Node(
        package='planning',
        executable='footprint_publisher',
        name='footprint_publisher',
        output='screen',
        parameters=[vehicle_params],
    )

    return LaunchDescription([
        use_sim_time,

        # event chain for ros2_control controllers
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[load_joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[load_ackermann_controller])),

        # Gazebo + robot
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        rviz,

        # PCD map + TF
        pcd_static,
        occupancy_grid,
        static_world_to_map,
        static_map_to_odom,
        odom_to_tf,
        footprint
    ])