import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, TextSubstitution

def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation'
    )

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

    # --- PCD Map Publisher (PCD -> PointCloud2) ---
    default_pcd_map_path = os.path.join(get_package_share_directory('hunter_launch'), 'maps', 'map.pcd')
    pcd_static = Node(
        package='planning',
        executable='static_pcd_publisher',
        name='static_pcd_publisher',
        arguments=[default_pcd_map_path, 'map'],
        output='screen'
    )

    # --- PointCloud Filter (PointCloud2 -> PointCloud2) ---
    pcd_filtered = Node(
            package='planning',
            executable='pcd_filter',
            name='pcd_filter_node',
            parameters=[{
                'input_topic': '/pcd_map',
                'output_topic': '/global_map',
                'z_min': 0.05,
                'z_max': 2.0,
            }],
            output='screen'
        )

    # --- Occupancy Grid Publisher (PointCloud2 -> OccupancyGrid) ---
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

    # --- Static TF: (world -> map) ---
    static_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map'],
        output='screen'
    )
    # --- Static TF: (map -> odom) ---
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
        output='screen',
        remappings=[('/odom', '/ackermann_like_controller/odom')]
    )

    vehicle_params = os.path.join(
        get_package_share_directory('hunter_launch'),
        'config',
        'vehicle_params.yaml'
    )

    # --- Footprint Publisher (JointState -> Footprint) ---
    footprint = Node(
        package='planning',
        executable='footprint_publisher',
        name='footprint_publisher',
        output='screen',
        parameters=[vehicle_params],
    )

    ################
    # planner_ros2 #
    ################

    planner_ros2_dir = get_package_share_directory('planner_ros2')
    grid_map_params = os.path.join(planner_ros2_dir, 'params', 'grid_map.yaml')
    hybrid_astar_params = os.path.join(planner_ros2_dir, 'params', 'hybrid_astar.yaml')
    optimizer_params = os.path.join(planner_ros2_dir, 'params', 'optimizer.yaml')
    trailer_params = os.path.join(planner_ros2_dir, 'params', 'trailer.yaml')
    controller_params = os.path.join(planner_ros2_dir, 'params', 'controller.yaml')

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

    set_casadipath = SetEnvironmentVariable(
        name='CASADIPATH', value='/opt/casadi-3.6.5/lib'
    )

    set_ld = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=[
            EnvironmentVariable('LD_LIBRARY_PATH'),
            TextSubstitution(text=':/opt/casadi-3.6.5/lib:/opt/qpOASES/lib')
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

    ######################
    # Planning -> Gazebo #
    ######################

    ack_to_twist = Node(
        package='planning', executable='ackermann_to_twist', name='ackermann_to_twist_bridge',
        parameters=[{'wheelbase': 0.65,
                     'src': '/trailer_cmd',
                     'dst': '/ackermann_like_controller/cmd_vel'}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_arg,
        use_sim_time,

        # event chain for ros2_control controllers
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[load_joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[load_ackermann_controller])),

        # Gazebo + Rviz Simulation
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        rviz,

        # PCD map + TF
        pcd_static,
        pcd_filtered,
        occupancy_grid,
        static_world_to_map,
        static_map_to_odom,
        odom_to_tf,
        footprint,

        # Planning + Control
        set_casadipath,
        set_ld,
        simulator_node,
        planner_node,
        mpc_node,
        ack_to_twist,
    ])