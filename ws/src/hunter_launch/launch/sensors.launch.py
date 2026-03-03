from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def has_iface(name: str) -> bool:
    return os.path.exists(os.path.join("/sys/class/net", name))

def generate_launch_description():
    # Args
    domain_id = DeclareLaunchArgument("ros_domain_id", default_value="25")
    can_if = DeclareLaunchArgument("can_if", default_value="can0")
    can_bitrate = DeclareLaunchArgument("can_bitrate", default_value="500000")

    # Feature toggles (dev-friendly defaults)
    enable_can    = DeclareLaunchArgument("enable_can",    default_value="true")
    enable_lidar  = DeclareLaunchArgument("enable_lidar",  default_value="true")
    enable_imu    = DeclareLaunchArgument("enable_imu",    default_value="true")
    enable_gps    = DeclareLaunchArgument("enable_gps",    default_value="true")
    enable_rviz   = DeclareLaunchArgument("enable_rviz",   default_value="true")
    enable_joy    = DeclareLaunchArgument("enable_joy",    default_value="false")
    enable_camera = DeclareLaunchArgument("enable_camera", default_value="false")

    ros_domain_id   = LaunchConfiguration("ros_domain_id")
    can_if_cfg      = LaunchConfiguration("can_if")
    can_bitrate_cfg = LaunchConfiguration("can_bitrate")

    set_domain_env = SetEnvironmentVariable(name="ROS_DOMAIN_ID", value=ros_domain_id)

    # ---- Nodes (independent) ----
    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_joy")),
    )

    # RViz: silence XDG warning in dev (creates a private runtime dir)
    rviz_env = {"XDG_RUNTIME_DIR": "/tmp/xdg-runtime-rviz"}
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("rslidar_sdk"), "rviz", "rviz2.rviz"])],
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        additional_env=rviz_env,
    )

    # ---- CAN bring-up ----
    can_down_cmd = ExecuteProcess(
        cmd=["/sbin/ifconfig", can_if_cfg, "down"],
        shell=False,
        output="screen",
    )
    can_type_cmd = ExecuteProcess(
        cmd=["/sbin/ip", "link", "set", can_if_cfg, "type", "can", "bitrate", can_bitrate_cfg],
        shell=False,
        output="screen",
    )
    can_up_cmd = ExecuteProcess(
        cmd=["/sbin/ifconfig", can_if_cfg, "up"],
        shell=False,
        output="screen",
    )
    ros2can_bridge_cmd = ExecuteProcess(
        cmd=["ros2can_bridge"],
        shell=False,
        output="screen",
    )
    canbridge = Node(
        package="canbridge",
        executable="canbridge",
        name="controller_canbridge",
        output="screen",
    )

    # Chain only when enabled AND iface exists; otherwise do nothing.
    def can_chain_fn(context):
        actions = []
        if str(context.launch_configurations.get("enable_can", "false")).lower() == "true":
            iface = context.launch_configurations.get("can_if", "can0")
            if has_iface(iface):
                actions += [
                    can_down_cmd,
                    RegisterEventHandler(OnProcessExit(target_action=can_down_cmd, on_exit=[can_type_cmd])),
                    RegisterEventHandler(OnProcessExit(target_action=can_type_cmd, on_exit=[can_up_cmd])),
                    RegisterEventHandler(OnProcessExit(target_action=can_up_cmd, on_exit=[ros2can_bridge_cmd])),
                    canbridge,
                ]
            else:
                # Interface missing: log once and skip bring-up/bridge
                actions.append(ExecuteProcess(
                    cmd=["/bin/echo", f"[launch] Skipping CAN: interface '{iface}' not present"],
                    shell=False,
                    output="screen",
                ))
        return actions

    can_chain = OpaqueFunction(function=can_chain_fn)

    # ---- Sensors ----
    rslidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("rslidar_sdk"), "launch", "start.py"])
        ),
        condition=IfCondition(LaunchConfiguration("enable_lidar")),
    )
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("witmotion_ros"), "launch", "wt905.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("enable_imu")),
    )
    gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ublox_gps"), "launch", "ublox_gps_node_zedf9p-launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("enable_gps")),
    )

    return LaunchDescription([
        # args + env
        domain_id, can_if, can_bitrate,
        enable_can, enable_lidar, enable_imu, enable_gps, enable_rviz, enable_joy,
        set_domain_env,

        # feature groups
        can_chain,
        rslidar,
        imu,
        gps,

        # independent
        joy,
        rviz2,
    ])