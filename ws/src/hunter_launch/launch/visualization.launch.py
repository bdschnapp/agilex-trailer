import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )
    use_sim_time_cfg = LaunchConfiguration("use_sim_time")

    set_domain = SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="25")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("hunter_description"), "description", "robot.urdf.xacro"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time_cfg}],
    )

    node_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_cfg}],
    )

    hunter_description_path = get_package_share_directory("hunter_description")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(hunter_description_path, "rviz", "robot_view.rviz")],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_cfg}],
    )

    static_world_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_world_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "1", "world", "odom"],
        output="screen",
    )

    return LaunchDescription([
        set_domain,
        use_sim_time,
        node_joint_state_publisher,
        node_robot_state_publisher,
        rviz,
        static_world_to_odom,
    ])