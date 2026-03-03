#!/usr/bin/env python3
"""
Converts geometry_msgs/PoseStamped (from lidarslam) to nav_msgs/Odometry
This allows lidarslam's /current_pose to be used by nodes expecting /odom
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class PoseToOdom(Node):
    def __init__(self):
        super().__init__('pose_to_odom_converter')

        # Declare parameters
        self.declare_parameter('input_topic', '/current_pose')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('child_frame_id', 'base_link')

        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        # Create publisher and subscriber
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            input_topic,
            self.pose_callback,
            10
        )

        self.get_logger().info(f'Converting {input_topic} (PoseStamped) to {output_topic} (Odometry)')
        self.get_logger().info(f'Using child_frame_id: {self.child_frame_id}')

    def pose_callback(self, pose_msg: PoseStamped):
        """Convert PoseStamped to Odometry message"""
        odom_msg = Odometry()

        # Copy header
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = self.child_frame_id

        # Copy pose
        odom_msg.pose.pose = pose_msg.pose

        # Set covariance to unknown (zeros indicate unknown)
        # If you have better covariance estimates, set them here
        odom_msg.pose.covariance = [0.0] * 36

        # Velocity is unknown (lidarslam only provides pose)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        odom_msg.twist.covariance = [0.0] * 36

        # Publish
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
