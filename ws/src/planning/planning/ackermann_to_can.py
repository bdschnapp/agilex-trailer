#!/usr/bin/env python3
"""
Converts AckermannDrive messages from the planner to Joy messages for canbridge
This allows the planning stack to control the vehicle via CAN
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy
import math


class AckermannToCan(Node):
    def __init__(self):
        super().__init__('ackermann_to_can_bridge')

        # Declare parameters
        self.declare_parameter('ackermann_topic', '/ackermann_cmd')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('wheelbase', 0.65)  # meters
        self.declare_parameter('max_speed', 0.5)  # m/s (software speed limit for safety)
        self.declare_parameter('max_steering_angle', 0.576)  # radians (max hardware turn value)

        # Get parameters
        ackermann_topic = self.get_parameter('ackermann_topic').get_parameter_value().string_value
        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value

        # Create publisher and subscriber
        self.joy_pub = self.create_publisher(Joy, joy_topic, 10)
        self.ackermann_sub = self.create_subscription(
            AckermannDrive,
            ackermann_topic,
            self.ackermann_callback,
            10
        )

        self.get_logger().info(f'Converting {ackermann_topic} (AckermannDrive) to {joy_topic} (Joy)')
        self.get_logger().info(f'Wheelbase: {self.wheelbase}m, Max speed: {self.max_speed}m/s')
        self.get_logger().info(f'Max steering angle: {self.max_steering_angle}rad')

    def ackermann_callback(self, ackermann_msg: AckermannDrive):
        """Convert AckermannDrive to Joy message for canbridge"""
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()

        # Initialize axes (canbridge expects axes[1] for throttle, axes[3] for steering)
        joy_msg.axes = [0.0] * 8  # Standard joystick has 8 axes
        joy_msg.buttons = [0] * 12  # Standard joystick has 12 buttons

        # Extract velocity and steering from Ackermann message
        desired_speed = ackermann_msg.speed  # m/s
        steering_angle = ackermann_msg.steering_angle  # radians (front wheel angle)

        # Clamp speed to safety limits
        speed_clamped = max(-self.max_speed, min(self.max_speed, desired_speed))

        # Convert to normalized axes values [-1, 1]
        # axes[1]: throttle (forward/backward)
        throttle_normalized = speed_clamped / self.max_speed if self.max_speed > 0 else 0.0

        # axes[3]: steering (left/right)
        # Clamp steering angle
        steering_clamped = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        steering_normalized = steering_clamped / self.max_steering_angle if self.max_steering_angle > 0 else 0.0

        # Set Joy axes
        joy_msg.axes[1] = throttle_normalized  # Throttle
        joy_msg.axes[3] = steering_normalized  # Steering

        # Publish
        self.joy_pub.publish(joy_msg)

        # Log for debugging
        if abs(throttle_normalized) > 0.01 or abs(steering_normalized) > 0.01:
            self.get_logger().debug(
                f'Speed: {speed_clamped:.2f}m/s ({throttle_normalized:.2f}), '
                f'Steering: {steering_clamped:.2f}rad ({steering_normalized:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = AckermannToCan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
