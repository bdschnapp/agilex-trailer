import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist

class AckToTwist(Node):
    def __init__(self):
        super().__init__('ack_to_twist_bridge')
        self.declare_parameter('wheelbase', 1.0)  # meters
        self.declare_parameter('src', '/trailer_cmd')
        self.declare_parameter('dst', '/ackermann_like_controller/cmd_vel')
        self.L = float(self.get_parameter('wheelbase').value)
        self.pub = self.create_publisher(Twist, self.get_parameter('dst').value, 10)
        self.sub = self.create_subscription(AckermannDrive, self.get_parameter('src').value, self.cb, 10)

    def cb(self, msg: AckermannDrive):
        v = float(msg.speed)
        delta = float(msg.steering_angle)
        w = v * math.tan(delta) / max(self.L, 1e-6)

        out = Twist()
        out.linear.x  = v
        out.angular.z = w
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(AckToTwist())
    rclpy.shutdown()

if __name__ == '__main__':
    main()