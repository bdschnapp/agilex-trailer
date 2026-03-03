import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry
from planner_ros2.msg import TrailerState  # your custom msg
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler

class TrailerOdomBuilder(Node):
    def __init__(self):
        super().__init__('trailer_odom_builder')

        # Params
        self.declare_parameter('tractor_odom_topic', '/ackermann_like_controller/odom')
        self.declare_parameter('tractor_base_link', 'base_link')
        self.declare_parameter('trailer_base_link', 'trailer_base_link')
        self.declare_parameter('target_frame', 'world')        # publish odoms in this frame
        self.declare_parameter('publish_topic', '/trailer_odom')
        self.declare_parameter('timeout_sec', 0.2)             # TF lookup timeout
        self.declare_parameter('pub_rate_hz', 30.0)            # if you prefer timer-driven publish

        self.tractor_odom_topic = self.get_parameter('tractor_odom_topic').get_parameter_value().string_value
        self.tractor_link = self.get_parameter('tractor_base_link').get_parameter_value().string_value
        self.trailer_link = self.get_parameter('trailer_base_link').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.pub_rate_hz = float(self.get_parameter('pub_rate_hz').value)

        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pub/Sub
        self.pub = self.create_publisher(TrailerState, self.publish_topic, 10)
        # Use tractor odom as a heartbeat; whenever it updates, rebuild TrailerState
        self.sub = self.create_subscription(Odometry, self.tractor_odom_topic, self._on_tractor_odom, 10)

        # Optional: also publish on a timer (uncomment if you want fixed-rate publish)
        # self.timer = self.create_timer(1.0/self.pub_rate_hz, self._publish_from_tf)

        self.get_logger().info(
            f"Building TrailerState on {self.publish_topic} with poses in frame '{self.target_frame}' "
            f"from TF({self.tractor_link}, {self.trailer_link}); trigger: {self.tractor_odom_topic}"
        )

    def _on_tractor_odom(self, _msg: Odometry):
        # Publish once per tractor odom update
        self._publish_from_tf()

    def _lookup_pose(self, child_frame: str):
        from geometry_msgs.msg import Pose
        now = rclpy.time.Time()
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, child_frame, now, timeout=Duration(seconds=self.timeout_sec)
            )
            p = Pose()
            p.position.x = tf.transform.translation.x
            p.position.y = tf.transform.translation.y
            p.position.z = tf.transform.translation.z
            p.orientation = tf.transform.rotation
            return p, True
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed {self.target_frame} <- {child_frame}: {e}")
            return Pose(), False

    def _publish_from_tf(self):
        # Get tractor and trailer poses in target_frame
        p_trac, ok1 = self._lookup_pose(self.tractor_link)
        p_trai, ok2 = self._lookup_pose(self.trailer_link)
        if not (ok1 and ok2):
            return  # skip publish until TF is ready

        # Build two Odometry messages (pose only, zero twist to match your sample)
        def odom_from_pose(pose: Pose):
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = self.target_frame
            odom.child_frame_id = ''  # matches your example
            odom.pose.pose = pose
            # leave covariance zeros
            # set twists to zero (as in your sample)
            return odom

        msg = TrailerState()
        msg.time_now = self.get_clock().now().to_msg()
        msg.odoms.append(odom_from_pose(p_trac))  # index 0: tractor
        msg.odoms.append(odom_from_pose(p_trai))  # index 1: trailer

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TrailerOdomBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()