import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PolygonStamped, Point32, Point
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, TimeoutException
from tf_transformations import quaternion_matrix

def rect_corners_xy(length, width, cx=0.0, cy=0.0, yaw=0.0):
    """Return 4 (x,y) rectangle corners (CCW) centered at (cx,cy) with heading yaw."""
    hl, hw = 0.5*length, 0.5*width
    corners_local = [ (+hl, +hw), (+hl, -hw), (-hl, -hw), (-hl, +hw) ]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c*x - s*y, cy + s*x + c*y) for (x,y) in corners_local]

def monotone_chain_convex_hull(pts):
    """Andrew’s monotone chain. pts: list of (x,y). Returns hull CCW without last=first dup."""
    pts = sorted(set(pts))
    if len(pts) <= 1:
        return pts
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]

class RobotTrailerFootprint(Node):
    def __init__(self):
        super().__init__('robot_trailer_footprint')

        # Parameters (override in launch or YAML)
        self.declare_parameter('publish_frame', 'map')     # frame to publish footprint in
        self.declare_parameter('base_frame', 'base_link')  # robot base frame
        self.declare_parameter('trailer_frame', 'trailer_chassis')  # trailer reference frame

        # Robot rectangle (meters) and optional offset/yaw in its own frame
        self.declare_parameter('robot_length', 1.5)
        self.declare_parameter('robot_width',  1.2)
        self.declare_parameter('robot_offset_x', 0.0)
        self.declare_parameter('robot_offset_y', 0.0)
        self.declare_parameter('robot_yaw', 0.0)

        # Trailer rectangle (meters) and optional offset/yaw in its own frame
        self.declare_parameter('trailer_length', 3.7)
        self.declare_parameter('trailer_width',  1.2)
        self.declare_parameter('trailer_offset_x', 0.0)
        self.declare_parameter('trailer_offset_y', 0.0)
        self.declare_parameter('trailer_yaw', 0.0)

        # rate
        self.declare_parameter('rate_hz', 10.0)

        # TF
        self.buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.listener = TransformListener(self.buffer, self)

        # pubs
        self.pub_poly = self.create_publisher(PolygonStamped, '/robot_trailer_footprint', 1)
        self.pub_marker = self.create_publisher(Marker, '/robot_trailer_footprint_marker', 1)

        # 1) WAIT TIMER: don’t publish until TF is ready
        self.wait_timer = self.create_timer(0.2, self.await_tf_ready)
        self.main_timer = None  # will be created later

    def await_tf_ready(self):
        pub_frame = self.get_parameter('publish_frame').value
        base_frame = self.get_parameter('base_frame').value
        trailer_frame = self.get_parameter('trailer_frame').value

        now = rclpy.time.Time()  # latest
        try:
            ok1 = self.buffer.can_transform(pub_frame, base_frame, now, timeout=Duration(seconds=0.1))
            ok2 = self.buffer.can_transform(pub_frame, trailer_frame, now, timeout=Duration(seconds=0.1))
        except Exception:
            ok1 = ok2 = False

        if ok1 and ok2:
            self.get_logger().info(
                f"TF ready: {pub_frame}←{base_frame} & {pub_frame}←{trailer_frame}. Starting footprint publisher.")
            # start main timer and stop the wait timer
            rate = float(self.get_parameter('rate_hz').value)
            self.main_timer = self.create_timer(1.0 / max(rate, 1e-3), self.tick)
            self.wait_timer.cancel()
            self.destroy_timer(self.wait_timer)

    def lookup_tf_xy(self, target_frame, source_frame, stamp):
        try:
            tf = self.buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=Duration(seconds=0.1)
            )
        except (LookupException, ExtrapolationException, TimeoutException) as e:
            self.get_logger().warn(f"TF {target_frame}<-{source_frame}: {e}")
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        m = quaternion_matrix([q.x, q.y, q.z, q.w])
        yaw = math.atan2(m[1, 0], m[0, 0])
        return (t.x, t.y, yaw)

    def tick(self):
        pub_frame     = self.get_parameter('publish_frame').value
        base_frame    = self.get_parameter('base_frame').value
        trailer_frame = self.get_parameter('trailer_frame').value
        now = rclpy.time.Time()

        base_pose = self.lookup_tf_xy(pub_frame, base_frame, now)
        trailer_pose = self.lookup_tf_xy(pub_frame, trailer_frame, now)
        if base_pose is None or trailer_pose is None:
            return  # skip this tick cleanly

        # Robot rect in publish_frame
        rl = float(self.get_parameter('robot_length').value)
        rw = float(self.get_parameter('robot_width').value)
        rdx = float(self.get_parameter('robot_offset_x').value)
        rdy = float(self.get_parameter('robot_offset_y').value)
        ryaw_extra = float(self.get_parameter('robot_yaw').value)
        bx, by, byaw = base_pose
        robot_pts = rect_corners_xy(rl, rw, rdx, rdy, byaw + ryaw_extra)
        robot_pts = [(x+bx, y+by) for (x,y) in robot_pts]

        # Trailer rect in publish_frame
        tl = float(self.get_parameter('trailer_length').value)
        tw = float(self.get_parameter('trailer_width').value)
        tdx = float(self.get_parameter('trailer_offset_x').value)
        tdy = float(self.get_parameter('trailer_offset_y').value)
        tyaw_extra = float(self.get_parameter('trailer_yaw').value)
        tx, ty, tyaw = trailer_pose
        trailer_pts = rect_corners_xy(tl, tw, tdx, tdy, tyaw + tyaw_extra)
        trailer_pts = [(x+tx, y+ty) for (x,y) in trailer_pts]

        # Combined convex hull
        hull = monotone_chain_convex_hull(robot_pts + trailer_pts)
        if not hull:
            return

        # Publish PolygonStamped
        poly = PolygonStamped()
        poly.header.frame_id = pub_frame
        poly.header.stamp = self.get_clock().now().to_msg()
        poly.polygon.points = [Point32(x=p[0], y=p[1], z=0.0) for p in hull]
        self.pub_poly.publish(poly)

        # Publish Marker (line strip for visualization)
        m = Marker()
        m.header = poly.header
        m.ns = "robot_trailer"
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0  # identity
        m.scale.x = 0.03  # line width (x used for LINE_STRIP)
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.2, 1.0

        # close the loop
        hull_closed = hull + [hull[0]]

        # Marker.points must be a list[geometry_msgs/Point]
        m.points = [Point(x=x, y=y, z=0.0) for (x, y) in hull_closed]

        self.pub_marker.publish(m)

def main():
    rclpy.init()
    rclpy.spin(RobotTrailerFootprint())
    rclpy.shutdown()

if __name__ == '__main__':
    main()