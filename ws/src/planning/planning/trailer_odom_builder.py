import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64

from sensor_msgs_py import point_cloud2

from planner_ros2.msg import TrailerState  # your custom msg


def yaw_from_quat(q):
    # geometry_msgs/Quaternion -> yaw (Z)
    # (x,y,z,w)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    # yaw -> geometry_msgs/Quaternion
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    # x=y=0 for planar yaw
    return (0.0, 0.0, qz, qw)


def odom_from_pose(pose: Pose, stamp_msg, frame_id: str) -> Odometry:
    odom = Odometry()
    odom.header.stamp = stamp_msg
    odom.header.frame_id = frame_id
    odom.child_frame_id = ''
    odom.pose.pose = pose
    return odom


class TractorTrailerOdomBuilder(Node):
    """
    - Sub /odom (tractor)
    - Sub /rslidar_points (raw XYZI)
    - Publish /rslidar_points_front (XYZI) for SLAM
    - Use back half for hitch angle estimation
    - Compute trailer pose from tractor odom + hitch angle + trailer length
    - Publish TrailerState (2 odoms) to /tractor_odom
    """

    def __init__(self):
        super().__init__('tractor_trailer_fusion')

        # -----------------------
        # Topics
        # -----------------------
        self.declare_parameter('tractor_odom_in_topic', '/odom')
        self.declare_parameter('raw_lidar_topic', '/rslidar_points')
        self.declare_parameter('front_lidar_topic', '/rslidar_points_front')
        self.declare_parameter('tractor_odom_out_topic', '/tractor_odom')

        # -----------------------
        # Frames / stamping
        # -----------------------
        self.declare_parameter('target_frame', 'map')  # Odometry header.frame_id

        # -----------------------
        # LiDAR split config
        # -----------------------
        # Front if |atan2(y,x)| <= pi/2 in LiDAR frame (x forward, y left).
        self.declare_parameter('front_half_angle_rad', math.pi / 2.0)

        # -----------------------
        # Trailer kinematics params
        # -----------------------
        # Trailer reference point definition:
        # We will publish the trailer pose at the TRAILER AXLE CENTER (or whichever point you intend),
        # located "trailer_length_m" behind the hitch along trailer forward axis.
        self.declare_parameter('trailer_length_m', 3.6576)

        # Hitch point relative to the tractor odom pose (in tractor base frame).
        # If your /odom pose is already at the hitch, leave these 0.
        # If /odom is at rear axle and hitch is behind/ahead, set accordingly.
        self.declare_parameter('hitch_offset_x_m', 0.0)
        self.declare_parameter('hitch_offset_y_m', 0.0)

        # Hitch angle conventions:
        # Your estimator currently returns angle_deg = degrees(atan2(det,dot)) + 90.
        # That means "straight" ends up near 90deg.
        # So we convert to a "relative yaw" by: rel_deg = estimated_deg + hitch_angle_offset_deg
        # Default offset = -90 so straight => ~0.
        self.declare_parameter('hitch_angle_offset_deg', -90.0)

        # -----------------------
        # Hitch estimator ROI (in LiDAR frame)
        # -----------------------
        self.declare_parameter('roi_x_min', -0.95)
        self.declare_parameter('roi_x_max', -0.30)
        self.declare_parameter('roi_y_min', -0.125)
        self.declare_parameter('roi_y_max', 0.125)
        self.declare_parameter('roi_z_min', -0.10)
        self.declare_parameter('roi_z_max', 0.70)

        # Debug / output
        self.declare_parameter('publish_hitch_angle', True)
        self.declare_parameter('hitch_angle_topic', '/hitch_angle_deg')

        # -----------------------
        # Load params
        # -----------------------
        self.tractor_odom_in_topic = self.get_parameter('tractor_odom_in_topic').value
        self.raw_lidar_topic = self.get_parameter('raw_lidar_topic').value
        self.front_lidar_topic = self.get_parameter('front_lidar_topic').value
        self.tractor_odom_out_topic = self.get_parameter('tractor_odom_out_topic').value

        self.target_frame = self.get_parameter('target_frame').value

        self.front_half_angle = float(self.get_parameter('front_half_angle_rad').value)

        self.trailer_length_m = float(self.get_parameter('trailer_length_m').value)
        self.hitch_offset_x_m = float(self.get_parameter('hitch_offset_x_m').value)
        self.hitch_offset_y_m = float(self.get_parameter('hitch_offset_y_m').value)
        self.hitch_angle_offset_deg = float(self.get_parameter('hitch_angle_offset_deg').value)

        self.roi_x_min = float(self.get_parameter('roi_x_min').value)
        self.roi_x_max = float(self.get_parameter('roi_x_max').value)
        self.roi_y_min = float(self.get_parameter('roi_y_min').value)
        self.roi_y_max = float(self.get_parameter('roi_y_max').value)
        self.roi_z_min = float(self.get_parameter('roi_z_min').value)
        self.roi_z_max = float(self.get_parameter('roi_z_max').value)

        self.publish_hitch_angle = bool(self.get_parameter('publish_hitch_angle').value)
        self.hitch_angle_topic = self.get_parameter('hitch_angle_topic').value

        # -----------------------
        # State
        # -----------------------
        self.last_tractor_odom: Odometry | None = None

        # Estimator internal state
        self.last_hitch_angle_deg_est: float = 90.0  # using your estimator convention (straight ~90)
        self.roi_adjustment: float = 0.0
        self.using_side_face: bool = False  # keep if you re-enable later

        # -----------------------
        # Pub/Sub
        # -----------------------
        self.odom_sub = self.create_subscription(
            Odometry,
            self.tractor_odom_in_topic,
            self._on_odom,
            qos_profile_sensor_data
        )

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.raw_lidar_topic,
            self._on_lidar,
            qos_profile_sensor_data
        )

        self.front_pub = self.create_publisher(PointCloud2, self.front_lidar_topic, qos_profile_sensor_data)
        self.state_pub = self.create_publisher(TrailerState, self.tractor_odom_out_topic, 10)

        self.hitch_pub = None
        if self.publish_hitch_angle:
            self.hitch_pub = self.create_publisher(Float64, self.hitch_angle_topic, 10)

        self.get_logger().info(
            "tractor_trailer_fusion started\n"
            f"  odom in:   {self.tractor_odom_in_topic}\n"
            f"  lidar in:  {self.raw_lidar_topic}\n"
            f"  lidar out(front XYZI): {self.front_lidar_topic}\n"
            f"  TrailerState out: {self.tractor_odom_out_topic}\n"
            f"  trailer_length_m: {self.trailer_length_m}\n"
            f"  hitch_offset_x/y: {self.hitch_offset_x_m}, {self.hitch_offset_y_m}\n"
            f"  hitch_angle_offset_deg: {self.hitch_angle_offset_deg} (default -90 to make straight ~0)\n"
        )

    # -------------------
    # Callbacks
    # -------------------
    def _on_odom(self, msg: Odometry):
        self.last_tractor_odom = msg
        self._publish_trailer_state()

    def _on_lidar(self, msg: PointCloud2):
        # Split & publish front cloud (XYZI), keep back points XYZ for hitch estimation
        front_msg, back_xyz = self._split_lidar_front_back_xyzi(msg)
        if front_msg is not None:
            self.front_pub.publish(front_msg)

        if back_xyz is None or back_xyz.shape[0] == 0:
            return

        hitch_pts = self._filter_trailer_face(back_xyz)
        if hitch_pts.shape[0] < 10:
            return

        est_deg = self._compute_hitch_angle_deg_estimator_convention(hitch_pts)
        self.last_hitch_angle_deg_est = float(est_deg)

        if self.hitch_pub is not None:
            out = Float64()
            out.data = self.last_hitch_angle_deg_est
            self.hitch_pub.publish(out)

    # -------------------
    # LiDAR split (XYZI)
    # -------------------
    def _split_lidar_front_back_xyzi(self, cloud_msg: PointCloud2):
        """
        Front publish: XYZI PointCloud2 (xyz + intensity).
        Back output: Nx3 XYZ numpy for hitch estimator (you only use xyz).

        If intensity is absent, it is set to 0.0 in the published front cloud.
        """
        # Detect intensity field presence
        field_names = [f.name for f in cloud_msg.fields]
        has_intensity = 'intensity' in field_names

        if has_intensity:
            pts = np.array(
                [p for p in point_cloud2.read_points(
                    cloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
                )],
                dtype=np.float32
            )
            xyz = pts[:, 0:3]
            intensity = pts[:, 3:4]
        else:
            xyz = np.array(
                [p for p in point_cloud2.read_points(
                    cloud_msg, field_names=("x", "y", "z"), skip_nans=True
                )],
                dtype=np.float32
            )
            if xyz.size == 0:
                return None, None
            intensity = np.zeros((xyz.shape[0], 1), dtype=np.float32)
            pts = np.hstack([xyz, intensity])

        if xyz.size == 0:
            return None, None

        az = np.arctan2(xyz[:, 1], xyz[:, 0])
        front_mask = np.abs(az) <= self.front_half_angle
        back_mask = ~front_mask

        front_pts = pts[front_mask]         # Nx4 (x,y,z,i)
        back_xyz = xyz[back_mask]           # Nx3 (x,y,z)

        # Build XYZI PointCloud2 for front points
        fields = [
            point_cloud2.PointField(name='x', offset=0,  datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4,  datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8,  datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='intensity', offset=12, datatype=point_cloud2.PointField.FLOAT32, count=1),
        ]
        header = cloud_msg.header
        front_cloud = point_cloud2.create_cloud(header, fields, front_pts.tolist())

        return front_cloud, back_xyz

    # -------------------
    # Hitch ROI + angle
    # -------------------
    def _filter_trailer_face(self, points_xyz: np.ndarray) -> np.ndarray:
        # Your ROI shift depends on previous angle => this is the “starts near 0” assumption part.
        if self.last_hitch_angle_deg_est <= (90.0 - 20.0):   # ~<= 70 in your convention
            self.roi_adjustment = -0.2
        elif self.last_hitch_angle_deg_est >= (90.0 + 20.0): # ~>= 110 in your convention
            self.roi_adjustment = 0.2
        else:
            self.roi_adjustment = 0.0

        x_min, x_max = self.roi_x_min, self.roi_x_max
        y_min = self.roi_y_min + self.roi_adjustment
        y_max = self.roi_y_max + self.roi_adjustment
        z_min, z_max = self.roi_z_min, self.roi_z_max

        m = (
            (points_xyz[:, 0] > x_min) & (points_xyz[:, 0] < x_max) &
            (points_xyz[:, 1] > y_min) & (points_xyz[:, 1] < y_max) &
            (points_xyz[:, 2] > z_min) & (points_xyz[:, 2] < z_max)
        )
        return points_xyz[m]

    def _compute_hitch_angle_deg_estimator_convention(self, points_xyz: np.ndarray) -> float:
        """
        Returns angle in YOUR CURRENT convention (includes +90 shift),
        so that your existing thresholds/ROI behavior still matches.
        """
        mean_point = np.mean(points_xyz, axis=0)
        centered = points_xyz - mean_point

        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eig(cov)
        normal = eigvecs[:, np.argmin(eigvals)]

        if float(np.dot(normal, mean_point)) > 0.0:
            normal = -normal

        z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        trailer_dir = np.cross(normal, z_axis)

        trailer_dir_xy = trailer_dir[:2]
        n = np.linalg.norm(trailer_dir_xy)
        if n < 1e-6:
            return self.last_hitch_angle_deg_est

        trailer_xy = trailer_dir_xy / n

        truck_forward = np.array([1.0, 0.0], dtype=np.float64)
        dot = float(np.dot(truck_forward, trailer_xy))
        det = float(truck_forward[0] * trailer_xy[1] - truck_forward[1] * trailer_xy[0])
        angle_rad = math.atan2(det, dot)
        angle_deg = math.degrees(angle_rad) + 90.0  # keep your current behavior
        return angle_deg

    # -------------------
    # Trailer pose compute (NO TF)
    # -------------------
    def _publish_trailer_state(self):
        if self.last_tractor_odom is None:
            return

        stamp = self.get_clock().now().to_msg()

        # Tractor pose from /odom
        tractor_pose_in = self.last_tractor_odom.pose.pose

        tractor_x = tractor_pose_in.position.x
        tractor_y = tractor_pose_in.position.y
        tractor_yaw = yaw_from_quat(tractor_pose_in.orientation)

        # Hitch point in map frame (apply hitch offset expressed in tractor frame)
        hx = tractor_x + math.cos(tractor_yaw) * self.hitch_offset_x_m - math.sin(tractor_yaw) * self.hitch_offset_y_m
        hy = tractor_y + math.sin(tractor_yaw) * self.hitch_offset_x_m + math.cos(tractor_yaw) * self.hitch_offset_y_m

        # Convert estimated hitch angle -> relative yaw offset (rad)
        # rel_deg ~ 0 when straight (with default offset=-90)
        rel_deg = self.last_hitch_angle_deg_est + self.hitch_angle_offset_deg
        rel_rad = math.radians(rel_deg)

        # Trailer yaw in map frame
        trailer_yaw = tractor_yaw + rel_rad

        # Trailer reference point = hitch minus trailer_length along trailer forward axis
        tx = hx - math.cos(trailer_yaw) * self.trailer_length_m
        ty = hy - math.sin(trailer_yaw) * self.trailer_length_m

        # Build trailer pose
        trailer_pose = Pose()
        trailer_pose.position.x = tx
        trailer_pose.position.y = ty
        trailer_pose.position.z = tractor_pose_in.position.z  # assume planar; keep same z
        qx, qy, qz, qw = quat_from_yaw(trailer_yaw)
        trailer_pose.orientation.x = qx
        trailer_pose.orientation.y = qy
        trailer_pose.orientation.z = qz
        trailer_pose.orientation.w = qw

        # Publish TrailerState with two odoms
        msg = TrailerState()
        msg.time_now = stamp

        # Tractor odom out: keep the incoming tractor pose (but set frame_id to target_frame)
        tractor_pose_out = Pose()
        tractor_pose_out.position = tractor_pose_in.position
        tractor_pose_out.orientation = tractor_pose_in.orientation

        msg.odoms.append(odom_from_pose(tractor_pose_out, stamp, self.target_frame))
        msg.odoms.append(odom_from_pose(trailer_pose, stamp, self.target_frame))

        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TractorTrailerOdomBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()