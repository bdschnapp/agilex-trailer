import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
import struct

from sensor_msgs_py import point_cloud2

class HitchAngleEstimator(Node):
    def __init__(self):
        super().__init__('hitch_angle_estimator')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.pointcloud_callback,
            10)
        
        self.ROIAdjustment=0 
        self.hitch_angle=0
        self.using_side_face=False

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to numpy array
        points = self.convert_pointcloud2_to_numpy(msg)
        if points.shape[0] == 0:
            self.get_logger().warn("Empty point cloud")
            return

        # Filter region near where trailer front face is expected
        trailer_face_points = self.filter_trailer_face(points)
        if trailer_face_points.shape[0] < 10:
            self.get_logger().warn("Not enough trailer face points detected")
            return

        # Compute hitch angle
        self.hitch_angle = self.compute_hitch_angle(trailer_face_points)
        self.get_logger().info(f"Hitch Angle: {self.hitch_angle:.2f}°")

    def convert_pointcloud2_to_numpy(self, cloud_msg):
        return np.array([
            [p[0], p[1], p[2]] for p in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        ])

    def filter_trailer_face(self, points):
        # Adjust these bounds based on trailer geometry and expected position

        '''
        #once hitch is rotated 40 degrees, start using side of truck trailer to derive angle 
        if self.hitch_angle<=-40.0:
            self.ROIAdjustment=0.15
            self.using_side_face=True
        elif self.hitch_angle>=40.0:
            self.ROIAdjustment+=0.15
            self.using_side_face=True
        else:
            self.using_side_face=False
            #slightly shifting ROI when hitch is at 20 deg 
            if self.hitch_angle<=-20.0:
                self.ROIAdjustment=-0.2
            elif self.hitch_angle>=20.0:
                self.ROIAdjustment=0.2
            else:
                self.ROIAdjustment=0.0
        '''

        #slightly shifting ROI when hitch is at 20 deg 
        if self.hitch_angle<=-20.0:
            self.ROIAdjustment=-0.2
        elif self.hitch_angle>=20.0:
            self.ROIAdjustment=0.2
        else:
            self.ROIAdjustment=0.0

        x_min, x_max = -0.95,-0.30
        y_min, y_max = -0.125+self.ROIAdjustment,0.125+self.ROIAdjustment
        z_min, z_max = -0.1,0.70

        mask = (
            (points[:, 0] > x_min) & (points[:, 0] < x_max) &
            (points[:, 1] > y_min) & (points[:, 1] < y_max) &
            (points[:, 2] > z_min) & (points[:, 2] < z_max)
        )
        return points[mask]
    
    def compute_hitch_angle(self, points):
        # Center points
        mean_point = np.mean(points, axis=0)
        centered = points - mean_point

        # PCA to get face normal
        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eig(cov)
        normal = eigvecs[:, np.argmin(eigvals)]  # least variance = normal

        # Flip normal to point away from LiDAR at origin
        if np.dot(normal, mean_point) > 0:
            normal = -normal

        # Trailer forward direction = horizontal vector perpendicular to the face normal
        z_axis = np.array([0, 0, 1])
        trailer_dir = np.cross(normal, z_axis)
        #trailer_dir = np.cross(z_axis, normal)



            # Rotate by 90 degrees if using side face (i.e., normal is side-facing)
        if self.using_side_face:
            # Rotate trailer_dir in XY plane by ±90 degrees
            trailer_dir_xy = trailer_dir[:2]
            # Rotate clockwise if turning left, CCW if turning right (sign flip based on angle)
            if self.hitch_angle > 0:
                trailer_dir_xy = np.array([-trailer_dir_xy[1], trailer_dir_xy[0]])  # +90°
            else:
                trailer_dir_xy = np.array([trailer_dir_xy[1], -trailer_dir_xy[0]])  # -90°
        else:
            trailer_dir_xy = trailer_dir[:2]



        # Normalize and project to XY
        trailer_dir /= np.linalg.norm(trailer_dir_xy)
        trailer_xy = trailer_dir[:2]

        # Compute hitch angle vs truck X-axis
        truck_forward = np.array([1.0, 0.0])
        dot = np.dot(truck_forward, trailer_xy)
        det = truck_forward[0]*trailer_xy[1] - truck_forward[1]*trailer_xy[0]
        angle_rad = np.arctan2(det, dot)
        angle_deg = np.degrees(angle_rad)+90

        return angle_deg

    
def main(args=None):
    rclpy.init(args=args)
    node = HitchAngleEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
