import math
from typing import Iterable, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class PcdFilter(Node):
    def __init__(self):
        super().__init__('pcd_filter')

        # --- Parameters ---
        self.declare_parameter('input_topic', '/points_raw')      # source cloud
        self.declare_parameter('output_topic', '/global_map')     # filtered cloud (planner subscribes here)
        self.declare_parameter('z_min', 0.05)                     # meters, drop ground below this
        self.declare_parameter('z_max', 2.0)                      # meters, drop overhead above this
        self.declare_parameter('voxel_leaf_xy', 0.0)              # meters, 0 disables voxel downsample (XY only)
        self.declare_parameter('frame_id_override', '')           # set non-empty to force outgoing frame

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.z_min = self.get_parameter('z_min').get_parameter_value().double_value
        self.z_max = self.get_parameter('z_max').get_parameter_value().double_value
        self.leaf = self.get_parameter('voxel_leaf_xy').get_parameter_value().double_value
        self.frame_id_override = self.get_parameter('frame_id_override').get_parameter_value().string_value

        qos = rclpy.qos.QoSProfile(depth=1)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cb, qos)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos)

        self.get_logger().info(
            f'pcd_filter: listening on {self.input_topic} -> publishing {self.output_topic} | '
            f'z∈[{self.z_min:.3f},{self.z_max:.3f}], voxel_leaf_xy={self.leaf}'
        )

    def cb(self, msg: PointCloud2):
        # Convert to Nx3 (only x,y,z) numpy array quickly
        # keep_organized=False flattens the cloud; skip_nans=True filters NaNs
        points_iter = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )
        # Convert generator to array
        pts = np.fromiter((p for p in points_iter), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        if pts.size == 0:
            # Nothing to publish; send an empty cloud with same header
            out = PointCloud2()
            out.header = msg.header
            if self.frame_id_override:
                out.header.frame_id = self.frame_id_override
            self.pub.publish(out)
            return

        xyz = np.vstack((pts['x'], pts['y'], pts['z'])).T  # shape (N, 3)

        # Z pass-through filter
        zmask = (xyz[:, 2] >= self.z_min) & (xyz[:, 2] <= self.z_max)
        xyz = xyz[zmask]
        if xyz.size == 0:
            out = PointCloud2()
            out.header = msg.header
            if self.frame_id_override:
                out.header.frame_id = self.frame_id_override
            self.pub.publish(out)
            return

        # Optional: XY voxel downsample (fast numpy grid)
        if self.leaf and self.leaf > 0.0:
            # Compute voxel indices in XY (ignore Z for grid spacing)
            ix = np.floor(xyz[:, 0] / self.leaf).astype(np.int64)
            iy = np.floor(xyz[:, 1] / self.leaf).astype(np.int64)
            key = ix * 73856093 ^ iy * 19349663  # simple 2D hash
            # Keep one point per voxel (take first index per unique key)
            _, keep_idx = np.unique(key, return_index=True)
            xyz = xyz[keep_idx]

        # Create PointCloud2 (xyz32)
        header = msg.header
        if self.frame_id_override:
            header.frame_id = self.frame_id_override
        out_msg = point_cloud2.create_cloud_xyz32(header, xyz.tolist())

        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = PcdFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()