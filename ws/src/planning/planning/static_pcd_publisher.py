import rclpy, struct
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import numpy as np

def load_pcd_xyz(pcd_path):
    # minimal ASCII/ASCII-like reader; for binary PCDs, use pcl or open3d
    xs, ys, zs = [], [], []
    with open(pcd_path, 'r') as f:
        header = True
        for line in f:
            if header:
                if line.strip().startswith('DATA'):
                    header = False
                continue
            vals = line.strip().split()
            if len(vals) >= 3:
                xs.append(float(vals[0]))
                ys.append(float(vals[1]))
                zs.append(float(vals[2]))
    return np.column_stack([xs, ys, zs]).astype(np.float32)

def make_cloud(points_xyz, frame_id='map', stamp=None):
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    if stamp is None:
        msg.header.stamp = Time(sec=0, nanosec=0)  # timeless for static map
    else:
        msg.header.stamp = stamp

    msg.height = 1
    msg.width = points_xyz.shape[0]
    msg.is_bigendian = False
    msg.is_dense = True
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.data = points_xyz.tobytes()
    return msg

class StaticPCDPublisher(Node):
    def __init__(self, pcd_path, frame='map'):
        super().__init__('static_pcd_publisher')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # latch
        )
        self.pub = self.create_publisher(PointCloud2, '/pcd_map', qos)
        pts = load_pcd_xyz(pcd_path)
        cloud = make_cloud(pts, frame_id=frame)
        self.pub.publish(cloud)
        self.get_logger().info(f'Published static map with {pts.shape[0]} points in frame "{frame}"')
        # keep the node alive so latched data remains available
        self.timer = self.create_timer(1.0, lambda: None)

def main():
    import sys
    rclpy.init()
    if len(sys.argv) < 2:
        print("usage: static_pcd_publisher.py /path/to/map.pcd [frame_id]")
        return
    frame = sys.argv[2] if len(sys.argv) > 2 else 'map'
    node = StaticPCDPublisher(sys.argv[1], frame)
    rclpy.spin(node)

if __name__ == '__main__':
    main()