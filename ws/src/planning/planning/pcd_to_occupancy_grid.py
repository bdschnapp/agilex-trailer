import math
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PcdToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('pcd_to_occupancy_grid')

        # --- Parameters ---
        # Input
        self.declare_parameter('cloud_topic', '/points')                  # PointCloud2 topic
        self.declare_parameter('frame_id', 'map')                         # Output map frame
        self.declare_parameter('z_min', -0.20)                            # meters: exclude floor below this
        self.declare_parameter('z_max',  2.00)                            # meters: exclude high points above this
        self.declare_parameter('downsample_voxel', 0.05)                  # meters: 0 disables (set <=0)

        # Grid
        self.declare_parameter('resolution', 0.05)                        # meters/cell
        self.declare_parameter('size_x', 60.0)                            # meters (width)
        self.declare_parameter('size_y', 60.0)                            # meters (height)
        self.declare_parameter('origin_x', -30.0)                         # meters (map origin lower-left X)
        self.declare_parameter('origin_y', -30.0)                         # meters (map origin lower-left Y)

        # Occupancy logic
        self.declare_parameter('min_hits_per_cell', 1)                    # threshold to mark occupied
        self.declare_parameter('inflate_radius', 0.20)                    # meters (0 = no inflation)
        self.declare_parameter('occupied_value', 100)                     # 0..100
        self.declare_parameter('free_value', 0)                           # 0..100
        self.declare_parameter('unknown_value', -1)                       # -1 for unknown
        self.declare_parameter('latched', True)                           # publish latched (latching via QoS not here)

        self.cloud_topic       = self.get_parameter('cloud_topic').value
        self.frame_id          = self.get_parameter('frame_id').value
        self.z_min             = float(self.get_parameter('z_min').value)
        self.z_max             = float(self.get_parameter('z_max').value)
        self.voxel             = float(self.get_parameter('downsample_voxel').value)

        self.res               = float(self.get_parameter('resolution').value)
        self.size_x_m          = float(self.get_parameter('size_x').value)
        self.size_y_m          = float(self.get_parameter('size_y').value)
        self.origin_x          = float(self.get_parameter('origin_x').value)
        self.origin_y          = float(self.get_parameter('origin_y').value)

        self.min_hits          = int(self.get_parameter('min_hits_per_cell').value)
        self.inflate_radius_m  = float(self.get_parameter('inflate_radius').value)
        self.occ_val           = int(self.get_parameter('occupied_value').value)
        self.free_val          = int(self.get_parameter('free_value').value)
        self.unknown_val       = int(self.get_parameter('unknown_value').value)

        # Derived sizes
        self.width_cells  = int(round(self.size_x_m / self.res))
        self.height_cells = int(round(self.size_y_m / self.res))
        self.get_logger().info(f"Grid: {self.width_cells}x{self.height_cells} cells @ {self.res:.3f} m")

        # Publisher
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', 1)

        # Subscriber
        self.sub = self.create_subscription(PointCloud2, self.cloud_topic, self.pc_cb, 10)

        # Precompute inflation offsets (disk mask)
        self.inflate_offsets = self._make_inflate_offsets(self.inflate_radius_m, self.res)

    def _make_inflate_offsets(self, radius_m, res):
        if radius_m <= 0.0:
            return []
        r_cells = int(math.floor(radius_m / res))
        off = []
        r2 = r_cells * r_cells
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                if dx*dx + dy*dy <= r2:
                    off.append((dx, dy))
        return off

    def pc_cb(self, msg: PointCloud2):
        # Extract XYZ
        points_iter = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.fromiter(points_iter, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        if pts.size == 0:
            self.get_logger().warn("Received empty point cloud.")
            return

        xyz = np.vstack((pts['x'], pts['y'], pts['z'])).T

        # Height crop
        mask = (xyz[:, 2] >= self.z_min) & (xyz[:, 2] <= self.z_max)
        xyz = xyz[mask]
        if xyz.shape[0] == 0:
            self.get_logger().warn("All points filtered by z_min/z_max.")
            return

        # Optional voxel downsample (simple grid hashing)
        if self.voxel > 0.0:
            v = self.voxel
            keys = np.floor(xyz[:, :3] / v).astype(np.int32)
            # unique by voxel key
            _, idx = np.unique(keys, axis=0, return_index=True)
            xyz = xyz[idx]

        # Project to 2D grid indices
        gx = np.floor((xyz[:, 0] - self.origin_x) / self.res).astype(np.int32)
        gy = np.floor((xyz[:, 1] - self.origin_y) / self.res).astype(np.int32)

        # Keep points inside bounds
        inb = (gx >= 0) & (gx < self.width_cells) & (gy >= 0) & (gy < self.height_cells)
        gx, gy = gx[inb], gy[inb]

        if gx.size == 0:
            self.get_logger().warn("No points within map bounds.")
            return

        # Accumulate hits per cell
        # Use dict of counts to avoid massive dense arrays for big maps
        hits = defaultdict(int)
        for x, y in zip(gx, gy):
            hits[(x, y)] += 1

        # Build occupancy array
        grid = np.full((self.height_cells, self.width_cells), self.unknown_val, dtype=np.int16)

        # Mark free as default known-free if you prefer (optional)
        # grid[:, :] = self.free_val

        # Occupied by threshold
        for (x, y), c in hits.items():
            if c >= self.min_hits:
                grid[y, x] = self.occ_val
            else:
                # You can optionally tag low-hit cells as free
                if self.free_val >= 0:
                    grid[y, x] = self.free_val

        # Inflate obstacles (in place)
        if self.inflate_offsets:
            occ_cells = np.argwhere(grid == self.occ_val)
            for (y, x) in occ_cells:
                for dx, dy in self.inflate_offsets:
                    xx, yy = x + dx, y + dy
                    if 0 <= xx < self.width_cells and 0 <= yy < self.height_cells:
                        if grid[yy, xx] != self.occ_val:
                            grid[yy, xx] = self.occ_val

        # Publish OccupancyGrid
        og = OccupancyGrid()
        og.header = Header()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = self.frame_id

        meta = MapMetaData()
        meta.resolution = float(self.res)
        meta.width = int(self.width_cells)
        meta.height = int(self.height_cells)
        meta.origin.position.x = float(self.origin_x)
        meta.origin.position.y = float(self.origin_y)
        meta.origin.position.z = 0.0
        meta.origin.orientation.w = 1.0
        og.info = meta

        # OccupancyGrid expects row-major 1D list, 0..100 or -1
        og.data = grid.flatten(order='C').astype(np.int8).tolist()

        self.pub_map.publish(og)
        self.get_logger().info(f"Published /map with {self.width_cells}x{self.height_cells} cells")

def main():
    rclpy.init()
    node = PcdToOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()