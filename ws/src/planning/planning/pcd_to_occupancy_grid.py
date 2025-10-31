import math
from collections import defaultdict, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PcdToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('pcd_to_occupancy_grid')

        # --- Parameters ---
        # Input
        self.declare_parameter('cloud_topic', '/points')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('z_min', -0.20)
        self.declare_parameter('z_max',  2.00)
        self.declare_parameter('downsample_voxel', 0.05)

        # Grid
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('size_x', 60.0)
        self.declare_parameter('size_y', 60.0)
        self.declare_parameter('origin_x', -30.0)
        self.declare_parameter('origin_y', -30.0)

        # Occupancy logic
        self.declare_parameter('min_hits_per_cell', 1)
        self.declare_parameter('occupied_value', 100)
        self.declare_parameter('free_value', 0)
        self.declare_parameter('unknown_value', -1)

        # NEW: gradient inflation (Nav2-like)
        self.declare_parameter('inflation_radius', 1.2)         # meters
        self.declare_parameter('inscribed_radius', 0.35)         # meters (robot radius / half-width)
        self.declare_parameter('cost_scaling_factor', 0.1)       # lower -> wider halo
        self.declare_parameter('inflate_into_unknown', False)    # typically False

        # Publication behavior
        self.declare_parameter('latched', True)  # publish once and stay latched for static maps

        # Read params
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
        self.occ_val           = int(self.get_parameter('occupied_value').value)
        self.free_val          = int(self.get_parameter('free_value').value)
        self.unknown_val       = int(self.get_parameter('unknown_value').value)

        self.infl_rad          = float(self.get_parameter('inflation_radius').value)
        self.insc_rad          = float(self.get_parameter('inscribed_radius').value)
        self.scale             = float(self.get_parameter('cost_scaling_factor').value)
        self.inflate_unknown   = bool(self.get_parameter('inflate_into_unknown').value)
        self.latched           = bool(self.get_parameter('latched').value)

        # Derived sizes
        self.width_cells  = int(round(self.size_x_m / self.res))
        self.height_cells = int(round(self.size_y_m / self.res))
        self.get_logger().info(f"Grid: {self.width_cells}x{self.height_cells} cells @ {self.res:.3f} m")

        # Publisher (latched / transient local if requested)
        if self.latched:
            qos = QoSProfile(depth=1)
            qos.history = QoSHistoryPolicy.KEEP_LAST
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.pub_map = self.create_publisher(OccupancyGrid, '/map', qos)
        else:
            self.pub_map = self.create_publisher(OccupancyGrid, '/map', 1)

        # Subscriber
        self.sub = self.create_subscription(PointCloud2, self.cloud_topic, self.pc_cb, 10)

    # -------------------- Helpers --------------------

    def _bfs_distance_map(self, occ_mask):
        """8-connected BFS distance (meters) from nearest occupied cell up to inflation_radius."""
        H, W = occ_mask.shape
        inf = np.inf
        dist = np.full((H, W), inf, dtype=np.float32)
        q = deque()

        # Seed with occupied cells
        ys, xs = np.where(occ_mask)
        for y, x in zip(ys, xs):
            dist[y, x] = 0.0
            q.append((y, x))

        if not q:
            return dist  # empty map

        step4 = self.res
        stepd = self.res * math.sqrt(2.0)
        neigh = [(-1,0,step4),(1,0,step4),(0,-1,step4),(0,1,step4),
                 (-1,-1,stepd),(-1,1,stepd),(1,-1,stepd),(1,1,stepd)]

        max_d = self.infl_rad

        while q:
            y, x = q.popleft()
            base = dist[y, x]
            if base > max_d:
                continue
            for dy, dx, step in neigh:
                yy, xx = y+dy, x+dx
                if 0 <= yy < H and 0 <= xx < W:
                    nd = base + step
                    if nd < dist[yy, xx] and nd <= max_d:
                        dist[yy, xx] = nd
                        q.append((yy, xx))
        return dist

    def _apply_gradient_inflation(self, grid):
        """Write graded costs (1..occ_val-1) around obstacles."""
        H, W = grid.shape
        occ = (grid == self.occ_val)

        # Distance (meters) to nearest obstacle, limited to inflation_radius
        dist = self._bfs_distance_map(occ)

        # Apply costs where distance is finite
        for y in range(H):
            for x in range(W):
                d = dist[y, x]
                if not np.isfinite(d):
                    continue  # untouched

                # Skip unknown if not inflating into it
                if (grid[y, x] == self.unknown_val) and not self.inflate_unknown:
                    continue

                if d <= self.insc_rad:
                    # lethal within inscribed radius
                    grid[y, x] = self.occ_val
                elif d <= self.infl_rad:
                    # Exponential decay like Nav2: cost in [1..occ_val-1]
                    c = (self.occ_val - 1) * math.exp(- self.scale * (d - self.insc_rad))
                    c = int(max(1, min(self.occ_val - 1, round(c))))
                    if grid[y, x] != self.occ_val:
                        grid[y, x] = max(grid[y, x], c)  # don’t overwrite higher costs

    # -------------------- Callback --------------------

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

        # Optional voxel downsample
        if self.voxel > 0.0:
            v = self.voxel
            keys = np.floor(xyz[:, :3] / v).astype(np.int32)
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
        hits = defaultdict(int)
        for x, y in zip(gx, gy):
            hits[(x, y)] += 1

        # Build occupancy array (unknown by default)
        grid = np.full((self.height_cells, self.width_cells), self.unknown_val, dtype=np.int16)

        # Mark free by default (optional but recommended for planners)
        grid[:, :] = self.free_val

        # Set occupied
        for (x, y), c in hits.items():
            if c >= self.min_hits:
                grid[y, x] = self.occ_val

        # Gradient inflation
        if self.infl_rad > 0.0 and self.scale > 0.0:
            self._apply_gradient_inflation(grid)

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

        og.data = grid.flatten(order='C').astype(np.int8).tolist()
        self.pub_map.publish(og)
        self.get_logger().info(f"Published /map with {self.width_cells}x{self.height_cells} cells")

        # If this map is static, you can unsubscribe after first publish
        if self.latched:
            self.get_logger().info("Latched /map published. Unsubscribing from point cloud.")
            self.destroy_subscription(self.sub)

def main():
    rclpy.init()
    node = PcdToOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()