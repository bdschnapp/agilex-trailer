#include "planner_ros2/grid_map.h"
#include <pcl/filters/passthrough.h>

namespace trailer_planner
{
    void GridMap::init(rclcpp::Node::SharedPtr node)
    {
        this->node_ = node;
        map_size[0] = node->declare_parameter("grid_map/map_size_x", map_size[0]);
        map_size[1] = node->declare_parameter("grid_map/map_size_y", map_size[1]);
        resolution = node->declare_parameter("grid_map/resolution", resolution);

        // NEW: z filtering params (defaults: keep -0.20 m up to +2.0 m)
        z_min_ = node->declare_parameter("grid_map/z_min", -0.20);
        z_max_ = node->declare_parameter("grid_map/z_max",  2.00);

        esdf_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/esdf_map", 1);
        cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/global_map", 1, std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));
        vis_timer = node->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&GridMap::visCallback, this));

        // origin, boundaries, resolution, voxel counts ... (unchanged)

        buffer_size    = voxel_num(0) * voxel_num(1);
        esdf_buffer    = std::vector<double>(buffer_size, 0.0);
        occ_buffer     = std::vector<char>(buffer_size, 0);        // 0 free, 1 occupied
        custom_buffer  = std::vector<double>(buffer_size, 0.0);
        grid_node_map  = new GridNodePtr[buffer_size];
        for (int i = 0; i < buffer_size; i++) grid_node_map[i] = new GridNode();
        map_ready = false;
    }

    void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // pcl verbosity (unchanged)
        pcl::PointCloud<pcl::PointXYZI> pc_in;
        pcl::fromROSMsg(*msg, pc_in);

        // 1) CLEAR occupancy each frame so stale cells don’t persist
        std::fill(occ_buffer.begin(), occ_buffer.end(), 0);

        // 2) Z-PASS-THROUGH FILTER (drop ground & any returns outside [z_min_, z_max_])
        pcl::PointCloud<pcl::PointXYZI> pc_z;
        {
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud(pc_in.makeShared());
            pass.setFilterFieldName("z");
            pass.setFilterLimits(z_min_, z_max_);
            // pass.setFilterLimitsNegative(false); // keep in-range
            pass.filter(pc_z);
        }

        auto& pc = pc_z;

        // 3) Mark occupancy from filtered points
        std::vector<Eigen::Vector2d> local_p;
        local_p.reserve(100);
        int idx = 0;
        for (size_t i=0; i<pc.points.size(); i++)
        {
            // NOTE: ensure msg frame is gravity-aligned. If not, transform first.
            Eigen::Vector2i id;
            posToIndex(Eigen::Vector2d(pc.points[i].x, pc.points[i].y), id);
            if (isInMap(id))
            {
                occ_buffer[toAddress(id)] = 1;
                if (idx < 100) { local_p.emplace_back(pc.points[i].x, pc.points[i].y); idx++; }
            }
        }

        int pts_size = std::min<int>(local_p.size(), 100);
        local_pts.resize(2, pts_size);
        for (int i = 0; i < pts_size; i++) local_pts.col(i) = local_p[i];

        // 4) Recompute ESDF and build a visualization cloud where z = distance
        updateESDF2d();
        pcl::PointCloud<pcl::PointXYZI> esdf_cloud_pc;
        esdf_cloud_pc.reserve(voxel_num(0) * voxel_num(1));
        for (int i=0; i<voxel_num(0); i++)
        {
            for (int j=0; j<voxel_num(1); j++)
            {
                Eigen::Vector2i id(i, j);
                Eigen::Vector2d pos;
                indexToPos(id, pos);
                pcl::PointXYZI p;
                p.x = pos(0);
                p.y = pos(1);
                p.z = esdf_buffer[toAddress(id)];
                p.intensity = 1.0;
                esdf_cloud_pc.push_back(p);
            }
        }
        esdf_cloud_pc.header.frame_id = "world";
        esdf_cloud_pc.width  = esdf_cloud_pc.size();
        esdf_cloud_pc.height = 1;
        esdf_cloud_pc.is_dense = true;
        pcl::toROSMsg(esdf_cloud_pc, esdf_cloud);  // class member sensor_msgs::msg::PointCloud2

        map_ready = true;
    }
}