#pragma once

#include <string.h>
#include <iostream>
#include <random>
#include <time.h>
#include <eigen3/Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "planner_ros2/trailer.hpp"
#include "planner_ros2/grid_map.h"
#include "planner_ros2/hybrid_astar.h"
#include "planner_ros2/arc_opt.h"

#include "planner_ros2/msg/arc_trailer_traj.hpp"
#include "planner_ros2/msg/trailer_state.hpp"

namespace trailer_planner
{
    class Planner : public rclcpp::Node
    {
        private:
            bool has_odom = false;
            bool in_plan = false;

            Eigen::Vector2d odom_vw;
            Eigen::VectorXd odom_pos;
            Eigen::VectorXd start_pos;
            Eigen::VectorXd end_pos;

            // trajs
            std::vector<Eigen::VectorXd> front_path;
            ArcTraj arc_traj;
            
            // members
            Trailer::Ptr trailer;
            GridMap::Ptr grid_map;
            HybridAstar hybrid_astar;
            ArcOpt arc_opt;

            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr front_pub, end_pub;
            rclcpp::Publisher<planner_ros2::msg::ArcTrailerTraj>::SharedPtr arc_traj_pub;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
            rclcpp::Subscription<planner_ros2::msg::TrailerState>::SharedPtr odom_sub;
            
        public:
            Planner(const std::string& node_name);
            void init();
            void rcvGoalCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
            void rcvOdomCallBack(const planner_ros2::msg::TrailerState::SharedPtr msg);
            bool plan(Eigen::VectorXd start, Eigen::VectorXd end);
            void vis_front();
            void vis_end();
            void pub_end();
    };
}
