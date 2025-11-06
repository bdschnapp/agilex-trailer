#pragma once

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string.h>
#include <algorithm>
#include <numeric>

#include <casadi/casadi.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "planner_ros2/msg/trailer_state.hpp"
#include "planner_ros2/trailer.hpp"
#include "utils/traj_anal.hpp"

using namespace std;
using namespace Eigen;
using namespace casadi;

class MPC
{
private:
    // parameters
    /// algorithm param
    double du_th = 0.1;
    double dt = 0.2;
    int Npre = 5;
    int delay_num;

    /// constraints
    trailer_planner::Trailer trailer;
    double max_speed = 55.0 / 3.6;
    double min_speed = -55.0 / 3.6;
    double max_accel = 1.0; 
    double max_dsteer = 1.0;

    // MPC dataset
    casadi::Opti nlp;
    casadi::DM X_sol;
    casadi::DM U_sol;
    casadi::DM x_0; // Initial state
    casadi::DM u_0; // Initial input

    casadi::MX J;
    casadi::MX X;
    casadi::MX U;

    casadi::MX Q;
    casadi::MX R;
    casadi::MX Rd;

    casadi::MX U_min;
    casadi::MX U_max;
    casadi::MX dU_min;
    casadi::MX dU_max;
    std::vector<TrajPoint> xref;
    std::vector<Eigen::Vector2d> output_buff;

    // control data
    bool has_odom;
    bool receive_traj = false;
    TrajPoint now_state;
    Eigen::VectorXd se2_now_state;
    TrajAnalyzer traj_analyzer;

    // ros interface
	rclcpp::Node::SharedPtr node;
    rclcpp::TimerBase::SharedPtr cmd_timer;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr cmd_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr predict_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ref_pub;
    rclcpp::Subscription<planner_ros2::msg::TrailerState>::SharedPtr odom_sub;
    rclcpp::Subscription<planner_ros2::msg::ArcTrailerTraj>::SharedPtr arc_traj_sub;
    ackermann_msgs::msg::AckermannDrive cmd;
    void cmdCallback();
    void rcvOdomCallBack(const planner_ros2::msg::TrailerState::SharedPtr msg);
    void rcvArcTrajCallBack(const planner_ros2::msg::ArcTrailerTraj::SharedPtr msg);

    // MPC function
    void getCmd(void);

    void normlize_theta(double& th)
    {
        while (th > M_PI)
            th -= M_PI * 2;
        while (th < -M_PI)
            th += M_PI * 2;
    }

    void smooth_yaw(vector<TrajPoint> &ref_points)
    {
        for (size_t j=0; j<TRAILER_NUM+1; j++)
        {
            double dyaw = ref_points[0].data(2+j) - now_state.data(2+j);

            while (dyaw >= M_PI / 2)
            {
                ref_points[0].data(2+j) -= M_PI * 2;
                dyaw = ref_points[0].data(2+j) - now_state.data(2+j);
            }
            while (dyaw <= -M_PI / 2)
            {
                ref_points[0].data(2+j) += M_PI * 2;
                dyaw = ref_points[0].data(2+j) - now_state.data(2+j);
            }

            for (int i = 0; i < Npre - 1; i++)
            {
                dyaw = ref_points[i + 1].data(2+j) - ref_points[i].data(2+j);
                while (dyaw >= M_PI / 2)
                {
                    ref_points[i + 1].data(2+j) -= M_PI * 2;
                    dyaw = ref_points[i + 1].data(2+j) - ref_points[i].data(2+j);
                }
                while (dyaw <= -M_PI / 2)
                {
                    ref_points[i + 1].data(2+j) += M_PI * 2;
                    dyaw = ref_points[i + 1].data(2+j) - ref_points[i].data(2+j);
                }
            }
        }

        return;
    }

    MX stateTrans(MX x_now, MX u_now)
    {
        MX x_next;
        MX x_next_x = dt * mtimes(cos(x_now(2)), u_now(0)) + x_now(0);
        MX x_next_y = dt * mtimes(sin(x_now(2)), u_now(0)) + x_now(1);
        MX v = u_now(0);
        MX x_next_theta = dt * u_now(0) * tan(u_now(1)) / trailer.wheel_base + x_now(2);
        x_next = vertcat(x_next_x, x_next_y, x_next_theta);
        for (size_t i=0; i<TRAILER_NUM; i++)
        {
            x_next = vertcat(x_next, x_now(i+2) + dt * v * sin(x_now(i+2)-x_now(i+3)) / trailer.Lhead[i]);
            v = v * cos(x_now(i+2)-x_now(i+3));
        }

        return x_next;
    }

    void drawPredictPath(void)
    {
        int id = 0;
        double sc = 0.05;
        visualization_msgs::msg::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = node->now();
        sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 1;
        sphere.color.b = line_strip.color.b = 0;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::msg::Point pt;
        
        Slice all;
        for (int i=0; i<Npre; i++)
        {
            DM pre_state = X_sol(all, i);
            pt.x = double(pre_state(0, 0));
            pt.y = double(pre_state(1, 0));
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        predict_pub->publish(line_strip);
    }

    void drawRefPath(void)
    {
        int id = 0;
        double sc = 0.05;
        visualization_msgs::msg::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = node->now();
        sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::msg::Point pt;
        
        for (int i=0; i<Npre; i++)
        {
            pt.x = xref[i].x();
            pt.y = xref[i].y();
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        ref_pub->publish(line_strip);
    }

public:
	MPC() {}
    void init(rclcpp::Node::SharedPtr nh);
	~MPC() {}
};
