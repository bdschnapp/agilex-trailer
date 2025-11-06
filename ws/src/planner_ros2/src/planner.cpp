#include "planner_ros2/planner.h"

namespace trailer_planner
{
    Planner::Planner(const std::string& node_name) : Node(node_name)
    {
    }

    void Planner::init()
    {
        trailer.reset(new Trailer);
        grid_map.reset(new GridMap);
        
        trailer->init(shared_from_this());
        grid_map->init(shared_from_this());

        hybrid_astar.init(shared_from_this());
        hybrid_astar.setTrailerEnv(trailer, grid_map);
        arc_opt.init(shared_from_this());
        arc_opt.setTrailerEnv(trailer, grid_map);

        front_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/front_path", 1);
        end_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/end_path", 1);
        arc_traj_pub = this->create_publisher<planner_ros2::msg::ArcTrailerTraj>("/arc_trailer_traj", 1);

        goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1,
            std::bind(&Planner::rcvGoalCallBack, this, std::placeholders::_1));
        
        odom_sub = this->create_subscription<planner_ros2::msg::TrailerState>(
            "~/odom", 1,
            std::bind(&Planner::rcvOdomCallBack, this, std::placeholders::_1));
    
        start_pos.resize(TRAILER_NUM+3);
        end_pos.resize(TRAILER_NUM+3);
        front_path.clear();

        return;
    }

    void Planner::rcvOdomCallBack(const planner_ros2::msg::TrailerState::SharedPtr msg)
    {
        has_odom = true;
        Eigen::VectorXd s;
        s.resize(TRAILER_NUM+3);
        s[0] = msg->odoms[0].pose.pose.position.x;
        s[1] = msg->odoms[0].pose.pose.position.y;
        s[2] = 2.0 * atan2(msg->odoms[0].pose.pose.orientation.z, msg->odoms[0].pose.pose.orientation.w);
        for (int i=1; i<=TRAILER_NUM; i++)
            s[2+i] = 2.0 * atan2(msg->odoms[i].pose.pose.orientation.z, msg->odoms[i].pose.pose.orientation.w);
        odom_pos = s;
        odom_vw[0] = msg->odoms[0].twist.twist.linear.x;
        odom_vw[1] = msg->odoms[0].twist.twist.angular.z;
        return;
    }

    void Planner::rcvGoalCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_WARN(this->get_logger(), "[Planner] Get target pose, begin planning.");

        Eigen::VectorXd far;
        far.resize(TRAILER_NUM+3);
        far.fill(1000);

        if (in_plan)
            return;

        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;
        Eigen::Vector3d wps(p.x, p.y, 2.0 * atan2(q.z, q.w));
        
        double dpos = (wps.head(2) - odom_pos.head(2)).norm();
        double dyaw = fabs(atan2(sin(wps(2) - odom_pos(2)), cos(wps(2) - odom_pos(2))));
        if (dpos < 0.1 && dyaw < 0.1)
            return;
        
        if (!has_odom)
        {
            RCLCPP_ERROR(this->get_logger(), "[Planner] No odom received, cannot plan.");
            return;
        }
        start_pos = odom_pos;
        end_pos = wps;
        trailer->setShowTerminal(wps);
        in_plan = true;
        if (plan(start_pos, end_pos))
        {
            RCLCPP_INFO(this->get_logger(), "[Planner] Planning done.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[Planner] Planning failed, check bugs!");
            front_path.clear();
        }
        in_plan = false;

        return;
    }

    bool Planner::plan(Eigen::VectorXd start, Eigen::VectorXd end)
    {
        assert(end.size() == 3);

        Eigen::VectorXd end_full = end;
        end_full.resize(TRAILER_NUM+3);
        end_full.setConstant(end(2));
        Eigen::Vector2d half_box(trailer->box_length/2.0, trailer->box_width/2.0);
        end_full.head(2) = end.head(2) + Eigen::Vector2d(cos(end(2)), sin(end(2))) 
                                        * (trailer->box_length/2.0 - trailer->length[0] + trailer->rear_length - trailer->terminal_tol/2.0);
        
        if (!hybrid_astar.isValid(end_full))
        {
            RCLCPP_ERROR(this->get_logger(), "[Planner] End state is not valid, cannot plan.");
            return false;

        }

        // use simple front end
        front_path = hybrid_astar.pureAstarPlan(start, end);

        if (front_path.empty())
            return false;

        vis_front();

        // arc opt
        if (!arc_opt.optimizeTraj(front_path, 0.0))
            return false;

        arc_traj = arc_opt.getTraj();

        vis_end();
        pub_end();

        return true;
    }

    void Planner::vis_front()
    {
        if (front_path.empty())
            return;

        visualization_msgs::msg::MarkerArray front_msg;
        std::vector<Eigen::VectorXd> se2_path; 
        for (size_t i=0; i<front_path.size(); i++)
        {
            Eigen::VectorXd se2_state;
            trailer->gainSE2State(front_path[i], se2_state);
            se2_path.push_back(se2_state);
        }
        for (size_t i=0; i<TRAILER_NUM+1; i++)
        {
            visualization_msgs::msg::Marker sphere, line_strip;
            sphere.header.frame_id = line_strip.header.frame_id = "world";
            sphere.header.stamp = line_strip.header.stamp = this->now();
            sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
            sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
            sphere.id = i;
            line_strip.id = i + 1000;

            sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
            sphere.color.r = line_strip.color.r = trailer->array_msg.markers[i].color.r;
            sphere.color.g = line_strip.color.g = trailer->array_msg.markers[i].color.g;
            sphere.color.b = line_strip.color.b = trailer->array_msg.markers[i].color.b;
            sphere.color.a = line_strip.color.a = 1;
            sphere.scale.x = 0.08;
            sphere.scale.y = 0.08;
            sphere.scale.z = 0.08;
            line_strip.scale.x = 0.08 / 2;
            geometry_msgs::msg::Point pt;
            
            for (auto p:se2_path)
            {
                pt.x = p[3*i];
                pt.y = p[3*i+1];
                pt.z = 0.0;
                line_strip.points.push_back(pt);
                sphere.points.push_back(pt);
            }
            front_msg.markers.push_back(line_strip);
            front_msg.markers.push_back(sphere);
        }
        front_pub->publish(front_msg);
    }

    void Planner::vis_end()
    {
        double ttime_duration;
        ttime_duration = arc_traj.getTotalDuration();
        if (ttime_duration<1e-3)
            return;

        visualization_msgs::msg::MarkerArray end_msg;
        std::vector<Eigen::VectorXd> se2_path;

        for (double i = 0; i < ttime_duration + 0.01; i+=0.01)
        {
            double t = i;
            if (i > ttime_duration)
                t = ttime_duration;
            
            Eigen::VectorXd se2_state;
            Eigen::VectorXd state;
            state = arc_traj.getState(t);
            trailer->gainSE2State(state, se2_state);
            se2_path.push_back(se2_state);
        }

        for (size_t i=0; i<TRAILER_NUM+1; i++)
        {
            double scale = 0.12;
            visualization_msgs::msg::Marker line_strip;
            line_strip.header.frame_id = "world";
            line_strip.header.stamp = this->now();
            line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_strip.action = visualization_msgs::msg::Marker::ADD;
            line_strip.id = i + 1000;

            line_strip.pose.orientation.w = 1.0;
            line_strip.color.r = trailer->array_msg.markers[i].color.r;
            line_strip.color.g = trailer->array_msg.markers[i].color.g;
            line_strip.color.b = trailer->array_msg.markers[i].color.b;
            line_strip.color.a = 1;
            line_strip.scale.x = scale / 2;
            geometry_msgs::msg::Point pt;
            
            for (auto p:se2_path)
            {
                pt.x = p[3*i];
                pt.y = p[3*i+1];
                pt.z = 0.0;
                line_strip.points.push_back(pt);
            }
            end_msg.markers.push_back(line_strip);
        }
        end_pub->publish(end_msg);
    }

    void Planner::pub_end()
    {
        planner_ros2::msg::ArcTrailerTraj traj_msg;
        traj_msg.head.start_time = traj_msg.tails.start_time = this->now();
        traj_msg.arc.start_time = this->now();
        Eigen::VectorXd durs = arc_traj.head.getDurations();
        for (int i=0; i<durs.size(); i++)
        {
            traj_msg.head.durations.push_back((float)durs[i]);
            std_msgs::msg::Float32MultiArray coeff_mat;
            for (int j=0; j<2; j++)
            {
                for (int k=0; k<6; k++)
                {
                    coeff_mat.data.push_back((float)(arc_traj.head[i].getCoeffMat()(j, k)));
                }
            }
            traj_msg.head.coeff.push_back(coeff_mat);
        }
        durs = arc_traj.tails.getDurations();
        for (int i=0; i<durs.size(); i++)
        {
            traj_msg.tails.durations.push_back((float)durs[i]);
            std_msgs::msg::Float32MultiArray coeff_mat;
            for (int j=0; j<TRAILER_NUM; j++)
            {
                for (int k=0; k<6; k++)
                {
                    coeff_mat.data.push_back((float)(arc_traj.tails[i].getCoeffMat()(j, k)));
                }
            }
            traj_msg.tails.coeff.push_back(coeff_mat);
        }
        durs = arc_traj.arc.getDurations();
        for (int i=0; i<durs.size(); i++)
        {
            traj_msg.arc.durations.push_back((float)durs[i]);
            std_msgs::msg::Float32MultiArray coeff_mat;
            for (int j=0; j<1; j++)
            {
                for (int k=0; k<6; k++)
                {
                    coeff_mat.data.push_back((float)(arc_traj.arc[i].getCoeffMat()(j, k)));
                }
            }
            traj_msg.arc.coeff.push_back(coeff_mat);
        }

        arc_traj_pub->publish(traj_msg);
        return;
    }

}
