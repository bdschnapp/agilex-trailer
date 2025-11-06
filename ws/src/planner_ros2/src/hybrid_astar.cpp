#include "planner_ros2/hybrid_astar.h"
#include <cmath>

namespace trailer_planner
{
    void HybridAstar::init(rclcpp::Node::SharedPtr node)
    {
        this->node_ = node;
        yaw_resolution = node->declare_parameter("hybrid_astar/yaw_resolution", yaw_resolution);
        lambda_heu = node->declare_parameter("hybrid_astar/lambda_heu", lambda_heu);
        weight_r2 = node->declare_parameter("hybrid_astar/weight_r2", weight_r2);
        weight_delta = node->declare_parameter("hybrid_astar/weight_delta", weight_delta);
        weight_v_change = node->declare_parameter("hybrid_astar/weight_v_change", weight_v_change);
        weight_delta_change = node->declare_parameter("hybrid_astar/weight_delta_change", weight_delta_change);
        time_interval = node->declare_parameter("hybrid_astar/time_interval", time_interval);
        oneshot_range = node->declare_parameter("hybrid_astar/oneshot_range", oneshot_range);
        check_ratio = node->declare_parameter("hybrid_astar/check_ratio", check_ratio);
        max_vel = node->declare_parameter("hybrid_astar/max_vel", max_vel);
        pos_tol = node->declare_parameter("hybrid_astar/pos_tol", pos_tol);
        theta_tol = node->declare_parameter("hybrid_astar/theta_tol", theta_tol);
        max_time_consume = node->declare_parameter("hybrid_astar/max_time_consume", max_time_consume);
        in_test = node->declare_parameter("hybrid_astar/in_test", in_test);
        heuristic_type = node->declare_parameter("hybrid_astar/heuristic_type", heuristic_type);

        if (in_test)
        {
            expanded_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/hybrid_astar/expanded_points", 10);
        }

        yaw_resolution_inv = 1.0 / yaw_resolution;

        return;
    }

    void HybridAstar::setFullPath(const std::vector<Eigen::VectorXd>& astar_path,
                                std::vector<Eigen::VectorXd>& path)
    {
        Eigen::VectorXd temp_state = path.back();
        for (size_t i=1; i<astar_path.size(); i++)
        {
            Eigen::VectorXd last_state = path.back();
            double yaw = astar_path[i].z();
            // Keep head yaw continuous w.r.t last state to avoid +/-pi flips
            normalizeAngle(last_state(2), yaw);

            // Use arc length between consecutive SE2 points as the forward
            // displacement for the head vehicle to avoid division by near-zero
            // cos/sin which can create unrealistic trailer angles.
            double dx = astar_path[i].x() - astar_path[i-1].x();
            double dy = astar_path[i].y() - astar_path[i-1].y();
            double v = std::hypot(dx, dy);
            double dyaw0 = normalizedAngle(yaw - last_state(2));

            // Set head position and yaw from Ackermann path
            temp_state.head(2) = astar_path[i].head(2);
            temp_state(2) = yaw;
            
            // Apply full truck-trailer kinematics matching stateTransitVel
            double v_propagated = v;
            double w_propagated = dyaw0;  // Start with head vehicle angular velocity
            
            for (size_t j=0; j<TRAILER_NUM; j++)
            {
                double sthetad = sin(last_state(j+2)-last_state(j+3));
                double cthetad = cos(last_state(j+2)-last_state(j+3));
                
                // Full trailer kinematics with proper angular velocity propagation
                double w_temp = w_propagated;
                w_propagated = (v_propagated * sthetad - trailer->Ltail[j] * w_propagated * cthetad) / trailer->Lhead[j];
                
                // Update trailer angle
                temp_state(j+3) = normalizedAngle(last_state(j+3) + w_propagated);
                
                // Enforce jack-knife constraints
                double thetad = temp_state(j+3) - temp_state(j+2);
                if (thetad > M_PI)
                    thetad = thetad - PI_X_2;
                else if (thetad < -M_PI)
                    thetad = thetad + PI_X_2;
                if (thetad > trailer->max_dtheta)
                    temp_state(j+3) = normalizedAngle(temp_state(j+2) + trailer->max_dtheta);
                else if (thetad < -trailer->max_dtheta)
                    temp_state(j+3) = normalizedAngle(temp_state(j+2) - trailer->max_dtheta);
                
                // Keep trailer yaw continuous w.r.t previous sample  
                normalizeAngle(last_state(j+3), temp_state(j+3));
                
                // Propagate velocity with full kinematics
                v_propagated = v_propagated * cthetad + trailer->Ltail[j] * w_temp * sthetad;
            }
            path.push_back(temp_state);
        }

        return;
    }

    std::vector<Eigen::VectorXd> HybridAstar::pureAstarPlan(const Eigen::VectorXd& start_state, const Eigen::VectorXd& end_state)
    {
        assert(end_state.size() == 3);

        if (!set_done)
        {
            RCLCPP_ERROR(node_->get_logger(), "[Hybrid A*] No Setting! Can't begin planning!");
            return front_end_path;
        }

        front_end_path.clear();
        if (!isValid(start_state))
        {
            RCLCPP_ERROR(node_->get_logger(), "[Hybrid A*] start is not free!!!");
            return front_end_path;
        }

        std::vector<double> errors{0.0, 0.0};
        std::vector<double> path_len{1.0e+10, 1.0e+10};
        std::vector<Eigen::VectorXd> ends;
        Eigen::VectorXd end_full;
        Eigen::VectorXd end_temp = end_state;
        trailer->setStateFromBox(end_temp, end_full);
        if (isValid(end_full))
            ends.push_back(end_full);

        end_temp(2) += M_PI;
        trailer->normYaw(end_temp(2));
        trailer->setStateFromBox(end_temp, end_full);
        if (isValid(end_full))
            ends.push_back(end_full);

        // auto astar_result = grid_map->astarPlan(start_state.head(2), end_state.head(2));
        // auto astar_path = astar_result.first;
        front_end_path = planAckermann(start_state, ends);

        return front_end_path;
    }

    std::vector<Eigen::VectorXd> HybridAstar::planAckermann(const Eigen::VectorXd& start_state_full, 
                                                            const std::vector<Eigen::VectorXd>& ends)
    {
        // reset
        int use_node_num = 0;
        int iter_num = 0;
        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
        open_set.swap(empty_queue);
        expanded_nodes.clear();
        expanded_points.clear();
        for (int i = 0; i < allocate_num; i++)
        {
            PathNodePtr node = path_node_pool[i];
            node->parent = NULL;
            node->node_state = NOT_EXPAND;
        }

        Eigen::Vector3d start_state = start_state_full.head(3);
        Eigen::Vector2d end_center = Eigen::Vector2d::Zero();
        std::vector<bool> path_ok;
        std::vector<double> errors;
        std::vector<Eigen::Vector3d> end_states;
        std::vector<std::vector<Eigen::VectorXd>> paths;
        int min_idx = -1;
        double min_error = 1.0e+10;
        for (size_t i=0; i<ends.size(); i++)
        {
            path_ok.push_back(false);
            errors.push_back(0.0);
            end_states.push_back(ends[i].head(3));
            std::vector<Eigen::VectorXd> path;
            paths.push_back(path);
            end_center += end_states[i].head(2);
        }
        end_center /= ends.size();

        rclcpp::Time t0 = node_->now();
        PathNodePtr cur_node = path_node_pool[0];
        cur_node->parent = NULL;
        cur_node->state = start_state;
        stateToIndexAckermann(cur_node->state, cur_node->index);
        cur_node->g_score = 0.0;
        cur_node->input = Eigen::Vector2d::Zero();
        cur_node->f_score = lambda_heu * tie_breaker * (cur_node->state.head(2) - end_center).norm();
        cur_node->node_state = OPEN;

        open_set.push(cur_node);
        use_node_num += 1;
        expanded_nodes.insert(cur_node->index, cur_node);
        
        while (!open_set.empty())
        {
            cur_node = open_set.top();

            visExpanded();

            for (size_t i=0; i<ends.size(); i++)
            {
                if (path_ok[i])
                    continue;

                if((cur_node->state.head(2) - end_states[i].head(2)).norm() < oneshot_range)
                {
                    // rclcpp::Time t1 = node_->now();
                    asignShotTrajAckermann(cur_node->state, end_states[i]);

                    if (!shot_path.empty())
                    {
                        // std::cout << "[Hybrid A*] one-shot time: " << (node_->now()-t1).seconds()*1000 << " ms"<<std::endl;
                        // std::cout << "[Hybrid A*] front once time: " << (node_->now()-t0).seconds()*1000 << " ms"<<std::endl;
                        std::vector<Eigen::VectorXd> ackermann_path;
                        for (int j=shot_path.size()-1; j>=0; j--)
                            ackermann_path.push_back(shot_path[j]);
                        ackermann_path.push_back(cur_node->state);
                        while (cur_node->parent != NULL)
                        {
                            cur_node = cur_node->parent;
                            ackermann_path.push_back(cur_node->state);
                        }
                        reverse(ackermann_path.begin(), ackermann_path.end());
                        std::vector<Eigen::VectorXd> full_path;
                        full_path.push_back(start_state_full);
                        setFullPath(ackermann_path, full_path);
                        path_ok[i] = true;
                        errors[i] = trailer->stateError(ends[i], full_path.back());
                        paths[i] = full_path;
                        if (errors[i] < min_error)
                        {
                            min_error = errors[i];
                            min_idx = i;
                        }
                    }
                }
            }

            bool full_ok = true;
            for (size_t i=0; i<ends.size(); i++)
            {
                if (!path_ok[i])
                {
                    full_ok = false;
                    break;
                }
            }

            if (full_ok)
            {
                std::cout << "[Hybrid A*] front all time: " << (node_->now()-t0).seconds()*1000 << " ms"<<std::endl;
                planning_time = (node_->now()-t0).seconds() * 1000.0;
                return paths[min_idx];
            }

            for (size_t i=0; i<ends.size(); i++)
            {
                if (path_ok[i])
                    continue;

                if ((cur_node->state.head(2)-end_states[i].head(2)).norm() < pos_tol)
                {
                    std::cout << "[Hybrid A*] front once time: " << (node_->now()-t0).seconds()*1000 << " ms"<<std::endl;
                    std::vector<Eigen::VectorXd> ackermann_path;
                    for (int j=shot_path.size()-1; j>=0; j--)
                            ackermann_path.push_back(shot_path[j]);
                    ackermann_path.push_back(cur_node->state);
                    while (cur_node->parent != NULL)
                    {
                        cur_node = cur_node->parent;
                        ackermann_path.push_back(cur_node->state);
                    }
                    reverse(ackermann_path.begin(), ackermann_path.end());
                    std::vector<Eigen::VectorXd> full_path;
                    full_path.push_back(start_state_full);
                    setFullPath(ackermann_path, full_path);
                    path_ok[i] = true;
                    errors[i] = trailer->stateError(ends[i], full_path.back());
                    paths[i] = full_path;
                    if (errors[i] < min_error)
                    {
                        min_error = errors[i];
                        min_idx = i;
                    }
                }
            }

            full_ok = true;
            for (size_t i=0; i<ends.size(); i++)
            {
                if (!path_ok[i])
                {
                    full_ok = false;
                    break;
                }
            }

            if (full_ok)
            {
                std::cout << "[Hybrid A*] front all time: " << (node_->now()-t0).seconds()*1000 << " ms"<<std::endl;
                planning_time = (node_->now()-t0).seconds() * 1000.0;
                return paths[min_idx];
            }

            double time_consume = (node_->now()-t0).seconds();
            if (time_consume > max_time_consume)
            {
                if (min_idx != -1)
                {
                    planning_time = (node_->now()-t0).seconds() * 1000.0;
                    return paths[min_idx];
                }
                else
                    std::cout << "[Hybrid A*] hybrid A* time out, front all time: " << time_consume*1000 << " ms"<<std::endl;
                return paths[0];
            }

            open_set.pop();
            cur_node->node_state = CLOSE;
            iter_num += 1;

            Eigen::VectorXd cur_state = cur_node->state;
            Eigen::VectorXd pro_state;
            Eigen::Vector2d ctrl_input;
            std::vector<Eigen::Vector2d> inputs;

            for (double v = 0.0; v <= max_vel + 1e-3; v += 0.5 * max_vel)
            {
                for (double steer = -trailer->max_steer; steer <= trailer->max_steer + 1e-3; steer += 0.5 * trailer->max_steer)
                {
                    ctrl_input << v, steer;
                    inputs.push_back(ctrl_input);
                }
            }
                
            for (size_t i=0; i<inputs.size(); i++)
            {
                Eigen::Vector2d input = inputs[i];

                stateTransitVelAckermann(cur_state, input, time_interval, pro_state);
                expanded_points.points.push_back(pcl::PointXYZ(pro_state(0), pro_state(1), 0.0));

                Eigen::Vector2d pp = pro_state.head(2);
                if (!grid_map->isInMap(pp))
                    continue;

                Eigen::VectorXi pro_id;
                PathNodePtr pro_node;

                stateToIndexAckermann(pro_state, pro_id);
                pro_node = expanded_nodes.find(pro_id);

                if (pro_node != NULL && pro_node->node_state == CLOSE)
                {
                    continue;
                }

                Eigen::VectorXd xt;
                bool valid = true;
                double temp_ct = check_ratio * time_interval;
                for (double t = 0.0; t < time_interval-1e-4; t+=temp_ct)
                {
                    stateTransitVelAckermann(cur_state, input, t, xt);
                    valid = isValidAckermann(xt);
                    if (!valid)
                        break;
                }
                if (!valid)
                    continue;

                double tmp_g_score = 0.0;
                double tmp_f_score = 0.0;
                double arc = fabs(input(0)) * time_interval;
                tmp_g_score += weight_r2 * arc;
                tmp_g_score += weight_delta * fabs(input(1)) * arc;
                tmp_g_score += weight_v_change * std::fabs(input(0)-cur_node->input(0));
                tmp_g_score += weight_delta_change * std::fabs(input(1)-cur_node->input(1));
                tmp_g_score += cur_node->g_score;
                tmp_f_score = tmp_g_score + lambda_heu * tie_breaker * (pro_state.head(2) - end_center).norm();
                

                if (pro_node == NULL)
                {
                    pro_node = path_node_pool[use_node_num];
                    pro_node->index = pro_id;
                    pro_node->state = pro_state;
                    pro_node->f_score = tmp_f_score;
                    pro_node->g_score = tmp_g_score;
                    pro_node->input = input;
                    pro_node->parent = cur_node;
                    pro_node->node_state = OPEN;
                    open_set.push(pro_node);

                    expanded_nodes.insert(pro_id, pro_node);
                    use_node_num ++;

                    if (use_node_num == allocate_num)
                    {
                        std::cout << "run out of memory." << std::endl;
                        return paths[0];
                    }
                }
                else if (pro_node->node_state == OPEN)
                {
                    if (tmp_g_score < pro_node->g_score)
                    {
                        pro_node->index = pro_id;
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->input = input;
                        pro_node->parent = cur_node;
                    }
                }
            }
        }

        std::cout << "Kino Astar Failed, No path!!!" << std::endl;

        visExpanded();

        return paths[0];
    }

    std::vector<Eigen::VectorXd> HybridAstar::plan(const Eigen::VectorXd& start_state, const Eigen::VectorXd& end_state)
    {
        if (!set_done)
        {
            RCLCPP_ERROR(node_->get_logger(), "[Hybrid A*] No Setting! Can't begin planning!");
            return front_end_path;
        }

        // reset
        int use_node_num = 0;
        int iter_num = 0;
        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
        open_set.swap(empty_queue);
        front_end_path.clear();
        expanded_nodes.clear();
        expanded_points.clear();
        for (int i = 0; i < allocate_num; i++)
        {
            PathNodePtr node = path_node_pool[i];
            node->parent = NULL;
            node->node_state = NOT_EXPAND;
        }

        if (!isValid(start_state))
        {
            RCLCPP_ERROR(node_->get_logger(), "[Hybrid A*] start is not free!!!");
            return front_end_path;
        }
        if (!isValid(end_state))
        {
            RCLCPP_ERROR(node_->get_logger(), "[Hybrid A*] goal is not free!!!");
            return front_end_path;
        }

        rclcpp::Time t0 = node_->now();
        PathNodePtr cur_node = path_node_pool[0];
        cur_node->parent = NULL;
        cur_node->state = start_state;
        stateToIndex(cur_node->state, cur_node->index);
        cur_node->g_score = 0.0;
        cur_node->input = Eigen::Vector2d::Zero();
        cur_node->f_score = lambda_heu * getHeu(cur_node->state, end_state, heuristic_type);
        cur_node->node_state = OPEN;

        Eigen::VectorXi end_index;
        stateToIndex(end_state, end_index);
        Eigen::VectorXd end_se2;
        trailer->gainSE2State(end_state, end_se2);

        open_set.push(cur_node);
        use_node_num += 1;
        expanded_nodes.insert(cur_node->index, cur_node);
        
        while (!open_set.empty())
        {
            cur_node = open_set.top();

            visExpanded();
            
            if((cur_node->state.head(2) - end_state.head(2)).norm() < oneshot_range)
            {
                rclcpp::Time t1 = node_->now();
                asignShotTraj(cur_node->state, end_state);
                if (!shot_path.empty())
                {
                    std::cout << "[Hybrid A*] one-shot time: " << (node_->now()-t1).seconds()*1000 << " ms"<<std::endl;
                    std::cout << "[Hybrid A*] front all time: " << (node_->now()-t0).seconds()*1000 << " ms"<<std::endl;
                    retrievePath(cur_node);
                    planning_time = (node_->now()-t0).seconds() * 1000.0;
                    return front_end_path;
                }
            }

            double time_consume = (node_->now()-t0).seconds();
            // if (isEnd(cur_node->index, end_index))
            if (isClose(cur_node->state, end_state))
            {
                std::cout << "[Hybrid A*] front all time: " << time_consume*1000 << " ms"<<std::endl;
                retrievePath(cur_node);
                planning_time = (node_->now()-t0).seconds() * 1000.0;
                return front_end_path;
            }
            
            if (time_consume > max_time_consume)
            {
                std::cout << "[Hybrid A*] hybrid A* time out, front all time: " << time_consume*1000 << " ms"<<std::endl;
                front_end_path.clear();
                return front_end_path;
                // retrievePath(cur_node);
                // return front_end_path;
            }

            open_set.pop();
            cur_node->node_state = CLOSE;
            iter_num += 1;

            Eigen::VectorXd cur_state = cur_node->state;
            Eigen::VectorXd pro_state;
            Eigen::Vector2d ctrl_input;
            std::vector<Eigen::Vector2d> inputs;

            for (double v = 0.0; v <= max_vel + 1e-3; v += 0.5 * max_vel)
            {
                for (double steer = -trailer->max_steer; steer <= trailer->max_steer + 1e-3; steer += 0.5 * trailer->max_steer)
                {
                    ctrl_input << v, steer;
                    inputs.push_back(ctrl_input);
                }
            }
                
            for (size_t i=0; i<inputs.size(); i++)
            {
                Eigen::Vector2d input = inputs[i];

                trailer->stateTransitVel(cur_state, input, time_interval, pro_state);
                expanded_points.points.push_back(pcl::PointXYZ(pro_state(0), pro_state(1), 0.0));

                Eigen::VectorXd pro_se2_state;
                trailer->gainSE2State(pro_state, pro_se2_state);
                bool in_map = true;
                for  (size_t j=0; j<=TRAILER_NUM; j++)
                {
                    Eigen::Vector2d p = pro_se2_state.segment(j*3, 2);
                    if (!grid_map->isInMap(p))
                    {
                        in_map = false;
                        break;
                    }
                }
                if (!in_map)
                    continue;

                Eigen::VectorXi pro_id;
                PathNodePtr pro_node;

                stateToIndex(pro_state, pro_id);
                pro_node = expanded_nodes.find(pro_id);

                if (pro_node != NULL && pro_node->node_state == CLOSE)
                {
                    continue;
                }

                Eigen::VectorXd xt;
                bool valid = true;
                double temp_ct = check_ratio * time_interval;
                for (double t = 0.0; t < time_interval-1e-4; t+=temp_ct)
                {
                    trailer->stateTransitVel(cur_state, input, t, xt);
                    valid = isValid(xt);
                    // valid = isValidAckermann(xt);
                    if (!valid)
                        break;
                }
                if (!valid)
                    continue;

                double tmp_g_score = 0.0;
                double tmp_f_score = 0.0;
                double arc = fabs(input(0)) * time_interval;
                tmp_g_score += weight_r2 * arc;
                tmp_g_score += weight_delta * fabs(input(1)) * arc;
                tmp_g_score += weight_v_change * std::fabs(input(0)-cur_node->input(0));
                tmp_g_score += weight_delta_change * std::fabs(input(1)-cur_node->input(1));
                tmp_g_score += cur_node->g_score;
                tmp_f_score = tmp_g_score + lambda_heu * getHeu(pro_state, end_state, heuristic_type);

                if (pro_node == NULL)
                {
                    pro_node = path_node_pool[use_node_num];
                    pro_node->index = pro_id;
                    pro_node->state = pro_state;
                    pro_node->f_score = tmp_f_score;
                    pro_node->g_score = tmp_g_score;
                    pro_node->input = input;
                    pro_node->parent = cur_node;
                    pro_node->node_state = OPEN;
                    open_set.push(pro_node);

                    expanded_nodes.insert(pro_id, pro_node);
                    use_node_num ++;

                    if (use_node_num == allocate_num)
                    {
                        std::cout << "run out of memory." << std::endl;
                        front_end_path.clear();
                        return front_end_path;
                    }
                }
                else if (pro_node->node_state == OPEN)
                {
                    if (tmp_g_score < pro_node->g_score)
                    {
                        pro_node->index = pro_id;
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->input = input;
                        pro_node->parent = cur_node;
                    }
                }
            }
        }

        std::cout << "Kino Astar Failed, No path!!!" << std::endl;

        visExpanded();
        front_end_path.clear();

        return front_end_path;
    }

    void HybridAstar::visExpanded()
    {
        if (in_test)
        {
            sensor_msgs::msg::PointCloud2 expanded_msg;
            pcl::toROSMsg(expanded_points, expanded_msg);
            expanded_msg.header.stamp = node_->now();
            expanded_msg.header.frame_id = "world";
            expanded_pub->publish(expanded_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(20));
        }
    }
}
