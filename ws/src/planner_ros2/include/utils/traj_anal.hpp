#pragma once

#include "planner_ros2/msg/arc_trailer_traj.hpp"
#include "planner_ros2/arc_opt.h"
#include "planner_ros2/msg/trailer_state.hpp"
#include <rclcpp/rclcpp.hpp>

class TrajPoint
{
public:
  Eigen::VectorXd data;
  TrajPoint() {}
  TrajPoint(Eigen::VectorXd d): data(d) {}
  double x() const { return data(0); }
  double y() const { return data(1); }
  double theta0() const { return data(2); }
  Eigen::VectorXd thetat() const { return data.tail(data.size()-3); }
};

class TrajAnalyzer
{
private:
  trailer_planner::ArcTraj arc_traj;
  rclcpp::Time start_time;
  double traj_duration;
  bool in_test = false;

public:
    bool at_goal = false;

    TrajAnalyzer() {}

    void setTraj(planner_ros2::msg::ArcTrailerTraj::SharedPtr msg)
    {
      std::vector<trailer_planner::CoefficientMat<1, 5>> cMats_arc;
      std::vector<trailer_planner::CoefficientMat<2, 5>> cMats;
      std::vector<trailer_planner::CoefficientMat<TRAILER_NUM, 5>> cMats_tails;
      std::vector<double> durs;
      for (size_t i=0; i<msg->head.durations.size(); i++)
      {
        durs.push_back((double)msg->head.durations[i]);
        trailer_planner::CoefficientMat<2, 5> p;
        const std::vector<float>& ci = msg->head.coeff[i].data;
        for (int j=0; j<2; j++)
        {
          for (int k=0; k<6; k++)
          {
            p(j, k) = (double) ci[j*6+k];
          }
        }
        cMats.push_back(p);
      }
      arc_traj.head = trailer_planner::PolyTrajectory<2, 5>(durs, cMats);
      durs.clear();
      for (size_t i=0; i<msg->tails.durations.size(); i++)
      {
        durs.push_back((double)msg->tails.durations[i]);
        trailer_planner::CoefficientMat<TRAILER_NUM, 5> p;
        const std::vector<float>& ci = msg->tails.coeff[i].data;
        for (int j=0; j<TRAILER_NUM; j++)
        {
          for (int k=0; k<6; k++)
          {
            p(j, k) = (double) ci[j*6+k];
          }
        }
        cMats_tails.push_back(p);
      }
      arc_traj.tails = trailer_planner::PolyTrajectory<TRAILER_NUM, 5>(durs, cMats_tails);
      durs.clear();
      for (size_t i=0; i<msg->arc.durations.size(); i++)
      {
        durs.push_back((double)msg->arc.durations[i]);
        trailer_planner::CoefficientMat<1, 5> p;
        const std::vector<float>& ci = msg->arc.coeff[i].data;
        for (int j=0; j<1; j++)
        {
          for (int k=0; k<6; k++)
          {
            p(j, k) = (double) ci[j*6+k];
          }
        }
        cMats_arc.push_back(p);
      }
      arc_traj.arc = trailer_planner::PolyTrajectory<1, 5>(durs, cMats_arc);
      
      traj_duration = arc_traj.getTotalDuration();
      start_time = rclcpp::Clock().now();
      at_goal = false;
      return;
    }

    std::vector<TrajPoint> getRefPoints(const int T, double dt)
    {
      std::vector<TrajPoint> P;
      P.clear();
      rclcpp::Time time_now = rclcpp::Clock().now();
      double t_cur = (time_now - start_time).seconds();
      int j=0;

      if (t_cur > traj_duration)
      {
        at_goal = true;
        return P;
      }
      else
      {
        at_goal = false;
      }

      for (double t=t_cur+dt; j<T; j++, t+=dt)
      {
        double temp = t;
        Eigen::VectorXd state;
        
        if (temp <= traj_duration)
          state = arc_traj.getState(temp);
        else
          state = arc_traj.getState(traj_duration);
        P.push_back(TrajPoint(state));
      }

      return P;
    }
   
    ~TrajAnalyzer() {}
};