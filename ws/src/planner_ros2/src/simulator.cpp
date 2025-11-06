#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <vector>
// #include <nav_msgs/Odometry.h>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "planner_ros2/trailer.hpp"
#include "planner_ros2/msg/trailer_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ros interface
std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr command_sub;
rclcpp::Subscription<planner_ros2::msg::TrailerState>::SharedPtr set_sub;
rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub;
rclcpp::Publisher<planner_ros2::msg::TrailerState>::SharedPtr odom_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr sensor_odom_pub;
rclcpp::TimerBase::SharedPtr simulate_timer;
rclcpp::Time get_cmdtime;
rclcpp::Node::SharedPtr node;

// simulator variables
std::default_random_engine generator;
std::normal_distribution<double> distribution{0.0, 1.0};
ackermann_msgs::msg::AckermannDrive im_cmd;
vector<ackermann_msgs::msg::AckermannDrive> cmd_buff;
trailer_planner::Trailer trailer;
Eigen::VectorXd state;
bool rcv_cmd = false;
bool initialized = false;

// simulator parameters
double time_resolution = 0.01;
double time_delay = 0.0;
double noise_std = 0.0;
double max_speed = 2.0;
double max_steer = 0.7;

void normYaw(double& th)
{
	while (th > M_PI)
		th -= M_PI * 2;
	while (th < -M_PI)
		th += M_PI * 2;
}

double guassRandom(double std)
{
	return std * distribution(generator);
}

void rcvCmdCallBack(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
{	
	if (rcv_cmd==false)
	{
		rcv_cmd = true;
		cmd_buff.push_back(*msg);
		get_cmdtime = node->now();
	}
	else
	{
		cmd_buff.push_back(*msg);
		if ((node->now() - get_cmdtime).seconds() > time_delay)
		{
			im_cmd = cmd_buff[0];
			cmd_buff.erase(cmd_buff.begin());
		}
	}
}

void rcvSetCallBack(const planner_ros2::msg::TrailerState::SharedPtr msg)
{
    state[0] = msg->odoms[0].pose.pose.position.x;
    state[1] = msg->odoms[0].pose.pose.position.y;
    for (int i = 0; i < TRAILER_NUM + 1; i++)
    {
        const auto &q = msg->odoms[i].pose.pose.orientation;
        state[2 + i] = 2.0 * atan2(q.z, q.w);
    }
}

void rcvInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    double yaw = 2.0 * atan2(q.z, q.w);
    
    // Initialize state vector if not already done
    if (state.size() == 0) {
        state.resize(TRAILER_NUM+3);
    }
    
    state[0] = p.x;
    state[1] = p.y;
    for (int i = 0; i < TRAILER_NUM + 1; i++)
        state[2 + i] = yaw; // aligned tractor and trailers
    
    initialized = true;
    RCLCPP_INFO(node->get_logger(), "Initial pose set: x=%.2f, y=%.2f, yaw=%.2f", p.x, p.y, yaw);
}

void simCallback()
{
    // Don't simulate until we receive initial pose from RViz
    if (!initialized) {
        return;
    }
    
    planner_ros2::msg::TrailerState new_odom;
	nav_msgs::msg::Odometry odom_temp;
	odom_temp.header.frame_id = "world";

	double v = max(min((double)im_cmd.speed, max_speed), -max_speed) + guassRandom(noise_std);
	double delta = max(min((double)im_cmd.steering_angle, max_steer), -max_steer) + guassRandom(noise_std);
	Eigen::VectorXd new_state, new_se2_state, v_state, w_state;
	new_state = state;
	v_state.resize(TRAILER_NUM+1);
	w_state.resize(TRAILER_NUM+1);

	// state transition use vel
	double w = v * tan(delta) / trailer.wheel_base;
	double y = time_resolution * w;

	if (fabs(w) > 1e-4)
	{
		new_state(0) = state(0) + v / w * (sin(state(2)+y) - sin(state(2)));
		new_state(1) = state(1) - v / w * (cos(state(2)+y) - cos(state(2)));
		new_state(2) = state(2) + y;
		normYaw(new_state(2));
	}
	else
	{
		new_state(0) = state(0) + v * time_resolution * cos(state(2));
		new_state(1) = state(1) + v * time_resolution * sin(state(2));
		new_state(2) = state(2);
	}
	v_state[0] = v;
	w_state[0] = w;

	for (size_t i=0; i<TRAILER_NUM; i++)
	{
		double sthetad = sin(state(i+2)-state(i+3));
		double cthetad = cos(state(i+2)-state(i+3));
		double w_temp = w;
		w = (v * sthetad - trailer.Ltail[i] * w * cthetad) /trailer.Lhead[i];
		v = v * cthetad + trailer.Ltail[i] * w_temp * sthetad;
		v_state[i+1] = v;
		w_state[i+1] = w;
		new_state(i+3) = state(i+3) + w * time_resolution;
		normYaw(new_state(i+3));
	}

	trailer.gainSE2State(new_state, new_se2_state);
	for (size_t i=0; i<TRAILER_NUM+1; i++)
	{
		odom_temp.pose.pose.position.x = new_se2_state[3*i];
		odom_temp.pose.pose.position.y = new_se2_state[3*i+1];
		double yy = new_se2_state[3*i+2];
		odom_temp.pose.pose.orientation.w = cos(yy/2.0);
		odom_temp.pose.pose.orientation.x = 0.0;
		odom_temp.pose.pose.orientation.y = 0.0;
		odom_temp.pose.pose.orientation.z = sin(yy/2.0);
		odom_temp.twist.twist.angular.z = w_state[i];
		odom_temp.twist.twist.linear.x = v_state[i];
		new_odom.odoms.push_back(odom_temp);
	}
	trailer.showTrailer(new_state, 1);
	new_odom.time_now = node->now();
	odom_pub->publish(new_odom);
	state = new_state;
	sensor_odom_pub->publish(new_odom.odoms[0]);

	geometry_msgs::msg::TransformStamped transform;
	transform.header.stamp = node->now();
	transform.header.frame_id = "world";
	transform.child_frame_id = "robot";
	transform.transform.translation.x = new_odom.odoms[0].pose.pose.position.x;
	transform.transform.translation.y = new_odom.odoms[0].pose.pose.position.y;
	transform.transform.translation.z = new_odom.odoms[0].pose.pose.position.z;
	transform.transform.rotation = new_odom.odoms[0].pose.pose.orientation;
	broadcaster->sendTransform(transform);
	return;
} 

// main loop
int main (int argc, char** argv) 
{        
    rclcpp::init (argc, argv);
    node = rclcpp::Node::make_shared("simulator_node");

    trailer.init(node);

    // Don't initialize state with hardcoded values - wait for RViz 2D Pose Estimate
    // state.resize(TRAILER_NUM+3);
	// for (size_t i=0; i<TRAILER_NUM+3; i++)
	//	state[i] = trailer.pthetas[i];

	im_cmd.speed = 0.0;
	im_cmd.steering_angle = 0.0;

    RCLCPP_INFO(node->get_logger(), "Simulator waiting for initial pose from RViz...");

    command_sub = node->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "~/cmd", 1000, rcvCmdCallBack);
    set_sub = node->create_subscription<planner_ros2::msg::TrailerState>(
        "/trailer_set", 1000, rcvSetCallBack);
    initpose_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, rcvInitialPose);
    odom_pub = node->create_publisher<planner_ros2::msg::TrailerState>("~/odom", 10);
    sensor_odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("~/sensor_odom", 10);
    simulate_timer = node->create_wall_timer(
        std::chrono::duration<double>(time_resolution), simCallback);
	broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

	rclcpp::spin(node);
	rclcpp::shutdown();

    return 0;

}
