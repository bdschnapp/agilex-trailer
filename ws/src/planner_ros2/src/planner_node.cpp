#include "planner_ros2/planner.h"
#include <rclcpp/rclcpp.hpp>

using namespace trailer_planner;

int main(int argc, char * argv[])
{ 
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<Planner>("planner_node");
    
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}