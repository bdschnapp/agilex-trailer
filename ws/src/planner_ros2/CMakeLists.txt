cmake_minimum_required(VERSION 3.8)
project(planner_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#! Choose the num of the trailer
add_definitions(-DTRAILER_NUM=1)
# add_definitions(-DTRAILER_NUM=2)
# add_definitions(-DTRAILER_NUM=3)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(casadi REQUIRED)

# Help the linker find casadi if variables are provided
if(DEFINED CASADI_LIBRARY_DIRS)
  link_directories(${CASADI_LIBRARY_DIRS})
endif()

set(BOOST_ROOT "/usr")
set(Boost_NO_SYSTEM_PATHS TRUE)
find_package(Boost REQUIRED COMPONENTS filesystem)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrailerState.msg"
  "msg/PolyTraj.msg"
  "msg/ArcTrailerTraj.msg"
  DEPENDENCIES std_msgs nav_msgs builtin_interfaces
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIR}
  ${CASADI_INCLUDE_DIRS}
)

# Create library
add_library(planner_lib 
  src/grid_map.cpp
  src/hybrid_astar.cpp
  src/arc_opt.cpp
  src/planner.cpp
)

ament_target_dependencies(planner_lib
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

target_link_libraries(planner_lib
  ${OMPL_LIBRARIES}
)

# Planner node
add_executable(planner_node src/planner_node.cpp)
ament_target_dependencies(planner_node
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

target_link_libraries(planner_node
  planner_lib
  ${PCL_LIBRARIES}
)

# Simulator node
add_executable(simulator_node src/simulator.cpp)
ament_target_dependencies(simulator_node
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  ackermann_msgs
)

# MPC node
add_executable(mpc_node src/mpc.cpp)
ament_target_dependencies(mpc_node
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  ackermann_msgs
  pcl_conversions
)

target_link_libraries(mpc_node
  ${CASADI_LIBRARIES}
  casadi
)

# If casadi provides an imported target, link it as well
if(TARGET casadi::casadi)
  target_link_libraries(mpc_node casadi::casadi)
endif()

# Make sure messages are generated before building executables
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(planner_lib "${cpp_typesupport_target}")
target_link_libraries(planner_node "${cpp_typesupport_target}")
target_link_libraries(simulator_node "${cpp_typesupport_target}")
target_link_libraries(mpc_node "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  planner_lib
  DESTINATION lib
)

install(TARGETS
  planner_node
  simulator_node
  mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
