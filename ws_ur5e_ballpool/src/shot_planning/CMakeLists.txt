cmake_minimum_required(VERSION 3.8)
project(shot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(shared_headers_pkg REQUIRED)  #mia libreria di headers .hpp

find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)      



add_executable(task_node src/task_node.cpp)
ament_target_dependencies(
  task_node
  "rclcpp"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "control_msgs"
  "Eigen3"
  "shared_headers_pkg"
)

install(TARGETS 
  task_node
DESTINATION lib/${PROJECT_NAME})


ament_package()
