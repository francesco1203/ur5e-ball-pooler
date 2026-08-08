cmake_minimum_required(VERSION 3.8)
project(shot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-cpp)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(shared_headers_pkg REQUIRED)  #mia libreria di headers .hpp
# find_package(interfaces_pkg REQUIRED)     #mia libreria di interfaces per servizi e azioni

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)      

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(ruckig REQUIRED)


add_executable(task_node src/task_node.cpp)
ament_target_dependencies(
  task_node
  "rclcpp"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "sensor_msgs"
  "Eigen3"
  "shared_headers_pkg"
  "tf2_ros"
  "tf2_geometry_msgs"
  "ruckig"
)

install(TARGETS 
  task_node
DESTINATION lib/${PROJECT_NAME})

########## INSTALL the config files ############
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
################################################


# ########## INSTALL the launch files ############
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME})
# ################################################



ament_package()
