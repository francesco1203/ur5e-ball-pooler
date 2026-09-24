cmake_minimum_required(VERSION 3.8)
project(logging_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-cpp)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(shared_headers_pkg REQUIRED)  #mia libreria di headers .hpp
find_package(interfaces_pkg REQUIRED)      #mia libreria di interfaces

find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
  

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)



add_executable(cartesian_pose_publisher src/cartesian_pose_publisher.cpp)
ament_target_dependencies(
  cartesian_pose_publisher
  "rclcpp"
  "geometry_msgs"
  "shared_headers_pkg"
  "std_srvs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "moveit_ros_planning_interface"
  "moveit_core"
)



add_executable(cartesian_velocity_publisher src/cartesian_velocity_publisher.cpp)
ament_target_dependencies(
  cartesian_velocity_publisher
  "rclcpp"
  "geometry_msgs"
  "shared_headers_pkg"
  "std_srvs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "moveit_ros_planning_interface"
  "moveit_core"
)




add_executable(bag_writer src/bag_writer_node.cpp)

ament_target_dependencies(
  bag_writer
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "control_msgs"
  "std_msgs"
  "rosbag2_cpp"
  "rosbag2_storage"
  "interfaces_pkg"
  "shared_headers_pkg"
)




install(TARGETS 
  cartesian_pose_publisher
  bag_writer
  cartesian_velocity_publisher
DESTINATION lib/${PROJECT_NAME})


# ########## INSTALL the config files ############
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME})
# ################################################


########## INSTALL the launch files ############
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
################################################



ament_package()
