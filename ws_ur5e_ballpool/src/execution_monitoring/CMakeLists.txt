cmake_minimum_required(VERSION 3.8)
project(execution_monitoring)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-cpp)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(shared_headers_pkg REQUIRED)  #mia libreria di headers .hpp
find_package(interfaces_pkg REQUIRED)      #mia libreria di interfaces

find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
  

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)


add_executable(cartesian_logger src/cartesian_logger.cpp)
ament_target_dependencies(
  cartesian_logger
  "rclcpp"
  "geometry_msgs"
  "shared_headers_pkg"
  "tf2_ros"
  "tf2_geometry_msgs"
  "interfaces_pkg"
  "moveit_ros_planning_interface"
  "moveit_core"
)

add_executable(joint_logger src/joint_logger.cpp)
ament_target_dependencies(
  joint_logger
  "rclcpp"
  "sensor_msgs"
  "shared_headers_pkg"
  "interfaces_pkg"
)

add_executable(torque_logger src/torque_logger.cpp)
ament_target_dependencies(
  torque_logger
  "rclcpp"
  "sensor_msgs"
  "shared_headers_pkg"
  "interfaces_pkg"
)

add_executable(controller_logger src/controller_logger.cpp)
ament_target_dependencies(
  controller_logger
  "rclcpp"
  # "geometry_msgs"
  "control_msgs"
  "shared_headers_pkg"
  "interfaces_pkg"
)



install(TARGETS 
  cartesian_logger
  joint_logger
  torque_logger
  controller_logger
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
