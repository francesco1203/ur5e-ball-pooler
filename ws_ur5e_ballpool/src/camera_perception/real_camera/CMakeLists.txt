cmake_minimum_required(VERSION 3.8)
project(real_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(shared_headers_pkg REQUIRED)  #mia libreria di headers .hpp

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(vision_node src/vision_node.cpp)


ament_target_dependencies(vision_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
  shared_headers_pkg
  cv_bridge
)

# Aggiungi esplicitamente ${cv_bridge_INCLUDE_DIRS}
target_include_directories(vision_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# Aggiungi esplicitamente la libreria di cv_bridge al linker
target_link_libraries(vision_node 
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

install(TARGETS
  vision_node
  DESTINATION lib/${PROJECT_NAME}
)

########## INSTALL the launch files ############
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})
################################################


ament_package()
