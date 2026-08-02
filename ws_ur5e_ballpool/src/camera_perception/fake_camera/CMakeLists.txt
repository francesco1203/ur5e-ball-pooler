cmake_minimum_required(VERSION 3.8)
project(fake_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(shared_headers_pkg REQUIRED)  #mia libreria di headers .hpp

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Trova la libreria yaml-cpp
find_package(yaml-cpp REQUIRED)

add_executable(fake_camera_node src/fake_camera.cpp)


ament_target_dependencies(fake_camera_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  shared_headers_pkg
)
target_link_libraries(fake_camera_node yaml-cpp)

install(TARGETS
  fake_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

########## INSTALL the launch files ############
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
################################################


########## INSTALL the config files ############
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})


ament_package()
