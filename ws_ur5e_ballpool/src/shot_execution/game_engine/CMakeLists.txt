cmake_minimum_required(VERSION 3.8)
project(game_engine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-cpp)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(shared_headers_pkg REQUIRED)  #mia libreria di headers .hpp
find_package(interfaces_pkg REQUIRED)      #mia libreria di interfaces

find_package(std_srvs REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(game_engine src/game_engine.cpp)
ament_target_dependencies(
  game_engine
  "rclcpp"
  "shared_headers_pkg"
  "interfaces_pkg"
  "std_srvs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "geometry_msgs"
)

add_executable(fake_game_engine src/fake_game_engine.cpp)
ament_target_dependencies(
  fake_game_engine
  "rclcpp"
  "shared_headers_pkg"
  "interfaces_pkg"
  "std_srvs"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)

install(TARGETS 
  game_engine
  fake_game_engine
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
