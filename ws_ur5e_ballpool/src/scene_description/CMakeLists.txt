cmake_minimum_required(VERSION 3.8)
project(scene_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Trova le dipendenze ROS 2 e di sistema
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Include la cartella locale
include_directories(include)

# Definizione del nodo
add_executable(scene_builder src/scene_builder_node.cpp)

# Aggiungi TUTTE le dipendenze qui dentro
ament_target_dependencies(
  scene_builder
  "rclcpp"
  "moveit_ros_planning_interface"
  "tf2_ros"
  "geometric_shapes"  # <-- Necessaria per addMESH
  "ament_index_cpp"   # <-- Necessaria per get_package_share_directory
  "Eigen3"            # <-- Necessaria per la manipolazione vettoriale di geometric_shapes
)

# Installazione del nodo e della cartella meshes
install(DIRECTORY 
  meshes
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS 
  scene_builder
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

