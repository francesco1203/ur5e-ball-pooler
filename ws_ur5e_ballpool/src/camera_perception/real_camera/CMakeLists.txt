cmake_minimum_required(VERSION 3.8)
project(real_camera)

# Se incontri muri di errori incomprensibili legati a GCC 13, 
# puoi sostituire -Wall -Wextra -Wpedantic con un semplice -w
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================================================
# 1. Trova le dipendenze ROS 2
# ========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(shared_headers_pkg REQUIRED)
find_package(visualization_msgs REQUIRED)
# --- NUOVE DIPENDENZE AGGIUNTE PER I QUATERNIONI ---
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# ---------------------------------------------------

# 2. Trova la libreria OpenCV di sistema
find_package(OpenCV REQUIRED)

# 3. Dichiara l'eseguibile 
add_executable(vision_node src/vision_node.cpp)

# 4. Includi gli header di OpenCV
target_include_directories(vision_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(tf_freezer_node src/tf_freezer_node.cpp)
ament_target_dependencies(tf_freezer_node 
rclcpp 
std_srvs 
shared_headers_pkg
geometry_msgs
 tf2 
  tf2_geometry_msgs    # <-- AGGIUNTO QUI
tf2_ros)
install(TARGETS tf_freezer_node DESTINATION lib/${PROJECT_NAME})

# 5. Collega le dipendenze ROS 2 all'eseguibile
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  shared_headers_pkg
  tf2                  # <-- AGGIUNTO QUI
  tf2_geometry_msgs    # <-- AGGIUNTO QUI
  visualization_msgs
  std_srvs
)

# 6. Collega le librerie OpenCV all'eseguibile
target_link_libraries(vision_node
  ${OpenCV_LIBRARIES}
)

# 7. Regola di installazione per ros2 run
install(TARGETS
  vision_node
  tf_freezer_node
  DESTINATION lib/${PROJECT_NAME}
)

########## INSTALL the directories ############
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)
################################################

ament_package()