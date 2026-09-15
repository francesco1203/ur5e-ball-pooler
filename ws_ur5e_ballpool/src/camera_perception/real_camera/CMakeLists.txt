# cmake_minimum_required(VERSION 3.8)
# project(fake_camera)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


# find_package(shared_headers_pkg REQUIRED)  #mia libreria di headers .hpp

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)

# # Trova la libreria yaml-cpp
# find_package(yaml-cpp REQUIRED)

# add_executable(fake_camera_node src/fake_camera.cpp)


# ament_target_dependencies(fake_camera_node
#   rclcpp
#   geometry_msgs
#   tf2
#   tf2_ros
#   shared_headers_pkg
# )
# target_link_libraries(fake_camera_node yaml-cpp)

# #toADD: VISION NODE

# install(TARGETS
#   fake_camera_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ########## INSTALL the launch files ############
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME})
# ################################################


# ########## INSTALL the config files ############
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME})


# ament_package()



cmake_minimum_required(VERSION 3.8)
project(fake_camera) # <-- INSERISCI QUI IL NOME DEL PACCHETTO

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Trova le dipendenze ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(shared_headers_pkg REQUIRED) # Il tuo pacchetto con gli header condivisi

# 2. Trova la libreria OpenCV di sistema
find_package(OpenCV REQUIRED)

# 3. Dichiara l'eseguibile (Assicurati che il percorso src/vision_node.cpp sia corretto)
add_executable(vision_node src/vision_node.cpp)

# 4. Includi gli header di OpenCV
target_include_directories(vision_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

# 5. Collega le dipendenze ROS 2 all'eseguibile
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  shared_headers_pkg
)

# 6. Collega le librerie OpenCV all'eseguibile
target_link_libraries(vision_node
  ${OpenCV_LIBRARIES}
)

# 7. Regola di installazione per ros2 run
install(TARGETS
  vision_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()