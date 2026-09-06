cmake_minimum_required(VERSION 3.8)
project(tool_point_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

# The core tool calibration library
add_library(${PROJECT_NAME}
  src/tool_point_calibration.cpp
)

include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  tf2
  tf2_eigen
)

target_link_libraries(${PROJECT_NAME}
  ${CERES_LIBRARIES}
  Eigen3::Eigen
)

# A utility for calibrating a named tool point on the command line
add_executable(console_calibration
  src/console_calibration.cpp
)

ament_target_dependencies(console_calibration
  rclcpp
  tf2
  tf2_eigen
  geometry_msgs
)

target_link_libraries(console_calibration
  ${CERES_LIBRARIES}
  tool_point_calibration
  Eigen3::Eigen
)

install(TARGETS console_calibration
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test
    test/utest.cpp
    test/test_tool.cpp
  )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()
endif()

ament_package()