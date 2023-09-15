cmake_minimum_required(VERSION 3.5)
project(planning)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

SET(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")
SET(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH} ~/workspace/driverless/driverless_ws/install;/opt/ros/foxy")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(GSL REQUIRED)

#add_executable(slam_test src/planning_codebase/EKF_SLAM.cpp)
#ament_target_dependencies(slam_test rclcpp message_filters eufs_msgs)
#ament_export_include_directories(source)

# Temporary fix to compile raceline
# add_library(frenet_test src/planning_codebase/raceline/frenet.cpp src/planning_codebase/raceline/raceline.cpp)
# ament_target_dependencies(frenet_test rclcpp)
# ament_export_include_directories(src)
# target_link_libraries(frenet_test GSL::gsl GSL::gslcblas)

add_executable(midline src/planning_codebase/midpoint/generator.cpp)
ament_target_dependencies(midline rclcpp std_msgs GSL interfaces)
target_include_directories(midline PRIVATE AFTER ${PROJECT_SOURCE_DIR}/..)
target_link_libraries(midline GSL::gsl GSL::gslcblas)

#install(TARGETS
#  slam_test
#  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
	# frenet_test
  midline
	DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks()
ament_package()
