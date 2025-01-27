cmake_minimum_required(VERSION 3.5)
project(planning)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(GSL REQUIRED)

add_executable(midline src/midpoint.cpp src/generator.cpp src/raceline/raceline.cpp)
ament_target_dependencies(midline rclcpp std_msgs eufs_msgs geometry_msgs GSL)



add_executable(talker pub.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

add_executable(listener sub.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  midline
  DESTINATION lib/${PROJECT_NAME})

ament_package()