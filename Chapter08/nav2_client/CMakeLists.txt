cmake_minimum_required(VERSION 3.8)
project(nav2_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(navigation_client
  src/main.cpp
  src/navigation_client.cpp)

ament_target_dependencies(navigation_client
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs)

install(TARGETS navigation_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()