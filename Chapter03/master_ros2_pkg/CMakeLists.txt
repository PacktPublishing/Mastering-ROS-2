cmake_minimum_required(VERSION 3.8)
project(master_ros2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(master_ros2_interface REQUIRED)


add_executable(publisher_node src/publisher_node.cpp)
ament_target_dependencies(publisher_node rclcpp std_msgs master_ros2_interface)

add_executable(subscriber_node src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node rclcpp std_msgs master_ros2_interface)

add_executable(publisher_node_params src/publisher_node_params.cpp)
ament_target_dependencies(publisher_node_params rclcpp std_msgs master_ros2_interface)

add_executable(simple_server src/simple_server.cpp)
ament_target_dependencies(simple_server rclcpp master_ros2_interface)

add_executable(simple_client src/simple_client.cpp)
ament_target_dependencies(simple_client rclcpp master_ros2_interface)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action master_ros2_interface)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action master_ros2_interface)


install(TARGETS
 publisher_node
 subscriber_node 
 publisher_node_params 
 simple_server
 simple_client
 action_server
 action_client
 DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
