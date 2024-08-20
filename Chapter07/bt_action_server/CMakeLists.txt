cmake_minimum_required(VERSION 3.8)
project(bt_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(behaviortree_cpp REQUIRED)


# Generate the action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ReachLocation.action"
  DEPENDENCIES std_msgs geometry_msgs
)

# Include directories
include_directories(include)

# Define the executable for the action server
add_executable(reach_location_server src/reach_location_server.cpp)
ament_target_dependencies(reach_location_server rclcpp rclcpp_action std_msgs geometry_msgs )
rosidl_target_interfaces(reach_location_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Define the executable for the action client
add_executable(reach_location_client src/reach_location_client.cpp)
ament_target_dependencies(reach_location_client rclcpp rclcpp_action  std_msgs geometry_msgs )
rosidl_target_interfaces(reach_location_client ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Install the executables
install(TARGETS 
  reach_location_server DESTINATION lib/${PROJECT_NAME})



# Install the executables
install(TARGETS 
  reach_location_client DESTINATION lib/${PROJECT_NAME})


add_executable( bt_action src/bt_action.cpp ) 
ament_target_dependencies(bt_action rclcpp rclcpp_action std_msgs geometry_msgs behaviortree_cpp )
rosidl_target_interfaces(bt_action ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Install the executables
install(TARGETS 
  bt_action DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch trees 
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
