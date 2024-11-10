cmake_minimum_required(VERSION 3.8)
project(panda_moveit_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package( moveit_ros_planning_interface  REQUIRED )



add_executable( joint_space_planning src/joint_space_planning.cpp ) 
ament_target_dependencies(joint_space_planning rclcpp std_msgs moveit_ros_planning_interface)

add_executable( cartesian_space_planning src/cartesian_space_planning.cpp ) 
ament_target_dependencies(cartesian_space_planning rclcpp std_msgs moveit_ros_planning_interface)


add_executable( planning_with_obstacles src/planning_with_obstacles.cpp ) 
ament_target_dependencies(planning_with_obstacles rclcpp std_msgs moveit_ros_planning_interface)


install(DIRECTORY launch  
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS joint_space_planning
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS cartesian_space_planning
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS planning_with_obstacles
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
