cmake_minimum_required(VERSION 3.22)
project(nav2_bt_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Install directories
install(DIRECTORY
  bt_xml launch config
  DESTINATION share/${PROJECT_NAME}
)


# Install C++ nodes
set(NAV2_BT_SOURCES
    src/navigation_behaviors.cpp 
)
set(TARGET_DEPENDS
    ament_index_cpp rclcpp rclcpp_action
    nav2_msgs behaviortree_cpp tf2 tf2_ros tf2_geometry_msgs yaml-cpp
)
include_directories(include)
add_executable(nav2_bt_node src/nav2_bt_node.cpp ${NAV2_BT_SOURCES})
install(TARGETS
    nav2_bt_node
    DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(nav2_bt_node ${TARGET_DEPENDS})
target_link_libraries(nav2_bt_node ${YAML_CPP_LIBRARIES})

ament_package()
