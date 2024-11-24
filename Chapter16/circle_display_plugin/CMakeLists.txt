cmake_minimum_required(VERSION 3.8)
project(circle_display_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(circle_msgs REQUIRED)
find_package(rviz_rendering REQUIRED)

set(BUILD_SHARED_LIBS ON)
set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/circle_display_plugin/circle_display.hpp
)

add_library(circle_display src/circle_display.cpp ${MOC_FILES})
target_include_directories(circle_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(circle_display
  pluginlib
  rviz_common
  circle_msgs
  rviz_rendering
)

install(TARGETS circle_display
        EXPORT export_rviz_circle_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_rviz_circle_plugin)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()
