cmake_minimum_required(VERSION 3.8)
project(cmd_vel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(geometry_msgs REQUIRED)

set(cmd_vel_SRCS
  src/cmd_vel.cpp
)

set(cmd_vel_HDRS
  include/cmd_vel/cmd_vel.h  
)

set(cmd_vel_UIS
  src/cmd_vel_ui.ui
)

include_directories( include )
include_directories( ${CMAKE_BINARY_DIR}  )
qt5_wrap_cpp(cmd_vel_MOCS ${cmd_vel_HDRS})
qt5_wrap_ui(cmd_vel_UIS_H ${cmd_vel_UIS})

add_library(${PROJECT_NAME} SHARED
  ${cmd_vel_SRCS}		
  ${cmd_vel_MOCS}
  ${cmd_vel_UIS_H}
)
	
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")


target_link_libraries(${PROJECT_NAME} PUBLIC
  ${rclcpp_TARGETS}
  ${qt_gui_cpp_TARGETS}
  ${rqt_gui_cpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  Qt5::Widgets
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)


pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export new-style CMake variables
ament_export_targets(${PROJECT_NAME})





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
