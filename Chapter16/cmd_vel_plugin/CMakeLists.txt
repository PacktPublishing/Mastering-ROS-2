cmake_minimum_required(VERSION 3.8)
project(cmd_vel_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-cmake3 REQUIRED)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_library(CmdVelPlugin SHARED src/pub_cmd_vel.cpp)

target_link_libraries(CmdVelPlugin 
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
ament_target_dependencies(CmdVelPlugin rclcpp std_msgs geometry_msgs)
install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})


ament_package()
