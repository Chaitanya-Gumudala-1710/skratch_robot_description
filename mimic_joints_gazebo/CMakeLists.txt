cmake_minimum_required(VERSION 3.5)
project(mimic_joints_gazebo)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# Compiler settings
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Plugin 1: mimic_joint_plugin
add_library(gazebo_mimic_joint_plugin SHARED
  src/mimic_joint_plugin.cpp
)

ament_target_dependencies(gazebo_mimic_joint_plugin
  rclcpp
  gazebo_ros
  control_toolbox
)

target_link_libraries(gazebo_mimic_joint_plugin
  ${GAZEBO_LIBRARIES}
)

# Plugin 2: disable_link_plugin
add_library(gazebo_disable_link_plugin SHARED
  src/disable_link_plugin.cpp
)

ament_target_dependencies(gazebo_disable_link_plugin
  rclcpp
  gazebo_ros
  control_toolbox
)

target_link_libraries(gazebo_disable_link_plugin
  ${GAZEBO_LIBRARIES}
)

# Install
install(TARGETS
  gazebo_mimic_joint_plugin
  gazebo_disable_link_plugin
  DESTINATION lib
)

ament_package()
