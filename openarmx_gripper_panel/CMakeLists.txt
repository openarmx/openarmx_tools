# Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
#
# Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
# https://www.openarmx.com
#
# This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
# 4.0 International License (CC BY-NC-SA 4.0).
#
# To view a copy of this license, visit:
# http://creativecommons.org/licenses/by-nc-sa/4.0/
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

cmake_minimum_required(VERSION 3.8)
project(openarmx_gripper_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# C++ RViz Plugin Library
include_directories(
  include
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(PLUGIN_SOURCES
  src/gripper_panel.cpp
)

set(PLUGIN_HEADERS
  include/openarmx_gripper_panel/gripper_panel.hpp
)

add_library(${PROJECT_NAME} SHARED
  ${PLUGIN_SOURCES}
  ${PLUGIN_HEADERS}
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  control_msgs
  rviz_common
  rviz_default_plugins
  pluginlib
)

# Install C++ library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description
install(FILES
  plugins/plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install resource files
install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

# Install documentation
install(FILES
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  control_msgs
  rviz_common
  pluginlib
)

ament_package()
