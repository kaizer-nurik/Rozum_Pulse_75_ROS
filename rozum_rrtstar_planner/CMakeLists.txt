cmake_minimum_required(VERSION 3.10)
project(rozum_rrtstar_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(class_loader REQUIRED)

add_library(rozum_rrtstar_planner SHARED
  src/rrtstar_interface.cpp
  src/rrtstar_planning_context.cpp
  src/rrtstar_planner_manager.cpp
)

ament_target_dependencies(rozum_rrtstar_planner
  rclcpp
  moveit_core
  moveit_ros_planning
  pluginlib
  class_loader
)

target_include_directories(rozum_rrtstar_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Чтобы класс был виден как плагин
pluginlib_export_plugin_description_file(moveit_core
  rrtstar_planner_plugin_description.xml)

install(TARGETS rozum_rrtstar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(FILES rrtstar_planner_plugin_description.xml
  DESTINATION share/${PROJECT_NAME})
  
ament_package()
