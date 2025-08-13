cmake_minimum_required(VERSION 3.16)
project(rozum_pulse_75_hardware)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(CURL REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rozum_system.cpp
)

get_filename_component(WS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(RAPIDJSON_LOCAL_INCLUDE "${WS_ROOT}/rapidjson/include")

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  CURL::libcurl
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${RAPIDJSON_LOCAL_INCLUDE}
)

pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()