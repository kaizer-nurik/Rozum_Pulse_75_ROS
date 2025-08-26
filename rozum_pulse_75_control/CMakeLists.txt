cmake_minimum_required(VERSION 3.16)
project(rozum_pulse_75_control)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)


get_filename_component(WS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})


ament_package()