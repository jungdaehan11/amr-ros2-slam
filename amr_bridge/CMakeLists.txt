cmake_minimum_required(VERSION 3.8)
project(amr_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bridge_node
  src/bridge_node.cpp
  src/Packet.cpp
  src/SerialPort.cpp)
target_include_directories(bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bridge_node rclcpp geometry_msgs std_msgs)

install(TARGETS bridge_node DESTINATION lib/${PROJECT_NAME})

ament_package()
