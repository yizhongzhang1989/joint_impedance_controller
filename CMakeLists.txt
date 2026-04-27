cmake_minimum_required(VERSION 3.16)
project(my_joint_impedance_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

generate_parameter_library(
  ${PROJECT_NAME}_parameters
  src/my_joint_impedance_controller.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/my_joint_impedance_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_NAME}_parameters
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    rclcpp
    rclcpp_lifecycle
    controller_interface
    hardware_interface
    pluginlib
    realtime_tools
    sensor_msgs
    urdf
)

pluginlib_export_plugin_description_file(
  controller_interface my_joint_impedance_controller.xml
)

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  generate_parameter_library
  sensor_msgs
  urdf
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
