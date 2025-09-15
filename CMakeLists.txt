cmake_minimum_required(VERSION 3.8)
project(so_arm101_ros2_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)

# ft_servo static library
add_library(ft_servo STATIC
  src/ft_servo/SCS.cpp
  src/ft_servo/SCSCL.cpp
  src/ft_servo/SCSerial.cpp
  src/ft_servo/SMS_STS.cpp
  src/ft_servo/SMSBL.cpp
  src/ft_servo/SMSCL.cpp
)

target_include_directories(ft_servo
  PUBLIC
    src/ft_servo
)

# Set position independent code for static library
set_target_properties(ft_servo PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Suppress warnings for third-party ft_servo library
target_compile_options(ft_servo PRIVATE
  -Wno-vla                    # Suppress variable length array warnings
  -Wno-maybe-uninitialized    # Suppress uninitialized variable warnings
)

# Hardware interface library
add_library(${PROJECT_NAME}_hardware_interface SHARED
  src/so_arm101_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME}_hardware_interface
  PRIVATE
    include
)

target_compile_definitions(${PROJECT_NAME}_hardware_interface
  PRIVATE "SO_ARM101_ROS2_CONTROL_BUILDING_LIBRARY"
)

target_link_libraries(${PROJECT_NAME}_hardware_interface
  ft_servo
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}_hardware_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

# Install library
install(
  TARGETS ${PROJECT_NAME}_hardware_interface ft_servo
  DESTINATION lib
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install urdf and config files for ros2_control
install(
  DIRECTORY
    config
    urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin description
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_hardware_interface ft_servo)
ament_export_dependencies(
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  yaml-cpp
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()