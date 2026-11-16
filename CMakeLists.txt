cmake_minimum_required(VERSION 3.8)
project(medlab_dynamixel_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMotorTorqueEnable.srv"
)

# Install the ROS 2 Python executable
install(
  PROGRAMS
    medlab_dynamixel_hardware_interface/scripts/dynamixel_hardware_interface.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dynamixel_hardware_interface
)

# Install our Python utility modules alongside generated ROS interfaces.
# Do not overwrite the generated package's __init__.py.
install(
  DIRECTORY medlab_dynamixel_hardware_interface/utils
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install launch files, configurations, and model definitions
install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
