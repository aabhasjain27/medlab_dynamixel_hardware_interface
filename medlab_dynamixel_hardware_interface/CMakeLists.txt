cmake_minimum_required(VERSION 3.8)
project(medlab_dynamixel_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_python REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMotorTorqueEnable.srv"
)

# Install Python executable
install(
  PROGRAMS
    medlab_dynamixel_hardware_interface/scripts/dynamixel_hardware_interface.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dynamixel_hardware_interface
)

# Install Python utility modules
install(
  DIRECTORY medlab_dynamixel_hardware_interface/utils/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/utils
  FILES_MATCHING PATTERN "*.py"
)

# Install configuration, launch, and model files
install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
