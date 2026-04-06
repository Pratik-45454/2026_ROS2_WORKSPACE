cmake_minimum_required(VERSION 3.8)
project(oakd_roi_detector_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDetections.srv"
)

# Export dependencies for other packages to find this
ament_export_dependencies(rosidl_default_runtime)

# Install the srv files (optional but recommended)
install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
