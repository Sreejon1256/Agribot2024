cmake_minimum_required(VERSION 3.5)
project(Agri)

# Find dependencies
find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fields2cover REQUIRED)
find_package(controller_manager REQUIRED)
Cmake# Install Python scripts
install(PROGRAMS
  scripts/fields2cover_coverage.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration and launch files
install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
