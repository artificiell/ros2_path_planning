cmake_minimum_required(VERSION 3.5)
project(ros2_path_planning)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanTrajectory.srv"
  DEPENDENCIES geometry_msgs
  DEPENDENCIES nav_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ros2_path_planning/planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
