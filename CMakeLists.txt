cmake_minimum_required(VERSION 3.8)

project(torque_vectoring_pkg)

# C++20 — required for the concepts in interfaces.hpp
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -DDEBUG)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(vehicle_params_cpp REQUIRED)
find_package(ros2can_msgs REQUIRED)
find_package(fs_msgs REQUIRED)

# C++ executable
add_executable(torque_vectoring_node src/main.cpp)

target_include_directories(torque_vectoring_node PRIVATE include)

ament_target_dependencies(torque_vectoring_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  fs_msgs
  ros2can_msgs
)

target_link_libraries(torque_vectoring_node
  Eigen3::Eigen
  vehicle_params_cpp::vehicle_params_cpp
  )

# Install C++ executables
install(TARGETS
  torque_vectoring_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
