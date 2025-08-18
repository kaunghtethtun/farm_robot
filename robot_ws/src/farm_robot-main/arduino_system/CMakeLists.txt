cmake_minimum_required(VERSION 3.8)
project(arduino_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED) 
find_package(tf2 REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add_executable(serial_node src/serial_node.cpp)
# ament_target_dependencies(serial_node rclcpp)
# install(TARGETS serial_node DESTINATION lib/${PROJECT_NAME})

add_executable(diff_controller src/twist_to_motor.cpp)
ament_target_dependencies(diff_controller rclcpp geometry_msgs std_msgs sensor_msgs)
install(TARGETS diff_controller DESTINATION lib/${PROJECT_NAME})

add_executable(odom src/base_controller.cpp)
ament_target_dependencies(odom rclcpp geometry_msgs std_msgs nav_msgs tf2_ros tf2 tf2_geometry_msgs sensor_msgs)
install(TARGETS odom DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
	launch
  config
	DESTINATION share/${PROJECT_NAME}
)
ament_package()
