cmake_minimum_required(VERSION 3.10)
project(linksee)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(
  PROGRAMS
    scripts/asr_node.py
    scripts/odom_remap_node.py
    scripts/voice_cmd_node.py
    scripts/voice_dispatcher_node.py
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
