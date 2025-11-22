cmake_minimum_required(VERSION 3.8)
project(sekirei_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(testn src/testn.cpp)
ament_target_dependencies(testn rclcpp moveit_ros_planning_interface)
install(TARGETS testn
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/fake_controller.py
  scripts/joy_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
