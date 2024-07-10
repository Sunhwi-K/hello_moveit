cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
  -isystem/usr/include/c++/11 \
  -isystem/usr/include/x86_64-linux-gnu/c++/11 \
  -isystem/usr/include/c++/11/backward \
  -isystem/usr/lib/gcc/x86_64-linux-gnu/11/include \
  -isystem/usr/local/include \
  -isystem/usr/include/x86_64-linux-gnu \
  -isystem/usr/include"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  hello_moveit_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  rclcpp
  rclcpp_components
  rclpy
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
  tf2_ros
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executables
add_executable(moveit_sample src/hello_moveit.cpp)
ament_target_dependencies(moveit_sample
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
)

add_executable(hello_ur src/hello_ur.cpp)
ament_target_dependencies(hello_ur
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
)

add_executable(moveit_client src/moveit_client.cpp)
ament_target_dependencies(moveit_client
  hello_moveit_msgs
  lifecycle_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

# Add executable for using a controller
set(SERVO_CONTROLLER_INPUT servo_controller_input)
add_library(${SERVO_CONTROLLER_INPUT} SHARED src/teleop_demo/joystick_servo_example.cpp)
ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "moveit_servo::JoyToServoPub")

# Install Libraries
install(
  TARGETS
    ${SERVO_CONTROLLER_INPUT}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install Cpp executables
install(TARGETS
  moveit_sample
  hello_ur
  moveit_client
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/moveit_client_main.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install data files which are included in package
install(DIRECTORY 
  launch
  rviz
  config
  cad
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
