cmake_minimum_required(VERSION 3.8)
project(demo_moveit_ur)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_srvs)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(moveit_servo_ur src/moveit_servo_ur.cpp)
target_include_directories(moveit_servo_ur PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_servo_ur PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_servo_ur
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
  "control_msgs"
  "controller_manager_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
)

add_executable(moveit_constrain_ur src/moveit_constrain_ur.cpp)
target_include_directories(moveit_constrain_ur PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_constrain_ur PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_constrain_ur
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)

add_executable(moveit_avoid_ur src/moveit_avoid_ur.cpp)
target_include_directories(moveit_avoid_ur PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_avoid_ur PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_avoid_ur
  "moveit_ros_planning_interface"
  "rclcpp"
)


add_executable(moveit_basic_ur src/moveit_basic_ur.cpp)
target_include_directories(moveit_basic_ur PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_basic_ur PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_basic_ur
  "moveit_ros_planning_interface"
  "rclcpp"
)


install(TARGETS 
  moveit_basic_ur
  moveit_avoid_ur
  moveit_constrain_ur
  moveit_servo_ur
DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
