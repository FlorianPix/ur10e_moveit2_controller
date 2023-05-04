cmake_minimum_required(VERSION 3.8)
project(ur10e_moveit2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(ur10e_moveit2_position_controller src/ur10e_moveit2_position_controller.cpp)
target_include_directories(ur10e_moveit2_position_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ur10e_moveit2_position_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ur10e_moveit2_position_controller
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(ur10e_moveit2_cylinder_section_controller src/ur10e_moveit2_cylinder_section_controller.cpp)
target_include_directories(ur10e_moveit2_cylinder_section_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ur10e_moveit2_cylinder_section_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ur10e_moveit2_cylinder_section_controller
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(ur10e_moveit2_cylinder_section_paused_controller src/ur10e_moveit2_cylinder_section_paused_controller.cpp)
target_include_directories(ur10e_moveit2_cylinder_section_paused_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ur10e_moveit2_cylinder_section_paused_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ur10e_moveit2_cylinder_section_paused_controller
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(ur10e_moveit2_rectangle_controller src/ur10e_moveit2_rectangle_controller.cpp)
target_include_directories(ur10e_moveit2_rectangle_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ur10e_moveit2_rectangle_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ur10e_moveit2_rectangle_controller
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(ur10e_moveit2_rectangle_paused_controller src/ur10e_moveit2_rectangle_paused_controller.cpp)
target_include_directories(ur10e_moveit2_rectangle_paused_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ur10e_moveit2_rectangle_paused_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ur10e_moveit2_rectangle_paused_controller
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(sim_camera_adapter src/sim_camera_adapter.cpp)
target_include_directories(sim_camera_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sim_camera_adapter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  sim_camera_adapter
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(controller src/controller.cpp)
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  controller
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS
  ur10e_moveit2_position_controller
  ur10e_moveit2_rectangle_controller
  ur10e_moveit2_rectangle_paused_controller
  ur10e_moveit2_cylinder_section_controller
  ur10e_moveit2_cylinder_section_paused_controller
  sim_camera_adapter
  controller
  DESTINATION lib/${PROJECT_NAME})

# Install config and launch files.
install(DIRECTORY
  config
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
