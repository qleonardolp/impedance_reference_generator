cmake_minimum_required(VERSION 3.16)
project(robot_impedance_analyzer VERSION 2.0.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(kinematic_pose_msgs REQUIRED)

find_package(generate_parameter_library REQUIRED)

# KinematicReference node
generate_parameter_library(
  kinematic_reference_parameters
  src/kinematic_reference_parameters.yaml
)

add_executable(kinematic_reference src/kinematic_reference.cpp)
target_include_directories(kinematic_reference PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(kinematic_reference PUBLIC c_std_99 cxx_std_17)

target_link_libraries(kinematic_reference PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  kinematic_reference_parameters
  ${kinematic_pose_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

# Impedance identification node
generate_parameter_library(
  impedance_id_parameters
  src/impedance_id_parameters.yaml
)

add_executable(identification src/impedance_id.cpp)
target_include_directories(identification PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(identification PUBLIC c_std_99 cxx_std_17)

target_link_libraries(identification PUBLIC
  Eigen3::Eigen
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  impedance_id_parameters
  ${kinematic_pose_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

# QuadrupedControl node
generate_parameter_library(
  quadruped_control_parameters
  src/quadruped_control_parameters.yaml
)

add_executable(quadruped_control src/quadruped_control.cpp)
target_include_directories(quadruped_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(quadruped_control PUBLIC c_std_99 cxx_std_17)

target_link_libraries(quadruped_control PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  quadruped_control_parameters
  ${kinematic_pose_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    kinematic_reference
    kinematic_reference_parameters
  EXPORT export_kinematic_reference
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    identification
    impedance_id_parameters
  EXPORT export_identification
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    quadruped_control
    quadruped_control_parameters
  EXPORT export_quadruped_control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_kinematic_reference HAS_LIBRARY_TARGET)
ament_export_targets(export_identification HAS_LIBRARY_TARGET)
ament_export_targets(export_quadruped_control HAS_LIBRARY_TARGET)
ament_package()
