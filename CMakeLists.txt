cmake_minimum_required(VERSION 3.16)
project(impedance_reference_generator VERSION 1.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(kinematic_pose_msgs REQUIRED)

find_package(generate_parameter_library REQUIRED)

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

install(
  TARGETS
    kinematic_reference
    kinematic_reference_parameters
  EXPORT export_kinematic_reference
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_kinematic_reference HAS_LIBRARY_TARGET)
ament_package()
