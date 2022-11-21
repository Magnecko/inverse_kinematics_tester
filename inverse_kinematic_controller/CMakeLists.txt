cmake_minimum_required(VERSION 3.8)
project(inverse_kinematic_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    generate_parameter_library
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  inverse_kinematic_controller_parameters
  src/inverse_kinematic_controller_parameters.yaml
)
generate_parameter_library(
  m#ulti_interface_forward_command_controller_parameters
  #src/multi_interface_forward_command_controller_parameters.yaml
)

add_library(forward_command_controller
  SHARED
  src/inverse_kinematic_controller_base.cpp
  src/inverse_kinematic_controller.cpp
  #src/multi_interface_forward_command_controller.cpp
)
target_include_directories(inverse_kinematic_controller PRIVATE include)
ament_target_dependencies(inverse_kinematic_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(inverse_kinematic_controller
  inverse_kinematic_controller_parameters
  #multi_interface_forward_command_controller_parameters
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(inverse_kinematic_controller PRIVATE "INVERSE_KINEMATIC_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface q_dot_command.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    inverse_kinematic_controller
    inverse_kinematic_controller_parameters
    #multi_interface_forward_command_controller_parameters
  EXPORT export_inverse_kinematic_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
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

ament_export_targets(export_forward_command_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
