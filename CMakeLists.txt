cmake_minimum_required(VERSION 3.8)
project(cga_ik)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Suppress warnings for Eigen
add_compile_options(-Wno-class-memacces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)  # <-- for using actions in C++
find_package(rclcpp_components REQUIRED)  # <-- for using actions in C++
find_package(cga_ik_action_interfaces REQUIRED)  # <-- customised action_interface for cga_ik
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


# Dependencies list
set(dependencies
    ament_cmake
    rclcpp
    rclcpp_action
    rclcpp_components
    cga_ik_action_interfaces
    Eigen3
    tf2_ros
    sensor_msgs
    visualization_msgs
)

# For ROS 2 action
ament_export_dependencies(rosidl_default_runtime)


# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Source files
set(CGA_TEST_TEST_SOURCES src/cga_test.cpp)
set(VISUALISE_COBOT_6DOF_SOURCES src/visualise_cobot_6dof.cpp)
set(VISUALISE_SPM_3DOF_SOURCES src/visualise_spm_3dof.cpp)
set(SPM_ACTION_SERVER_SOURCES src/spm_action_server.cpp)

# Add executables
add_executable(cga_test ${CGA_TEST_TEST_SOURCES})
add_executable(visualise_cobot_6dof ${VISUALISE_COBOT_6DOF_SOURCES})
add_executable(visualise_spm_3dof ${VISUALISE_SPM_3DOF_SOURCES})
add_executable(spm_action_server ${SPM_ACTION_SERVER_SOURCES})

target_link_libraries(spm_action_server)

# Link libraries
ament_target_dependencies(cga_test ${dependencies})
ament_target_dependencies(visualise_cobot_6dof ${dependencies})
ament_target_dependencies(visualise_spm_3dof ${dependencies})
ament_target_dependencies(spm_action_server ${dependencies})

# Install targets
install(TARGETS
  cga_test
  visualise_cobot_6dof
  visualise_spm_3dof
  spm_action_server
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(
  DIRECTORY launch config meshes
  DESTINATION share/${PROJECT_NAME}
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
