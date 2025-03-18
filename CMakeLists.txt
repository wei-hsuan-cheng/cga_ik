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
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Source files
set(CGA_TEST_TEST_SOURCES src/cga_test.cpp)
set(CGA_IK_VALIDATION_SOURCES src/cga_ik_validate.cpp)
set(VISUALISE_ROBOT_TF_SOURCES src/visualise_robot_tf.cpp)

# Add executables
add_executable(cga_test ${CGA_TEST_TEST_SOURCES})
add_executable(cga_ik_validate ${CGA_IK_VALIDATION_SOURCES})
add_executable(visualise_robot_tf ${VISUALISE_ROBOT_TF_SOURCES})

# Link libraries
ament_target_dependencies(cga_test
  rclcpp
  Eigen3
)

ament_target_dependencies(cga_ik_validate
  rclcpp
  Eigen3
)

ament_target_dependencies(visualise_robot_tf
  rclcpp
  Eigen3
  tf2_ros
)


# Install targets
install(TARGETS
  cga_test
  cga_ik_validate
  visualise_robot_tf
  

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
