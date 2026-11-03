cmake_minimum_required(VERSION 3.8)
project(iahrs_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------
# Generate Libraries
# --------------------------------------------------------------
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/lib/serialib/lib)

add_library(serial src/serial.cpp)
add_library(iahrs_driver src/iahrs_driver.cpp)

# --------------------------------------------------------------
# Generate Executable
# --------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/iahrs_ros2_driver_node.cpp)

# --------------------------------------------------------------
# Fine Dependencies
# --------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(iahrs_ros2_driver_msgs REQUIRED)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  tf2_ros

  iahrs_ros2_driver_msgs
)

target_link_libraries(iahrs_driver serial)
target_link_libraries(${PROJECT_NAME}_node iahrs_driver)

# --------------------------------------------------------------
# Node Install
# --------------------------------------------------------------
install(TARGETS
  serial
  iahrs_driver
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# --------------------------------------------------------------
# Shared Directory
# --------------------------------------------------------------
install(DIRECTORY
  launch param
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()