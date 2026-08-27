cmake_minimum_required(VERSION 3.16)
project(collision_checker_server)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ROS 2 Jazzy requires C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ik_solver_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(cnr_param REQUIRED)
find_package(cnr_yaml REQUIRED)
find_package(cnr_logger REQUIRED)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/ik_collision_checker.cpp
)

# Use target-specific include directories instead of global include_directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ik_solver_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
)

target_link_libraries(${PROJECT_NAME}
    cnr_param::cnr_param
    cnr_yaml::cnr_yaml
    cnr_logger::cnr_logger
)
# Create executable
add_executable(${PROJECT_NAME}_node src/node.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES 
  OUTPUT_NAME ik_collision_checker 
  PREFIX ""
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  ik_solver_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
) 

# Install targets
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Ament exports
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  ik_solver_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
)

ament_package()