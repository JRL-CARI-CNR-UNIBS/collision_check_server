cmake_minimum_required(VERSION 3.0.2)
project(collision_checker_server)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED COMPONENTS
  ik_solver_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  roscpp
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES collision_checker_server
  CATKIN_DEPENDS ik_solver_msgs moveit_core moveit_msgs moveit_ros_planning roscpp
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/ik_collision_checker.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/node.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ik_collision_checker PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
