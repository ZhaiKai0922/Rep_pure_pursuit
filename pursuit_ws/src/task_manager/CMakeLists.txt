cmake_minimum_required(VERSION 3.0.2)
project(task_manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  inspur_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf
  tf2_ros
)

find_package(OpenCV 3 REQUIRED)
find_package(Bullet REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES task_manager
#  CATKIN_DEPENDS roscpp stdmsgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
  /usr/local/include/yaml-cpp
)

set(JSONCPP /usr/lib/x86_64-linux-gnu/cmake/jsoncpp)

link_directories(
  /usr/local/lib
)

add_executable(task_manager_node
src/task_manager_node.cpp
src/pub_path.cpp
src/pure_pursuit.cpp
src/task_manager.cpp
)

add_dependencies(task_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(task_manager_node ${PROJECT_NAME}_gencfg)

target_link_libraries(task_manager_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  jsoncpp
)
