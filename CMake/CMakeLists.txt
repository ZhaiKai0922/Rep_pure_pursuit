cmake_minimum_required(VERSION 3.0.2)
project(task_manager)

set(catkin_RUN_PACKAGES
  clean_avoidance
  angles
  cob_map_accessibility_analysis
  cv_bridge
  eigen_conversions
  geometry_msgs
  laser_geometry
  move_base_msgs
  dynamic_reconfigure
  nav_msgs
  roslib
  sensor_msgs
  std_srvs
  roscpp
  rospy
  std_msgs
  inspur_msgs
  tf2
  tf
  tf2_ros
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS
	${catkin_RUN_PACKAGES}
)
find_package(OpenCV  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system chrono thread)


catkin_package()


include_directories(
  include
  common/include
  include/ccpp
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
	common/src/boustrophedon_explorator.cpp
	common/src/room_rotator.cpp
	common/src/meanshift2d.cpp
	common/src/A_star_pathplanner.cpp
	common/src/concorde_TSP.cpp
	common/src/genetic_TSP.cpp
	common/src/nearest_neighbor_TSP.cpp
	common/src/node.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	libcoverage_check_server
)


add_executable(task_manager_node
	src/task_manager_node.cpp
	src/task_manager.cpp
)
add_dependencies(task_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_manager_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${orocos_kdl_LIBRARIES}
	${YAML_CPP_LIBRARIES}
	${Boost_LIBRARIES}
	libcoverage_check_server
	jsoncpp
)


add_executable(complete_cov_path_plan
	src/ccpp/complete_coverage_path_planning.cpp
	src/ccpp/fov_to_robot_mapper.cpp
)
add_dependencies(complete_cov_path_plan ${catkin_EXPORTED_TARGETS})
target_link_libraries(complete_cov_path_plan
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${PROJECT_NAME}
	libcoverage_check_server
)


add_library(libcoverage_check_server
	src/ccpp/coverage_check_server.cpp
)
add_dependencies(libcoverage_check_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(libcoverage_check_server
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
)


add_executable(coverage_check_server
	src/ccpp/coverage_check_server_main.cpp
)
add_dependencies(coverage_check_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_check_server
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	libcoverage_check_server
)


add_executable(ccpp_trigger_client
	src/ccpp_trigger_client.cpp
)
add_dependencies(ccpp_trigger_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(ccpp_trigger_client
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
)


add_executable(room_exploration_client
	src/ccpp/room_exploration_action_client.cpp
)
add_dependencies(room_exploration_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(room_exploration_client
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
)
