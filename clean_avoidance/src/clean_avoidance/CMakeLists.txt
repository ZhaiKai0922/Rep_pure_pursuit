cmake_minimum_required(VERSION 3.0.2)
project(clean_avoidance)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        geometry_msgs
        costmap_2d
        map_msgs
        nav_msgs
        pluginlib
        roscpp
	rospy
        sensor_msgs
        std_msgs
        tf2
        tf2_geometry_msgs
        tf2_ros
        tf2_sensor_msgs
        visualization_msgs
        voxel_grid
        actionlib
        actionlib_msgs
	nav_core
	navfn
	cmake_modules
	message_generation
        )

## Generate actions in the 'action' folder
 add_action_files(
    FILES
    CleanAvoidance.action
    )

 generate_messages(
    DEPENDENCIES
    actionlib_msgs  # Or other packages containing msgs
    )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES clean_avoidance
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

##add_executable(clean_avoidance_node src/clean_avoidance.cpp)
##add_dependencies(clean_avoidance_node ${catkin_EXPORTED_TARGETS})
##target_link_libraries(clean_avoidance_node ${catkin_LIBRARIES})

##add_executable(clean_avoidance_double_node src/clean_avoidance_double.cpp src/pure_pursuit.cpp)
##add_dependencies(clean_avoidance_double_node ${catkin_EXPORTED_TARGETS})
##target_link_libraries(clean_avoidance_double_node ${catkin_LIBRARIES})

add_executable(clean_avoidance_node src/clean_avoidance.cpp src/pure_pursuit.cpp)
add_dependencies(clean_avoidance_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(clean_avoidance_node ${catkin_LIBRARIES})
