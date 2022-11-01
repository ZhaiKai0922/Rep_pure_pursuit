# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "clean_avoidance: 7 messages, 0 services")

set(MSG_I_FLAGS "-Iclean_avoidance:/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(clean_avoidance_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg" NAME_WE)
add_custom_target(_clean_avoidance_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "clean_avoidance" "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg" "actionlib_msgs/GoalID:clean_avoidance/CleanAvoidanceGoal:actionlib_msgs/GoalStatus:clean_avoidance/CleanAvoidanceResult:clean_avoidance/CleanAvoidanceFeedback:clean_avoidance/CleanAvoidanceActionGoal:std_msgs/Header:clean_avoidance/CleanAvoidanceActionResult:clean_avoidance/CleanAvoidanceActionFeedback"
)

get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg" NAME_WE)
add_custom_target(_clean_avoidance_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "clean_avoidance" "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg" ""
)

get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg" NAME_WE)
add_custom_target(_clean_avoidance_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "clean_avoidance" "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg" ""
)

get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg" NAME_WE)
add_custom_target(_clean_avoidance_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "clean_avoidance" "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg" "clean_avoidance/CleanAvoidanceFeedback:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg" NAME_WE)
add_custom_target(_clean_avoidance_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "clean_avoidance" "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg" "actionlib_msgs/GoalID:clean_avoidance/CleanAvoidanceGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg" NAME_WE)
add_custom_target(_clean_avoidance_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "clean_avoidance" "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:clean_avoidance/CleanAvoidanceResult:std_msgs/Header"
)

get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg" NAME_WE)
add_custom_target(_clean_avoidance_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "clean_avoidance" "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/clean_avoidance
)
_generate_msg_cpp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/clean_avoidance
)
_generate_msg_cpp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/clean_avoidance
)
_generate_msg_cpp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/clean_avoidance
)
_generate_msg_cpp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/clean_avoidance
)
_generate_msg_cpp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/clean_avoidance
)
_generate_msg_cpp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/clean_avoidance
)

### Generating Services

### Generating Module File
_generate_module_cpp(clean_avoidance
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/clean_avoidance
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(clean_avoidance_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(clean_avoidance_generate_messages clean_avoidance_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_cpp _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_cpp _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_cpp _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_cpp _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_cpp _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_cpp _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_cpp _clean_avoidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(clean_avoidance_gencpp)
add_dependencies(clean_avoidance_gencpp clean_avoidance_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS clean_avoidance_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/clean_avoidance
)
_generate_msg_eus(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/clean_avoidance
)
_generate_msg_eus(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/clean_avoidance
)
_generate_msg_eus(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/clean_avoidance
)
_generate_msg_eus(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/clean_avoidance
)
_generate_msg_eus(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/clean_avoidance
)
_generate_msg_eus(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/clean_avoidance
)

### Generating Services

### Generating Module File
_generate_module_eus(clean_avoidance
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/clean_avoidance
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(clean_avoidance_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(clean_avoidance_generate_messages clean_avoidance_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_eus _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_eus _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_eus _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_eus _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_eus _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_eus _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_eus _clean_avoidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(clean_avoidance_geneus)
add_dependencies(clean_avoidance_geneus clean_avoidance_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS clean_avoidance_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/clean_avoidance
)
_generate_msg_lisp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/clean_avoidance
)
_generate_msg_lisp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/clean_avoidance
)
_generate_msg_lisp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/clean_avoidance
)
_generate_msg_lisp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/clean_avoidance
)
_generate_msg_lisp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/clean_avoidance
)
_generate_msg_lisp(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/clean_avoidance
)

### Generating Services

### Generating Module File
_generate_module_lisp(clean_avoidance
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/clean_avoidance
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(clean_avoidance_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(clean_avoidance_generate_messages clean_avoidance_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_lisp _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_lisp _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_lisp _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_lisp _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_lisp _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_lisp _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_lisp _clean_avoidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(clean_avoidance_genlisp)
add_dependencies(clean_avoidance_genlisp clean_avoidance_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS clean_avoidance_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/clean_avoidance
)
_generate_msg_nodejs(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/clean_avoidance
)
_generate_msg_nodejs(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/clean_avoidance
)
_generate_msg_nodejs(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/clean_avoidance
)
_generate_msg_nodejs(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/clean_avoidance
)
_generate_msg_nodejs(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/clean_avoidance
)
_generate_msg_nodejs(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/clean_avoidance
)

### Generating Services

### Generating Module File
_generate_module_nodejs(clean_avoidance
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/clean_avoidance
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(clean_avoidance_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(clean_avoidance_generate_messages clean_avoidance_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_nodejs _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_nodejs _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_nodejs _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_nodejs _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_nodejs _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_nodejs _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_nodejs _clean_avoidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(clean_avoidance_gennodejs)
add_dependencies(clean_avoidance_gennodejs clean_avoidance_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS clean_avoidance_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/clean_avoidance
)
_generate_msg_py(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/clean_avoidance
)
_generate_msg_py(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/clean_avoidance
)
_generate_msg_py(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/clean_avoidance
)
_generate_msg_py(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/clean_avoidance
)
_generate_msg_py(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/clean_avoidance
)
_generate_msg_py(clean_avoidance
  "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/clean_avoidance
)

### Generating Services

### Generating Module File
_generate_module_py(clean_avoidance
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/clean_avoidance
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(clean_avoidance_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(clean_avoidance_generate_messages clean_avoidance_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_py _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_py _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_py _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_py _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_py _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_py _clean_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg" NAME_WE)
add_dependencies(clean_avoidance_generate_messages_py _clean_avoidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(clean_avoidance_genpy)
add_dependencies(clean_avoidance_genpy clean_avoidance_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS clean_avoidance_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/clean_avoidance)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/clean_avoidance
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(clean_avoidance_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/clean_avoidance)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/clean_avoidance
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(clean_avoidance_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/clean_avoidance)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/clean_avoidance
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(clean_avoidance_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/clean_avoidance)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/clean_avoidance
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(clean_avoidance_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/clean_avoidance)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/clean_avoidance\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/clean_avoidance
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(clean_avoidance_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
