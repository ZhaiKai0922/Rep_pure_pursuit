# Install script for directory: /home/zk/zk/obstacle_avoidance/clean_avoidance/src/clean_avoidance

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zk/zk/obstacle_avoidance/clean_avoidance/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/clean_avoidance/action" TYPE FILE FILES "/home/zk/zk/obstacle_avoidance/clean_avoidance/src/clean_avoidance/action/CleanAvoidance.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/clean_avoidance/msg" TYPE FILE FILES
    "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceAction.msg"
    "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionGoal.msg"
    "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionResult.msg"
    "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceActionFeedback.msg"
    "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceGoal.msg"
    "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceResult.msg"
    "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/clean_avoidance/msg/CleanAvoidanceFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/clean_avoidance/cmake" TYPE FILE FILES "/home/zk/zk/obstacle_avoidance/clean_avoidance/build/clean_avoidance/catkin_generated/installspace/clean_avoidance-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/include/clean_avoidance")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/roseus/ros/clean_avoidance")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/common-lisp/ros/clean_avoidance")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/share/gennodejs/ros/clean_avoidance")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/lib/python2.7/dist-packages/clean_avoidance")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/zk/zk/obstacle_avoidance/clean_avoidance/devel/lib/python2.7/dist-packages/clean_avoidance")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zk/zk/obstacle_avoidance/clean_avoidance/build/clean_avoidance/catkin_generated/installspace/clean_avoidance.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/clean_avoidance/cmake" TYPE FILE FILES "/home/zk/zk/obstacle_avoidance/clean_avoidance/build/clean_avoidance/catkin_generated/installspace/clean_avoidance-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/clean_avoidance/cmake" TYPE FILE FILES
    "/home/zk/zk/obstacle_avoidance/clean_avoidance/build/clean_avoidance/catkin_generated/installspace/clean_avoidanceConfig.cmake"
    "/home/zk/zk/obstacle_avoidance/clean_avoidance/build/clean_avoidance/catkin_generated/installspace/clean_avoidanceConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/clean_avoidance" TYPE FILE FILES "/home/zk/zk/obstacle_avoidance/clean_avoidance/src/clean_avoidance/package.xml")
endif()

