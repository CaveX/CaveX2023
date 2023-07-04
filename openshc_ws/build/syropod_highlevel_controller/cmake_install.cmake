# Install script for directory: /home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_highlevel_controller

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install" TYPE PROGRAM FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install" TYPE PROGRAM FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install/setup.bash;/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install" TYPE FILE FILES
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/setup.bash"
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install/setup.sh;/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install" TYPE FILE FILES
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/setup.sh"
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install/setup.zsh;/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install" TYPE FILE FILES
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/setup.zsh"
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/install" TYPE FILE FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/syropod_highlevel_controller" TYPE FILE FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/devel/.private/syropod_highlevel_controller/include/syropod_highlevel_controller/DynamicConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/syropod_highlevel_controller" TYPE FILE FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/devel/.private/syropod_highlevel_controller/lib/python2.7/dist-packages/syropod_highlevel_controller/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/devel/.private/syropod_highlevel_controller/lib/python2.7/dist-packages/syropod_highlevel_controller/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/syropod_highlevel_controller" TYPE DIRECTORY FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/devel/.private/syropod_highlevel_controller/lib/python2.7/dist-packages/syropod_highlevel_controller/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/syropod_highlevel_controller/msg" TYPE FILE FILES
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_highlevel_controller/msg/LegState.msg"
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_highlevel_controller/msg/TipState.msg"
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_highlevel_controller/msg/TargetTipPose.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/syropod_highlevel_controller/cmake" TYPE FILE FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/syropod_highlevel_controller-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/devel/.private/syropod_highlevel_controller/include/syropod_highlevel_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/devel/.private/syropod_highlevel_controller/share/roseus/ros/syropod_highlevel_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/devel/.private/syropod_highlevel_controller/share/common-lisp/ros/syropod_highlevel_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/devel/.private/syropod_highlevel_controller/share/gennodejs/ros/syropod_highlevel_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/devel/.private/syropod_highlevel_controller/lib/python2.7/dist-packages/syropod_highlevel_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/devel/.private/syropod_highlevel_controller/lib/python2.7/dist-packages/syropod_highlevel_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/syropod_highlevel_controller.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/syropod_highlevel_controller/cmake" TYPE FILE FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/syropod_highlevel_controller-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/syropod_highlevel_controller/cmake" TYPE FILE FILES
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/syropod_highlevel_controllerConfig.cmake"
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/catkin_generated/installspace/syropod_highlevel_controllerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/syropod_highlevel_controller" TYPE FILE FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_highlevel_controller/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syropod_highlevel_controller/syropod_highlevel_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syropod_highlevel_controller/syropod_highlevel_controller_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syropod_highlevel_controller/syropod_highlevel_controller_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/syropod_highlevel_controller" TYPE EXECUTABLE FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/devel/.private/syropod_highlevel_controller/lib/syropod_highlevel_controller/syropod_highlevel_controller_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syropod_highlevel_controller/syropod_highlevel_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syropod_highlevel_controller/syropod_highlevel_controller_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syropod_highlevel_controller/syropod_highlevel_controller_node"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syropod_highlevel_controller/syropod_highlevel_controller_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/syropod_highlevel_controller" TYPE DIRECTORY FILES "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_highlevel_controller/include/syropod_highlevel_controller/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/syropod_highlevel_controller" TYPE DIRECTORY FILES
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_highlevel_controller/config"
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_highlevel_controller/launch"
    "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_highlevel_controller/rviz_cfg"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/syropod_highlevel_controller/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
