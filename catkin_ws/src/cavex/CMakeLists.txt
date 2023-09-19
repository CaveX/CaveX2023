cmake_minimum_required(VERSION 3.0.2)
project(cavex
#  VERSION   0.1
#  LANGUAGES CXX CUDA
)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg nodelet)

# find_package(CUDA REQUIRED)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INLCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN3_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(PCL REQUIRED)

# set(VTK_DIR "/usr/include/vtk-6.3")
set(Ceres_DIR "/usr/local/lib/cmake/Ceres")

find_package(VTK REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${VTK_USE_FILE}
  ${CERES_INCLUDE_DIRS}
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#link_directories(
#  include
#  ${PCL_LIBRARY_DIRS}
#)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#  DIRECTORY msg
#  FILES
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

# SET(CUDA_NVCC_FLAGS "-arch=compute_30" CACHE STRING "nvcc flags" FORCE)
# SET(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
# SET(LIB_TYPE STATIC)
# CUDA_ADD_LIBRARY

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
  CATKIN_DEPENDS roscpp rospy std_msgs nodelet
  DEPENDS EIGEN3 PCL Ceres
  INCLUDE_DIRS include
#  INCLUDE_DIRS include
  LIBRARIES floamNodelet
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
# include
# ${catkin_INCLUDE_DIRS}
#)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/cavex.cpp
# )

add_library(floamNodelet src/nodelets/floamNodelet.cpp)

target_link_libraries(floamNodelet
  ${catkin_LIBRARIES}
  # ${PCL_LIBRARIES}
  # Ceres::ceres
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(floamNodelet ${catkin_EXPORTED_LIBRARIES})
endif()

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(publisher_sample_node src/publisher_sample_node.cpp)
add_executable(velodyneReaderNode
              src/lidar/velodyneReaderNode.cpp 
              src/lidar/velodyneSocketReader.cpp 
              src/lidar/velodynePCAPReader.cpp 
              src/lidar/velodyneUtils.cpp
              src/floam_cpu/laserMappingClass.cpp 
              src/floam_cpu/laserProcessingClass.cpp 
              src/floam_cpu/odomEstimationClass.cpp 
              src/floam_cpu/lidarOptimisation.cpp
              src/object_detection/objCluster.cpp
              src/object_detection/objPointCloudProcessor.cpp
              src/object_detection/objRansac.cpp
              src/object_detection/objRender.cpp
)
# add_executable(robotStateNode src/robot_state/robotStateNode.cpp)

add_executable(nodeletManagerNode src/nodeletManagerNode.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

add_dependencies(publisher_sample_node ${publisher_sample_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(velodyneReaderNode ${velodyneReaderNode_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(publisher_sample_node
  ${catkin_LIBRARIES}
)

target_link_libraries(velodyneReaderNode
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Ceres::ceres
)

# target_link_libraries(robotStateNode
#   ${catkin_LIBRARIES}
# )

target_link_libraries(nodeletManagerNode
  floamNodelet
  ${catkin_LIBRARIES}
)
