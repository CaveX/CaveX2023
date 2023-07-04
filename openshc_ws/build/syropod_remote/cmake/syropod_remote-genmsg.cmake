# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "syropod_remote: 2 messages, 0 services")

set(MSG_I_FLAGS "-Isyropod_remote:/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(syropod_remote_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidJoy.msg" NAME_WE)
add_custom_target(_syropod_remote_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "syropod_remote" "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidJoy.msg" "std_msgs/String:geometry_msgs/Point:std_msgs/Bool:std_msgs/Int8:std_msgs/Header"
)

get_filename_component(_filename "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidSensor.msg" NAME_WE)
add_custom_target(_syropod_remote_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "syropod_remote" "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidSensor.msg" "std_msgs/Int8:std_msgs/Bool:std_msgs/String:std_msgs/Header:std_msgs/Float64:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(syropod_remote
  "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidJoy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/syropod_remote
)
_generate_msg_cpp(syropod_remote
  "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidSensor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/syropod_remote
)

### Generating Services

### Generating Module File
_generate_module_cpp(syropod_remote
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/syropod_remote
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(syropod_remote_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(syropod_remote_generate_messages syropod_remote_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidJoy.msg" NAME_WE)
add_dependencies(syropod_remote_generate_messages_cpp _syropod_remote_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidSensor.msg" NAME_WE)
add_dependencies(syropod_remote_generate_messages_cpp _syropod_remote_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(syropod_remote_gencpp)
add_dependencies(syropod_remote_gencpp syropod_remote_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS syropod_remote_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(syropod_remote
  "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidJoy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/syropod_remote
)
_generate_msg_eus(syropod_remote
  "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidSensor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/syropod_remote
)

### Generating Services

### Generating Module File
_generate_module_eus(syropod_remote
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/syropod_remote
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(syropod_remote_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(syropod_remote_generate_messages syropod_remote_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidJoy.msg" NAME_WE)
add_dependencies(syropod_remote_generate_messages_eus _syropod_remote_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidSensor.msg" NAME_WE)
add_dependencies(syropod_remote_generate_messages_eus _syropod_remote_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(syropod_remote_geneus)
add_dependencies(syropod_remote_geneus syropod_remote_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS syropod_remote_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(syropod_remote
  "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidJoy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/syropod_remote
)
_generate_msg_lisp(syropod_remote
  "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidSensor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/syropod_remote
)

### Generating Services

### Generating Module File
_generate_module_lisp(syropod_remote
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/syropod_remote
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(syropod_remote_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(syropod_remote_generate_messages syropod_remote_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidJoy.msg" NAME_WE)
add_dependencies(syropod_remote_generate_messages_lisp _syropod_remote_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidSensor.msg" NAME_WE)
add_dependencies(syropod_remote_generate_messages_lisp _syropod_remote_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(syropod_remote_genlisp)
add_dependencies(syropod_remote_genlisp syropod_remote_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS syropod_remote_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(syropod_remote
  "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidJoy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/syropod_remote
)
_generate_msg_nodejs(syropod_remote
  "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidSensor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/syropod_remote
)

### Generating Services

### Generating Module File
_generate_module_nodejs(syropod_remote
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/syropod_remote
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(syropod_remote_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(syropod_remote_generate_messages syropod_remote_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidJoy.msg" NAME_WE)
add_dependencies(syropod_remote_generate_messages_nodejs _syropod_remote_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidSensor.msg" NAME_WE)
add_dependencies(syropod_remote_generate_messages_nodejs _syropod_remote_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(syropod_remote_gennodejs)
add_dependencies(syropod_remote_gennodejs syropod_remote_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS syropod_remote_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(syropod_remote
  "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidJoy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/syropod_remote
)
_generate_msg_py(syropod_remote
  "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidSensor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/syropod_remote
)

### Generating Services

### Generating Module File
_generate_module_py(syropod_remote
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/syropod_remote
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(syropod_remote_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(syropod_remote_generate_messages syropod_remote_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidJoy.msg" NAME_WE)
add_dependencies(syropod_remote_generate_messages_py _syropod_remote_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/src/syropod_remote/msg/AndroidSensor.msg" NAME_WE)
add_dependencies(syropod_remote_generate_messages_py _syropod_remote_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(syropod_remote_genpy)
add_dependencies(syropod_remote_genpy syropod_remote_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS syropod_remote_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/syropod_remote)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/syropod_remote
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(syropod_remote_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(syropod_remote_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/syropod_remote)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/syropod_remote
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(syropod_remote_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(syropod_remote_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/syropod_remote)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/syropod_remote
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(syropod_remote_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(syropod_remote_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/syropod_remote)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/syropod_remote
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(syropod_remote_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(syropod_remote_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/syropod_remote)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/syropod_remote\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/syropod_remote
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(syropod_remote_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(syropod_remote_generate_messages_py geometry_msgs_generate_messages_py)
endif()
