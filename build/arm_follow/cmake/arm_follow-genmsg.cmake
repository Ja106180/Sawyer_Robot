# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "arm_follow: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iarm_follow:/home/mycar/catkin_ws/src/arm_follow/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iintera_core_msgs:/home/mycar/catkin_ws/src/sawyer_noetic/intera_common/intera_core_msgs/msg;-Iintera_core_msgs:/home/mycar/catkin_ws/devel/share/intera_core_msgs/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(arm_follow_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg" NAME_WE)
add_custom_target(_arm_follow_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_follow" "/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/mycar/catkin_ws/src/arm_follow/msg/PersonsKeypoints.msg" NAME_WE)
add_custom_target(_arm_follow_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_follow" "/home/mycar/catkin_ws/src/arm_follow/msg/PersonsKeypoints.msg" "std_msgs/Header:arm_follow/PersonKeypoints"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(arm_follow
  "/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_follow
)
_generate_msg_cpp(arm_follow
  "/home/mycar/catkin_ws/src/arm_follow/msg/PersonsKeypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_follow
)

### Generating Services

### Generating Module File
_generate_module_cpp(arm_follow
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_follow
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(arm_follow_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(arm_follow_generate_messages arm_follow_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg" NAME_WE)
add_dependencies(arm_follow_generate_messages_cpp _arm_follow_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/arm_follow/msg/PersonsKeypoints.msg" NAME_WE)
add_dependencies(arm_follow_generate_messages_cpp _arm_follow_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_follow_gencpp)
add_dependencies(arm_follow_gencpp arm_follow_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_follow_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(arm_follow
  "/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_follow
)
_generate_msg_eus(arm_follow
  "/home/mycar/catkin_ws/src/arm_follow/msg/PersonsKeypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_follow
)

### Generating Services

### Generating Module File
_generate_module_eus(arm_follow
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_follow
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(arm_follow_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(arm_follow_generate_messages arm_follow_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg" NAME_WE)
add_dependencies(arm_follow_generate_messages_eus _arm_follow_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/arm_follow/msg/PersonsKeypoints.msg" NAME_WE)
add_dependencies(arm_follow_generate_messages_eus _arm_follow_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_follow_geneus)
add_dependencies(arm_follow_geneus arm_follow_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_follow_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(arm_follow
  "/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_follow
)
_generate_msg_lisp(arm_follow
  "/home/mycar/catkin_ws/src/arm_follow/msg/PersonsKeypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_follow
)

### Generating Services

### Generating Module File
_generate_module_lisp(arm_follow
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_follow
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(arm_follow_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(arm_follow_generate_messages arm_follow_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg" NAME_WE)
add_dependencies(arm_follow_generate_messages_lisp _arm_follow_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/arm_follow/msg/PersonsKeypoints.msg" NAME_WE)
add_dependencies(arm_follow_generate_messages_lisp _arm_follow_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_follow_genlisp)
add_dependencies(arm_follow_genlisp arm_follow_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_follow_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(arm_follow
  "/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_follow
)
_generate_msg_nodejs(arm_follow
  "/home/mycar/catkin_ws/src/arm_follow/msg/PersonsKeypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_follow
)

### Generating Services

### Generating Module File
_generate_module_nodejs(arm_follow
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_follow
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(arm_follow_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(arm_follow_generate_messages arm_follow_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg" NAME_WE)
add_dependencies(arm_follow_generate_messages_nodejs _arm_follow_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/arm_follow/msg/PersonsKeypoints.msg" NAME_WE)
add_dependencies(arm_follow_generate_messages_nodejs _arm_follow_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_follow_gennodejs)
add_dependencies(arm_follow_gennodejs arm_follow_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_follow_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(arm_follow
  "/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_follow
)
_generate_msg_py(arm_follow
  "/home/mycar/catkin_ws/src/arm_follow/msg/PersonsKeypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_follow
)

### Generating Services

### Generating Module File
_generate_module_py(arm_follow
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_follow
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(arm_follow_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(arm_follow_generate_messages arm_follow_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mycar/catkin_ws/src/arm_follow/msg/PersonKeypoints.msg" NAME_WE)
add_dependencies(arm_follow_generate_messages_py _arm_follow_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/arm_follow/msg/PersonsKeypoints.msg" NAME_WE)
add_dependencies(arm_follow_generate_messages_py _arm_follow_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_follow_genpy)
add_dependencies(arm_follow_genpy arm_follow_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_follow_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_follow)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_follow
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(arm_follow_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET intera_core_msgs_generate_messages_cpp)
  add_dependencies(arm_follow_generate_messages_cpp intera_core_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(arm_follow_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(arm_follow_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_follow)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_follow
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(arm_follow_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET intera_core_msgs_generate_messages_eus)
  add_dependencies(arm_follow_generate_messages_eus intera_core_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(arm_follow_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(arm_follow_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_follow)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_follow
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(arm_follow_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET intera_core_msgs_generate_messages_lisp)
  add_dependencies(arm_follow_generate_messages_lisp intera_core_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(arm_follow_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(arm_follow_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_follow)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_follow
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(arm_follow_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET intera_core_msgs_generate_messages_nodejs)
  add_dependencies(arm_follow_generate_messages_nodejs intera_core_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(arm_follow_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(arm_follow_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_follow)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_follow\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_follow
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(arm_follow_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET intera_core_msgs_generate_messages_py)
  add_dependencies(arm_follow_generate_messages_py intera_core_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(arm_follow_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(arm_follow_generate_messages_py std_msgs_generate_messages_py)
endif()
