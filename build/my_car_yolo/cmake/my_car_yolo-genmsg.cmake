# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "my_car_yolo: 4 messages, 0 services")

set(MSG_I_FLAGS "-Imy_car_yolo:/home/mycar/catkin_ws/src/my_car_yolo/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(my_car_yolo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ImuProcessed.msg" NAME_WE)
add_custom_target(_my_car_yolo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_car_yolo" "/home/mycar/catkin_ws/src/my_car_yolo/msg/ImuProcessed.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/WheelEncoders.msg" NAME_WE)
add_custom_target(_my_car_yolo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_car_yolo" "/home/mycar/catkin_ws/src/my_car_yolo/msg/WheelEncoders.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg" NAME_WE)
add_custom_target(_my_car_yolo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_car_yolo" "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetections.msg" NAME_WE)
add_custom_target(_my_car_yolo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_car_yolo" "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetections.msg" "std_msgs/Header:my_car_yolo/ObjectDetection"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ImuProcessed.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_car_yolo
)
_generate_msg_cpp(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/WheelEncoders.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_car_yolo
)
_generate_msg_cpp(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_car_yolo
)
_generate_msg_cpp(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetections.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_car_yolo
)

### Generating Services

### Generating Module File
_generate_module_cpp(my_car_yolo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_car_yolo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(my_car_yolo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(my_car_yolo_generate_messages my_car_yolo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ImuProcessed.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_cpp _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/WheelEncoders.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_cpp _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_cpp _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetections.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_cpp _my_car_yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_car_yolo_gencpp)
add_dependencies(my_car_yolo_gencpp my_car_yolo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_car_yolo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ImuProcessed.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_car_yolo
)
_generate_msg_eus(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/WheelEncoders.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_car_yolo
)
_generate_msg_eus(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_car_yolo
)
_generate_msg_eus(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetections.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_car_yolo
)

### Generating Services

### Generating Module File
_generate_module_eus(my_car_yolo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_car_yolo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(my_car_yolo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(my_car_yolo_generate_messages my_car_yolo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ImuProcessed.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_eus _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/WheelEncoders.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_eus _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_eus _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetections.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_eus _my_car_yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_car_yolo_geneus)
add_dependencies(my_car_yolo_geneus my_car_yolo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_car_yolo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ImuProcessed.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_car_yolo
)
_generate_msg_lisp(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/WheelEncoders.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_car_yolo
)
_generate_msg_lisp(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_car_yolo
)
_generate_msg_lisp(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetections.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_car_yolo
)

### Generating Services

### Generating Module File
_generate_module_lisp(my_car_yolo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_car_yolo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(my_car_yolo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(my_car_yolo_generate_messages my_car_yolo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ImuProcessed.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_lisp _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/WheelEncoders.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_lisp _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_lisp _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetections.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_lisp _my_car_yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_car_yolo_genlisp)
add_dependencies(my_car_yolo_genlisp my_car_yolo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_car_yolo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ImuProcessed.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_car_yolo
)
_generate_msg_nodejs(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/WheelEncoders.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_car_yolo
)
_generate_msg_nodejs(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_car_yolo
)
_generate_msg_nodejs(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetections.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_car_yolo
)

### Generating Services

### Generating Module File
_generate_module_nodejs(my_car_yolo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_car_yolo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(my_car_yolo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(my_car_yolo_generate_messages my_car_yolo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ImuProcessed.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_nodejs _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/WheelEncoders.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_nodejs _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_nodejs _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetections.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_nodejs _my_car_yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_car_yolo_gennodejs)
add_dependencies(my_car_yolo_gennodejs my_car_yolo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_car_yolo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ImuProcessed.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_car_yolo
)
_generate_msg_py(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/WheelEncoders.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_car_yolo
)
_generate_msg_py(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_car_yolo
)
_generate_msg_py(my_car_yolo
  "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetections.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_car_yolo
)

### Generating Services

### Generating Module File
_generate_module_py(my_car_yolo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_car_yolo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(my_car_yolo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(my_car_yolo_generate_messages my_car_yolo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ImuProcessed.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_py _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/WheelEncoders.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_py _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetection.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_py _my_car_yolo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mycar/catkin_ws/src/my_car_yolo/msg/ObjectDetections.msg" NAME_WE)
add_dependencies(my_car_yolo_generate_messages_py _my_car_yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_car_yolo_genpy)
add_dependencies(my_car_yolo_genpy my_car_yolo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_car_yolo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_car_yolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_car_yolo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(my_car_yolo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_car_yolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_car_yolo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(my_car_yolo_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_car_yolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_car_yolo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(my_car_yolo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_car_yolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_car_yolo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(my_car_yolo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_car_yolo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_car_yolo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_car_yolo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(my_car_yolo_generate_messages_py std_msgs_generate_messages_py)
endif()
