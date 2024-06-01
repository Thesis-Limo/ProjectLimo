# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "limo_yolo: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ilimo_yolo:/home/thesis/ROS/src/limo_yolo/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(limo_yolo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/thesis/ROS/src/limo_yolo/msg/map.msg" NAME_WE)
add_custom_target(_limo_yolo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "limo_yolo" "/home/thesis/ROS/src/limo_yolo/msg/map.msg" "std_msgs/Header:geometry_msgs/Point:geometry_msgs/PointStamped"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(limo_yolo
  "/home/thesis/ROS/src/limo_yolo/msg/map.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_yolo
)

### Generating Services

### Generating Module File
_generate_module_cpp(limo_yolo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_yolo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(limo_yolo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(limo_yolo_generate_messages limo_yolo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/thesis/ROS/src/limo_yolo/msg/map.msg" NAME_WE)
add_dependencies(limo_yolo_generate_messages_cpp _limo_yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_yolo_gencpp)
add_dependencies(limo_yolo_gencpp limo_yolo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_yolo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(limo_yolo
  "/home/thesis/ROS/src/limo_yolo/msg/map.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_yolo
)

### Generating Services

### Generating Module File
_generate_module_eus(limo_yolo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_yolo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(limo_yolo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(limo_yolo_generate_messages limo_yolo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/thesis/ROS/src/limo_yolo/msg/map.msg" NAME_WE)
add_dependencies(limo_yolo_generate_messages_eus _limo_yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_yolo_geneus)
add_dependencies(limo_yolo_geneus limo_yolo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_yolo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(limo_yolo
  "/home/thesis/ROS/src/limo_yolo/msg/map.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_yolo
)

### Generating Services

### Generating Module File
_generate_module_lisp(limo_yolo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_yolo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(limo_yolo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(limo_yolo_generate_messages limo_yolo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/thesis/ROS/src/limo_yolo/msg/map.msg" NAME_WE)
add_dependencies(limo_yolo_generate_messages_lisp _limo_yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_yolo_genlisp)
add_dependencies(limo_yolo_genlisp limo_yolo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_yolo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(limo_yolo
  "/home/thesis/ROS/src/limo_yolo/msg/map.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_yolo
)

### Generating Services

### Generating Module File
_generate_module_nodejs(limo_yolo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_yolo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(limo_yolo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(limo_yolo_generate_messages limo_yolo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/thesis/ROS/src/limo_yolo/msg/map.msg" NAME_WE)
add_dependencies(limo_yolo_generate_messages_nodejs _limo_yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_yolo_gennodejs)
add_dependencies(limo_yolo_gennodejs limo_yolo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_yolo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(limo_yolo
  "/home/thesis/ROS/src/limo_yolo/msg/map.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_yolo
)

### Generating Services

### Generating Module File
_generate_module_py(limo_yolo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_yolo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(limo_yolo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(limo_yolo_generate_messages limo_yolo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/thesis/ROS/src/limo_yolo/msg/map.msg" NAME_WE)
add_dependencies(limo_yolo_generate_messages_py _limo_yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_yolo_genpy)
add_dependencies(limo_yolo_genpy limo_yolo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_yolo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_yolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_yolo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(limo_yolo_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_yolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_yolo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(limo_yolo_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_yolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_yolo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(limo_yolo_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_yolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_yolo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(limo_yolo_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_yolo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_yolo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_yolo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(limo_yolo_generate_messages_py geometry_msgs_generate_messages_py)
endif()
