# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "limo_behaviour_tree: 0 messages, 3 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(limo_behaviour_tree_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/PathType.srv" NAME_WE)
add_custom_target(_limo_behaviour_tree_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "limo_behaviour_tree" "/home/thesis/ROS/src/limo_behaviour_tree/srv/PathType.srv" ""
)

get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/EndGoal.srv" NAME_WE)
add_custom_target(_limo_behaviour_tree_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "limo_behaviour_tree" "/home/thesis/ROS/src/limo_behaviour_tree/srv/EndGoal.srv" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/TypeObjectTracking.srv" NAME_WE)
add_custom_target(_limo_behaviour_tree_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "limo_behaviour_tree" "/home/thesis/ROS/src/limo_behaviour_tree/srv/TypeObjectTracking.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/PathType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_behaviour_tree
)
_generate_srv_cpp(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/EndGoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_behaviour_tree
)
_generate_srv_cpp(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/TypeObjectTracking.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_behaviour_tree
)

### Generating Module File
_generate_module_cpp(limo_behaviour_tree
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_behaviour_tree
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(limo_behaviour_tree_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(limo_behaviour_tree_generate_messages limo_behaviour_tree_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/PathType.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_cpp _limo_behaviour_tree_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/EndGoal.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_cpp _limo_behaviour_tree_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/TypeObjectTracking.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_cpp _limo_behaviour_tree_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_behaviour_tree_gencpp)
add_dependencies(limo_behaviour_tree_gencpp limo_behaviour_tree_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_behaviour_tree_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/PathType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_behaviour_tree
)
_generate_srv_eus(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/EndGoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_behaviour_tree
)
_generate_srv_eus(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/TypeObjectTracking.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_behaviour_tree
)

### Generating Module File
_generate_module_eus(limo_behaviour_tree
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_behaviour_tree
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(limo_behaviour_tree_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(limo_behaviour_tree_generate_messages limo_behaviour_tree_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/PathType.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_eus _limo_behaviour_tree_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/EndGoal.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_eus _limo_behaviour_tree_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/TypeObjectTracking.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_eus _limo_behaviour_tree_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_behaviour_tree_geneus)
add_dependencies(limo_behaviour_tree_geneus limo_behaviour_tree_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_behaviour_tree_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/PathType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_behaviour_tree
)
_generate_srv_lisp(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/EndGoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_behaviour_tree
)
_generate_srv_lisp(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/TypeObjectTracking.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_behaviour_tree
)

### Generating Module File
_generate_module_lisp(limo_behaviour_tree
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_behaviour_tree
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(limo_behaviour_tree_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(limo_behaviour_tree_generate_messages limo_behaviour_tree_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/PathType.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_lisp _limo_behaviour_tree_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/EndGoal.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_lisp _limo_behaviour_tree_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/TypeObjectTracking.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_lisp _limo_behaviour_tree_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_behaviour_tree_genlisp)
add_dependencies(limo_behaviour_tree_genlisp limo_behaviour_tree_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_behaviour_tree_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/PathType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_behaviour_tree
)
_generate_srv_nodejs(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/EndGoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_behaviour_tree
)
_generate_srv_nodejs(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/TypeObjectTracking.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_behaviour_tree
)

### Generating Module File
_generate_module_nodejs(limo_behaviour_tree
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_behaviour_tree
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(limo_behaviour_tree_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(limo_behaviour_tree_generate_messages limo_behaviour_tree_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/PathType.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_nodejs _limo_behaviour_tree_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/EndGoal.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_nodejs _limo_behaviour_tree_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/TypeObjectTracking.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_nodejs _limo_behaviour_tree_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_behaviour_tree_gennodejs)
add_dependencies(limo_behaviour_tree_gennodejs limo_behaviour_tree_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_behaviour_tree_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/PathType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_behaviour_tree
)
_generate_srv_py(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/EndGoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_behaviour_tree
)
_generate_srv_py(limo_behaviour_tree
  "/home/thesis/ROS/src/limo_behaviour_tree/srv/TypeObjectTracking.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_behaviour_tree
)

### Generating Module File
_generate_module_py(limo_behaviour_tree
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_behaviour_tree
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(limo_behaviour_tree_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(limo_behaviour_tree_generate_messages limo_behaviour_tree_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/PathType.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_py _limo_behaviour_tree_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/EndGoal.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_py _limo_behaviour_tree_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/thesis/ROS/src/limo_behaviour_tree/srv/TypeObjectTracking.srv" NAME_WE)
add_dependencies(limo_behaviour_tree_generate_messages_py _limo_behaviour_tree_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_behaviour_tree_genpy)
add_dependencies(limo_behaviour_tree_genpy limo_behaviour_tree_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_behaviour_tree_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_behaviour_tree)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_behaviour_tree
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(limo_behaviour_tree_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(limo_behaviour_tree_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_behaviour_tree)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_behaviour_tree
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(limo_behaviour_tree_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(limo_behaviour_tree_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_behaviour_tree)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_behaviour_tree
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(limo_behaviour_tree_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(limo_behaviour_tree_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_behaviour_tree)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_behaviour_tree
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(limo_behaviour_tree_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(limo_behaviour_tree_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_behaviour_tree)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_behaviour_tree\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_behaviour_tree
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(limo_behaviour_tree_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(limo_behaviour_tree_generate_messages_py std_msgs_generate_messages_py)
endif()
