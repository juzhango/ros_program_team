# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "learning_msg: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ilearning_msg:/home/teabot/my_workSpace/a/ros_program_team/catkin_ws/src/learning_msg/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(learning_msg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/teabot/my_workSpace/a/ros_program_team/catkin_ws/src/learning_msg/msg/Num.msg" NAME_WE)
add_custom_target(_learning_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "learning_msg" "/home/teabot/my_workSpace/a/ros_program_team/catkin_ws/src/learning_msg/msg/Num.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(learning_msg
  "/home/teabot/my_workSpace/a/ros_program_team/catkin_ws/src/learning_msg/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_msg
)

### Generating Services

### Generating Module File
_generate_module_cpp(learning_msg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_msg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(learning_msg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(learning_msg_generate_messages learning_msg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/teabot/my_workSpace/a/ros_program_team/catkin_ws/src/learning_msg/msg/Num.msg" NAME_WE)
add_dependencies(learning_msg_generate_messages_cpp _learning_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(learning_msg_gencpp)
add_dependencies(learning_msg_gencpp learning_msg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS learning_msg_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(learning_msg
  "/home/teabot/my_workSpace/a/ros_program_team/catkin_ws/src/learning_msg/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_msg
)

### Generating Services

### Generating Module File
_generate_module_lisp(learning_msg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_msg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(learning_msg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(learning_msg_generate_messages learning_msg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/teabot/my_workSpace/a/ros_program_team/catkin_ws/src/learning_msg/msg/Num.msg" NAME_WE)
add_dependencies(learning_msg_generate_messages_lisp _learning_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(learning_msg_genlisp)
add_dependencies(learning_msg_genlisp learning_msg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS learning_msg_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(learning_msg
  "/home/teabot/my_workSpace/a/ros_program_team/catkin_ws/src/learning_msg/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_msg
)

### Generating Services

### Generating Module File
_generate_module_py(learning_msg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_msg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(learning_msg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(learning_msg_generate_messages learning_msg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/teabot/my_workSpace/a/ros_program_team/catkin_ws/src/learning_msg/msg/Num.msg" NAME_WE)
add_dependencies(learning_msg_generate_messages_py _learning_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(learning_msg_genpy)
add_dependencies(learning_msg_genpy learning_msg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS learning_msg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_msg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(learning_msg_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(learning_msg_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(learning_msg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_msg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(learning_msg_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(learning_msg_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(learning_msg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_msg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_msg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_msg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(learning_msg_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(learning_msg_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(learning_msg_generate_messages_py std_msgs_generate_messages_py)
endif()
