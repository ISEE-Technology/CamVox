# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "isee_synchronize: 3 messages, 2 services")

set(MSG_I_FLAGS "-Iisee_synchronize:/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(isee_synchronize_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/GPS.msg" NAME_WE)
add_custom_target(_isee_synchronize_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "isee_synchronize" "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/GPS.msg" "std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/refLLAUpdate.srv" NAME_WE)
add_custom_target(_isee_synchronize_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "isee_synchronize" "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/refLLAUpdate.srv" ""
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg" NAME_WE)
add_custom_target(_isee_synchronize_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "isee_synchronize" "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg" ""
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomMsg.msg" NAME_WE)
add_custom_target(_isee_synchronize_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "isee_synchronize" "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomMsg.msg" "isee_synchronize/CustomPoint:std_msgs/Header"
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/FirmwareUpdate.srv" NAME_WE)
add_custom_target(_isee_synchronize_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "isee_synchronize" "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/FirmwareUpdate.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/isee_synchronize
)
_generate_msg_cpp(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/isee_synchronize
)
_generate_msg_cpp(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/isee_synchronize
)

### Generating Services
_generate_srv_cpp(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/refLLAUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/isee_synchronize
)
_generate_srv_cpp(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/FirmwareUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/isee_synchronize
)

### Generating Module File
_generate_module_cpp(isee_synchronize
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/isee_synchronize
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(isee_synchronize_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(isee_synchronize_generate_messages isee_synchronize_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/GPS.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_cpp _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/refLLAUpdate.srv" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_cpp _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_cpp _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomMsg.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_cpp _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/FirmwareUpdate.srv" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_cpp _isee_synchronize_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(isee_synchronize_gencpp)
add_dependencies(isee_synchronize_gencpp isee_synchronize_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS isee_synchronize_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/isee_synchronize
)
_generate_msg_eus(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/isee_synchronize
)
_generate_msg_eus(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/isee_synchronize
)

### Generating Services
_generate_srv_eus(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/refLLAUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/isee_synchronize
)
_generate_srv_eus(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/FirmwareUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/isee_synchronize
)

### Generating Module File
_generate_module_eus(isee_synchronize
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/isee_synchronize
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(isee_synchronize_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(isee_synchronize_generate_messages isee_synchronize_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/GPS.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_eus _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/refLLAUpdate.srv" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_eus _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_eus _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomMsg.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_eus _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/FirmwareUpdate.srv" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_eus _isee_synchronize_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(isee_synchronize_geneus)
add_dependencies(isee_synchronize_geneus isee_synchronize_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS isee_synchronize_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/isee_synchronize
)
_generate_msg_lisp(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/isee_synchronize
)
_generate_msg_lisp(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/isee_synchronize
)

### Generating Services
_generate_srv_lisp(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/refLLAUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/isee_synchronize
)
_generate_srv_lisp(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/FirmwareUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/isee_synchronize
)

### Generating Module File
_generate_module_lisp(isee_synchronize
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/isee_synchronize
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(isee_synchronize_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(isee_synchronize_generate_messages isee_synchronize_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/GPS.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_lisp _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/refLLAUpdate.srv" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_lisp _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_lisp _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomMsg.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_lisp _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/FirmwareUpdate.srv" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_lisp _isee_synchronize_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(isee_synchronize_genlisp)
add_dependencies(isee_synchronize_genlisp isee_synchronize_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS isee_synchronize_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/isee_synchronize
)
_generate_msg_nodejs(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/isee_synchronize
)
_generate_msg_nodejs(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/isee_synchronize
)

### Generating Services
_generate_srv_nodejs(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/refLLAUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/isee_synchronize
)
_generate_srv_nodejs(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/FirmwareUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/isee_synchronize
)

### Generating Module File
_generate_module_nodejs(isee_synchronize
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/isee_synchronize
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(isee_synchronize_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(isee_synchronize_generate_messages isee_synchronize_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/GPS.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_nodejs _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/refLLAUpdate.srv" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_nodejs _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_nodejs _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomMsg.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_nodejs _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/FirmwareUpdate.srv" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_nodejs _isee_synchronize_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(isee_synchronize_gennodejs)
add_dependencies(isee_synchronize_gennodejs isee_synchronize_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS isee_synchronize_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/isee_synchronize
)
_generate_msg_py(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/isee_synchronize
)
_generate_msg_py(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/isee_synchronize
)

### Generating Services
_generate_srv_py(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/refLLAUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/isee_synchronize
)
_generate_srv_py(isee_synchronize
  "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/FirmwareUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/isee_synchronize
)

### Generating Module File
_generate_module_py(isee_synchronize
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/isee_synchronize
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(isee_synchronize_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(isee_synchronize_generate_messages isee_synchronize_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/GPS.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_py _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/refLLAUpdate.srv" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_py _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomPoint.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_py _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/msg/CustomMsg.msg" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_py _isee_synchronize_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/camvox/isee-synchronize/srv/FirmwareUpdate.srv" NAME_WE)
add_dependencies(isee_synchronize_generate_messages_py _isee_synchronize_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(isee_synchronize_genpy)
add_dependencies(isee_synchronize_genpy isee_synchronize_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS isee_synchronize_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/isee_synchronize)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/isee_synchronize
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(isee_synchronize_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(isee_synchronize_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/isee_synchronize)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/isee_synchronize
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(isee_synchronize_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(isee_synchronize_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/isee_synchronize)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/isee_synchronize
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(isee_synchronize_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(isee_synchronize_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/isee_synchronize)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/isee_synchronize
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(isee_synchronize_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(isee_synchronize_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/isee_synchronize)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/isee_synchronize\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/isee_synchronize
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(isee_synchronize_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(isee_synchronize_generate_messages_py geometry_msgs_generate_messages_py)
endif()
