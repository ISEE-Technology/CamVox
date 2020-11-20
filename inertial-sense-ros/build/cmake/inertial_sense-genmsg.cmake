# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "inertial_sense: 12 messages, 2 services")

set(MSG_I_FLAGS "-Iinertial_sense:/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(inertial_sense_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/PreIntIMU.msg" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/PreIntIMU.msg" "std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObsVec.msg" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObsVec.msg" "inertial_sense/GNSSObservation:inertial_sense/GTime:std_msgs/Header"
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg" ""
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKRel.msg" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKRel.msg" "std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GlonassEphemeris.msg" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GlonassEphemeris.msg" "inertial_sense/GTime"
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/INL2States.msg" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/INL2States.msg" "geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPS.msg" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPS.msg" "std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg" ""
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSEphemeris.msg" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSEphemeris.msg" "inertial_sense/GTime:std_msgs/Header"
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKInfo.msg" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKInfo.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/refLLAUpdate.srv" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/refLLAUpdate.srv" ""
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg" "inertial_sense/GTime:std_msgs/Header"
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPSInfo.msg" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPSInfo.msg" "inertial_sense/SatInfo:std_msgs/Header"
)

get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/FirmwareUpdate.srv" NAME_WE)
add_custom_target(_inertial_sense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inertial_sense" "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/FirmwareUpdate.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/PreIntIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)
_generate_msg_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObsVec.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg;/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)
_generate_msg_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPSInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)
_generate_msg_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKRel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)
_generate_msg_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GlonassEphemeris.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)
_generate_msg_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/INL2States.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)
_generate_msg_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)
_generate_msg_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)
_generate_msg_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSEphemeris.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)
_generate_msg_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)
_generate_msg_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)
_generate_msg_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)

### Generating Services
_generate_srv_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/FirmwareUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)
_generate_srv_cpp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/refLLAUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
)

### Generating Module File
_generate_module_cpp(inertial_sense
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(inertial_sense_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(inertial_sense_generate_messages inertial_sense_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/PreIntIMU.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObsVec.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKRel.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GlonassEphemeris.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/INL2States.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPS.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSEphemeris.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/refLLAUpdate.srv" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPSInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/FirmwareUpdate.srv" NAME_WE)
add_dependencies(inertial_sense_generate_messages_cpp _inertial_sense_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inertial_sense_gencpp)
add_dependencies(inertial_sense_gencpp inertial_sense_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inertial_sense_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/PreIntIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)
_generate_msg_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObsVec.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg;/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)
_generate_msg_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPSInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)
_generate_msg_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKRel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)
_generate_msg_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GlonassEphemeris.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)
_generate_msg_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/INL2States.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)
_generate_msg_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)
_generate_msg_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)
_generate_msg_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSEphemeris.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)
_generate_msg_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)
_generate_msg_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)
_generate_msg_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)

### Generating Services
_generate_srv_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/FirmwareUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)
_generate_srv_eus(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/refLLAUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
)

### Generating Module File
_generate_module_eus(inertial_sense
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(inertial_sense_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(inertial_sense_generate_messages inertial_sense_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/PreIntIMU.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObsVec.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKRel.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GlonassEphemeris.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/INL2States.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPS.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSEphemeris.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/refLLAUpdate.srv" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPSInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/FirmwareUpdate.srv" NAME_WE)
add_dependencies(inertial_sense_generate_messages_eus _inertial_sense_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inertial_sense_geneus)
add_dependencies(inertial_sense_geneus inertial_sense_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inertial_sense_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/PreIntIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)
_generate_msg_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObsVec.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg;/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)
_generate_msg_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPSInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)
_generate_msg_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKRel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)
_generate_msg_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GlonassEphemeris.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)
_generate_msg_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/INL2States.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)
_generate_msg_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)
_generate_msg_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)
_generate_msg_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSEphemeris.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)
_generate_msg_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)
_generate_msg_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)
_generate_msg_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)

### Generating Services
_generate_srv_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/FirmwareUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)
_generate_srv_lisp(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/refLLAUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
)

### Generating Module File
_generate_module_lisp(inertial_sense
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(inertial_sense_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(inertial_sense_generate_messages inertial_sense_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/PreIntIMU.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObsVec.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKRel.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GlonassEphemeris.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/INL2States.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPS.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSEphemeris.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/refLLAUpdate.srv" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPSInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/FirmwareUpdate.srv" NAME_WE)
add_dependencies(inertial_sense_generate_messages_lisp _inertial_sense_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inertial_sense_genlisp)
add_dependencies(inertial_sense_genlisp inertial_sense_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inertial_sense_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/PreIntIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)
_generate_msg_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObsVec.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg;/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)
_generate_msg_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPSInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)
_generate_msg_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKRel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)
_generate_msg_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GlonassEphemeris.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)
_generate_msg_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/INL2States.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)
_generate_msg_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)
_generate_msg_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)
_generate_msg_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSEphemeris.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)
_generate_msg_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)
_generate_msg_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)
_generate_msg_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)

### Generating Services
_generate_srv_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/FirmwareUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)
_generate_srv_nodejs(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/refLLAUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
)

### Generating Module File
_generate_module_nodejs(inertial_sense
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(inertial_sense_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(inertial_sense_generate_messages inertial_sense_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/PreIntIMU.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObsVec.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKRel.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GlonassEphemeris.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/INL2States.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPS.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSEphemeris.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/refLLAUpdate.srv" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPSInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/FirmwareUpdate.srv" NAME_WE)
add_dependencies(inertial_sense_generate_messages_nodejs _inertial_sense_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inertial_sense_gennodejs)
add_dependencies(inertial_sense_gennodejs inertial_sense_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inertial_sense_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/PreIntIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)
_generate_msg_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObsVec.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg;/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)
_generate_msg_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPSInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)
_generate_msg_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKRel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)
_generate_msg_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GlonassEphemeris.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)
_generate_msg_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/INL2States.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)
_generate_msg_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)
_generate_msg_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)
_generate_msg_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSEphemeris.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)
_generate_msg_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)
_generate_msg_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg"
  "${MSG_I_FLAGS}"
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)
_generate_msg_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)

### Generating Services
_generate_srv_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/FirmwareUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)
_generate_srv_py(inertial_sense
  "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/refLLAUpdate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
)

### Generating Module File
_generate_module_py(inertial_sense
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(inertial_sense_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(inertial_sense_generate_messages inertial_sense_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/PreIntIMU.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObsVec.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/SatInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKRel.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GlonassEphemeris.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/INL2States.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPS.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GTime.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSEphemeris.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/RTKInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/refLLAUpdate.srv" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GNSSObservation.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/msg/GPSInfo.msg" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyw/catkin_ws/src/isee_camvox/inertial-sense-ros/srv/FirmwareUpdate.srv" NAME_WE)
add_dependencies(inertial_sense_generate_messages_py _inertial_sense_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inertial_sense_genpy)
add_dependencies(inertial_sense_genpy inertial_sense_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inertial_sense_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inertial_sense
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(inertial_sense_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(inertial_sense_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inertial_sense
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(inertial_sense_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(inertial_sense_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inertial_sense
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(inertial_sense_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(inertial_sense_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inertial_sense
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(inertial_sense_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(inertial_sense_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inertial_sense
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(inertial_sense_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(inertial_sense_generate_messages_py geometry_msgs_generate_messages_py)
endif()
