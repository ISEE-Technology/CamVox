# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;sensor_msgs;geometry_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-linertial_sense_ros".split(';') if "-linertial_sense_ros" != "" else []
PROJECT_NAME = "inertial_sense"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "1.1.1"
