CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

project(InertialSenseSDK)

#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -Wall -Wextra -Wno-unused-function -Wno-unused-value -Wno-unused-parameter -Wno-missing-field-initializers")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Wextra -Wno-unused-function -Wno-unused-value -Wno-unused-parameter -Wno-missing-field-initializers")


SET(BUILD_INERTIAL_SENSE_SDK 1)

add_definitions(-DRTK_EMBEDDED)
find_package(Threads REQUIRED)

add_library(InertialSense
  src/cltool.cpp
  src/cltool_main.cpp
  src/convert_ins.cpp
  src/com_manager.c
  src/data_sets.c
  src/DataChunk.cpp
  src/DataChunkSorted.cpp
  src/DataCSV.cpp
  src/DataJSON.cpp
  src/DataKML.cpp
  src/DeviceLog.cpp
  src/DeviceLogCSV.cpp
  src/DeviceLogJSON.cpp
  src/DeviceLogKML.cpp
  src/DeviceLogSerial.cpp
  src/DeviceLogSorted.cpp
  src/InertialSense.cpp
  src/inertialSenseBootLoader.c
  src/ISComm.c
  src/ISDataMappings.cpp
  src/ISDisplay.cpp
  src/ISEarth.c
  src/ISFileManager.cpp
  src/ISLogFile.cpp
  src/ISLogger.cpp
  src/ISMatrix.c
  src/ISPose.c
  src/ISSerialPort.cpp
  src/ISStream.cpp
  src/ISTcpClient.cpp
  src/ISTcpServer.cpp
  src/ISUtilities.cpp
  src/linked_list.c
  src/ring_buffer.c
  src/serialPort.c
  src/serialPortPlatform.c
  src/time_conversion.c
  src/tinystr.cpp
  src/tinyxml.cpp
  src/tinyxmlerror.cpp
  src/tinyxmlparser.cpp
  )
target_include_directories(InertialSense PUBLIC src external)
target_link_libraries(InertialSense pthread)
target_compile_options(InertialSense PUBLIC -fPIC)

add_subdirectory(CLTool)
add_subdirectory(ExampleProjects)

