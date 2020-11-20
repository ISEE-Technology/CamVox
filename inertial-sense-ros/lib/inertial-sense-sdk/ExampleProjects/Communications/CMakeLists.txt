CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

project(ISCommunicationsExample)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

add_executable(ISCommunicationsExample ISCommunicationsExample.c)
target_link_libraries(ISCommunicationsExample InertialSense)

else()	# Build only this project

add_executable(ISCommunicationsExample
    ISCommunicationsExample.c
    ../../src/data_sets.c
    ../../src/ISComm.c
    ../../src/serialPort.c
    ../../src/serialPortPlatform.c
)
target_link_libraries(ISCommunicationsExample m)

endif()
