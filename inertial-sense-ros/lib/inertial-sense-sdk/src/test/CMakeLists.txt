cmake_minimum_required(VERSION 2.8.3)
project(SDK_test)

add_library(SDK_test
	test_com_manager.cpp
	test_ring_buffer.cpp
	../com_manager.c
	../data_sets.c
	../ISComm.c
	../ISEarth.c
	../ISMatrix.c
	../ISPose.c
	../protocol_nmea.cpp
	../linked_list.c
	../ring_buffer.c
	)

find_package(GTest)
if (${GTEST_FOUND})
    add_executable(run_tests 
	test_com_manager.cpp
  	test_ring_buffer.cpp
	../com_manager.c
	../data_sets.c
	../ISComm.c
	../ISEarth.c
	../ISMatrix.c
	../ISPose.c
	../protocol_nmea.cpp
	../linked_list.c
	../ring_buffer.c
	)
    target_link_libraries(run_tests gtest gtest_main pthread ${GTEST_LIBRARIES})
    target_include_directories(run_tests PUBLIC ${CMAKE_CURRENT_LIST_DIR})
    add_test(NAME run_tests
             COMMAND run_tests --gtest_color=true)
endif()

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions")
