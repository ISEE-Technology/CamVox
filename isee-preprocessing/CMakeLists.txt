cmake_minimum_required(VERSION 2.8.3)
project(isee_preprocessing)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -fext-numeric-literals")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  image_transport
  sensor_msgs 
  geometry_msgs
  camera_info_manager
)

find_package( OpenCV 3.2.0 REQUIRED )

## Generate messages in the 'msg' folder
add_message_files(
	FILES
    CustomPoint.msg
    CustomMsg.msg
     GPS.msg
)
add_service_files(
  FILES
  FirmwareUpdate.srv
  refLLAUpdate.srv
  )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    geometry_msgs
)



include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include /opt/MVS/include/)
LINK_DIRECTORIES( /opt/MVS/lib/64/ )

add_executable(isee_livox_repub src/livox_repub.cpp)
target_link_libraries(isee_livox_repub ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(imu_processing src/data_process_node.cpp src/data_process.cpp src/gyr_int.cpp )
target_link_libraries(imu_processing ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(isee_synchronize src/isee_synchronize.cpp src/camera.cc)
add_dependencies(isee_synchronize ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(isee_synchronize ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
                      GCBase_gcc421_v3_0
                      MvCameraControl
                      GenApi_gcc421_v3_0
                      MVGigEVisionSDK
                      log4cpp_gcc421_v3_0
                      MVRender
                      Log_gcc421_v3_0
                      MvUsb3vTL
                      MathParser_gcc421_v3_0
                      NodeMapData_gcc421_v3_0
                      MediaProcess
                      XmlParser_gcc421_v3_0
                      X11
                     )




