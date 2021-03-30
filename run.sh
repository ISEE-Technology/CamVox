#!/bin/bash

#open ros master
source ~/catkin_ws/devel/setup.bash
{
   gnome-terminal -x bash -c "roscore;exec bash"
}&

# launch livox_ros_driver
sleep 3s
{
   gnome-terminal  -x bash -c "roslaunch livox_ros_driver livox_lidar.launch ;exec bash"
}&

# launch inertial_sense RTK 
sleep 3s
{
   gnome-terminal  -x bash -c "roslaunch inertial_sense test.launch;exec bash"
}&

# launch lidar_camera_RTK synchronization
sleep 5s
{
   gnome-terminal  -x bash -c "roslaunch isee_synchronize synchronize.launch;exec bash"
}&

# run CamVox-SLAM node
sleep 5s
{
   gnome-terminal  -x bash -c "cd /home/zyw/catkin_ws/src/camvox/isee-camvox/;rosrun online camvox Vocabulary/ORBvoc.bin camvox/online/Livox.yaml 400;exec bash"
}



# record /isee_rgb /isee_depth /livox/lidar /imu topic
#sleep 5s
#{
#  gnome-terminal  -x bash -c "rosbag record -O /home/zyw/bag/camvox.bag -b 4096 /isee_rgb /isee_depth /livox/lidar /imu;exec bash"
#}


