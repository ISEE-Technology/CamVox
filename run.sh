#!/bin/bash

#open ros master
source ~/catkin_ws/devel/setup.bash
{
   gnome-terminal -x bash -c "roscore;exec bash"
}&

# launch livox_ros_driver
sleep 3s
{
   gnome-terminal  -x bash -c "roslaunch livox_ros_driver livox_lidar.launch bd_list:="1HDDH1200102591";exec bash"
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

# run camvox slam node
sleep 5s
{
   gnome-terminal  -x bash -c "cd /home/zyw/catkin_ws/src/camvox/isee-camvox/;rosrun Camvox RGBD Vocabulary/ORBvoc.bin Examples/ROS/Camvox/Livox.yaml;exec bash"
}


# record /isee_rgb /isee_depth topic
#sleep 5s
#{
#	gnome-terminal  -x bash -c "rosbag record -O /home/zyw/bag/camvox.bag -b 4096 /isee_rgb /isee_depth;exec bash"
#}


