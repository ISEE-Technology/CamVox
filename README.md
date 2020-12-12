# CamVox
## A Low-cost and Accurate Lidar-assisted Visual SLAM System

<img src="./pics/CamVox.gif" alt="show" />

We propose **CamVox** by adapting Livox lidars into visual SLAM (ORB-SLAM2) by exploring the lidars’ unique features. Based on the non-repeating nature of Livox lidars, we propose an automatic lidar-camera calibration method that will work in uncontrolled scenes. The long depth detection range also beneﬁt a more efﬁcient mapping. Comparison of CamVox with visual SLAM (VINS-mono) and lidar SLAM (livox_horizon_loam) are evaluated on the same dataset to demonstrate the performance.

<div align="center">
    <img src="pics/compare.png" width = 100% >
</div>

**Developer:** [Yuewen Zhu](https://github.com/zywok), [Chunran Zheng](https://github.com/xuankuzcr), [Chongjian Yuan](https://github.com/ChongjianYUAN)

**Our related video**: our related videos are now available on [[YouTube Video](https://www.youtube.com/watch?v=AUnZNBB-uUE)] [[bilibili Video](https://www.bilibili.com/video/BV1fZ4y1V795/)]. 

**Paper**: our related paper has been posted on [arXiv](https://arxiv.org/abs/2011.11357).

## 1. Prerequisites
### 1.1 Ubuntu and ROS
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. Follow [ROS Installation](http://wiki.ros.org/ROS/Installation).

### 1.2 Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Follow [Pangolin Installation](https://github.com/stevenlovegrove/Pangolin).

### 1.3 OpenCV
We use OpenCV to manipulate images and features. Follow [Opencv Installation](http://opencv.org). **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

### 1.4 Eigen3
Follow [Eigen Installation](http://eigen.tuxfamily.org). **Required at least 3.1.0**.

### 1.5 Ceres Solver
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.6 MVS camera driver
Install the [HIKROBOT](https://en.hikrobotics.com/) camera driver as follows.
```bash
    tar zxvf MVS-2.0.0_x86_64_20191126.tar.gz
    cd ./MVS-2.0.0_x86_64_20191126
    chmod +x setup.sh
    sudo ./setup.sh
```
## 2. Build CamVox
Clone the repository and catkin_make:


### 2.1 supplement
like ORB_SLAM, Add the path including /CamVox/isee-camvox/camvox to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:
```bash
   gedit ~/.bashrc
```
and add at the end the following line. Replace PATH by the folder where you cloned camvox:
```bash
   export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/CamVox/isee-camvox/camvox
```
make sure the path  is added correctly: 
```bash
   source ~/.bashrc
   echo $ROS_PACKAGE_PATH
```

build:
```bash
    cd ~/catkin_ws/src
    git clone https://github.com/ISEE-Technology/CamVox
    cd CamVox/isee-camvox && chmod +x build.sh && chmod +x build_ros.sh	
    ./build_ros.sh
    ./build.sh
    source ~/catkin_ws/devel/setup.bash
```
## 3. Run with Hardware
### 3.1 Hardware

 <table>
	<tr>
	    <th>Platform</th>
	    <th>Item</th>
	    <th>Pics</th>  
	    <th>Shopping Link</th> 
	</tr >
	<tr >
	    <td rowspan="5"><img src="./pics/platform.png" /></td>
	    <td>Livox Horizon </td>
	    <td align="center" valign="middle"><img src=  "./pics/horizon.jpg" width=25% /></td>
            <td align="center" valign="middle">  <a href ="https://www.livoxtech.com/horizon"> Lidar </a> </td>
	</tr>
	<tr>
	    <td> MV-CE060-10UC</td>
	    <td align="center" valign="middle"><img src="./pics/camera.png" width=19% /></td>
	    <td align="center" valign="middle">  <a href ="https://en.hikrobotics.com/vision/visioninfo.htm?type=42&oid=2451"> Camera </a> </td>
	</tr>
	<tr>
	    <td> Inertial Sense uINS </td>
	    <td align="center" valign="middle"> <img src="./pics/Inertial_Sense_uINS.png" width=22% /> </td>
	    <td align="center" valign="middle"> <a href ="https://inertialsense.com/product/rugged-µins/"> RTK </a> </td>
	</tr>
	<tr>
	    <td>Manifold2C</td>
	    <td align="center" valign="middle"><img src="./pics/Manifold2C.jpg" width=22% /></td>
            <td align="center" valign="middle">  <a href ="https://www.dji.com/cn/manifold-2"> Onboard-Computer </a> </td>
	</tr>
	<tr>
	    <td> Scout-mini </td>
	    <td align="center" valign="middle"><img src="./pics/Scout-mini.jpg" width=28% /></td>
	    <td align="center" valign="middle">  <a href ="http://www.agilex.ai/index/product/id/3?lang=zh-cn"> Robot Chassis </a> </td>
	</tr>
</table>

### 3.2 Hard Synchronization

Hard synchronization is performed with all of these sensors by a trigger signal of 10 Hz. The camera output at each trigger signal(10 Hz). The lidar keeps a clock (synced with GPS-RTK) and continuously outputs the scanned point with an accurate timestamp. In the meantime, the IMU outputs at a frequency of 200 Hz synced with the trigger. The Hardware Synchronization diagram is as follows.

<img src="./pics/synchronization.jpg" width=100% />

### 3.3 Running

Connect to your PC to Livox Horizon lidar by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver).

```bash
    chmod +x run.sh
    ./run.sh
```


## 4. Run with Rosbag Example

### 4.1 SUSTech Dataset (Loop Closure)

We open sourced our dataset in SUSTech campus with loop closure. [Download here](https://1drv.ms/u/s!ArbY73Gm3ncciawc5zwMWb6tuvEkpA). </br>
Other framework data formats  for comparison. [VINS-mono](https://1drv.ms/u/s!ArbY73Gm3ncciawbxaSpc0ZYXogGVQ) | [livox_loam_horizon](https://1drv.ms/u/s!ArbY73Gm3ncciawYk8nceYiPfDXsVg).

### 4.2 Rosbag Example with static scenes (Automatic Calibration trigger)

We provide a rosbag file with static scenes to test the automatic calibration thread. [Download here](https://1drv.ms/u/s!ArbY73Gm3nccia03bTsyrdgtss9deQ?e=kJ0ok3). <br/>
When the car detects more than **20 frames of still images (about 2 seconds)**, the automatic calibration thread starts to work. The thread will be interrupted to enter the SLAM mode if the car starts to move before the end of calibration. 
The effects of automatic calibration is shown as follows.

<img src="./pics/runyang.gif" alt="show" width = 49% /> <img src="./pics/flat.gif" alt="show" width = 49%  />

### 4.3 Running
```bash
    cd CamVox/isee-camvox
    rosrun online camvox Vocabulary/ORBvoc.bin camvox/offline/Livox.yaml 1000
    rosbag play YOUR_DOWNLOADED.bag
```

## 5. Acknowledgements
The authors are grateful for the pioneering work from [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2), [ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras](https://ieeexplore.ieee.org/document/7946260). The authors also sincerely thank colleagues at Livox Technology for help in data aquisition and discussion.<br/>
This work is from [ISEE Research Group](https://isee.technology/) at SUSTech.

## 6. License

The source code is released under [GPLv2.0](http://www.gnu.org/licenses/) license. 

If you use CamVox in an academic work, please cite:

      @misc{zhu2020camvox,
      title={CamVox: A Low-cost and Accurate Lidar-assisted Visual SLAM System}, 
      author={Yuewen Zhu and Chunran Zheng and Chongjian Yuan and Xu Huang and Xiaoping Hong},
      year={2020},
      eprint={2011.11357},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
      }
