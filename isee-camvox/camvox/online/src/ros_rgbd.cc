#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "System.h"
#include "pointcloudmapping.h"
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include "ctime"
#include "time.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "CustomMsg.h"
#include "common.h"

using namespace Eigen;
using namespace cv;
using namespace std;


int num = 0, Activate_flag = 0, Cumulative_flag = 0;
rosbag::Bag bag_record;
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;
livox_ros_driver::CustomMsg livox_cloud;

struct pointData
{
    float x;
    float y;
    float z;
    int i;
};
vector<pointData> vector_data;

#if 0
string int2string(int value)
{
    stringstream ss;
    ss<<value;
    return ss.str();
}
#endif

class ImageGrabber
{
public:
    int count = 0;
    ImageGrabber(Camvox::System *pSLAM) : mpSLAM(pSLAM) {}
    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);
    Camvox::System *mpSLAM;
};

//****************************pcd title***********************************//
void writeTitle(const string filename, unsigned long point_num) 
{
    ofstream outfile(filename.c_str(), ios_base::out);
    if (!outfile) 
    {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else 
    {
        outfile << "# .PCD v.7 - Point Cloud Data file format" << endl;
        outfile << "VERSION .7" << endl;
        outfile << "FIELDS x y z intensity" << endl;
        outfile << "SIZE 4 4 4 4" << endl;
        outfile << "TYPE F F F F" << endl;
        outfile << "COUNT 1 1 1 1" << endl;
        outfile << "WIDTH " << long2str(point_num) << endl;
        outfile << "HEIGHT 1" << endl;
        outfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
        outfile << "POINTS " << long2str(point_num) << endl;
        outfile << "DATA ascii" << endl;
    }
    ROS_INFO("Save file %s", filename.c_str());
}
//*************************pcd write to file**************************************//
void writePointCloud(const string filename, const vector<pointData> singlePCD) 
{
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) 
    {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else 
    {   
        for (unsigned long i = 0; i < singlePCD.size(); ++i) 
        {
            outfile << float2str(singlePCD[i].x) << " " << float2str(singlePCD[i].y) << " " << float2str(singlePCD[i].z) << " " << int2str(singlePCD[i].i) << endl;
        }
    }
}

void LivoxMsgCbk(const livox_ros_driver::CustomMsgConstPtr &livox_msg_in)
{
    if (Activate_flag)
    {
        livox_cloud = *(livox_msg_in);
        for(uint i = 0; i < livox_cloud.point_num; ++i) 
        {
            pointData myPoint;
            myPoint.x = livox_cloud.points[i].x;
            myPoint.y = livox_cloud.points[i].y;
            myPoint.z = livox_cloud.points[i].z;
            myPoint.i = livox_cloud.points[i].reflectivity;
            vector_data.push_back(myPoint);
        }
        ++num;
        cout << "SLAM-ActivateCalibrationMode-count = " << num << endl;
        if (num == 90)
        {
           writeTitle("./camvox/calibration/calibration.pcd", vector_data.size());
           writePointCloud("./camvox/calibration/calibration.pcd", vector_data);
           vector_data.clear();
           num = 0;
           Cumulative_flag = 1;
        }
    }
    if (!(Activate_flag))
    {
       num = 0 ;
       livox_data.clear();
       vector_data.clear();
       Cumulative_flag = 0;
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "camvox");

#if 0
    ros::Time::init();
#endif
    ros::start();
    if (argc != 4)
    {
        cerr << endl
             << "Usage: rosrun online camvox path_to_vocabulary path_to_settings frame_num" << endl;
        ros::shutdown();
        return 1;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    Camvox::System SLAM(argv[1], argv[2], Camvox::System::RGBD, true);
    ImageGrabber igb(&SLAM);
    ros::NodeHandle nh;

    ros::Subscriber sub_livox_msg = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 1, LivoxMsgCbk);
#if 0
    ROS_INFO("start message filter");
    cerr << endl << "start message filter" << endl;
    time_t t=std::time(0);
    struct tm * now = std::localtime( & t );
    string file_name;
    //the name of bag file is better to be determined by the system time
    file_name="/home/zyw/bag/"+int2string(now->tm_year + 1900)+
          '-'+int2string(now->tm_mon + 1)+
          '-'+int2string(now->tm_mday)+
          '-'+int2string(now->tm_hour)+
          '-'+int2string(now->tm_min)+
          '-'+int2string(now->tm_sec)+
           ".bag";
    bag_record.open(file_name, rosbag::bagmode::Write);
#endif
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/isee_rgb", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/isee_depth", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    //ros::spin();
    while (ros::ok() and igb.count <= atoi(argv[3]))
    {
        ros::spinOnce();
        if (SLAM.CalibrationFlag == 1)
        {
           Activate_flag = 1;
        }
        
        if (SLAM.CalibrationFlag == 2)
        {
           Activate_flag = 0;
        }

        if (Cumulative_flag == 0)
        {
            SLAM.mpCalibratingter->cumulative_flag = false;
        }

        if (Cumulative_flag == 1)
        {
            SLAM.mpCalibratingter->cumulative_flag = true;
            //cout<<"revise content"<<SLAM.mpCalibratingter->cumulative_flag<<endl;
        }
        /*...TODO...*/
    }

    std::cout << "After spin: \n";

    while (SLAM.mpPointCloudMapping->loopbusy || SLAM.mpPointCloudMapping->cloudbusy) //* 点云建图忙时，什么都不做
    {
        cout << "";
    }
    //Tracking time statistics
    SLAM.mpPointCloudMapping->bStop = true;

#if 0
    bag_record.close();
#endif
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("OnlineKeyFrameTrajectory.txt");
    SLAM.save();
    // Stop all threads
    SLAM.Shutdown();
    ros::shutdown();
    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{

#if 0
    bag_record.write("/isee_rgb", msgRGB->header.stamp.now(), msgRGB);
    bag_record.write("/isee_depth", msgD->header.stamp.now(), msgD);
#endif
    // Copy the ros image message to cv::Mat

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    ++count;
}
