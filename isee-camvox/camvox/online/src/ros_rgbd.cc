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

using namespace Eigen;
using namespace cv;
using namespace std;

rosbag::Bag bag_record;

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


