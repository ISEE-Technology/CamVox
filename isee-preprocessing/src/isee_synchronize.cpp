#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <isee_synchronize/CustomMsg.h>
#include <isee_synchronize/GPS.h>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <pcl/common/transforms.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <camera_info_manager/camera_info_manager.h>
#include "camera.h"
#include <chrono>

using namespace std;
using namespace cv;
using namespace MVS;

typedef pcl::PointXYZI PointTypeXYZI;

vector<sensor_msgs::Image> camera_datas;
vector<sensor_msgs::PointCloud2ConstPtr> lidar_datas;
vector<nav_msgs::Odometry> imu_datas;
vector<isee_synchronize::GPS> rtk_datas;

double lidar_delta_time = 0.1;                              //! 10Hz lidar data, you can change this parameter by your sensor

const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
Eigen::Matrix4d rtk2lidar;                                  //!the extrinsic parameter
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor_pcd(new pcl::PointCloud<pcl::PointXYZRGB>());

ros::Publisher pub_cloud, pub_odometry,pub_isee_cloud,pub_isee_image,pub_isee_depth;  //! adverse
ros::Publisher pub_apx;
ros::Publisher pub_apx_p2;
ros::Publisher pub_apx_rpy;

/*********************************************************/
void livox_lidar2depth(uint32_t num_lidar);
bool lidar_imu_rtk_process(uint32_t num_lidar, uint32_t num_imu, uint32_t num_rtk);               //!  lidar_imu_rtk
bool average_quaternion(nav_msgs::Odometry &start, nav_msgs::Odometry &end, Eigen::Quaterniond &result, double t);
bool mercator_proj(double B0, double L0, double B, double L, double &X, double&Y);
void RGBTrans(PointTypeXYZI const * const pi, pcl::PointXYZRGB * const po);

/*********************************************************/
inline double to_time(sensor_msgs::Image camera_data)                               //!    UTC-time
{
    return (camera_data.header.stamp.sec )*1.0 + (camera_data.header.stamp.nsec / 1000000000.0);
}

inline double to_time(sensor_msgs::PointCloud2ConstPtr lidar_data)                               
{
    return (lidar_data->header.stamp.sec )*1.0 + (lidar_data->header.stamp.nsec / 1000000000.0);
}

inline double to_time(nav_msgs::Odometry imu_data)
{
    return (imu_data.header.stamp.sec )*1.0 + (imu_data.header.stamp.nsec / 1000000000.0);
}

inline double to_time(isee_synchronize::GPS rtk_data)   //sensor_msgs::NavSatFix 
{
    return (rtk_data.header.stamp.sec )*1.0 + (rtk_data.header.stamp.nsec / 1000000000.0);
}

/*********************************************************/ 
//isee_synchronize::GPS:: sensor_msgs::NavSatFix::ConstPtr
void rtkCbk(const isee_synchronize::GPS::ConstPtr &msg)
{
    rtk_datas.push_back(*msg);   //! gps_save
}

// transform sensor_msgs::Imu to nav_msgs::Odometry
void apximuCbk(const sensor_msgs::Imu::ConstPtr &msg) 
{
    nav_msgs::Odometry tempOdo;
    tempOdo.header.stamp = msg->header.stamp;
    tempOdo.pose.pose.orientation.x = msg->orientation.x;
    tempOdo.pose.pose.orientation.y = msg->orientation.y;
    tempOdo.pose.pose.orientation.z = msg->orientation.z;
    tempOdo.pose.pose.orientation.w = msg->orientation.w;
    tempOdo.header.frame_id = "/camera_init";

    pub_apx_rpy.publish(tempOdo);
}


void lidarCbk(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if(lidar_datas.size() > 0)
    {
        if(to_time(lidar_datas[lidar_datas.size()-1]) > to_time(msg))
        {
            ROS_INFO("lidar time error");
            return;
        }
    }
    /**********************************************************************/
    lidar_datas.push_back(msg);  //! lidar_save
}

void imuCbk(const nav_msgs::Odometry::ConstPtr &msg)
{
    imu_datas.push_back(*msg);   //! imu_save
}


/*****************************************************************************/
/*****************************************************************************/
int main(int argc, char **argv)
{
    //update the extrinsic parameter
    rtk2lidar <<  1,0,0,0,
                  0,1,0,0,
                  0,0,1,0,
                  0,0,0,1;

    ros::init(argc, argv, "isee_synchronize");
    ros::NodeHandle n;

    cv::Mat src;
    MVS::Camera MVS_cap(n);
    image_transport::ImageTransport cam_image(n);
    image_transport::CameraPublisher image_pub = cam_image.advertiseCamera("/isee_rgb", 1);
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info;
    sensor_msgs::Image image_msg;
    cam_info.reset(new camera_info_manager::CameraInfoManager(n, "main_camera", ""));

    if (!cam_info->isCalibrated())
    {
        cam_info->setCameraName("/dev/video0");
        sensor_msgs::CameraInfo cam_info_;
        cam_info_.header.frame_id = image_msg.header.frame_id;
        cam_info_.width = 1520;
        cam_info_.height = 568;
        cam_info->setCameraInfo(cam_info_);
    }
    
    sensor_msgs::CameraInfoPtr camera_info;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;                     

    ros::Subscriber sub_rtk = n.subscribe("/gps", 1000, rtkCbk);                 //! inertial_gps 10hz
    ros::Subscriber sub_apx_rpy = n.subscribe("/imu", 10000, apximuCbk);         //! inertial_imu 100hz
    pub_apx_rpy = n.advertise<nav_msgs::Odometry> ("pub_apx_rpy", 1000);                      

    ros::Subscriber sub_point = n.subscribe("/livox_undistort", 1000, lidarCbk); //! livox_lidar 10hz
    ros::Subscriber sub_imu = n.subscribe("pub_apx_rpy", 1000, imuCbk);          //! inertial_imu_need

    pub_cloud = n.advertise<sensor_msgs::PointCloud2>("pub_pointcloud2", 1);     
    pub_odometry = n.advertise<nav_msgs::Odometry>("pub_odometry", 1);

    pub_isee_cloud = n.advertise<sensor_msgs::PointCloud2>("/isee_cloud", 1);     
    pub_isee_depth = n.advertise<sensor_msgs::Image>("/isee_depth", 1);  
   // pub_isee_imu = n.advertise<sensor_msgs::Image>("/isee_imu", 1);  

    std::string map_file_path;
    ros::param::get("~map_file_path",map_file_path);
      
    uint32_t num_camera = 0;
    uint32_t num_lidar = 0;
    uint32_t num_imu = 1;
    uint32_t num_rtk = 1;
    
    uint32_t num_camera_last = 0;
    uint32_t num_lidar_last = 0;
    uint32_t num_imu_last = 1;
    uint32_t num_rtk_last = 1;
    
    bool init_flag = false; 
    
    //timestamp align
    while(n.ok())
    {
        ros::spinOnce();
        
        if(num_lidar < lidar_datas.size())
        {
            bool imu_flag = false;
            bool rtk_flag = false;
            
            // imu data align
            if(num_imu < imu_datas.size())
            {
                if(to_time(imu_datas[num_imu-1]) <= to_time(lidar_datas[num_lidar]))
                {
                    if(to_time(imu_datas[num_imu]) >= to_time(lidar_datas[num_lidar]))
                    {
                        imu_flag = true;   
                        rtk_flag = true;
                        
                    }
                    else
                    {
                        num_imu++;
                    }
                }
                else
                {
                    num_lidar++;
                    num_camera++;
                    continue;
                }
            }

            //rtk data align
            // if(num_rtk < rtk_datas.size())
            // {
            //     if(to_time(rtk_datas[num_rtk-1]) <= to_time(lidar_datas[num_lidar]))
            //     {
            //         if(to_time(rtk_datas[num_rtk]) >= to_time(lidar_datas[num_lidar]))
            //         {
            //             rtk_flag = true;
            //         }
            //         else
            //         {
            //             num_rtk++;
            //         }
            //     }
            //     else
            //     {
            //         num_lidar++;
            //         num_camera++;
            //         continue;
            //     }
            // }

            if(imu_flag && rtk_flag)
            {
                if(init_flag)
                {

                    // if(!lidar_imu_rtk_process(num_lidar_last, num_imu_last-1, num_rtk_last))
                    // {
                    //     cout << "error happened" << endl;
                    //     return -1;
                    // }

                    /**********************************************************/
                    MVS_cap >> src;     //! publish rgb image      
                    if (src.empty())
                    {
                        continue;
                    }
                    cv_ptr->image = src;
                    image_msg = *(cv_ptr->toImageMsg());
                    image_msg.header.stamp = lidar_datas[num_lidar_last]->header.stamp;
                    image_msg.header.frame_id =  "camera_init";
                    camera_info = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(cam_info->getCameraInfo()));
                    camera_info->header.frame_id = image_msg.header.frame_id;
                    camera_info->header.stamp = image_msg.header.stamp;      
                    image_pub.publish(image_msg, *camera_info);
                    
                    /**********************************************************/
                    /**********************************************************/
auto t0 = std::chrono::steady_clock::now();

                    livox_lidar2depth(num_lidar_last);    //! publish depth image   

 std::cout <<"point cloud 2 depth 時間"<< std::chrono::duration_cast<std::chrono::milliseconds>
    			(std::chrono::steady_clock::now() - t0).count()
              << '\n';

                    /**********************************************************/
                    //  sensor_msgs::PointCloud2ConstPtr Cloud;
                    //  Cloud = lidar_datas[num_lidar_last];                               
                    //  pub_isee_cloud.publish(Cloud);     
                    //  std::cout<<"lidar stamp: "<< Cloud->header.stamp << std::endl;
                    /**********************************************************/
                    /**********************************************************/
                    std::cout<<setprecision(20);    
                    std::cout<<"camera stamp: "<< image_msg.header.stamp << std::endl; 
                    std::cout<<"imu stamp: "<< to_time(imu_datas[num_imu_last]) << std::endl;
                    //std::cout<<"rtk stamp: "<< to_time(rtk_datas[num_rtk_last]) << std::endl;

                    std::cout<<num_imu_last<< std::endl;
                    // std::cout<<num_rtk_last<< std::endl;
                    std::cout<<num_lidar_last<< std::endl;
                    std::cout<<num_camera_last<< std::endl;
                    /**********************************************************/
                    /**********************************************************/
                    num_lidar_last = num_lidar;
                    num_camera_last = num_camera;
                    num_imu_last = num_imu;
                    //   num_rtk_last = num_rtk;
                    
                }
                else
                {
                    std::cout<< " RTK service start ! " << std::endl;
                    init_flag = true;
                    num_lidar_last = num_lidar;
                    num_camera_last = num_camera;
                    num_imu_last = num_imu;
                    //num_rtk_last = num_rtk;
                }

                //  std::cout<<setprecision(20);                
                //  std::cout<<"lidar_datas : "<< to_time(lidar_datas[num_lidar]) << std::endl;
                //  std::cout<<"camera_datas : "<< to_time(camera_datas[num_camera]) << std::endl;
                //  std::cout<<"imu_datas[num_imu] : "<< to_time(imu_datas[num_imu]) << std::endl;
                //  std::cout<<"imu_datas[num_imu+1] : "<< to_time(imu_datas[num_imu+1]) << std::endl;
                //  std::cout<<"imu_datas[num_rtk] : "<< to_time(rtk_datas[num_rtk]) << std::endl;
                   
                num_lidar++;
                num_camera++;
            }
        }
    }
    return 0;
}

/***************************************************************************/
/**********Livox_lidar & Camera & Imu Synchronization***********************/
void livox_lidar2depth(uint32_t num_lidar)
{

    pcl::PointCloud<pcl::PointXYZI> CloudIn;
    pcl::fromROSMsg(*lidar_datas[num_lidar],CloudIn);

    int   width=1520,height=568,size=2,type=0;
    float fx=1732.78774,fy=1724.88401,cx=798.426021,cy=312.570668;

    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(-0.082984,-0.0221759,0.0727962);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
    float noiseLevel=0.0;           //设置噪声水平
    float minRange = 0.0f;          //成像时考虑该阈值外的点

    pcl::RangeImagePlanar::Ptr rangeImage(new pcl::RangeImagePlanar);
    rangeImage->createFromPointCloudWithFixedSize(CloudIn,width,height,cx,cy,fx,fy,sensorPose,coordinate_frame);

    // 给range_image设置header
    rangeImage->header.seq = CloudIn.header.seq;
    rangeImage->header.frame_id = CloudIn.header.frame_id;
    rangeImage->header.stamp = CloudIn.header.stamp;

    int cols = rangeImage->width;
    int rows = rangeImage->height;   

    // 转换因子
    float factor = 1.0f / (130.0f - 0.5f);
    float offset = -0.5;

    // rangeimage转成图片才能以msg发送出去
    cv::Mat _rangeImage;  
    _rangeImage = cv::Mat::zeros(rows, cols, cv_bridge::getCvType("mono16"));  //最后的OpenCV格式的图像
    // 遍历每一个点 生成OpenCV格式的图像
    for (int i=0; i<cols; ++i)
    {
        for (int j=0; j<rows; ++j)
        {
            float r = rangeImage->getPoint(i, j).range;
            float range = (!std::isinf(r))? std::max(0.0f, std::min(1.0f, factor * (r + offset))) : 0.0;
            _rangeImage.at<ushort>(j, i) = static_cast<ushort>((range) * std::numeric_limits<ushort>::max());
        }
    }
    // 最后发布的消息格式
    sensor_msgs::ImagePtr msg;  
    msg=cv_bridge::CvImage(std_msgs::Header(),"mono16",_rangeImage).toImageMsg();    
    pcl_conversions::fromPCL(rangeImage->header, msg->header);   // header的转变
    msg->header.stamp = lidar_datas[num_lidar]->header.stamp;
    pub_isee_depth.publish(msg);
    std::cout<<"depth stamp: "<< msg->header.stamp << std::endl;
    //std::cout<<"---------------------end----------------------"<<std::endl;
}


