#ifndef Calibrating_H
#define Calibrating_H

#include "Tracking.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include <mutex>

#include <math.h>
#include <numeric>
#include <chrono>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>   
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace cv;
using namespace std;

namespace Camvox
{
    class Tracking;
    class LoopClosing;
    class LocalMapping;
  
    class Calibrating
    {
    public:
    /**************************************************************************************************************************************************/
        typedef Matrix<double, 6, 1> Vector6d;
        enum ProjectionType
        {
            INTENSITY,
            DEPTH,
            BOTH
        };

    public:
        // matrix vector
        float best_r_, best_p_, best_y_;
        int optimize_type;
        Rect rect_;
        Matrix3d Rcl;
        Mat origin_RGB_, imdepth_, imRGB_, imGray_, imIntensity_, imdepth_true_, imdepth8_, imIntensity8_, rgb_canny_, intensity_canny_, depth_canny_;
        vector<Vec4i> hierarchy_RGB_, hierarchy_imdepth_, hierarchy_intensity_, hierarchy_depth_intensity_;
        vector<vector<Point>> contours_RGB_, contours_imdepth_, contours_intensity_, contours_depth_intensity_;
        ProjectionType projection_type_;

        

        Calibrating(string _strSettingPath, string _RGB_path, string _Pcd_path, string _Projection_type, bool _isEnhanceImg, bool _isFillImg);                                           //构造
        bool loadPcd(const std::string &pcd_file);                                                                                                                                       //加载PCD
        bool loadParams(const std::string &filename);                                                                                                                                    //加载参数
        void Projection(const ProjectionType projection_type, const Vector6d &extrinsic_params);                                                                                         //点云投影
        Mat convertByIntensity(const Vector6d &calib_params);                                                                                                                            //强度图转换
        Mat convertByDepth(const Vector6d &calib_params, const bool is_fillImg);                                                                                                         //深度图转换
        void EdgeDetction(const ProjectionType projection_type, const int rgb_threshold, const int depth_threshold, const int intensity_threshold);                                      //边缘检测
        void EdgeFilter(const ProjectionType projection_type, const int rgb_edge_threshod, const int depth_edge_threshold, const int intensity_edge_threshold);                          //边缘滤波
        void convertToRgbCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &xyz_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &xyzrgb_cloud, Eigen::Vector3i rgb);                               //转换成RGB点云
        void convertToXYZCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &xyzrgb_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &xyz_cloud);                                                    //转换成XYZ点云
        void BuildRgbCloud();                                                                                                                                                            //生成彩色点云
        void BuildLidarCloud();                                                                                                                                                          //生成雷达点云
        float costFunction(const double *abc);                                                                                                                                           //cost Function
        float RpyCostFunction(const Eigen::Vector3f &rpy);                                                                                                                               //rpy cost Function
        Point3f backprojection(Point2f p, float z);                                                                                                                                      //点云投影
        vector<Point3f> unproject_all(Mat &imdepth_);                                                                                                                                    //深度图投影点云
        void showOrgPointClouds(const string name);                                                                                                                                      //显示原始点云
        void showClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud1, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud2);                                                     //显示点云
        void showNearestPointClouds(float radius);                                                                                                                                       //显示最近邻
        void showCurvaturesFilter(int show_num);                                                                                                                                         //显示曲率滤波
        void testRegister(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);                                                                       //测试匹配效果
        bool initRgbUseClouds();                                                                                                                                                         //初始化RGB使用的点云
        float calcDistance(pcl::PointXYZ p1, pcl::PointXYZ p2);                                                                                                                          //计算距离
        float calcMeanDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2);                                                                //计算平均距离
        void extractEdge(const bool is_filter, const Mat &img, const vector<vector<Point>> &contous, const int min_len, Mat &edge_img, pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_cloud); //提取边缘
        void color2d(const bool use_depth, const string info);                                                                                                                           //2D上色
        void color3d(const Vector6d &extrinsic_params, const int density, const string pcd_path);           

    /**************************************************************************************************************************************************/
    /**************************************************************************************************************************************************/
        void SetLoopCloser(LoopClosing *pLoopCloser);
        void SetTracker(Tracking *pTracker);
        void SetLocalMapper(LocalMapping *pLocalMapper);
        // Main function
        void Run();
        // Thread Synch
        void RequestStop();
        void RequestReset();
        bool Stop();
        void Release();
        bool isStopped();
        bool stopRequested();
        bool SetNotStop(bool flag);
        void RequestFinish();
        bool isFinished();
        bool mbCalibrating;
        bool mbOptimizing;
        void InformCalibrating(const bool &flag);
        void InformOptimizing(const bool &flag);
        

    protected:

        LoopClosing *mpLoopCloser;
        Tracking *mpTracker;
        LocalMapping *mpLocalMapper;

        void ResetIfRequested();
        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        bool mbNotStop;
        std::mutex mMutexStop;
        std::mutex mMutexAccept;

    /**************************************************************************************************************************************************/
    private:
        // Camera parameters
        float fx_, fy_, cx_, cy_, k1_, k2_, p1_, p2_, k3_, bf, mDepthMapFactor_;
        int width_, height_;
        // Canny
        int depth_edge_threshold_, rgb_canny_threshold_, depth_canny_threshold_, intensity_canny_threshold_;
        bool is_enhancement_, is_fillImg_;
        // KD-tree
        Mat K, I;
        Eigen::Vector3d T_;  
    /**************************************************************************************************************************************************/
    
    };
}

#endif //Calibrating_H