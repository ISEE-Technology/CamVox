#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include "PointCloude.h"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace Camvox
{
    class PointCloude;
    class PointCloudMapping //! this need to be changed to insert into namespace ORB_SLAM2
    {
    public:
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;

        PointCloudMapping(double resolution_, double meank_, double thresh_);
        void save();
        // 插入一个keyframe，会更新一次地图
        void insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth, long int idk, vector<KeyFrame *> vpKFs);
        void shutdown();
        void viewer();
        int loopcount = 0;
        vector<KeyFrame *> currentvpKFs;
        bool cloudbusy;
        bool loopbusy;
        void updatecloud();
        bool bStop = false;

    protected:
        PointCloud::Ptr generatePointCloud(KeyFrame *kf, cv::Mat &color, cv::Mat &depth);

        PointCloud::Ptr globalMap;
        shared_ptr<thread> viewerThread;

        bool shutDownFlag = false;
        mutex shutDownMutex;

        condition_variable keyFrameUpdated;
        mutex keyFrameUpdateMutex;
        vector<PointCloude> pointcloud;
        // data to generate point clouds
        vector<KeyFrame *> keyframes;
        vector<cv::Mat> colorImgs;
        vector<cv::Mat> depthImgs;
        vector<cv::Mat> colorImgks;
        vector<cv::Mat> depthImgks;
        vector<int> ids;
        mutex keyframeMutex;
        uint16_t lastKeyframeSize = 0;

        double resolution = 0.04;
        double meank = 50;
        double thresh = 1;
        pcl::VoxelGrid<PointT> voxel;
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
    };
} // namespace Camvox
#endif
