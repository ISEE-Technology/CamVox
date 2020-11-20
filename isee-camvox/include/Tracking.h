/* @file  Namespace::class -> Camvox::Tracking
 * @brief - 
 * @funct   Tracking()构造
 * @funct   SetLocalMapper()/SetLoopClosing()/SetViewer() 三个设置函数
 * @funct   GrabImageRGBD() 图像提取函数
 * @funct   ChangeCalibratingtion() 加载新设置
 * @funct   InformOnlyTracking() 仅仅定位相机
 * @funct   Track() 跟踪主接口   初始化RGBD->相机位置跟踪->局部地图跟踪->是否生成关键帧->生成关键帧->计算记录位姿信息
 * @funct   Reset() 重置函数
 * @value - 
 * @state - mState mLastProcessedState
 * @flag  - mbOnlyTracking mbRGB
 * thread - Tracking
 */ 

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <mutex>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"

// for pointcloud mapping and viewing   
#include "pointcloudmapping.h"    

class PointCloudMapping;

namespace Camvox
{

    class Viewer;             
    class FrameDrawer;               
    class Map;
    class LocalMapping;
    class LoopClosing;
    class System;

    class Tracking
    {

    public:
        //Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
        //KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
        Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, shared_ptr<PointCloudMapping> pPointCloud,
                 KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor);

        // Preprocess the input and call Track(). Extract features and performs stereo matching.
        cv::Mat GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp);

        void SetLocalMapper(LocalMapping *pLocalMapper);
        void SetLoopClosing(LoopClosing *pLoopClosing);
        void SetViewer(Viewer *pViewer);
        void SetCalibratingter(Calibrating *pCalibratingter);

        // Load new settings
        // The focal lenght should be similar or scale prediction will fail when projecting points
        // TODO: Modify MapPoint::PredictScale to take into account focal lenght
        void ChangeCalibratingtion(const string &strSettingPath);

        // Use this function if you have deactivated local mapping and you only want to localize the camera.
        void InformOnlyTracking(const bool &flag);

    public:
        // Tracking states
        enum eTrackingState
        {
            SYSTEM_NOT_READY = -1,
            NO_IMAGES_YET = 0,
            NOT_INITIALIZED = 1,
            OK = 2,
            LOST = 3
        };

        eTrackingState mState;
        eTrackingState mLastProcessedState;

        // Input sensor
        int mSensor;

        // Current Frame
        Frame mCurrentFrame;
        cv::Mat mImGray;
        cv::Mat mImRGB;
        cv::Mat mImDepth; // adding mImDepth member to realize pointcloud view

        // Initialization Variables (Monocular)
        std::vector<int> mvIniLastMatches;
        std::vector<int> mvIniMatches;
        std::vector<cv::Point2f> mvbPrevMatched;
        std::vector<cv::Point3f> mvIniP3D;
        Frame mInitialFrame;

        // Lists used to recover the full camera trajectory at the end of the execution.
        // Basically we store the reference keyframe for each frame and its relative transformation
        list<cv::Mat> mlRelativeFramePoses;
        list<KeyFrame *> mlpReferences;
        list<double> mlFrameTimes;
        list<bool> mlbLost;

        // True if local mapping is deactivated and we are performing only localization
        bool mbOnlyTracking;

        void Reset();

    protected:
        // Main tracking function. It is independent of the input sensor.
        void Track();

        // Map initialization for stereo and RGB-D
        void StereoInitialization();

        void CheckReplacedInLastFrame();
        bool TrackReferenceKeyFrame();
        void UpdateLastFrame();
        bool TrackWithMotionModel();

        bool Relocalization();

        void UpdateLocalMap();
        void UpdateLocalPoints();
        void UpdateLocalKeyFrames();

        bool TrackLocalMap();
        void SearchLocalPoints();

        bool NeedNewKeyFrame();
        void CreateNewKeyFrame();

        // In case of performing only localization, this flag is true when there are no matches to
        // points in the map. Still tracking will continue if there are enough matches with temporal points.
        // In that case we are doing visual odometry. The system will try to do relocalization to recover
        // "zero-drift" localization to the map.
        bool mbVO;

        //Other Thread Pointers
        LocalMapping *mpLocalMapper;
        LoopClosing *mpLoopClosing;
        Calibrating *mpCalibratingter;

        //ORB
        ORBextractor *mpORBextractorLeft, *mpORBextractorRight;             //! mpORBextractorRight need to be delete
        ORBextractor *mpIniORBextractor;

        //BoW
        ORBVocabulary *mpORBVocabulary;
        KeyFrameDatabase *mpKeyFrameDB;

        //Local Map
        KeyFrame *mpReferenceKF;
        std::vector<KeyFrame *> mvpLocalKeyFrames;
        std::vector<MapPoint *> mvpLocalMapPoints;

        // System
        System *mpSystem;

        //Drawers
        Viewer *mpViewer;
        FrameDrawer *mpFrameDrawer;
        MapDrawer *mpMapDrawer;

        //Map
        Map *mpMap;

        //Calibratingtion matrix
        cv::Mat mK;
        cv::Mat mDistCoef;
        float mbf;

        //New KeyFrame rules (according to fps)
        int mMinFrames;
        int mMaxFrames;

        // Threshold close/far points
        // Points seen as close by the stereo/RGBD sensor are considered reliable
        // and inserted from just one frame.                                                              //!Far points requiere a match in two keyframes.
        float mThDepth;

        // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
        float mDepthMapFactor;

        //Current matches in frame
        int mnMatchesInliers;

        //Last Frame, KeyFrame and Relocalisation Info
        KeyFrame *mpLastKeyFrame;
        Frame mLastFrame;
        unsigned int mnLastKeyFrameId;
        unsigned int mnLastRelocFrameId;

        //Motion Model
        cv::Mat mVelocity;

        //Color order (true RGB, false BGR, ignored if grayscale)                                         //! if grayscale ?
        bool mbRGB;

        list<MapPoint *> mlpTemporalPoints;

        // for point cloud viewing
        shared_ptr<PointCloudMapping> mpPointCloudMapping;                                                //*  PointcloudMapping 对象声明
    };

} // namespace Camvox

#endif 
