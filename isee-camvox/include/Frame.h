/* @file  Namespace::Class -> Camvox::Frame
 * @brief - 
 * @funct   Frame() 
 * @funct   ExtractORB() 
 * @funct   ComputeBoW() 
 * @funct   SetPose()/UpdatePoseMatrices(/GetCameraCenter()/GetRotationInverse() 
 * @funct   isInFrustum()/PosInGrid()/GetFeaturesInArea() 
 * @funct   UnprojectStereo() 
 * @value - mbf mThDepth mvpMapPoints mfGridElementWidthInv mfGridElementHeightInv
 * @state - 
 * @flag  - mGrid
 * thread - Tracking
 */ 

#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <opencv2/opencv.hpp>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

namespace Camvox
{
    #define FRAME_GRID_ROWS 48
    #define FRAME_GRID_COLS 64

    class MapPoint;
    class KeyFrame;

    class Frame
    {
    public:
        Frame();

        // Copy constructor.
        Frame(const Frame &frame);
      
        // Constructor for RGB-D cameras.
        Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

        // Extract ORB on the image. 0 for left image and 1 for right image.
        void ExtractORB(int flag, const cv::Mat &im);

        // Compute Bag of Words representation.
        void ComputeBoW();

        // Set the camera pose.
        void SetPose(cv::Mat Tcw);

        // Computes rotation, translation and camera center matrices from the camera pose.
        void UpdatePoseMatrices();

        // Returns the camera center.
        inline cv::Mat GetCameraCenter()
        {
            return mOw.clone();
        }

        // Returns inverse of rotation
        inline cv::Mat GetRotationInverse()
        {
            return mRwc.clone();
        }

        // Check if a MapPoint is in the frustum of the camera
        // and fill variables of the MapPoint to be used by the tracking
        bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

        // Compute the cell of a keypoint (return false if outside the grid)
        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

        vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel = -1, const int maxLevel = -1) const;

        // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.                    
        void ComputeStereoFromRGBD(const cv::Mat &imDepth);                                                             

        // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
        cv::Mat UnprojectStereo(const int &i);

    public:
        // Vocabulary used for relocalization.
        ORBVocabulary *mpORBvocabulary;

        // Feature extractor. The right is used only in the stereo case.
        ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

        // Frame timestamp.
        double mTimeStamp;

        // Calibratingtion matrix and OpenCV distortion parameters.
        cv::Mat mK;
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float invfx; // 1/fx
        static float invfy; // 1/fy
        cv::Mat mDistCoef; 

        // Stereo baseline multiplied by fx.
        float mbf; //b*f

        // Stereo baseline in meters.
        float mb; //b

        // Threshold close/far points. Close points are inserted from 1 view. 
        // Far points are inserted as in the monocular case from 2 views.
        float mThDepth;                                                                 //! 这里近点从相机的一个位姿视点插入，远点从类似于单目一样从相机的两个位姿视点插入

        // Number of KeyPoints.
        int N;

        // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
        // In the stereo case, mvKeysUn is redundant as images must be rectified.
        // In the RGB-D case, RGB images can be distorted.
        std::vector<cv::KeyPoint> mvKeys, mvKeysRight;                                  //! this code need to be changed
        std::vector<cv::KeyPoint> mvKeysUn;

        // Corresponding stereo coordinate and depth for each keypoint.
        // "Monocular" keypoints have a negative value.
        std::vector<float> mvuRight;                                                    //! this code may be need to be changed
        std::vector<float> mvDepth;

        // Bag of Words Vector structures.
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        // ORB descriptor, each row associated to a keypoint.
        cv::Mat mDescriptors, mDescriptorsRight;                                        //! this code may be need to be changed

        // MapPoints associated to keypoints, NULL pointer if no association.
        std::vector<MapPoint *> mvpMapPoints;             

        // Flag to identify outlier associations.
        std::vector<bool> mvbOutlier;

        // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;

        // FRAME_GRID_ROWS 48
        // FRAME_GRID_COLS 64
        std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        // Camera pose.
        cv::Mat mTcw; 

        // Current and Next Frame id.
        static long unsigned int nNextId;
        long unsigned int mnId; ///< Current Frame id.

        // Reference Keyframe.
        KeyFrame *mpReferenceKF; 

        // Scale pyramid info.
        int mnScaleLevels;  
        float mfScaleFactor; 
        float mfLogScaleFactor;
        vector<float> mvScaleFactors;
        vector<float> mvInvScaleFactors;
        vector<float> mvLevelSigma2;
        vector<float> mvInvLevelSigma2;

        // Undistorted Image Bounds (computed once).
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        static bool mbInitialComputations;

    private:
        // Undistort keypoints given OpenCV distortion parameters.
        // Only for the RGB-D case. Stereo must be already rectified!
        // (called in the constructor).
        void UndistortKeyPoints();

        // Computes image bounds for the undistorted image (called in the constructor).
        void ComputeImageBounds(const cv::Mat &imLeft);

        // Assign keypoints to the grid for speed up feature matching (called in the constructor).
        void AssignFeaturesToGrid();

        // Rotation, translation and camera center
        cv::Mat mRcw;
        cv::Mat mtcw;
        cv::Mat mRwc;
        cv::Mat mOw; //==mtwc
    };

} // namespace Camvox

#endif 
