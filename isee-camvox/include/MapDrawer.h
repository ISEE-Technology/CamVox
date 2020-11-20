/* @file  Namespace::Class -> Camvox::MapDrawer
 * @brief - This class dont need change !
 * @funct   MapDrawer() 
 * @funct   DrawMapPoints() 
 * @funct   DrawKeyFrames() 
 * @funct   DrawCurrentCamera() 
 * @funct   SetCurrentCameraPose() 
 * @funct   SetReferenceKeyFrame() 
 * @funct   GetCurrentOpenGLCameraMatrix() 
 * @value - mCameraPose mpMap
 * @state - 
 * @flag  - 
 * thread - Viewer
 */ 

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace Camvox
{

    class MapDrawer
    {
    public:
        MapDrawer(Map *pMap, const string &strSettingPath);

        Map *mpMap;

        void DrawMapPoints();
        void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
        void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
        void SetCurrentCameraPose(const cv::Mat &Tcw);
        void SetReferenceKeyFrame(KeyFrame *pKF);
        void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    private:
        float mKeyFrameSize;
        float mKeyFrameLineWidth;
        float mGraphLineWidth;
        float mPointSize;
        float mCameraSize;
        float mCameraLineWidth;

        cv::Mat mCameraPose;

        std::mutex mMutexCamera;
    };

} // namespace Camvox

#endif // MAPDRAWER_H
