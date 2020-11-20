/* @file  Namespace::Class -> Camvox::LocalMapping
 * @brief - 
 * @funct   LocalMapping()
 * @funct   SetLoopCloser()/SetTracker() 
 * @funct   Run() 
 * @funct   InsertKeyFrame()/KeyframesInQueue() 
 * @funct   Thread Synch 
 * @funct   InterruptBA() 
 * @funct   RequestFinish()/isFinished() 
 * @value - 
 * @state - 
 * @flag  - mbAbortBA mbStopped mbStopRequested mbNotStop mbAcceptKeyFrames
 * thread - LocalMapping
 */ 

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "Calibrating.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>

namespace Camvox
{
    class Calibrating;
    class Tracking;
    class LoopClosing;
    class Map;

    class LocalMapping
    {
    public:
        LocalMapping(Map *pMap, const float bMonocular);                                  //! This code may be need change 

        void SetLoopCloser(LoopClosing *pLoopCloser);

        void SetTracker(Tracking *pTracker);
        void SetCalibratingter(Calibrating *pCalibratingter);

        // Main function
        void Run();

        void InsertKeyFrame(KeyFrame *pKF);

        // Thread Synch
        void RequestStop();
        void RequestReset();
        bool Stop();
        void Release();
        bool isStopped();
        bool stopRequested();
        bool AcceptKeyFrames();
        void SetAcceptKeyFrames(bool flag);
        bool SetNotStop(bool flag);

        void InterruptBA();     

        void RequestFinish(); 
        bool isFinished();

        int KeyframesInQueue()
        {
            unique_lock<std::mutex> lock(mMutexNewKFs);
            return mlNewKeyFrames.size();
        }

    protected:
        bool CheckNewKeyFrames();   
        void ProcessNewKeyFrame();
        void CreateNewMapPoints();

        void MapPointCulling();
        void SearchInNeighbors();

        void KeyFrameCulling();

        cv::Mat ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);

        cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

        bool mbMonocular;                                                                  //! This code may be need to change

        void ResetIfRequested();
        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        Map *mpMap;

        LoopClosing *mpLoopCloser;
        Tracking *mpTracker;
        Calibrating *mpCalibratingter;

        std::list<KeyFrame *> mlNewKeyFrames; 

        KeyFrame *mpCurrentKeyFrame;

        std::list<MapPoint *> mlpRecentAddedMapPoints;

        std::mutex mMutexNewKFs;

        bool mbAbortBA;

        bool mbStopped;
        bool mbStopRequested;
        bool mbNotStop;
        std::mutex mMutexStop;

        bool mbAcceptKeyFrames;
        std::mutex mMutexAccept;
    };

} // namespace Camvox

#endif // LOCALMAPPING_H
