#include "System.h"
#include "Converter.h"

#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <time.h>

string strSettingPath = "/home/zyw/catkin_ws/src/camvox/isee-camvox/camvox/config/camera.yaml";
string RGBPath = "/home/zyw/catkin_ws/src/camvox/isee-camvox/camvox/calibration/calibration.bmp";
string PcdPath = "/home/zyw/catkin_ws/src/camvox/isee-camvox/camvox/calibration/calibration.pcd"; 
string projectionType = "both";
bool isEnhanceImg = false;
bool isFillImg = true;

bool has_suffix(const std::string &str, const std::string &suffix)
{
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace Camvox
{
    System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,const bool bUseViewer) : mSensor(sensor), mpViewer(static_cast<Viewer *>(NULL)), mbReset(false), mbActivateLocalizationMode(false),mbDeactivateLocalizationMode(false),mbActivateCalibrationMode(false),mbDeactivateCalibrationMode(false),mbActivateCalibrationOptimizingMode(false),mbDeactivateCalibrationOptimizingMode(false),CalibrationFlag(0)
    {
        // Output welcome message
        cout << endl
             << "Camvox Copyright (C) 2020 ISEE, University of SUSTech." << endl
             << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
             << "This is free software, and you are welcome to redistribute it" << endl
             << "under certain conditions. See LICENSE.txt." << endl
             << endl;

        cout << "Input sensor was set to: ";

        if (mSensor == RGBD)
            cout << "RGB-D" << endl;

        //Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

        // for point cloud resolution                                                                                        //* pointcloudMappint -> setting yaml files
        float resolution = fsSettings["PointCloudMapping.Resolution"];
        float meank = fsSettings["meank"];
        float thresh = fsSettings["thresh"];
        //Load ORB Vocabulary
        cout << endl
             << "Loading ORB Vocabulary. This could take a while..." << endl;

        clock_t tStart = clock();
        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = false; // chose loading method based on file extension
        if (has_suffix(strVocFile, ".txt"))
            bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        else
            bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
        if (!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Failed to open at: " << strVocFile << endl;
            exit(-1);
        }
        printf("Vocabulary loaded in %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);                                 //!  time caculation CLOCKS_PER_SEC

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        //Create the Map
        mpMap = new Map();  

        //Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(mpMap);
        mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

        // Initialize pointcloud mapping
        mpPointCloudMapping = make_shared<PointCloudMapping>(resolution, meank, thresh);                                    //*动态内存中分配一个对象并初始化它 pointcouldMapping r m t setting

        //Initialize the Calibrating thread and launch
        mpCalibratingter = new Calibrating(strSettingPath,RGBPath,PcdPath,projectionType,isEnhanceImg,isFillImg);
        mptCalibrating = new thread(&Camvox::Calibrating::Run, mpCalibratingter);

        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer, mpMap, mpPointCloudMapping, mpKeyFrameDatabase, strSettingsFile, mSensor);   //* tracking thread include PointcloudMapping
        
        //Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR);
        mptLocalMapping = new thread(&Camvox::LocalMapping::Run, mpLocalMapper);

        //Initialize the Loop Closing thread and launch
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR, mpPointCloudMapping);
        mptLoopClosing = new thread(&Camvox::LoopClosing::Run, mpLoopCloser);

        //Initialize the Viewer thread and launch
        if (bUseViewer)
        {
            mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
        }

        //Set pointers between threads                                                                                      
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);
        mpTracker->SetCalibratingter(mpCalibratingter);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);
        mpLocalMapper->SetCalibratingter(mpCalibratingter);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
        mpLoopCloser->SetCalibratingter(mpCalibratingter);

        mpCalibratingter->SetLocalMapper(mpLocalMapper);
        mpCalibratingter->SetLoopCloser(mpLoopCloser);
        mpCalibratingter->SetTracker(mpTracker);
    }

    cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
    {
        if (mSensor != RGBD)
        {
            cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
            exit(-1);
        }

        // Check Localization mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();
                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }
                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check Calibratingtion mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateCalibrationMode)
            {
                CalibrationFlag = 1 ;
                mpCalibratingter->InformCalibrating(true);
                mbActivateCalibrationMode = false;
            }
            if (mbDeactivateCalibrationMode)
            {
                CalibrationFlag = 2 ;
                mpCalibratingter->InformCalibrating(false);
                mpCalibratingter->Release();
                mbDeactivateCalibrationMode = false;
            }
        }
        
        // Check CalibratingtionOptimizing mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateCalibrationOptimizingMode)
            {
                
                mpCalibratingter->InformOptimizing(true);
                mbActivateCalibrationOptimizingMode = false;
            }
            if (mbDeactivateCalibrationOptimizingMode)
            {
                
                mpCalibratingter->InformOptimizing(false);
                
                mbDeactivateCalibrationOptimizingMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
            }
        }
        cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);
        unique_lock<mutex> lock2(mMutexState);                                                                    
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }

                                                                                                                   
    void System::ActivateCalibrationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateCalibrationMode = true;
    }

    void System::DeactivateCalibrationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateCalibrationMode = true;
    }

    void System::ActivateCalibrationOptimizingMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateCalibrationOptimizingMode = true;
    }

    void System::DeactivateCalibrationOptimizingMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateCalibrationOptimizingMode = true;
    }

    void System::ActivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }

    bool System::MapChanged()
    {
        static int n = 0;
        int curn = mpMap->GetLastBigChangeIdx();
        if (n < curn)
        {
            n = curn;
            return true;
        }
        else
            return false;
    }

    void System::Reset()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::Shutdown()
    {
        mpLocalMapper->RequestFinish(); 
        mpLoopCloser->RequestFinish(); 
        mpCalibratingter->RequestFinish(); 
        mpPointCloudMapping->shutdown(); 

        if (mpViewer)
        {
            mpViewer->RequestFinish();
            while (!mpViewer->isFinished())
                usleep(5000);
        }

        // Wait until all thread have effectively stopped
        while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
        {
            usleep(5000);
        }

        if (mpViewer)
            pangolin::BindToContext("Camvox: Map Viewer");
    }

    void System::SaveTrajectoryTUM(const string &filename)
    {
        cout << endl
             << "Saving camera trajectory to " << filename << " ..." << endl;
       
        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<Camvox::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();
        for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                     lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            while (pKF->isBad())
            {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            cv::Mat Tcw = (*lit) * Trw;
            cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            // twc: world coordinate
            cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

            vector<float> q = Converter::toQuaternion(Rwc);

            f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
        f.close();
        cout << endl
             << "trajectory saved!" << endl;
    }

    void System::SaveKeyFrameTrajectoryTUM(const string &filename)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            pKF->SetPose(pKF->GetPose()*Two);

            if (pKF->isBad())
                continue;

            cv::Mat R = pKF->GetRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }

        f.close();
        cout << endl
             << "trajectory saved!" << endl;
    }

    void System::SaveTrajectoryKITTI(const string &filename)
    {
        cout << endl
             << "Saving camera trajectory to " << filename << " ..." << endl;       
        
        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<Camvox::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++)
        {
            Camvox::KeyFrame *pKF = *lRit;

            cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

            while (pKF->isBad())
            {
                //  cout << "bad parent" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            cv::Mat Tcw = (*lit) * Trw;
            cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

            f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " " << twc.at<float>(0) << " " << Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1) << " " << Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << endl;
        }
        f.close();
        cout << endl
             << "trajectory saved!" << endl;
    }

    int System::GetTrackingState()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackingState;
    }

    vector<MapPoint *> System::GetTrackedMapPoints()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedMapPoints;
    }

    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedKeyPointsUn;
    }
    void System::SavePointCloud()
    {
        mpPointCloudMapping->save();
    }
    int System::getloopcount()
    {
        return mpLoopCloser->loopcount;
    }
} // namespace Camvox
