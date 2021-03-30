#include "FrameDrawer.h"
#include "Tracking.h"
#include "fstream"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>

namespace Camvox
{
    FrameDrawer::FrameDrawer(Map *pMap) : mpMap(pMap)
    {
        mState = Tracking::SYSTEM_NOT_READY;
        mIm = cv::Mat(568, 1520, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    cv::Mat FrameDrawer::DrawFrame()
    {
        cv::Mat im;
        vector<cv::KeyPoint> vIniKeys;     // Initialization: KeyPoints in reference frame
        vector<int> vMatches;              // Initialization: correspondeces with reference keypoints
        vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
        vector<bool> vbVO, vbMap;          // Tracked MapPoints in current frame
        int state;                         // Tracking state

        //Copy variables within scoped mutex
        {
            unique_lock<mutex> lock(mMutex);
            state = mState;
            if (mState == Tracking::SYSTEM_NOT_READY)
                mState = Tracking::NO_IMAGES_YET;

            mIm.copyTo(im);

            if (mState == Tracking::NOT_INITIALIZED)
            {
                vCurrentKeys = mvCurrentKeys;
                vIniKeys = mvIniKeys;
                vMatches = mvIniMatches;
            }
            else if (mState == Tracking::OK)
            {
                vCurrentKeys = mvCurrentKeys;
                vbVO = mvbVO;
                vbMap = mvbMap;
            }
            else if (mState == Tracking::LOST)
            {
                vCurrentKeys = mvCurrentKeys;
            }
        } // destroy scoped mutex -> release mutex

        if (im.channels() < 3)  //this should be always true
            cvtColor(im, im, CV_GRAY2BGR);

        //Draw
        if (state == Tracking::NOT_INITIALIZED) //INITIALIZING
        {
            for (unsigned int i = 0; i < vMatches.size(); i++)
            {
                if (vMatches[i] >= 0)
                {
                    cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt,
                             cv::Scalar(0, 255, 0));
                }
            }
        }
        else if (state == Tracking::OK) //TRACKING
        {
            mnTracked = 0;
            mnTrackedVO = 0;
            const float r = 5;
            const int n = vCurrentKeys.size();
            for (int i = 0; i < n; i++)
            {
                if (vbVO[i] || vbMap[i])
                {
                    cv::Point2f pt1, pt2;
                    pt1.x = vCurrentKeys[i].pt.x - r;
                    pt1.y = vCurrentKeys[i].pt.y - r;
                    pt2.x = vCurrentKeys[i].pt.x + r;
                    pt2.y = vCurrentKeys[i].pt.y + r;

                    // This is a match to a MapPoint in the map
                    if (vbMap[i])
                    {
                        cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
                        cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(255, 0, 0), -1);
                        mnTracked++;
                    }
                    else // This is match to a "visual odometry" MapPoint created in the last frame
                    {
                        cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
                        cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(0, 255, 0), -1);
                        mnTrackedVO++;
                    }
                }
            }
        }

        cv::Mat imWithInfo;
        DrawTextInfo(im, state, imWithInfo);

        return imWithInfo;
    }

    void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
    {
        stringstream s;
        if (nState == Tracking::NO_IMAGES_YET)
            s << " WAITING FOR IMAGES";
        else if (nState == Tracking::NOT_INITIALIZED)
            s << " TRYING TO INITIALIZE ";
        else if (nState == Tracking::OK)
        {
            if (!mbOnlyTracking)
                s << "SLAM MODE |  ";
            else
                s << "LOCALIZATION | ";
            int nKFs = mpMap->KeyFramesInMap();
            int nMPs = mpMap->MapPointsInMap();
            s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;

            // ofstream outfile;
            // outfile.open("/home/zyw/data.txt",ios::binary | ios::app | ios::in | ios::out);
            // outfile << mnTracked  <<endl;
            // outfile.close();

            if (mnTrackedVO > 0)
                s << ", + VO matches: " << mnTrackedVO;
        }
        else if (nState == Tracking::LOST)
        {
            s << " TRACK LOST. TRYING TO RELOCALIZE ";
        }
        else if (nState == Tracking::SYSTEM_NOT_READY)
        {
            s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
        }

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

        imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
        im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
        imText.rowRange(im.rows, imText.rows) = cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
        cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
    }
    
    void FrameDrawer::Update(Tracking *pTracker)
    {
        unique_lock<mutex> lock(mMutex);
        pTracker->mImRGB.copyTo(mIm);
        mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
        N = mvCurrentKeys.size();
        mvbVO = vector<bool>(N, false);
        mvbMap = vector<bool>(N, false);
        mbOnlyTracking = pTracker->mbOnlyTracking;

        if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED)
        {
            mvIniKeys = pTracker->mInitialFrame.mvKeys;
            mvIniMatches = pTracker->mvIniMatches;
        }
        else if (pTracker->mLastProcessedState == Tracking::OK)
        {
            for (int i = 0; i < N; i++)
            {
                MapPoint *pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
                if (pMP)
                {
                    if (!pTracker->mCurrentFrame.mvbOutlier[i])
                    {
                        if (pMP->Observations() > 0)
                            mvbMap[i] = true;
                        else
                            mvbVO[i] = true;
                    }
                }
            }
        }
        mState = static_cast<int>(pTracker->mLastProcessedState);
    }
} // namespace Camvox
