#include "Viewer.h"
#include <pangolin/pangolin.h>   
#include <iostream>
#include "opencv2/opencv.hpp"
#include <math.h>
#include <numeric>
#include <chrono>

#include <mutex>

using namespace std;
using namespace cv;

cv::Mat last_img,current_img,im_copy,im;

namespace Camvox
{

    Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath) : mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
                                                                                                                                           mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if (fps < 1)
            fps = 30;
        mT = 1e3 / fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if (mImageWidth < 1 || mImageHeight < 1)
        {
            mImageWidth = 1520;
            mImageHeight = 568;
        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];
    }

    void Viewer::Run()
    {
        int number = 0;
        int sflag = 0;
        mbFinished = false;
        mbStopped = false;

        pangolin::CreateWindowAndBind("Camvox: Map Viewer", 1024, 768);                              

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuCalibratingtion("menu.camvox Calibratingte", false, true);
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
        pangolin::Var<bool> menuReset("menu.Reset", false, false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f).SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        
        cv::namedWindow("Camvox: Current Frame");                                                       

        bool bFollow = true;
        bool bLocalizationMode = false;
        bool bCalibratingtionMode = false;

        while (1)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
            
            if (menuCalibratingtion && !bCalibratingtionMode)                 
            {
                mpSystem->ActivateCalibratingtionMode();
                bCalibratingtionMode = true;
            }
            else if (!menuCalibratingtion && bCalibratingtionMode)
            {
                mpSystem->DeactivateCalibratingtionMode();
                bCalibratingtionMode = false;
            }

            if (menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if (menuFollowCamera && !bFollow)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else if (!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            if (menuLocalizationMode && !bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if (!menuLocalizationMode && bLocalizationMode)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            mpMapDrawer->DrawCurrentCamera(Twc);
            if (menuShowKeyFrames || menuShowGraph)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
            if (menuShowPoints)
                mpMapDrawer->DrawMapPoints();
            pangolin::FinishFrame();
            
            
            last_img = im;                               
            im = mpFrameDrawer->DrawFrame();             
           //im = mpMapDrawer->DrawCurrentCamera(Twc);
            current_img = im;
            im_copy = im.clone();

            /*...TODO...*/
            if((!im.empty())&&(!last_img.empty()))
            {
                double t = (double)cvGetTickCount();
                bool result = judgemovement(last_img,current_img); 
                t = (double)cvGetTickCount() - t;
                //std::cout << "Single frame consumes " << t / ((double)cvGetTickFrequency() * 1000.) << " ms" << endl;
                if (result == true)
                {
                    number = 0;
                    putText(im, "Robot is moving", Point(50, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 4, 8); 
                }
                else
                {
                    number++;
                    putText(im, "Robot is still", Point(50, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 4, 8); 
                }
                if(number==10)
                {
                    cv::imwrite("./camvox/calibration/calibration.bmp", im_copy);
                    cout << "Robot keep 1s still !" << endl;
                    putText(im, "Robot keep 1s still !", Point(1200, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 4, 8); 
                    mpSystem->ActivateCalibratingtionMode();
                    sflag = 1;
                }
                else if(number == 100)
                {
                    number = 0;
                    sflag = 0;
                }
                if((sflag==1)&&(number==0))
                {
                    cout << "cancel calibration !" << endl;
                    putText(im, "cancel calibration !", Point(1200, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 4, 8); //在图片上写文字
                    mpSystem->DeactivateCalibratingtionMode();
                    sflag = 0;
                }
            } 
            /*...TODO...*/
            
            cv::imshow("Camvox: Current Frame", im);
            cv::waitKey(mT);

            if (menuReset)
            {
                menuShowGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                menuLocalizationMode = false;
                if (bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                menuCalibratingtion = false;
                if (bCalibratingtionMode)
                    mpSystem->DeactivateCalibratingtionMode();
                bCalibratingtionMode = false;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->Reset();
                menuReset = false;
            }

            if (Stop())
            {
                while (isStopped())
                {
                    usleep(3000);
                }
            }

            if (CheckFinish())
                break;
        }

        SetFinish();
    }

/*********************************************************************************/
    bool Viewer::judgemovement( cv::Mat &pre_frame,  cv::Mat &curr_frame)
    {
        vector<vector<Point>>contours;
        vector<Vec4i>hierarchy;   
        Mat element = getStructuringElement(MORPH_RECT,Size(1,1));
        Mat img_delta;
        int count = 0;
        cvtColor(pre_frame,pre_frame,CV_BGR2GRAY);
        cvtColor(curr_frame,curr_frame,CV_BGR2GRAY);
        GaussianBlur(pre_frame,pre_frame,Size(3,3),0);     //why blur
        GaussianBlur(curr_frame,curr_frame,Size(3,3),0);
        absdiff(pre_frame,curr_frame,img_delta); 
        threshold(img_delta,img_delta,25,255,CV_THRESH_BINARY);
        dilate(img_delta,img_delta,cv::Mat());
        findContours(img_delta, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        for(int i=0;i<contours.size();i++)  
        {
        if(contourArea(contours[i])<1300) 
            continue;
        else                          
            count++;
        }    
        if(count>=7)                               
            return true;
        else
            return false;
    }
/*********************************************************************************/

    void Viewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if (!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if (mbFinishRequested)
            return false;
        else if (mbStopRequested)
        {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;
    }

    void Viewer::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

} // namespace Camvox
