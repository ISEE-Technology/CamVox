#include "Tracking.h"
#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Optimizer.h"
#include "PnPsolver.h"

#include <iostream>
#include <mutex>

#include <chrono>

#include <stdlib.h>
#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <time.h>

long int idk = 1;

using namespace std;

namespace Camvox
{

    Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, shared_ptr<PointCloudMapping> pPointCloud, KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor) : mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
                                                                                                                                                                                                                                             mpPointCloudMapping(pPointCloud),
                                                                                                                                                                                                                                             mpKeyFrameDB(pKFDB), mpSystem(pSys), mpViewer(NULL),
                                                                                                                                                                                                                                             mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
    {
        // Load camera parameters from settings file

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        //       |fx  0   cx|
        // K =   |0   fy  cy|
        //       |0   0   1 |
        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        // 图像矫正系数
        // [k1 k2 p1 p2 k3]
        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];   
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        // 双目摄像头baseline * fx 50
        mbf = fSettings["Camera.bf"];

        float fps = fSettings["Camera.fps"];
        if (fps == 0)
            fps = 30;

        // Max/Min Frames to insert keyframes and to check relocalisation
        mMinFrames = 0;
        mMaxFrames = fps;

        cout << endl
             << "Camera Parameters: " << endl;
        cout << "- fx: " << fx << endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        if (DistCoef.rows == 5)
            cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- p1: " << DistCoef.at<float>(2) << endl;
        cout << "- p2: " << DistCoef.at<float>(3) << endl;
        cout << "- fps: " << fps << endl;

        // 1:RGB 0:BGR
        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;                           

        if (mbRGB)
            cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            cout << "- color order: BGR (ignored if grayscale)" << endl;

        // Load ORB parameters

        // 每一帧提取的特征点数 1000
        int nFeatures = fSettings["ORBextractor.nFeatures"];
        // 图像建立金字塔时的变化尺度 1.2
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        // 尺度金字塔的层数 8
        int nLevels = fSettings["ORBextractor.nLevels"];
        // 提取fast特征点的默认阈值 20
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        // 如果默认阈值提取不出足够fast特征点，则使用最小阈值 8                                                
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];

        // tracking过程都会用到mpORBextractorLeft作为特征点提取器
        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        cout << endl
             << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

        if (sensor == System::RGBD)
        {
            // 判断一个3D点远/近的阈值 mbf * 35 / fx
            mThDepth = mbf * (float)fSettings["ThDepth"] / fx;                                          //! mbf ThDepth   ->   threshold
            cout << endl
                 << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }

        if (sensor == System::RGBD)                                                                     //! mDepthMapFactor -> CV_32F
        {
            // 深度相机disparity转化为depth时的因子
            mDepthMapFactor = fSettings["DepthMapFactor"];
            if (fabs(mDepthMapFactor) < 1e-5) //求绝对值
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        }
    }

    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper = pLocalMapper;
    }

    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
    {
        mpLoopClosing = pLoopClosing;
    }

    void Tracking::SetViewer(Viewer *pViewer)
    {
        mpViewer = pViewer;
    }

    void Tracking::SetCalibratingter(Calibrating *pCalibratingter)
    {
        mpCalibratingter = pCalibratingter;
    }
    
    // 输入左目RGB或RGBA图像和深度图
    // 1、将图像转为mImGray和imDepth并初始化mCurrentFrame
    // 2、进行tracking过程
    // 输出世界坐标系到该帧相机坐标系的变换矩阵
    cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp)
    {
        mImRGB = imRGB;
        mImGray = imRGB;
        mImDepth = imD;
        // 步骤1：将RGB或RGBA图像转为灰度图像
        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB) 
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
            // 改为彩色图像
            /* if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2BGR); */
        }
        // 步骤2：将深度相机的disparity转为Depth
        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || mImDepth.type() != CV_32F)
            mImDepth.convertTo(mImDepth, CV_32F, mDepthMapFactor);    
        ofstream OutFile("/home/zyw/save_timestamp.txt");                             //! has get real depth-> d                     
        // 步骤3：构造Frame

        //!  --------------------------------------------------->   Time1  -------------------------------------------------------------------------- //
        //auto t0 = std::chrono::steady_clock::now();

        mCurrentFrame = Frame(mImGray, mImDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);   
        OutFile << "frame: "<<"delt_t"<<endl;  

        //  std::cout <<"構造幀時間"<< std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count()<< '\n';
     
        // 步骤4：跟踪

        Track();  

        OutFile.close();  
        return mCurrentFrame.mTcw.clone();
    }

    /**
     * @brief Main tracking function. It is independent of the input sensor.
     * Tracking 线程                                                                               //!  Main Tracking Function                       
     */
    void Tracking::Track()
    {
        // track包含两部分：估计运动、跟踪局部地图

        // mState为tracking的状态机
        // SYSTME_NOT_READY, NO_IMAGE_YET, NOT_INITIALIZED, OK, LOST
        // 如果图像复位过、或者第一次运行，则为NO_IMAGE_YET状态
        if (mState == NO_IMAGES_YET)
        {
            mState = NOT_INITIALIZED; //判断系统是否初始化，没有初始化就跳转到初始化函数，这是单目所独有的过程；
        }
        // mLastProcessedState存储了Tracking最新的状态，用于FrameDrawer中的绘制
        mLastProcessedState = mState;

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        //! 步骤1：初始化
        if (mState == NOT_INITIALIZED)
        {
            if (mSensor == System::RGBD)
                StereoInitialization();
            else
                cout << " " << endl;  //!D-C! MonocularInitialization();

            mpFrameDrawer->Update(this);  

            if (mState != OK)
                return;
        }
        //! 步骤2：跟踪   Camera pose track and Track Local Map
        else            
        {
            // System is initialized. Track Frame.
            // bOK为临时变量，用于表示每个函数是否执行成功
            bool bOK;

            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            // 在viewer中有个开关menuLocalizationMode，有它控制是否ActivateLocalizationMode，并最终管控mbOnlyTracking
            // mbOnlyTracking等于false表示正常VO模式（有地图更新），mbOnlyTracking等于true表示用户手动选择定位模式
            if (!mbOnlyTracking)
            {
                // Local Mapping is activated. This is the normal behaviour, unless
                // you explicitly activate the "only tracking" mode.

                // 正常初始化成功                                                       //! Camera Pose tracking
                if (mState == OK)
                {
                    // Local Mapping might have changed some MapPoints tracked in last frame
                    // 检查并更新上一帧被替换的MapPoints
                    // 更新Fuse函数和SearchAndFuse函数替换的MapPoints
                    CheckReplacedInLastFrame(); //更新最近一帧 lastframe 路标点
                    // 步骤2.1：跟踪上一帧或者参考帧或者重定位

                    // 运动模型是空的或刚完成重定位
                    // mCurrentFrame.mnId<mnLastRelocFrameId+2这个判断不应该有
                    // 应该只要mVelocity不为空，就优先选择TrackWithMotionModel
                    // mnLastRelocFrameId上一次重定位的那一帧
                    if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                    {
                        // 将上一帧的位姿作为当前帧的初始位姿
                        // 通过BoW的方式在参考帧中找当前帧特征点的匹配点
                        // 优化每个特征点都对应3D点重投影误差即可得到位姿
                        bOK = TrackReferenceKeyFrame();
                    }
                    else
                    {
                        // 根据恒速模型设定当前帧的初始位姿
                        // 通过投影的方式在参考帧中找当前帧特征点的匹配点
                        // 优化每个特征点所对应3D点的投影误差即可得到位姿
                        bOK = TrackWithMotionModel();
                        if (!bOK)
                            // 当使用运动模式匹配到的特征点数较少时，就会选用关键帧模式。即尝试和最近一个关键帧去做匹配。为了快速匹配，本文利用了bag of words（BoW）来加速。首先，计算当前帧的BoW，并设定初始位姿为上一帧的位姿；其次，根据位姿和BoW词典来寻找特征匹配，使用函数matcher.SearchByBoW()；最后，利用匹配的特征优化位姿。TrackReferenceKeyFrame是跟踪参考帧，不能根据固定运动速度模型预测当前帧的位姿态，通过bow加速匹配（SearchByBow）
                            // 最后通过优化得到优化后的位姿
                            bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // BOW搜索，PnP求解位姿
                    bOK = Relocalization();
                }
            }
            else
            {
                // Localization Mode: Local Mapping is deactivated
                // 只进行跟踪tracking，局部地图不工作

                // 步骤2.1：跟踪上一帧或者参考帧或者重定位

                // tracking跟丢了
                if (mState == LOST)
                {
                    bOK = Relocalization();
                }
                else
                {
                    // mbVO是mbOnlyTracking为true时的才有的一个变量
                    // mbVO为false表示此帧匹配了很多的MapPoints，跟踪很正常，
                    // mbVO为true表明此帧匹配了很少的MapPoints，少于10个，要跪的节奏
                    if (!mbVO)
                    {
                        // In last frame we tracked enough MapPoints in the map
                        // mbVO为0则表明此帧匹配了很多的3D map点，非常好
                        if (!mVelocity.empty())
                        {
                            bOK = TrackWithMotionModel();
                            // 这个地方是不是应该加上：
                            // if(!bOK)
                            //    bOK = TrackReferenceKeyFrame();
                        }
                        else
                        {
                            bOK = TrackReferenceKeyFrame();
                        }
                    }
                    else
                    {
                        // In last frame we tracked mainly "visual odometry" points.

                        // We compute two camera poses, one from motion model and one doing relocalization.
                        // If relocalization is sucessfull we choose that solution, otherwise we retain
                        // the "visual odometry" solution.
                        // mbVO为1，则表明此帧匹配了很少的3D map点，少于10个，要跪的节奏，既做跟踪又做定位

                        bool bOKMM = false;
                        bool bOKReloc = false;
                        vector<MapPoint *> vpMPsMM;
                        vector<bool> vbOutMM;
                        cv::Mat TcwMM;
                        if (!mVelocity.empty())
                        {
                            bOKMM = TrackWithMotionModel();
                            // 这三行没啥用？
                            vpMPsMM = mCurrentFrame.mvpMapPoints;
                            vbOutMM = mCurrentFrame.mvbOutlier;
                            TcwMM = mCurrentFrame.mTcw.clone();
                        }
                        bOKReloc = Relocalization();

                        // 重定位没有成功，但是如果跟踪成功
                        if (bOKMM && !bOKReloc)
                        {
                            // 这三行没啥用？
                            mCurrentFrame.SetPose(TcwMM);
                            mCurrentFrame.mvpMapPoints = vpMPsMM;
                            mCurrentFrame.mvbOutlier = vbOutMM;

                            if (mbVO)
                            {
                                // 这段代码是不是有点多余？应该放到TrackLocalMap函数中统一做
                                // 更新当前帧的MapPoints被观测程度
                                for (int i = 0; i < mCurrentFrame.N; i++)
                                {
                                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                    {
                                        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                    }
                                }
                            }
                        }
                        else if (bOKReloc) // 只要重定位成功整个跟踪过程正常进行（定位与跟踪，更相信重定位）
                        {
                            mbVO = false;
                        }

                        bOK = bOKReloc || bOKMM;
                    }
                }
            }
            // 将最新的关键帧作为reference frame
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // If we have an initial estimation of the camera pose and matching. Track the local map.
            // 步骤2.2：在帧间匹配得到初始的姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
            // local map:当前帧、当前帧的MapPoints、当前关键帧与其它关键帧共视关系
            // 在步骤2.1中主要是两两跟踪（恒速模型跟踪上一帧、跟踪参考帧），这里搜索局部关键帧后搜集所有局部MapPoints，
            // 然后将局部MapPoints和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化            
                                                                                                                  //! Track Local Map
            if (!mbOnlyTracking)
            {
                if (bOK)
                    bOK = TrackLocalMap();
            }
            else
            {
                // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                // the camera we will use the local map again.
                // 重定位成功
                if (bOK && !mbVO)
                    bOK = TrackLocalMap();
            }

            if (bOK)
                mState = OK;
            else
                mState = LOST;

            // Update drawer
            mpFrameDrawer->Update(this);

            // If tracking were good, check if we insert a keyframe
            if (bOK)
            {
                // Update motion model
                if (!mLastFrame.mTcw.empty())
                {
                    // 步骤2.3：更新恒速运动模型TrackWithMotionModel中的mVelocity
                    cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                    mVelocity = mCurrentFrame.mTcw * LastTwc; // Tcl
                }
                else
                    mVelocity = cv::Mat();

                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

                // Clean VO matches                                                                                      //! clear MapPoints
                // 步骤2.4：清除UpdateLastFrame中为当前帧临时添加的MapPoints
                for (int i = 0; i < mCurrentFrame.N; i++)
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];                       
                    if (pMP)
                        // 排除UpdateLastFrame函数中为了跟踪增加的MapPoints
                        if (pMP->Observations() < 1)
                        {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                        }
                }

                // Delete temporal MapPoints
                // 步骤2.5：清除临时的MapPoints，这些MapPoints在TrackWithMotionModel的UpdateLastFrame函数里生成（仅双目和rgbd）  //! For RGBD and stereo ,generate the MapPoints for Lastframe
                // 步骤2.4中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
                // 这里生成的仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
                for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++)
                {
                    MapPoint *pMP = *lit;
                    delete pMP;
                }
                // 这里不仅仅是清除mlpTemporalPoints，通过delete pMP还删除了指针指向的MapPoint
                mlpTemporalPoints.clear();

                // Check if we need to insert a new keyframe
                // 步骤2.6：检测并插入关键帧，对于双目会产生新的MapPoints                                                       //! insert a new keyframe
                if (NeedNewKeyFrame())
                    CreateNewKeyFrame();
                idk++; //上个版本木有
                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame.
                // 删除那些在bundle adjustment中检测为outlier的3D map点
                for (int i = 0; i < mCurrentFrame.N; i++)
                {
                    if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }
            }

            // Reset if the camera get lost soon after initialization
            // 跟踪失败，并且relocation也没有搞定，只能重新Reset
            if (mState == LOST)
            {
                if (mpMap->KeyFramesInMap() <= 5)
                {
                    cout << "Track lost soon after initialisation, reseting..." << endl;
                    mpSystem->Reset();
                    return;
                }
            }

            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // 保存上一帧的数据
            mLastFrame = Frame(mCurrentFrame);
        }

        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        //! 步骤3：记录位姿信息，用于轨迹复现
        if (!mCurrentFrame.mTcw.empty())
        {
            // 计算相对姿态T_currentFrame_referenceKeyFrame
            cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState == LOST);
        }
        else
        {
            // This can happen if tracking is lost
            //! 如果跟踪失败，则相对位姿使用上一次值
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState == LOST);
        }
    }

   /**
    * @brief 双目和rgbd的地图初始化
    *
    * 由于具有深度信息，直接生成MapPoints
    */
    void Tracking::StereoInitialization()
    {
        if (mCurrentFrame.N > 500)
        {
            // Set Frame pose to the origin
            // 步骤1：设定初始位姿
            mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F)); //对角矩阵  

            // Create KeyFrame
            // 步骤2：将当前帧构造为初始关键帧
            // mCurrentFrame的数据类型为Frame
            // KeyFrame包含Frame、地图3D点、以及BoW
            // KeyFrame里有一个mpMap，Tracking里有一个mpMap，而KeyFrame里的mpMap都指向Tracking里的这个mpMap
            // KeyFrame里有一个mpKeyFrameDB，Tracking里有一个mpKeyFrameDB，而KeyFrame里的mpMap都指向Tracking里的这个mpKeyFrameDB
            KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB); 

            // Insert KeyFrame in the map
            // KeyFrame中包含了地图、反过来地图中也包含了KeyFrame，相互包含
            // 步骤3：在地图中添加该初始关键帧
            mpMap->AddKeyFrame(pKFini);

            // Create MapPoints and asscoiate to KeyFrame
            // 步骤4：为每个特征点构造MapPoint
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0)
                {
                    // 步骤4.1：通过反投影得到该特征点的3D坐标
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    // 步骤4.2：将3D点构造为MapPoint
                    MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap);

                    // 步骤4.3：为该MapPoint添加属性：
                    // a.观测到该MapPoint的关键帧
                    // b.该MapPoint的描述子
                    // c.该MapPoint的平均观测方向和深度范围

                    // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
                    pNewMP->AddObservation(pKFini, i); //输入当前KeyFrame以及第i个特征点
                    // 步骤4.5：表示该KeyFrame的哪个特征点可以观测到哪个3D点
                    pKFini->AddMapPoint(pNewMP, i);
                    // b.从众多观测到该MapPoint的特征点中挑选区分读最高的描述子，上一步每个关键帧都拥有一个特征点
                    pNewMP->ComputeDistinctiveDescriptors();
                    // c.更新该MapPoint平均观测方向以及观测距离的范围
                    /*地图点到所有观测到的关键帧相机中心向量，归一化后相加。                                         //!  choose the level according by depth
        深度范围：地图点到参考帧（只有一帧）相机中心距离，乘上参考帧中描述子获取时金字塔放大尺度，得到最大距离mfMaxDistance；
        最大距离除以整个金字塔最高层的放大尺度得到最小距离mfMinDistance。通常来说，距离较近的地图点，
        将在金字塔层数较高的地方提取出，距离较远的地图点，在金字塔层数较低的地方提取出（金字塔层数越低，分辨率越高，才能识别出远点）
        。因此通过地图点的信息（主要是对应描述子），我们可以获得该地图点对应的金字塔层级：
        const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
        从而预测该地图点在什么距离范围内能够被观测到！*/
                    pNewMP->UpdateNormalAndDepth();           
                    // 步骤4.4：在地图中添加该MapPoint
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                }
            }
            
            cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;  
            // 步骤4：在局部地图中添加该初始关键帧  
            mpLocalMapper->InsertKeyFrame(pKFini);

            mLastFrame = Frame(mCurrentFrame);     //作为最近一帧
            mnLastKeyFrameId = mCurrentFrame.mnId; //最近一个关键帧的id
            mpLastKeyFrame = pKFini;               //将上述关键帧操作作为最近一个关键帧

            mvpLocalKeyFrames.push_back(pKFini);          //局部关键帧数组
            mvpLocalMapPoints = mpMap->GetAllMapPoints(); //局部关键帧关键点路标集合 
            mpReferenceKF = pKFini;                       //关键帧作为参考帧 
            mCurrentFrame.mpReferenceKF = pKFini;         //关键帧作为参考帧？？？？？？？？？？？

            // 把当前（最新的）局部MapPoints作为ReferenceMapPoints
            // ReferenceMapPoints是DrawMapPoints函数画图的时候用的
            mpMap->SetReferenceMapPoints(mvpLocalMapPoints);  

            mpMap->mvpKeyFrameOrigins.push_back(pKFini);

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw); 

            mState = OK;
        }
    }

    /**
     * @brief 检查上一帧中的MapPoints是否被替换
     * Local Mapping线程可能会将关键帧中某些MapPoints进行替换，由于tracking中需要用到mLastFrame，这里检查并更新上一帧中被替换的MapPoints
     * @see LocalMapping::SearchInNeighbors()
     */
    void Tracking::CheckReplacedInLastFrame()
    {
        for (int i = 0; i < mLastFrame.N; i++)
        {
            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            if (pMP)
            {
                MapPoint *pRep = pMP->GetReplaced();
                if (pRep)
                {
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
    }
  
    /**
     * @brief 对参考关键帧的MapPoints进行跟踪
     * 
     * 1. 计算当前帧的词包，将当前帧的特征点分到特定层的nodes上
     * 2. 对属于同一node的描述子进行匹配
     * 3. 根据匹配对估计当前帧的姿态
     * 4. 根据姿态剔除误匹配
     * @return 如果匹配数大于10，返回true
     */
    bool Tracking::TrackReferenceKeyFrame()
    {
        // Compute Bag of Words vector
        // 步骤1：将当前帧的描述子转化为BoW向量
        mCurrentFrame.ComputeBoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, true);        //定义ORB匹配
        vector<MapPoint *> vpMapPointMatches; //定义匹配到的路标点

        // 步骤2：通过特征点的BoW加快当前帧与参考帧之间的特征点匹配
        // 特征点的匹配关系由MapPoints进行维护  
        int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches); //获取匹配的对数

        if (nmatches < 15)
            return false;
        // 步骤3:将上一帧的位姿态作为当前帧位姿的初始值
        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.mTcw); // 用上一次的Tcw设置初值，在PoseOptimization可以收敛快一些
        // 步骤4:通过优化3D-2D的重投影误差来获得位姿
        Optimizer::PoseOptimization(&mCurrentFrame); //优化

        // Discard outliers
        // 步骤5：剔除优化后的outlier匹配点（MapPoints）
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        return nmatchesMap >= 10;
    }

    /**
     * @brief 双目或rgbd摄像头根据深度值为上一帧产生新的MapPoints
     *
     * 在双目和rgbd情况下，选取一些深度小一些的点（可靠一些） 
     * 可以通过深度值产生一些新的MapPoints
     */
    void Tracking::UpdateLastFrame()
    {
        // Update pose according to reference keyframe
        // 步骤1：更新最近一帧的位姿
        KeyFrame *pRef = mLastFrame.mpReferenceKF;
        cv::Mat Tlr = mlRelativeFramePoses.back();

        mLastFrame.SetPose(Tlr * pRef->GetPose()); // Tlr*Trw = Tlw 1:last r:reference w:world

        // 如果上一帧为关键帧，或者单目的情况，则退出
        if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || !mbOnlyTracking)          //! this code need to be changed!!!
            return;
 
        // 步骤2：对于双目或rgbd摄像头，为上一帧临时生成新的MapPoints
        // 注意这些MapPoints不加入到Map中，在tracking的最后会删除                                                //! Choose the close points, these Mappoints dont insert Map
        // 跟踪过程中需要将将上一帧的MapPoints投影到当前帧可以缩小匹配范围，加快当前帧与上一帧进行特征点匹配

        // Create "visual odometry" MapPoints                                                              //! the first reference VO
        // We sort points according to their measured depth by the stereo/RGB-D sensor                     //! why sort Points according to depth ?
        // 步骤2.1：得到上一帧有深度值的特征点   
        vector<pair<float, int>> vDepthIdx;
        vDepthIdx.reserve(mLastFrame.N);
        for (int i = 0; i < mLastFrame.N; i++)
        {
            float z = mLastFrame.mvDepth[i];
            if (z > 0)
            {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (vDepthIdx.empty())
            return;

        // 步骤2.2：按照深度从小到大排序
        sort(vDepthIdx.begin(), vDepthIdx.end());

        // We insert all close points (depth<mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        // 步骤2.3：将距离比较近的点包装成MapPoints
        int nPoints = 0;
        for (size_t j = 0; j < vDepthIdx.size(); j++)
        {
            int i = vDepthIdx[j].second;

            bool bCreateNew = false;

            MapPoint *pMP = mLastFrame.mvpMapPoints[i];
            if (!pMP)
                bCreateNew = true;
            else if (pMP->Observations() < 1)
            {
                bCreateNew = true;
            }

            if (bCreateNew)
            {
                // 这些生成MapPoints后并没有通过：
                // a.AddMapPoint、
                // b.AddObservation、
                // c.ComputeDistinctiveDescriptors、
                // d.UpdateNormalAndDepth添加属性，
                // 这些MapPoint仅仅为了提高双目和RGBD的跟踪成功率
                cv::Mat x3D = mLastFrame.UnprojectStereo(i); //获取该特征点三维坐标
                MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

                mLastFrame.mvpMapPoints[i] = pNewMP; // 添加新的MapPoint

                // 标记为临时添加的MapPoint，之后在CreateNewKeyFrame之前会全部删除
                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            }
            else
            {
                nPoints++;
            }

            if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                break;
        }
    }

    /**
     * @brief 根据匀速度模型对上一帧的MapPoints进行跟踪
     * 
     * 1. 非单目情况，需要对上一帧产生一些新的MapPoints（临时）
     * 2. 将上一帧的MapPoints投影到当前帧的图像平面上，在投影的位置进行区域匹配
     * 3. 根据匹配对估计当前帧的姿态
     * 4. 根据姿态剔除误匹配
     * @return 如果匹配数大于10，返回true
     * @see V-B Initial Pose Estimation From Previous Frame
     */
    bool Tracking::TrackWithMotionModel()
    {
        ORBmatcher matcher(0.9, true);

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        //! 步骤1：对于双目或rgbd摄像头，根据深度值为上一关键帧生成新的MapPoints
        // （跟踪过程中需要将当前帧与上一帧进行特征点匹配，将上一帧的MapPoints投影到当前帧可以缩小匹配范围）
        // 在跟踪过程中，去除outlier的MapPoint，如果不及时增加MapPoint会逐渐减少
        // 这个函数的功能就是补充增加RGBD和双目相机上一帧的MapPoints数
        UpdateLastFrame();

        // 根据Const Velocity Model(认为这两帧之间的相对运动和之前两帧间相对运动相同)估计当前帧的位姿，nani                   //! Const velocity Model
        mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);

        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

        // Project points seen in previous frame
        int th;

        th = 7;

        // 步骤2：根据匀速度模型进行对上一帧的MapPoints进行跟踪
        // 根据上一帧特征点对应的3D点投影的位置缩小特征点匹配范围

         int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);

        // If few matches, uses a wider window search
        // 如果跟踪的点少，则扩大搜索半径再来一次

        if (nmatches < 20)
        {
            fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
            nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
        }

        if (nmatches < 20)
            return false;

        // Optimize frame pose with all matches
        // 步骤3：优化位姿
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        // 步骤4：优化位姿后剔除outlier的mvpMapPoints
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        if (mbOnlyTracking)
        {
            mbVO = nmatchesMap < 10;
            return nmatches > 20;
        }

        return nmatchesMap >= 10;
    }

    /**
     * @brief 对Local Map的MapPoints进行跟踪
     * 
     * 1. 更新局部地图，包括局部关键帧和关键点
     * 2. 对局部MapPoints进行投影匹配
     * 3. 根据匹配对估计当前帧的姿态
     * 4. 根据姿态剔除误匹配
     * @return true if success
     * @see V-D track Local Map
     */
    bool Tracking::TrackLocalMap()
    {
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.

        // Update Local KeyFrames and Local Points
        // 步骤1：更新局部关键帧mvpLocalKeyFrames和局部地图点mvpLocalMapPoints
        UpdateLocalMap();

        // 步骤2：在局部地图中查找与当前帧匹配的MapPoints
        SearchLocalPoints();

        // Optimize Pose
        // 在这个函数之前，在Relocalization、TrackReferenceKeyFrame、TrackWithMotionModel中都有位姿优化，
        // 步骤3：更新局部所有MapPoints后对位姿再次优化
        Optimizer::PoseOptimization(&mCurrentFrame);
        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        // 步骤3：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                // 由于当前帧的MapPoints可以被当前帧观测到，其被观测统计量加1
                if (!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking)
                    {
                        // 该MapPoint被其它关键帧观测到过
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    }
                    else
                        // 记录当前帧跟踪到的MapPoints，用于统计跟踪效果
                        mnMatchesInliers++;
                }
                //!D-C! 删除双目有关当前帧地图点
                /* else if (mSensor == System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL); */
                //!D-C!
            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        // 步骤4：决定是否跟踪成功
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
            return false;

        if (mnMatchesInliers < 30)
            return false;
        else
            return true;
    }

    /**
     * @brief 断当前帧是否为关键帧
     * @return true if needed
     */
    bool Tracking::NeedNewKeyFrame()
    {
        // 步骤1：如果用户在界面上选择重定位，那么将不插入关键帧
        // 由于插入关键帧过程中会生成MapPoint，因此用户选择重定位后地图上的点云和关键帧都不会再增加
        if (mbOnlyTracking)
            return false;

        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        // 如果局部地图被闭环检测使用，则不插入关键帧                                               //! Loopclosing dont insert KF
        if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
            return false;

        const int nKFs = mpMap->KeyFramesInMap();

        // Do not insert keyframes if not enough frames have passed from last relocalisation
        // 步骤2：判断是否距离上一次插入关键帧的时间太短
        // mCurrentFrame.mnId是当前帧的ID
        // mnLastRelocFrameId是最近一次重定位帧的ID
        // mMaxFrames等于图像输入的帧率
        // 如果关键帧比较少，则考虑插入关键帧
        // 或距离上一次重定位超过1s，则考虑插入关键帧
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
            return false;

        // Tracked MapPoints in the reference keyframe
        // 步骤3：得到参考关键帧跟踪到的MapPoints数量
        // 在UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
        int nMinObs = 3;
        if (nKFs <= 2)
            nMinObs = 2;
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

        // Local Mapping accept keyframes?
        // 步骤4：查询局部地图管理器是否繁忙
        bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose = 0;
        if (mSensor != System::MONOCULAR)                                                           //! this code need to be changed!!!
        {
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth)
                {
                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                        nTrackedClose++;
                    else
                        nNonTrackedClose++;
                }
            }
        }

        bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

        // 步骤6：决策是否需要插入关键帧
        // Thresholds
        // 设定inlier阈值，和之前帧特征点匹配的inlier比例
        float thRefRatio = 0.75f;
        if (nKFs < 2)
            thRefRatio = 0.4f; // 关键帧只有一帧，那么插入关键帧的阈值设置很低
    
        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        // 很长时间没有插入关键帧
        const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        // localMapper处于空闲状态
        const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
        //Condition 1c: tracking is weak
        // 跟踪要跪的节奏，0.25和0.3是一个比较低的阈值
        const bool c1c = mSensor != System::MONOCULAR && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        // 阈值比c1c要高，与之前参考帧（最近的一个关键帧）重复度不是太高
        const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

        if ((c1a || c1b || c1c) && c2)
        {
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise send a signal to interrupt BA                                         //! insert KF or interrupt BA
            if (bLocalMappingIdle)
            {
                return true;
            }
            else
            {
                mpLocalMapper->InterruptBA();
                if (mSensor != System::MONOCULAR)
                {
                    // 队列里不能阻塞太多关键帧
                    // tracking插入关键帧不是直接插入，而且先插入到mlNewKeyFrames中，
                    // 然后localmapper再逐个pop出来插入到mspKeyFrames  
                    if (mpLocalMapper->KeyframesInQueue() < 3)
                        return true;
                    else
                        return false;
                }
                else
                    return false;
            }
        }
        else
            return false;
    }

    /**
     * @brief 创建新的关键帧
     *
     * 对于非单目的情况，同时创建新的MapPoints
     */
    void Tracking::CreateNewKeyFrame()
    {
        if (!mpLocalMapper->SetNotStop(true))
            return;
        //! 步骤1：将当前帧构造成关键帧
        KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

        //! 步骤2：将当前关键帧共识度最高的关键帧设置为当前帧的参考关键帧
        // 在UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;

        // 这段代码和UpdateLastFrame中的那一部分代码功能相同
        //! 步骤3：对于双目或rgbd摄像头，为当前帧生成新的MapPoints                                      //!  RGBD create KeyFrame and new Mappoints
        if (mSensor != System::MONOCULAR)
        {
            // 根据Tcw计算mRcw、mtcw和mRwc、mOw
            mCurrentFrame.UpdatePoseMatrices();

            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            // 步骤3.1：得到当前帧深度小于阈值的特征点
            // 创建新的MapPoint, depth < mThDepth                                                //! mThDepth use as Theashold  to create MapPoint
            vector<pair<float, int>> vDepthIdx;
            vDepthIdx.reserve(mCurrentFrame.N);
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0)
                {
                    vDepthIdx.push_back(make_pair(z, i));
                }
            }

            if (!vDepthIdx.empty())
            {
                // 步骤3.2：按照深度从小到大排序                                                    //! sort the mappoints by depth
                sort(vDepthIdx.begin(), vDepthIdx.end());

                // 步骤3.3：将距离比较近的点包装成MapPoints                                         //! The close was packed as MapPoints
                int nPoints = 0;
                for (size_t j = 0; j < vDepthIdx.size(); j++)
                {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (!pMP)
                        bCreateNew = true;
                    else if (pMP->Observations() < 1)
                    {
                        bCreateNew = true;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }

                    if (bCreateNew)
                    {
                        cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                        MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap);
                        // 这些添加属性的操作是每次创建MapPoint后都要做的                             //! add property for the new MapPoints
                        pNewMP->AddObservation(pKF, i);
                        pKF->AddMapPoint(pNewMP, i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpMap->AddMapPoint(pNewMP);

                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                        nPoints++;
                    }
                    else
                    {
                        nPoints++;
                    }

                    // 这里决定了双目和rgbd摄像头时地图点云的稠密程度
                    // 但是仅仅为了让地图稠密直接改这些不太好，
                    // 因为这些MapPoints会参与之后整个slam过程
                    if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                        break;
                }
            }
        }

        mpLocalMapper->InsertKeyFrame(pKF);                                                                     //! there has get into Localmappint thread

        mpLocalMapper->SetNotStop(false);
        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        // insert Key Frame into point cloud viewer
        mpPointCloudMapping->insertKeyFrame(pKF, this->mImRGB, this->mImDepth, idk, vpKFs);                     //! PointCloudMapping update by KF

        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;
    }
    /**
 * @brief 对Local MapPoints进行跟踪
 * 
 * 在局部地图中查找在当前帧视野范围内的点，将视野范围内的点和当前帧的特征点进行投影匹配
 */
    void Tracking::SearchLocalPoints()
    {
        // Do not search map points already matched   
        // 步骤1：遍历当前帧的mvpMapPoints，标记这些MapPoints不参与之后的搜索   
        // 因为当前的mvpMapPoints一定在当前帧的视野中   
        for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
            {
                if (pMP->isBad())
                {
                    *vit = static_cast<MapPoint *>(NULL);
                }
                else
                {
                    // 更新能观测到该点的帧数加1
                    pMP->IncreaseVisible();
                    // 标记该点被当前帧观测到
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    // 标记该点将来不被投影，因为已经匹配过
                    pMP->mbTrackInView = false;
                }
            }
        }

        int nToMatch = 0;

        // Project points in frame and check its visibility
        // 步骤2：将所有局部MapPoints投影到当前帧，判断是否在视野范围内，然后进行投影匹配
        for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            // 已经被当前帧观测到MapPoint不再判断是否能被当前帧观测到
            if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if (pMP->isBad())
                continue;
            // Project (this fills MapPoint variables for matching)
            // 步骤2.1：判断LocalMapPoints中的点是否在在视野内
            if (mCurrentFrame.isInFrustum(pMP, 0.5))
            {
                // 观测到该点的帧数加1，该MapPoint在某些帧的视野范围内
                pMP->IncreaseVisible();
                // 只有在视野范围内的MapPoints才参与之后的投影匹配
                nToMatch++;
            }
        }

        if (nToMatch > 0)
        {
            ORBmatcher matcher(0.8);
            int th = 1;
            if (mSensor == System::RGBD)
                th = 3;
            // If the camera has been relocalised recently, perform a coarser search
            // 如果不久前进行过重定位，那么进行一个更加宽泛的搜索，阈值需要增大
            if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                th = 5;
            // 步骤2.2：对视野范围内的MapPoints通过投影进行特征点匹配
            matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
        }
    }

    /**
     * @brief 更新LocalMap
     *
     * 局部地图包括： \n
     * - K1个关键帧、K2个临近关键帧和参考关键帧
     * - 由这些关键帧观测到的MapPoints
     */
    void Tracking::UpdateLocalMap()
    {
        // This is for visualization
        // 这行程序放在UpdateLocalPoints函数后面是不是好一些
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        // Update
        // 更新局部关键帧和局部MapPoints
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

    /**
     * @brief 更新局部关键点，called by UpdateLocalMap()
     * 
     * 局部关键帧mvpLocalKeyFrames的MapPoints，更新mvpLocalMapPoints
     */
    void Tracking::UpdateLocalPoints()
    {
        // 步骤1：清空局部MapPoints
        mvpLocalMapPoints.clear();

        // 步骤2：遍历局部关键帧mvpLocalKeyFrames
        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
        {
            KeyFrame *pKF = *itKF;
            const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

            // 步骤2：将局部关键帧的MapPoints添加到mvpLocalMapPoints
            for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
            {
                MapPoint *pMP = *itMP;
                if (!pMP)
                    continue;
                // mnTrackReferenceForFrame防止重复添加局部MapPoint
                if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                    continue;
                if (!pMP->isBad())
                {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
            }
        }
    }

    /**
     * @brief 更新局部关键帧，called by UpdateLocalMap()
     *
     * 遍历当前帧的MapPoints，将观测到这些MapPoints的关键帧和相邻的关键帧取出，更新mvpLocalKeyFrames
     */
    void Tracking::UpdateLocalKeyFrames()
    {
        // Each map point vote for the keyframes in which it has been observed
        // 步骤1：遍历当前帧的MapPoints，记录所有能观测到当前帧MapPoints的关键帧
        map<KeyFrame *, int> keyframeCounter;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP->isBad())
                {
                    // 能观测到当前帧MapPoints的关键帧
                    const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                    for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i] = NULL;
                }
            }
        }

        if (keyframeCounter.empty())
            return;

        int max = 0;
        KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

        // 步骤2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧有三个策略
        // 先清空局部关键帧
        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        // V-D K1: shares the map points with current frame
        // 策略1：能观测到当前帧MapPoints的关键帧作为局部关键帧
        for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
        {
            KeyFrame *pKF = it->first;

            if (pKF->isBad())
                continue;

            if (it->second > max)
            {
                max = it->second;
                pKFmax = pKF;
            }

            mvpLocalKeyFrames.push_back(it->first);
            // mnTrackReferenceForFrame防止重复添加局部关键帧
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }

        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        // V-D K2: neighbors to K1 in the covisibility graph
        // 策略2：与策略1得到的局部关键帧共视程度很高的关键帧作为局部关键帧
        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
        {
            // Limit the number of keyframes
            if (mvpLocalKeyFrames.size() > 80)
                break;

            KeyFrame *pKF = *itKF;

            // 策略2.1:最佳共视的10帧
            const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

            for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
            {
                KeyFrame *pNeighKF = *itNeighKF;
                if (!pNeighKF->isBad())
                {
                    // mnTrackReferenceForFrame防止重复添加局部关键帧
                    if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            // 策略2.2:自己的子关键帧
            const set<KeyFrame *> spChilds = pKF->GetChilds();
            for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
            {
                KeyFrame *pChildKF = *sit;
                if (!pChildKF->isBad())
                {
                    if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            // 策略2.3:自己的父关键帧
            KeyFrame *pParent = pKF->GetParent();
            if (pParent)
            {
                // mnTrackReferenceForFrame防止重复添加局部关键帧
                if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // V-D Kref： shares the most map points with current frame
        // 步骤3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
        if (pKFmax)
        {
            mpReferenceKF = pKFmax;
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }

    bool Tracking::Relocalization()
    {
        // Compute Bag of Words Vector
        // 步骤1：计算当前帧特征点的Bow映射，下面函数要用
        mCurrentFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        // 步骤2：找到与当前帧相似的候选关键帧
        vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

        if (vpCandidateKFs.empty())
            return false;

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75, true);

        vector<PnPsolver *> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint *>> vvpMapPointMatches; //关键帧（向量）的路标点（向量）
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates = 0;

        //bow对每一个候选关键帧匹配
        for (int i = 0; i < nKFs; i++)
        {
            KeyFrame *pKF = vpCandidateKFs[i];
            if (pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                // 步骤3：通过BoW进行匹配
                int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
                if (nmatches < 15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    // 初始化PnPsolver
                    PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9, true);

        while (nCandidates > 0 && !bMatch)
        {
            for (int i = 0; i < nKFs; i++)
            {
                if (vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                // 步骤4：通过EPnP算法估计姿态
                PnPsolver *pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore)
                {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if (!Tcw.empty())
                {
                    Tcw.copyTo(mCurrentFrame.mTcw);

                    set<MapPoint *> sFound; //从小到大排列的容器

                    const int np = vbInliers.size();

                    for (int j = 0; j < np; j++)
                    {
                        if (vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j] = NULL;
                    }

                    // 步骤5：通过PoseOptimization对姿态进行优化求解
                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if (nGood < 10)
                        continue;

                    for (int io = 0; io < mCurrentFrame.N; io++)
                        if (mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    // 步骤6：如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，再进行优化求解
                    if (nGood < 50)
                    {
                        int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                        if (nadditional + nGood >= 50)
                        {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if (nGood > 30 && nGood < 50)
                            {
                                sFound.clear();
                                for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                    if (mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                                // Final optimization
                                if (nGood + nadditional >= 50)
                                {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for (int io = 0; io < mCurrentFrame.N; io++)
                                        if (mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io] = NULL;
                                }
                            }
                        }
                    }

                    // If the pose is supported by enough inliers stop ransacs and continue
                    if (nGood >= 50)
                    {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        if (!bMatch)
        {
            return false;
        }
        else
        {
            mnLastRelocFrameId = mCurrentFrame.mnId;
            return true;
        }
    }

    void Tracking::Reset()
    {

        cout << "System Reseting" << endl;
        if (mpViewer)
        {
            mpViewer->RequestStop();
            while (!mpViewer->isStopped())
                usleep(3000);
        }

        // Reset Local Mapping
        cout << "Reseting Local Mapper...";
        mpLocalMapper->RequestReset();
        cout << " done" << endl;

        // Reset Loop Closing
        cout << "Reseting Loop Closing...";
        mpLoopClosing->RequestReset();
        cout << " done" << endl;

        // Clear BoW Database
        cout << "Reseting Database..."; 
        mpKeyFrameDB->clear();
        cout << " done" << endl;

        // Clear Map (this erase MapPoints and KeyFrames)
        mpMap->clear();

        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;

        // if (mpInitializer)
        // {
        //     delete mpInitializer;
        //     mpInitializer = static_cast<Initializer *>(NULL);
        // }

        mlRelativeFramePoses.clear();
        mlpReferences.clear();
        mlFrameTimes.clear();
        mlbLost.clear();

        if (mpViewer)
            mpViewer->Release();
    }

    void Tracking::ChangeCalibratingtion(const string &strSettingPath)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0)                                                            //! this code need to be changed
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        Frame::mbInitialComputations = true;
    }

    void Tracking::InformOnlyTracking(const bool &flag)
    {
        mbOnlyTracking = flag;
    }

} // namespace Camvox
