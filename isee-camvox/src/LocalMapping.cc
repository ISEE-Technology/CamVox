#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
                   
#include <mutex>    
#include <chrono>                                      
               
namespace Camvox
{
    LocalMapping::LocalMapping(Map *pMap, const float bMonocular) : mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
                                                                    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
    {
    }

    void LocalMapping::SetLoopCloser(LoopClosing *pLoopCloser) //system中调用
    {
        mpLoopCloser = pLoopCloser;
    }

    void LocalMapping::SetTracker(Tracking *pTracker)
    {
        mpTracker = pTracker;
    }

    void LocalMapping::SetCalibratingter(Calibrating *pCalibratingter)
    {
        mpCalibratingter = pCalibratingter;
    }

    void LocalMapping::Run()
    {

        mbFinished = false;

        while (1)
        {
            
            // Tracking will see that Local Mapping is busy
            // 告诉Tracking，LocalMapping正处于繁忙状态，
            // LocalMapping线程处理的关键帧都是Tracking线程发过的
            // 在LocalMapping线程还没有处理完关键帧之前Tracking线程最好不要发送太快
            SetAcceptKeyFrames(false);                                                               //! Tracking should not send KF too fast

            // Check if there are keyframes in the queue
            // 等待处理的关键帧列表不为空
            if (CheckNewKeyFrames())
            {   
                // auto t2 = std::chrono::steady_clock::now();

                // BoW conversion and insertion in Map
                // VI-A keyframe insertion
                // 计算关键帧特征点的BoW映射，将关键帧插入地图
                ProcessNewKeyFrame();

                // Check recent MapPoints
                // VI-B recent map points culling
                // 剔除ProcessNewKeyFrame函数中引入的不合格MapPoints
                MapPointCulling();

                // Triangulate new MapPoints
                // VI-C new map points creation
                // 相机运动过程中与相邻关键帧通过三角化恢复出一些MapPoints
                CreateNewMapPoints();

                // 已经处理完队列中的最后的一个关键帧
                if (!CheckNewKeyFrames())
                {
                    // Find more matches in neighbor keyframes and fuse point duplications
                    // 检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints
                    SearchInNeighbors();
                }

                // 已经处理完队列中的最后的一个关键帧，并且闭环检测没有请求停止LocalMapping            //! Local BA --- process the final KF and LoopClosing not requeste stop
                mbAbortBA = false;

                if (!CheckNewKeyFrames() && !stopRequested())
                {
                    //  VI-D Local BA
                    if (mpMap->KeyFramesInMap() > 2)
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap);

                    // Check redundant local Keyframes
                    // VI-E local keyframes culling
                    //! 检测并剔除当前帧相邻的关键帧中冗余的关键帧
                    // 剔除的标准是：该关键帧的90%的MapPoints可以被其它关键帧观测到
                    // trick!
                    // Tracking中先把关键帧交给LocalMapping线程
                    // 并且在Tracking中InsertKeyFrame函数的条件比较松，交给LocalMapping线程的关键帧会比较密
                    // 在这里再删除冗余的关键帧
                    KeyFrameCulling();                                                       //! 90% MapPoints was seen, need delete the KF
                }

                // 将当前帧加入到闭环检测队列中
                mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);                                                      //! there has get into LoopClosing thread

            }

            else if (Stop())
            {
                // Safe area to stop
                while (isStopped() && !CheckFinish())
                {
                    usleep(3000); //std::this_thread::sleep_for(std::chrono::milliseconds(3));
                }
                if (CheckFinish())
                    break;
            }
            
            ResetIfRequested();

            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(true);

            if (CheckFinish())
                break;

            usleep(3000);
        }

        SetFinish();
    }
    
    /**
     * @brief 插入关键帧
     *
     * 将关键帧插入到地图中，以便将来进行局部地图优化
     * 这里仅仅是将关键帧插入到列表中进行等待
     * @param pKF KeyFrame
     */
    void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mlNewKeyFrames.push_back(pKF);
        mbAbortBA = true;
    }

    /**
     * @brief 查看列表中是否有等待被插入的关键帧
     * @return 如果存在，返回true
     */
    bool LocalMapping::CheckNewKeyFrames()
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        return (!mlNewKeyFrames.empty());
    }      

    /**
     * @brief 处理列表中的关键帧
     * 
     * - 计算Bow，加速三角化新的MapPoints
     * - 关联当前关键帧至MapPoints，并更新MapPoints的平均观测方向和观测距离范围
     * - 插入关键帧，更新Covisibility图和Essential图
     * @see VI-A keyframe insertion
     */
    void LocalMapping::ProcessNewKeyFrame()
    {
        // 步骤1：从缓冲队列中取出一帧关键帧
        // Tracking线程向LocalMapping中插入关键帧存在该队列中
        {
            unique_lock<mutex> lock(mMutexNewKFs);
            // 从列表中获得一个等待被插入的关键帧
            mpCurrentKeyFrame = mlNewKeyFrames.front();
            mlNewKeyFrames.pop_front();
        }

        // Compute Bags of Words structures
        // 步骤2：计算该关键帧特征点的Bow映射关系
        mpCurrentKeyFrame->ComputeBoW();

        // Associate MapPoints to the new keyframe and update normal and descriptor
        // 步骤3：跟踪局部地图过程中新匹配上的MapPoints和当前关键帧绑定
        // 在TrackLocalMap函数中将局部地图中的MapPoints与当前帧进行了匹配，
        // 但没有对这些匹配上的MapPoints与当前帧进行关联
        const vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

        for (size_t i = 0; i < vpMapPointMatches.size(); i++)
        {
            MapPoint *pMP = vpMapPointMatches[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    // 非当前帧生成的MapPoints
                    // 为当前帧在tracking过程跟踪到的MapPoints更新属性
                    if (!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                    {
                        // 添加观测
                        pMP->AddObservation(mpCurrentKeyFrame, i);
                        // 获得该点的平均观测方向和观测距离范围
                        pMP->UpdateNormalAndDepth();
                        // 加入关键帧后，更新3d点的最佳描述子
                        pMP->ComputeDistinctiveDescriptors();
                    }
                    else // this can only happen for new stereo points inserted by the Tracking
                    {
                        // 当前帧生成的MapPoints
                        // 将双目或RGBD跟踪过程中新插入的MapPoints放入mlpRecentAddedMapPoints，等待检查
                        // CreateNewMapPoints函数中通过三角化也会生成MapPoints
                        // 这些MapPoints都会经过MapPointCulling函数的检验
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }
            }
        }

        // Update links in the Covisibility Graph
        // 步骤4：更新关键帧间的连接关系，Covisibility图和Essential图(tree)
        mpCurrentKeyFrame->UpdateConnections();

        // Insert Keyframe in Map
        // 步骤5：将该关键帧插入到地图中
        mpMap->AddKeyFrame(mpCurrentKeyFrame);
    }

    /**
     * @brief 剔除ProcessNewKeyFrame和CreateNewMapPoints函数中引入的质量不好的MapPoints
     * @see VI-B recent map points culling
     */
    void LocalMapping::MapPointCulling()
    {
        // Check Recent Added MapPoints
        list<MapPoint *>::iterator lit = mlpRecentAddedMapPoints.begin();
        const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

        int nThObs;
        if (mbMonocular)
            nThObs = 2;
        else
            nThObs = 3;
        const int cnThObs = nThObs;

        // 遍历等待检查的MapPoints
        while (lit != mlpRecentAddedMapPoints.end())
        {
            MapPoint *pMP = *lit;
            if (pMP->isBad())
            {
                // 步骤1：已经是坏点的MapPoints直接从检查链表中删除
                lit = mlpRecentAddedMapPoints.erase(lit);
            }
            else if (pMP->GetFoundRatio() < 0.25f)
            {
                // 步骤2：将不满足VI-B条件的MapPoint剔除
                // VI-B 条件1：
                // 跟踪到该MapPoint的Frame数相比预计可观测到该MapPoint的Frame数的比例需大于25%
                // IncreaseFound / IncreaseVisible < 25%，注意不一定是关键帧。
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            }
            else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 2 && pMP->Observations() <= cnThObs)
            {
                // 步骤3：将不满足VI-B条件的MapPoint剔除
                // VI-B 条件2：从该点建立开始，到现在已经过了不小于2个关键帧
                // 但是观测到该点的关键帧数却不超过cnThObs帧，那么该点检验不合格
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            }
            else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 3)
                // 步骤4：从建立该点开始，已经过了3个关键帧而没有被剔除，则认为是质量高的点
                // 因此没有SetBadFlag()，仅从队列中删除，放弃继续对该MapPoint的检测
                lit = mlpRecentAddedMapPoints.erase(lit);
            else
                lit++;
        }
    }

    /**
     * 相机运动过程中和共视程度比较高的关键帧通过三角化恢复出一些MapPoints
     */
    void LocalMapping::CreateNewMapPoints()
    {
        // Retrieve neighbor keyframes in covisibility graph
        int nn = 10;
        if (mbMonocular)
            nn = 20;
        // 步骤1：在当前关键帧的共视关键帧中找到共视程度最高的nn帧相邻帧vpNeighKFs
        const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

        ORBmatcher matcher(0.6, false);

        cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
        cv::Mat Rwc1 = Rcw1.t();
        cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
        cv::Mat Tcw1(3, 4, CV_32F);
        Rcw1.copyTo(Tcw1.colRange(0, 3));
        tcw1.copyTo(Tcw1.col(3));
        // 得到当前关键帧在世界坐标系中的坐标
        cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

        const float &fx1 = mpCurrentKeyFrame->fx;
        const float &fy1 = mpCurrentKeyFrame->fy;
        const float &cx1 = mpCurrentKeyFrame->cx;
        const float &cy1 = mpCurrentKeyFrame->cy;
        const float &invfx1 = mpCurrentKeyFrame->invfx;
        const float &invfy1 = mpCurrentKeyFrame->invfy;

        const float ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;

        int nnew = 0;

        // Search matches with epipolar restriction and triangulate
        // 步骤2：遍历相邻关键帧vpNeighKFs
        for (size_t i = 0; i < vpNeighKFs.size(); i++)
        {
            if (i > 0 && CheckNewKeyFrames())
                return;

            KeyFrame *pKF2 = vpNeighKFs[i];

            // Check first that baseline is not too short
            // 邻接的关键帧在世界坐标系中的坐标
            cv::Mat Ow2 = pKF2->GetCameraCenter();
            // 基线向量，两个关键帧间的相机位移
            cv::Mat vBaseline = Ow2 - Ow1;
            // 基线长度
            const float baseline = cv::norm(vBaseline);

            // 步骤3：判断相机运动的基线是不是足够长
            if (!mbMonocular)
            {
                // 如果是立体相机，关键帧间距太小时不生成3D点
                if (baseline < pKF2->mb)
                    continue;
            }
          
            // Compute Fundamental Matrix
            // 步骤4：根据两个关键帧的位姿计算它们之间的基本矩阵
            cv::Mat F12 = ComputeF12(mpCurrentKeyFrame, pKF2);

            // Search matches that fullfil epipolar constraint
            // 步骤5：通过极线约束限制匹配时的搜索范围，进行特征点匹配
            vector<pair<size_t, size_t>> vMatchedIndices;
            matcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, F12, vMatchedIndices, false);

            cv::Mat Rcw2 = pKF2->GetRotation();
            cv::Mat Rwc2 = Rcw2.t();
            cv::Mat tcw2 = pKF2->GetTranslation();
            cv::Mat Tcw2(3, 4, CV_32F);
            Rcw2.copyTo(Tcw2.colRange(0, 3));
            tcw2.copyTo(Tcw2.col(3));

            const float &fx2 = pKF2->fx;
            const float &fy2 = pKF2->fy;
            const float &cx2 = pKF2->cx;
            const float &cy2 = pKF2->cy;
            const float &invfx2 = pKF2->invfx;
            const float &invfy2 = pKF2->invfy;

            // Triangulate each match
            // 步骤6：对每对匹配通过三角化生成3D点,he Triangulate函数差不多
            const int nmatches = vMatchedIndices.size();
            for (int ikp = 0; ikp < nmatches; ikp++)
            {
                // 步骤6.1：取出匹配特征点

                // 当前匹配对在当前关键帧中的索引
                const int &idx1 = vMatchedIndices[ikp].first;
                // 当前匹配对在邻接关键帧中的索引
                const int &idx2 = vMatchedIndices[ikp].second;

                // 当前匹配在当前关键帧中的特征点
                const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
                // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
                const float kp1_ur = mpCurrentKeyFrame->mvuRight[idx1];
                bool bStereo1 = kp1_ur >= 0;

                // 当前匹配在邻接关键帧中的特征点
                const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
                // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
                const float kp2_ur = pKF2->mvuRight[idx2];
                bool bStereo2 = kp2_ur >= 0;

                // Check parallax between rays
                // 步骤6.2：利用匹配点反投影得到视差角
                // 特征点反投影
                cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1) * invfx1, (kp1.pt.y - cy1) * invfy1, 1.0);
                cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx2) * invfx2, (kp2.pt.y - cy2) * invfy2, 1.0);

                // 由相机坐标系转到世界坐标系，得到视差角余弦值
                cv::Mat ray1 = Rwc1 * xn1;
                cv::Mat ray2 = Rwc2 * xn2;
                const float cosParallaxRays = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));

                // 加1是为了让cosParallaxStereo随便初始化为一个很大的值
                float cosParallaxStereo = cosParallaxRays + 1;
                float cosParallaxStereo1 = cosParallaxStereo;
                float cosParallaxStereo2 = cosParallaxStereo;

                // 步骤6.3：对于双目，利用双目得到视差角
                if (bStereo1) //双目，且有深度
                    cosParallaxStereo1 = cos(2 * atan2(mpCurrentKeyFrame->mb / 2, mpCurrentKeyFrame->mvDepth[idx1]));
                else if (bStereo2) //双目，且有深度
                    cosParallaxStereo2 = cos(2 * atan2(pKF2->mb / 2, pKF2->mvDepth[idx2]));

                // 得到双目观测的视差角
                cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

                // 步骤6.4：三角化恢复3D点
                cv::Mat x3D;
                // cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998)表明视差角正常
                // cosParallaxRays<cosParallaxStereo表明视差角很小
                // 视差角度小时用三角法恢复3D点，视差角大时用双目恢复3D点（双目以及深度有效）
                if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 && (bStereo1 || bStereo2 || cosParallaxRays < 0.9998))
                {
                    // Linear Triangulation Method
                    // 见Initializer.cpp的Triangulate函数
                    cv::Mat A(4, 4, CV_32F);
                    A.row(0) = xn1.at<float>(0) * Tcw1.row(2) - Tcw1.row(0);
                    A.row(1) = xn1.at<float>(1) * Tcw1.row(2) - Tcw1.row(1);
                    A.row(2) = xn2.at<float>(0) * Tcw2.row(2) - Tcw2.row(0);
                    A.row(3) = xn2.at<float>(1) * Tcw2.row(2) - Tcw2.row(1);

                    cv::Mat w, u, vt;
                    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

                    x3D = vt.row(3).t();

                    if (x3D.at<float>(3) == 0)
                        continue;

                    // Euclidean coordinates
                    x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
                }
                else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2)
                {
                    x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
                }
                else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1)
                {
                    x3D = pKF2->UnprojectStereo(idx2);
                }
                else
                    continue; //No stereo and very low parallax

                cv::Mat x3Dt = x3D.t();

                //Check triangulation in front of cameras
                // 步骤6.5：检测生成的3D点是否在相机前方
                float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
                if (z1 <= 0)
                    continue;

                float z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);
                if (z2 <= 0)
                    continue;

                //Check reprojection error in first keyframe
                // 步骤6.6：计算3D点在当前关键帧下的重投影误差
                const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
                const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1.at<float>(0);
                const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1.at<float>(1);
                const float invz1 = 1.0 / z1;

                if (!bStereo1)
                {
                    float u1 = fx1 * x1 * invz1 + cx1;
                    float v1 = fy1 * y1 * invz1 + cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
                    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
                        continue;
                }
                else
                {
                    float u1 = fx1 * x1 * invz1 + cx1;
                    float u1_r = u1 - mpCurrentKeyFrame->mbf * invz1;
                    float v1 = fy1 * y1 * invz1 + cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    float errX1_r = u1_r - kp1_ur;
                    if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) > 7.8 * sigmaSquare1)
                        continue;
                }

                //Check reprojection error in second keyframe
                // 计算3D点在另一个关键帧下的重投影误差
                const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
                const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2.at<float>(0);
                const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2.at<float>(1);
                const float invz2 = 1.0 / z2;
                if (!bStereo2)
                {
                    float u2 = fx2 * x2 * invz2 + cx2;
                    float v2 = fy2 * y2 * invz2 + cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
                        continue;
                }
                else
                {
                    float u2 = fx2 * x2 * invz2 + cx2;
                    float u2_r = u2 - mpCurrentKeyFrame->mbf * invz2;
                    float v2 = fy2 * y2 * invz2 + cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    float errX2_r = u2_r - kp2_ur;
                    // 基于卡方检验计算出的阈值（假设测量有一个一个像素的偏差）
                    if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) > 7.8 * sigmaSquare2)
                        continue;
                }

                //Check scale consistency
                // 步骤6.7：检查尺度连续性

                // 世界坐标系下，3D点与相机间的向量，方向由相机指向3D点
                cv::Mat normal1 = x3D - Ow1;
                float dist1 = cv::norm(normal1);

                cv::Mat normal2 = x3D - Ow2;
                float dist2 = cv::norm(normal2);

                if (dist1 == 0 || dist2 == 0)
                    continue;

                // ratioDist是不考虑金字塔尺度下的距离比例
                const float ratioDist = dist2 / dist1;
                // 金字塔尺度因子的比例
                const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave] / pKF2->mvScaleFactors[kp2.octave];

                /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
                // ratioDist*ratioFactor < ratioOctave 或 ratioDist/ratioOctave > ratioFactor表明尺度变化是连续的
                if (ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor)
                    continue;

                // Triangulation is succesfull
                // 步骤6.8：三角化生成3D点成功，构造成MapPoint
                MapPoint *pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpMap);

                // 步骤6.9：为该MapPoint添加属性：
                // a.观测到该MapPoint的关键帧
                // b.该MapPoint的描述子
                // c.该MapPoint的平均观测方向和深度范围
                pMP->AddObservation(mpCurrentKeyFrame, idx1);
                pMP->AddObservation(pKF2, idx2);

                mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
                pKF2->AddMapPoint(pMP, idx2);

                pMP->ComputeDistinctiveDescriptors();

                pMP->UpdateNormalAndDepth();

                mpMap->AddMapPoint(pMP);
                // 步骤6.8：将新产生的点放入检测队列
                // 这些MapPoints都会经过MapPointCulling函数的检验
                mlpRecentAddedMapPoints.push_back(pMP);

                nnew++;
            }
        }
    }
    
    /**
     * 检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints
     */
    void LocalMapping::SearchInNeighbors()
    {
        // Retrieve neighbor keyframes
        // 步骤1：获得当前关键帧在covisibility图中权重排名前nn的邻接关键帧
        // 找到当前帧一级相邻与二级相邻关键帧
        int nn = 10;
        if (mbMonocular)
            nn = 20;
        const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
        vector<KeyFrame *> vpTargetKFs;
        for (vector<KeyFrame *>::const_iterator vit = vpNeighKFs.begin(), vend = vpNeighKFs.end(); vit != vend; vit++)
        {
            KeyFrame *pKFi = *vit;
            if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi);                       // 加入一级相邻帧
            pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId; // 并标记已经加入

            // Extend to some second neighbors
            const vector<KeyFrame *> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
            for (vector<KeyFrame *>::const_iterator vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end(); vit2 != vend2; vit2++)
            {
                KeyFrame *pKFi2 = *vit2;
                if (pKFi2->isBad() || pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->mnId || pKFi2->mnId == mpCurrentKeyFrame->mnId)
                    continue;
                vpTargetKFs.push_back(pKFi2); // 存入二级相邻帧
            }
        }

        // Search matches by projection from current KF in target KFs
        ORBmatcher matcher;
        // 步骤2：将当前帧的MapPoints分别与一级二级相邻帧(的MapPoints)进行融合
        vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for (vector<KeyFrame *>::iterator vit = vpTargetKFs.begin(), vend = vpTargetKFs.end(); vit != vend; vit++)
        {
            KeyFrame *pKFi = *vit;

            // 投影当前帧的MapPoints到相邻关键帧pKFi中，并判断是否有重复的MapPoints
            // 1.如果MapPoint能匹配关键帧的特征点，并且该点有对应的MapPoint，那么将两个MapPoint合并（选择观测数多的）
            // 2.如果MapPoint能匹配关键帧的特征点，并且该点没有对应的MapPoint，那么为该点添加MapPoint
            matcher.Fuse(pKFi, vpMapPointMatches);
        }

        // Search matches by projection from target KFs in current KF
        // 用于存储一级邻接和二级邻接关键帧所有MapPoints的集合
        vector<MapPoint *> vpFuseCandidates;
        vpFuseCandidates.reserve(vpTargetKFs.size() * vpMapPointMatches.size());

        // 步骤3：将一级二级相邻帧的MapPoints分别与当前帧（的MapPoints）进行融合
        // 遍历每一个一级邻接和二级邻接关键帧
        for (vector<KeyFrame *>::iterator vitKF = vpTargetKFs.begin(), vendKF = vpTargetKFs.end(); vitKF != vendKF; vitKF++)
        {
            KeyFrame *pKFi = *vitKF;

            vector<MapPoint *> vpMapPointsKFi = pKFi->GetMapPointMatches();

            // 遍历当前一级邻接和二级邻接关键帧中所有的MapPoints
            for (vector<MapPoint *>::iterator vitMP = vpMapPointsKFi.begin(), vendMP = vpMapPointsKFi.end(); vitMP != vendMP; vitMP++)
            {
                MapPoint *pMP = *vitMP;
                if (!pMP)
                    continue;
                // 判断MapPoints是否为坏点，或者是否已经加进集合vpFuseCandidates
                if (pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                    continue;
                // 加入集合，并标记已经加入
                pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
                vpFuseCandidates.push_back(pMP);
            }
        }

        matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);

        // Update points
        // 步骤4：更新当前帧MapPoints的描述子，深度，观测主方向等属性
        vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for (size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++)
        {
            MapPoint *pMP = vpMapPointMatches[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    // 在所有找到pMP的关键帧中，获得最佳的描述子
                    pMP->ComputeDistinctiveDescriptors();
                    // 更新平均观测方向和观测距离
                    pMP->UpdateNormalAndDepth();
                }
            }
        }

        // Update connections in covisibility graph
        // 步骤5：更新当前帧的MapPoints后更新与其它帧的连接关系
        // 更新covisibility图
        mpCurrentKeyFrame->UpdateConnections();
    }

    /**
     * 根据两关键帧的姿态计算两个关键帧之间的基本矩阵
     * @param  pKF1 关键帧1
     * @param  pKF2 关键帧2
     * @return      基本矩阵
     */
    cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
    {
        // Essential Matrix: t12叉乘R12
        // Fundamental Matrix: inv(K1)*E*inv(K2)
        cv::Mat R1w = pKF1->GetRotation();
        cv::Mat t1w = pKF1->GetTranslation();
        cv::Mat R2w = pKF2->GetRotation();
        cv::Mat t2w = pKF2->GetTranslation();

        cv::Mat R12 = R1w * R2w.t();
        cv::Mat t12 = -R1w * R2w.t() * t2w + t1w;

        cv::Mat t12x = SkewSymmetricMatrix(t12);

        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pKF2->mK;

        return K1.t().inv() * t12x * R12 * K2.inv();
    }

    void LocalMapping::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopRequested = true;
        unique_lock<mutex> lock2(mMutexNewKFs);
        mbAbortBA = true;
    }

    bool LocalMapping::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if (mbStopRequested && !mbNotStop)
        {
            mbStopped = true;
            cout << "Local Mapping STOP" << endl;
            return true;
        }

        return false;
    }

    bool LocalMapping::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool LocalMapping::stopRequested()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopRequested;
    }

    void LocalMapping::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);
        if (mbFinished)
            return;
        mbStopped = false;
        mbStopRequested = false;
        for (list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++)
            delete *lit;
        mlNewKeyFrames.clear();

        cout << "Local Mapping RELEASE" << endl;
    }

    bool LocalMapping::AcceptKeyFrames()
    {
        unique_lock<mutex> lock(mMutexAccept);
        return mbAcceptKeyFrames;
    }

    void LocalMapping::SetAcceptKeyFrames(bool flag)
    {
        unique_lock<mutex> lock(mMutexAccept);
        mbAcceptKeyFrames = flag;
    }

    bool LocalMapping::SetNotStop(bool flag)
    {
        unique_lock<mutex> lock(mMutexStop);

        if (flag && mbStopped)
            return false;

        mbNotStop = flag;

        return true;
    }

    void LocalMapping::InterruptBA()
    {
        mbAbortBA = true;
    }

    /**
     * @brief 关键帧剔除
     * 
     * 在Covisibility Graph中的关键帧，其90%以上的MapPoints能被其他关键帧（至少3个）观测到，则认为该关键帧为冗余关键帧。
     * @see VI-E Local Keyframe Culling
     */
    void LocalMapping::KeyFrameCulling()
    {
        // Check redundant keyframes (only local keyframes)
        // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
        // in at least other 3 keyframes (in the same or finer scale)
        // We only consider close stereo points
        // 步骤1：根据Covisibility Graph提取当前帧的共视关键帧
        vector<KeyFrame *> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

        // 对所有的局部关键帧进行遍历
        for (vector<KeyFrame *>::iterator vit = vpLocalKeyFrames.begin(), vend = vpLocalKeyFrames.end(); vit != vend; vit++)
        {
            KeyFrame *pKF = *vit;
            if (pKF->mnId == 0)
                continue;
            // 步骤2：提取每个共视关键帧的MapPoints
            const vector<MapPoint *> vpMapPoints = pKF->GetMapPointMatches();

            int nObs = 3;
            const int thObs = nObs;
            int nRedundantObservations = 0;
            int nMPs = 0;
            // 步骤3：遍历该局部关键帧的MapPoints，判断是否90%以上的MapPoints能被其它关键帧（至少3个）观测到
            for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++)
            {
                MapPoint *pMP = vpMapPoints[i];
                if (pMP)
                {
                    if (!pMP->isBad())
                    {
                        if (!mbMonocular)
                        {
                            // 对于双目，仅考虑近处的MapPoints，不超过mbf * 35 / fx
                            if (pKF->mvDepth[i] > pKF->mThDepth || pKF->mvDepth[i] < 0)
                                continue;
                        }

                        nMPs++;
                        // MapPoints至少被三个关键帧观测到
                        if (pMP->Observations() > thObs)
                        {
                            const int &scaleLevel = pKF->mvKeysUn[i].octave;
                            const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                            // 判断该MapPoint是否同时被三个关键帧观测到
                            int nObs = 0;
                            for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
                            {
                                KeyFrame *pKFi = mit->first;
                                if (pKFi == pKF)
                                    continue;
                                const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                                // Scale Condition
                                // 尺度约束，要求MapPoint在该局部关键帧的特征尺度大于（或近似于）其它关键帧的特征尺度
                                if (scaleLeveli <= scaleLevel + 1)
                                {
                                    nObs++;
                                    // 已经找到三个同尺度的关键帧可以观测到该MapPoint，不用继续找了
                                    if (nObs >= thObs)
                                        break;
                                }
                            }
                            // 该MapPoint至少被三个关键帧观测到
                            if (nObs >= thObs)
                            {
                                nRedundantObservations++;
                            }
                        }
                    }
                }
            }

            // 步骤4：该局部关键帧90%以上的MapPoints能被其它关键帧（至少3个）观测到，则认为是冗余关键帧
            if (nRedundantObservations > 0.9 * nMPs)
                pKF->SetBadFlag();
        }
    }

    cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
    {
        return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1),
                v.at<float>(2), 0, -v.at<float>(0),
                -v.at<float>(1), v.at<float>(0), 0);
    }

    void LocalMapping::RequestReset()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetRequested = true;
        }

        while (1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if (!mbResetRequested)
                    break;
            }
            usleep(3000);
        }
    }

    void LocalMapping::ResetIfRequested()
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbResetRequested)
        {
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            mbResetRequested = false;
        }
    }

    void LocalMapping::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool LocalMapping::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void LocalMapping::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
        unique_lock<mutex> lock2(mMutexStop);
        mbStopped = true;
    }

    bool LocalMapping::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }
} // namespace Camvox
