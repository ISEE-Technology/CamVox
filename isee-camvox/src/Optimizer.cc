#include "Optimizer.h"
#include "Converter.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include <Eigen/StdVector>
#include <mutex>

namespace Camvox
{
    // pMap中所有的MapPoints和关键帧做bundle adjustment优化
    // 这个全局BA优化在本程序中有两个地方使用：
    // a.单目初始化：CreateInitialMapMonocular函数                                                                                          //! GBA also was used as MonocularInitializer  
    // b.闭环优化：RunGlobalBundleAdjustment函数  
    void Optimizer::GlobalBundleAdjustemnt(Map *pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    {
        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
        BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
    }

    /**
     * @brief bundle adjustment Optimization
     * 
     * 3D-2D 最小化重投影误差 e = (u,v) - project(Tcw*Pw) \n
     * 
     * 1. Vertex: g2o::VertexSE3Expmap()，即当前帧的Tcw
     *            g2o::VertexSBAPointXYZ()，MapPoint的mWorldPos
     * 2. Edge:
     *     - g2o::EdgeSE3ProjectXYZ()，BaseBinaryEdge
     *         + Vertex：待优化当前帧的Tcw
     *         + Vertex：待优化MapPoint的mWorldPos
     *         + measurement：MapPoint在当前帧中的二维位置(u,v)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *         
     * @param   vpKFs    关键帧 
     *          vpMP     MapPoints
     *          nIterations 迭代次数（20次）
     *          pbStopFlag  是否强制暂停
     *          nLoopKF  关键帧的个数
     *          bRobust  是否使用核函数
     */
    void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                     int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    {
        vector<bool> vbNotIncludedMP;
        vbNotIncludedMP.resize(vpMP.size());
        //1.确定线性求解器
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;                              //优化6维，误差3维
        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>(); //
        //2.确定求解块维度
        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
        //3.确定求解算法
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        //4.设置优化器
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        long unsigned int maxKFid = 0;

        // 步骤2：向优化器添加顶点
        // Set KeyFrame vertices
        // 步骤2.1：向优化器添加关键帧位姿顶点
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose())); //四元数
            vSE3->setId(pKF->mnId);                                  //123456
            vSE3->setFixed(pKF->mnId == 0);                          //第一帧固定不优化
            optimizer.addVertex(vSE3);                               //添加点
            if (pKF->mnId > maxKFid)
                maxKFid = pKF->mnId;
        }

        const float thHuber2D = sqrt(5.99); //核函数参数
        const float thHuber3D = sqrt(7.815);

        // Set MapPoint vertices
        // 步骤2.2：向优化器添加MapPoints顶点
        for (size_t i = 0; i < vpMP.size(); i++)
        {
            MapPoint *pMP = vpMP[i];
            if (pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos())); //Eigen::Matrix<double,3,1>
            const int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true); //schur消元
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, size_t> observations = pMP->GetObservations(); //查找看到该关键点的所有帧，格式1,1  .2,1

            int nEdges = 0;
            //SET EDGES
            // 步骤3：向优化器添加投影边边
            for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(); mit != observations.end(); mit++)
            {

                KeyFrame *pKF = mit->first;
                if (pKF->isBad() || pKF->mnId > maxKFid)
                    continue;

                nEdges++;

                const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second]; //该mp在当前关键帧的特征点

                // 单目或RGBD相机
                if (pKF->mvuRight[mit->second] < 0) //只有双目才有值，其余都是-1，属于Frame成员变量
                {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();
                    //建立每个mp与帧的边 BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap> 2，边的类型，两个节点的类型
                    //g2o如下定义，所以0 1不能对换 不然报错
                    //    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
                    //    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
                    //_error = obs-cam_project(v1->estimate().map(v2->estimate()));
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));        //VertexSBAPointXYZ
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId))); //VertexSE3Expmap
                    e->setMeasurement(obs);                                                                      //Vector2d Eigen::Matrix<double,2,1>
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    if (bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber2D);
                    }

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;

                    optimizer.addEdge(e);
                }
                else // 双目相机
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKF->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    if (bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber3D);
                    }

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;
                    e->bf = pKF->mbf;

                    optimizer.addEdge(e);
                }
            }

            if (nEdges == 0)
            {
                optimizer.removeVertex(vPoint);
                vbNotIncludedMP[i] = true;
            }
            else
            {
                vbNotIncludedMP[i] = false;
            }
        }

        // Optimize!
        // 步骤4：开始优化
        optimizer.initializeOptimization();
        optimizer.optimize(nIterations);

        // Recover optimized data
        // 步骤5：得到优化的结果

        //Keyframes
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            if (nLoopKF == 0)
            {
                pKF->SetPose(Converter::toCvMat(SE3quat));
            }
            else
            {
                pKF->mTcwGBA.create(4, 4, CV_32F);                //KF中仅仅声明了，并未具体大小
                Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA); //更新后的位姿放入mTcwGBA
                pKF->mnBAGlobalForKF = nLoopKF;                   //第几次回环
            }
        }

        //Points
        for (size_t i = 0; i < vpMP.size(); i++)
        {
            if (vbNotIncludedMP[i])
                continue;

            MapPoint *pMP = vpMP[i];

            if (pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));

            if (nLoopKF == 0)
            {
                pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
                pMP->UpdateNormalAndDepth();
            }
            else
            {
                pMP->mPosGBA.create(3, 1, CV_32F);
                Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
                pMP->mnBAGlobalForKF = nLoopKF; //跟上同理
            }
        }
    }

    /**
     * @brief Pose Only Optimization
     * 位姿图优化
     * 3D-2D 最小化重投影误差 e = (u,v) - project(Tcw*Pw) \n
     * 只优化Frame的Tcw，不优化MapPoints的坐标
     * 
     * 1. Vertex: g2o::VertexSE3Expmap()，即当前帧的Tcw
     * 2. Edge:
     *     - g2o::EdgeSE3ProjectXYZOnlyPose()，BaseUnaryEdge
     *         + Vertex：待优化当前帧的Tcw
     *         + measurement：MapPoint在当前帧中的二维位置(u,v)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *     - g2o::EdgeStereoSE3ProjectXYZOnlyPose()，BaseUnaryEdge
     *         + Vertex：待优化当前帧的Tcw
     *         + measurement：MapPoint在当前帧中的二维位置(ul,v,ur)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *
     * @param   pFrame Frame
     * @return  inliers数量
     */
    int Optimizer::PoseOptimization(Frame *pFrame)
    {
        // 该优化函数主要用于Tracking线程中：运动跟踪、参考帧跟踪、地图跟踪、重定位

        // 步骤1：构造g2o优化器
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        int nInitialCorrespondences = 0;

        // Set Frame vertex
        // 步骤2：添加顶点：待优化当前帧的Tcw
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        vSE3->setId(0);
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);

        // Set MapPoint vertices
        const int N = pFrame->N;

        // for Monocular
        vector<g2o::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
        vector<size_t> vnIndexEdgeMono;
        vpEdgesMono.reserve(N);
        vnIndexEdgeMono.reserve(N);

        // for Stereo
        vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesStereo.reserve(N);
        vnIndexEdgeStereo.reserve(N);

        const float deltaMono = sqrt(5.991);
        const float deltaStereo = sqrt(7.815);

        // 步骤3：添加一元边：相机投影模型
        {
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            for (int i = 0; i < N; i++)
            {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (pMP)
                {
                    // Monocular observation
                    // 单目情况, 也有可能在双目下, 当前帧的左兴趣点找不到匹配的右兴趣点
                    if (pFrame->mvuRight[i] < 0)
                    {
                        nInitialCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                        //该优化优化的是 Tcw,根据给定的相机参数，点的三维坐标，投影到该帧，计算重投影误差
                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        rk->setDelta(deltaMono);
                        e->setRobustKernel(rk);

                        e->fx = pFrame->fx;
                        e->fy = pFrame->fy;
                        e->cx = pFrame->cx;
                        e->cy = pFrame->cy;
                        cv::Mat Xw = pMP->GetWorldPos();
                        e->Xw[0] = Xw.at<float>(0);
                        e->Xw[1] = Xw.at<float>(1);
                        e->Xw[2] = Xw.at<float>(2);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                    else // Stereo observation双目
                    {
                        nInitialCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        //SET EDGE
                        Eigen::Matrix<double, 3, 1> obs; // 这里和单目不同
                        const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                        const float &kp_ur = pFrame->mvuRight[i];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur; // 这里和单目不同

                        g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose(); // 这里和单目不同

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        rk->setDelta(deltaStereo);
                        e->setRobustKernel(rk);

                        e->fx = pFrame->fx;
                        e->fy = pFrame->fy;
                        e->cx = pFrame->cx;
                        e->cy = pFrame->cy;
                        e->bf = pFrame->mbf;
                        cv::Mat Xw = pMP->GetWorldPos();
                        e->Xw[0] = Xw.at<float>(0);
                        e->Xw[1] = Xw.at<float>(1);
                        e->Xw[2] = Xw.at<float>(2);

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }
                }
            }
        }

        if (nInitialCorrespondences < 3)
            return 0;

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        // 步骤4：开始优化，总共优化四次，每次优化后，将观测分为outlier和inlier，outlier不参与下次优化
        // 由于每次优化后是对所有的观测进行outlier和inlier判别，因此之前被判别为outlier有可能变成inlier，反之亦然
        // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）----不知道怎么算的，查表可知 下面两个分别是服从 2和3自由度的卡方分布 0.5的值
        const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
        const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
        const int its[4] = {10, 10, 10, 10}; // 四次迭代，每次迭代的次数

        int nBad = 0;
        for (size_t it = 0; it < 4; it++)
        {

            vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw)); //每次优化重新确定节点待优化数值
            optimizer.initializeOptimization(0);                   // 对level为0的边进行优化
            optimizer.optimize(its[it]);

            nBad = 0;
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                g2o::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

                const size_t idx = vnIndexEdgeMono[i]; //单目中，每个特征点都会连接边，这是每个边的特征点的id

                if (pFrame->mvbOutlier[idx]) //如果这个特征点是out了 计算他的误差
                {
                    e->computeError(); // NOTE g2o只会计算active edge的误差
                }

                const float chi2 = e->chi2(); //计算卡方

                if (chi2 > chi2Mono[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1); // 设置为outlier
                    nBad++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0); // 设置为inlier
                }

                if (it == 2)
                    e->setRobustKernel(0); // 除了前两次优化需要RobustKernel以外, 其余的优化都不需要
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

                const size_t idx = vnIndexEdgeStereo[i];

                if (pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Stereo[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    e->setLevel(0);
                    pFrame->mvbOutlier[idx] = false;
                }

                if (it == 2)
                    e->setRobustKernel(0);
            }

            if (optimizer.edges().size() < 10)
                break;
        }

        // Recover optimized pose and return number of inliers
        g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
        g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
        cv::Mat pose = Converter::toCvMat(SE3quat_recov);
        pFrame->SetPose(pose);

        return nInitialCorrespondences - nBad; //返回inliers个数
    }

    /**
     * @brief Local Bundle Adjustment
     *
     * 1. Vertex:
     *     - g2o::VertexSE3Expmap()，LocalKeyFrames，即当前关键帧的位姿、与当前关键帧相连的关键帧的位姿
     *     - g2o::VertexSE3Expmap()，FixedCameras，即能观测到LocalMapPoints的关键帧（并且不属于LocalKeyFrames）的位姿，在优化中这些关键帧的位姿不变
     *     - g2o::VertexSBAPointXYZ()，LocalMapPoints，即LocalKeyFrames能观测到的所有MapPoints的位置
     * 2. Edge:
     *     - g2o::EdgeSE3ProjectXYZ()，BaseBinaryEdge
     *         + Vertex：关键帧的Tcw，MapPoint的Pw
     *         + measurement：MapPoint在关键帧中的二维位置(u,v)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *     - g2o::EdgeStereoSE3ProjectXYZ()，BaseBinaryEdge
     *         + Vertex：关键帧的Tcw，MapPoint的Pw
     *         + measurement：MapPoint在关键帧中的二维位置(ul,v,ur)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *         
     * @param pKF        KeyFrame
     * @param pbStopFlag 是否停止优化的标志
     * @param pMap       在优化后，更新状态时需要用到Map的互斥量mMutexMapUpdate
     */
    void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap)
    {
        // 该优化函数用于LocalMapping线程的局部BA优化
        // Local KeyFrames: First Breath Search from Current Keyframe
        list<KeyFrame *> lLocalKeyFrames;

        // 步骤1：将当前关键帧加入lLocalKeyFrames
        lLocalKeyFrames.push_back(pKF);
        pKF->mnBALocalForKF = pKF->mnId;

        // 步骤2：找到关键帧连接的关键帧（一级相连），加入lLocalKeyFrames中
        const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
        for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
        {
            KeyFrame *pKFi = vNeighKFs[i];
            pKFi->mnBALocalForKF = pKF->mnId; //记录局部优化id，该数为不断变化，数值等于局部化的关键帧的id，该id用于防止重复添加
            if (!pKFi->isBad())
                lLocalKeyFrames.push_back(pKFi);
        }

        // Local MapPoints seen in Local KeyFrames
        // 步骤3：遍历lLocalKeyFrames中关键帧，将它们观测的MapPoints加入到lLocalMapPoints
        list<MapPoint *> lLocalMapPoints;
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
            for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
            {
                MapPoint *pMP = *vit;
                if (pMP)               //如果存在
                    if (!pMP->isBad()) //且木有被抛弃
                        if (pMP->mnBALocalForKF != pKF->mnId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF = pKF->mnId; // 防止重复添加
                        }
            }
        }

        // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
        // 步骤4：得到能被局部MapPoints观测到，但不属于局部关键帧的关键帧，这些关键帧在局部BA优化时不优化
        list<KeyFrame *> lFixedCameras;
        for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        {
            map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                // pKFi->mnBALocalForKF!=pKF->mnId表示局部关键帧，
                // 其它的关键帧虽然能观测到，但不属于局部关键帧
                if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
                {
                    pKFi->mnBAFixedForKF = pKF->mnId; // 防止重复添加
                    if (!pKFi->isBad())
                        lFixedCameras.push_back(pKFi);
                }
            }
        }

        // Setup optimizer
        // 步骤5：构造g2o优化器
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        unsigned long maxKFid = 0;

        // Set Local KeyFrame vertices
        // 步骤6：添加顶点：Pose of Local KeyFrame
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
            vSE3->setId(pKFi->mnId);
            vSE3->setFixed(pKFi->mnId == 0); //第一帧位置固定
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
        }

        // Set Fixed KeyFrame vertices
        // 步骤7：添加顶点：Pose of Fixed KeyFrame，注意这里调用了vSE3->setFixed(true)。
        for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
            vSE3->setId(pKFi->mnId);
            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
        }

        // Set MapPoint vertices
        // 步骤7：添加3D顶点
        const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

        vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        const float thHuberMono = sqrt(5.991);
        const float thHuberStereo = sqrt(7.815);

        for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        {
            // 添加顶点：MapPoint
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
            int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, size_t> observations = pMP->GetObservations();

            //Set edges
            // 步骤8：对每一对关联的MapPoint和KeyFrame构建边
            for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (!pKFi->isBad())
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                    // Monocular observation
                    if (pKFi->mvuRight[mit->second] < 0)
                    {
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }
                    else // Stereo observation
                    {
                        Eigen::Matrix<double, 3, 1> obs;
                        const float kp_ur = pKFi->mvuRight[mit->second];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;
                        e->bf = pKFi->mbf;

                        optimizer.addEdge(e);
                        vpEdgesStereo.push_back(e);
                        vpEdgeKFStereo.push_back(pKFi);
                        vpMapPointEdgeStereo.push_back(pMP);
                    }
                }
            }
        }

        if (pbStopFlag)
            if (*pbStopFlag)
                return;

        // 步骤9：开始优化
        optimizer.initializeOptimization();
        optimizer.optimize(5);

        bool bDoMore = true;

        if (pbStopFlag)
            if (*pbStopFlag)
                bDoMore = false;

        if (bDoMore)
        {

            // Check inlier observations
            // 步骤10：检测outlier，并设置下次不优化
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
                MapPoint *pMP = vpMapPointEdgeMono[i];

                if (pMP->isBad())
                    continue;

                // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
                if (e->chi2() > 5.991 || !e->isDepthPositive())
                {
                    e->setLevel(1); // 不优化
                }

                e->setRobustKernel(0); // 不使用核函数
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
                MapPoint *pMP = vpMapPointEdgeStereo[i];

                if (pMP->isBad())
                    continue;

                if (e->chi2() > 7.815 || !e->isDepthPositive())
                {
                    e->setLevel(1);
                }

                e->setRobustKernel(0);
            }

            // Optimize again without the outliers
            // 步骤11：排除误差较大的outlier后再次优化
            optimizer.initializeOptimization(0);
            optimizer.optimize(10);
        }

        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

        // Check inlier observations
        // 步骤12：在优化后重新计算误差，剔除连接误差比较大的关键帧和MapPoint
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // 连接偏差比较大，在关键帧中剔除对该MapPoint的观测
        // 连接偏差比较大，在MapPoint中剔除对该关键帧的观测
        if (!vToErase.empty())
        {
            for (size_t i = 0; i < vToErase.size(); i++)
            {
                KeyFrame *pKFi = vToErase[i].first;
                MapPoint *pMPi = vToErase[i].second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        // Recover optimized data
        // 步骤13：优化后更新关键帧位姿以及MapPoints的位置、平均观测方向等属性

        //Keyframes
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKF = *lit;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }

        //Points
        for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
    }

    /**
     * @brief 闭环检测后，EssentialGraph优化 
     *                                                                               
     * 1. Vertex:
     *     - g2o::VertexSim3Expmap，Essential graph中关键帧的位姿
     * 2. Edge:
     *     - g2o::EdgeSim3()，BaseBinaryEdge
     *         + Vertex：关键帧的Tcw，MapPoint的Pw
     *         + measurement：经过CorrectLoop函数步骤2，Sim3传播校正后的位姿
     *         + InfoMatrix: 单位矩阵     
     *
     * @param pMap               全局地图
     * @param pLoopKF            闭环匹配上的关键帧
     * @param pCurKF             当前关键帧
     * @param NonCorrectedSim3   未经过Sim3传播调整过的关键帧位姿
     * @param CorrectedSim3      经过Sim3传播调整过的关键帧位姿
     * @param LoopConnections    因闭环时MapPoints调整而新生成的边
     */
    void Optimizer::OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                           const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                           const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                           const map<KeyFrame *, set<KeyFrame *>> &LoopConnections, const bool &bFixScale)
    {
        // Setup optimizer
        // 步骤1：构造优化器
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);
        // 指定线性方程求解器使用Eigen的块求解器
        g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
            new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
        // 构造线性求解器
        g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
        // 使用LM算法进行非线性迭代
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        solver->setUserLambdaInit(1e-16);
        optimizer.setAlgorithm(solver);

        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

        const unsigned int nMaxKFid = pMap->GetMaxKFid();

        // 仅经过Sim3传播调整，未经过优化的keyframe的pose
        vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
        // 经过Sim3传播调整，经过优化的keyframe的pose
        vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);
        // 这个变量没有用
        vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

        const int minFeat = 100;

        // Set KeyFrame vertices
        // 步骤2：将地图中所有keyframe的pose作为顶点添加到优化器
        // 尽可能使用经过Sim3调整的位姿
        for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

            const int nIDi = pKF->mnId;

            LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

            // 如果该关键帧在闭环时通过Sim3传播调整过，用校正后的位姿
            if (it != CorrectedSim3.end())
            {
                vScw[nIDi] = it->second;
                VSim3->setEstimate(it->second);
            }
            else // 如果该关键帧在闭环时没有通过Sim3传播调整过，用自身的位姿
            {
                Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
                Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
                g2o::Sim3 Siw(Rcw, tcw, 1.0);
                vScw[nIDi] = Siw;
                VSim3->setEstimate(Siw);
            }

            // 闭环匹配上的帧不进行位姿优化
            if (pKF == pLoopKF)
                VSim3->setFixed(true);

            VSim3->setId(nIDi);
            VSim3->setMarginalized(false);
            VSim3->_fix_scale = bFixScale;

            optimizer.addVertex(VSim3);

            // 优化前的pose顶点，后面代码中没有使用
            vpVertices[nIDi] = VSim3;
        }

        set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

        const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

        // Set Loop edges
        // 步骤3：添加边：LoopConnections是闭环时因为MapPoints调整而出现的新关键帧连接关系（不是当前帧与闭环匹配帧之间的连接关系）
        for (map<KeyFrame *, set<KeyFrame *>>::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end(); mit != mend; mit++)
        {
            KeyFrame *pKF = mit->first;
            const long unsigned int nIDi = pKF->mnId;
            const set<KeyFrame *> &spConnections = mit->second;
            const g2o::Sim3 Siw = vScw[nIDi];
            const g2o::Sim3 Swi = Siw.inverse();

            for (set<KeyFrame *>::const_iterator sit = spConnections.begin(), send = spConnections.end(); sit != send; sit++)
            {
                const long unsigned int nIDj = (*sit)->mnId;
                if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && pKF->GetWeight(*sit) < minFeat)
                    continue;

                const g2o::Sim3 Sjw = vScw[nIDj];
                // 得到两个pose间的Sim3变换
                const g2o::Sim3 Sji = Sjw * Swi;

                g2o::EdgeSim3 *e = new g2o::EdgeSim3();
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                // 根据两个Pose顶点的位姿算出相对位姿作为边，那还存在误差？优化有用？因为闭环MapPoints调整新形成的边不优化？（wubo???）
                e->setMeasurement(Sji);

                e->information() = matLambda;

                optimizer.addEdge(e);

                sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
            }
        }

        // Set normal edges
        // 步骤4：添加跟踪时形成的边、闭环匹配成功形成的边
        for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
        {
            KeyFrame *pKF = vpKFs[i];

            const int nIDi = pKF->mnId;

            g2o::Sim3 Swi;

            LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

            // 尽可能得到未经过Sim3传播调整的位姿
            if (iti != NonCorrectedSim3.end())
                Swi = (iti->second).inverse();
            else
                Swi = vScw[nIDi].inverse();

            KeyFrame *pParentKF = pKF->GetParent();

            // Spanning tree edge
            // 步骤4.1：只添加扩展树的边（有父关键帧）
            if (pParentKF)
            {
                int nIDj = pParentKF->mnId;

                g2o::Sim3 Sjw;

                LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

                // 尽可能得到未经过Sim3传播调整的位姿
                if (itj != NonCorrectedSim3.end())
                    Sjw = itj->second;
                else
                    Sjw = vScw[nIDj];

                g2o::Sim3 Sji = Sjw * Swi;

                g2o::EdgeSim3 *e = new g2o::EdgeSim3();
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                e->setMeasurement(Sji);

                e->information() = matLambda;
                optimizer.addEdge(e);
            }

            // Loop edges
            // 步骤4.2：添加在CorrectLoop函数中AddLoopEdge函数添加的闭环连接边（当前帧与闭环匹配帧之间的连接关系）
            // 使用经过Sim3调整前关键帧之间的相对关系作为边
            const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
            for (set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++)
            {
                KeyFrame *pLKF = *sit;
                if (pLKF->mnId < pKF->mnId)
                {
                    g2o::Sim3 Slw;

                    LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                    // 尽可能得到未经过Sim3传播调整的位姿
                    if (itl != NonCorrectedSim3.end())
                        Slw = itl->second;
                    else
                        Slw = vScw[pLKF->mnId];

                    g2o::Sim3 Sli = Slw * Swi;
                    g2o::EdgeSim3 *el = new g2o::EdgeSim3();
                    el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
                    el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                    // 根据两个Pose顶点的位姿算出相对位姿作为边，那还存在误差？优化有用？（wubo???）
                    el->setMeasurement(Sli);
                    el->information() = matLambda;
                    optimizer.addEdge(el);
                }
            }

            // Covisibility graph edges
            // 步骤4.3：最有很好共视关系的关键帧也作为边进行优化
            // 使用经过Sim3调整前关键帧之间的相对关系作为边
            const vector<KeyFrame *> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
            for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++)
            {
                KeyFrame *pKFn = *vit;
                if (pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
                {
                    if (!pKFn->isBad() && pKFn->mnId < pKF->mnId)
                    {
                        if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId), max(pKF->mnId, pKFn->mnId))))
                            continue;

                        g2o::Sim3 Snw;

                        LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                        // 尽可能得到未经过Sim3传播调整的位姿
                        if (itn != NonCorrectedSim3.end())
                            Snw = itn->second;
                        else
                            Snw = vScw[pKFn->mnId];

                        g2o::Sim3 Sni = Snw * Swi;

                        g2o::EdgeSim3 *en = new g2o::EdgeSim3();
                        en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
                        en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                        en->setMeasurement(Sni);
                        en->information() = matLambda;
                        optimizer.addEdge(en);
                    }
                }
            }
        }

        // Optimize!
        // 步骤5：开始g2o优化
        optimizer.initializeOptimization();
        optimizer.optimize(20);

        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
        // 步骤6：设定优化后的位姿
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];

            const int nIDi = pKFi->mnId;

            g2o::VertexSim3Expmap *VSim3 = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
            g2o::Sim3 CorrectedSiw = VSim3->estimate();
            vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
            Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = CorrectedSiw.translation();
            double s = CorrectedSiw.scale();

            eigt *= (1. / s); //[R t/s;0 1]

            cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

            pKFi->SetPose(Tiw);
        }

        // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
        // 步骤7：步骤5和步骤6优化得到关键帧的位姿后，MapPoints根据参考帧优化前后的相对关系调整自己的位置
        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
        {
            MapPoint *pMP = vpMPs[i];

            if (pMP->isBad())
                continue;

            int nIDr;
            // 该MapPoint经过Sim3调整过，(LoopClosing.cpp，CorrectLoop函数，步骤2.2_
            if (pMP->mnCorrectedByKF == pCurKF->mnId)
            {
                nIDr = pMP->mnCorrectedReference;
            }
            else
            {
                // 通过情况下MapPoint的参考关键帧就是创建该MapPoint的那个关键帧
                KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
                nIDr = pRefKF->mnId;
            }

            // 得到MapPoint参考关键帧步骤5优化前的位姿
            g2o::Sim3 Srw = vScw[nIDr];
            // 得到MapPoint参考关键帧优化后的位姿
            g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

            cv::Mat P3Dw = pMP->GetWorldPos();
            Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
            Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

            cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
            pMP->SetWorldPos(cvCorrectedP3Dw);

            pMP->UpdateNormalAndDepth();
        }
    }

    /**
     * @brief 形成闭环时进行Sim3优化
     *
     * 1. Vertex:
     *     - g2o::VertexSim3Expmap()，两个关键帧的位姿
     *     - g2o::VertexSBAPointXYZ()，两个关键帧共有的MapPoints
     * 2. Edge:
     *     - g2o::EdgeSim3ProjectXYZ()，BaseBinaryEdge
     *         + Vertex：关键帧的Sim3，MapPoint的Pw
     *         + measurement：MapPoint在关键帧中的二维位置(u,v)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *     - g2o::EdgeInverseSim3ProjectXYZ()，BaseBinaryEdge
     *         + Vertex：关键帧的Sim3，MapPoint的Pw
     *         + measurement：MapPoint在关键帧中的二维位置(u,v)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *         
     * @param pKF1        KeyFrame
     * @param pKF2        KeyFrame
     * @param vpMatches1  两个关键帧的匹配关系
     * @param g2oS12      两个关键帧间的Sim3变换
     * @param th2         核函数阈值
     * @param bFixScale   是否优化尺度，弹目进行尺度优化，双目不进行尺度优化
     */
    int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
    {
        // 步骤1：初始化g2o优化器
        // 先构造求解器
        g2o::SparseOptimizer optimizer;
        // 构造线性方程求解器，Hx = -b的求解器
        g2o::BlockSolverX::LinearSolverType *linearSolver;
        // 使用dense的求解器，（常见非dense求解器有cholmod线性求解器和shur补线性求解器）
        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
        // 使用L-M迭代
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        // Calibratingtion
        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pKF2->mK;

        // Camera poses
        const cv::Mat R1w = pKF1->GetRotation();
        const cv::Mat t1w = pKF1->GetTranslation();
        const cv::Mat R2w = pKF2->GetRotation();
        const cv::Mat t2w = pKF2->GetTranslation();

        // Set Sim3 vertex
        // 步骤2.1 添加Sim3顶点
        g2o::VertexSim3Expmap *vSim3 = new g2o::VertexSim3Expmap();
        vSim3->_fix_scale = bFixScale;
        vSim3->setEstimate(g2oS12);
        vSim3->setId(0);
        vSim3->setFixed(false);                           // 优化Sim3顶点
        vSim3->_principle_point1[0] = K1.at<float>(0, 2); // 光心横坐标cx
        vSim3->_principle_point1[1] = K1.at<float>(1, 2); // 光心纵坐标cy
        vSim3->_focal_length1[0] = K1.at<float>(0, 0);    // 焦距 fx
        vSim3->_focal_length1[1] = K1.at<float>(1, 1);    // 焦距 fy
        vSim3->_principle_point2[0] = K2.at<float>(0, 2);
        vSim3->_principle_point2[1] = K2.at<float>(1, 2);
        vSim3->_focal_length2[0] = K2.at<float>(0, 0);
        vSim3->_focal_length2[1] = K2.at<float>(1, 1);
        optimizer.addVertex(vSim3);

        // Set MapPoint vertices
        const int N = vpMatches1.size();
        const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
        vector<g2o::EdgeSim3ProjectXYZ *> vpEdges12;        //pKF2对应的MapPoints到pKF1的投影
        vector<g2o::EdgeInverseSim3ProjectXYZ *> vpEdges21; //pKF1对应的MapPoints到pKF2的投影
        vector<size_t> vnIndexEdge;

        vnIndexEdge.reserve(2 * N);
        vpEdges12.reserve(2 * N);
        vpEdges21.reserve(2 * N);

        const float deltaHuber = sqrt(th2);

        int nCorrespondences = 0;

        for (int i = 0; i < N; i++)
        {
            if (!vpMatches1[i])
                continue;

            // pMP1和pMP2是匹配的MapPoints
            MapPoint *pMP1 = vpMapPoints1[i];
            MapPoint *pMP2 = vpMatches1[i];

            const int id1 = 2 * i + 1;
            const int id2 = 2 * (i + 1);

            const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

            if (pMP1 && pMP2)
            {
                if (!pMP1->isBad() && !pMP2->isBad() && i2 >= 0)
                {
                    // 步骤2.2 添加PointXYZ顶点
                    g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
                    cv::Mat P3D1w = pMP1->GetWorldPos();
                    cv::Mat P3D1c = R1w * P3D1w + t1w;
                    vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                    vPoint1->setId(id1);
                    vPoint1->setFixed(true);
                    optimizer.addVertex(vPoint1);

                    g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
                    cv::Mat P3D2w = pMP2->GetWorldPos();
                    cv::Mat P3D2c = R2w * P3D2w + t2w;
                    vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                    vPoint2->setId(id2);
                    vPoint2->setFixed(true);
                    optimizer.addVertex(vPoint2);
                }
                else
                    continue;
            }
            else
                continue;

            nCorrespondences++;

            // Set edge x1 = S12*X2
            Eigen::Matrix<double, 2, 1> obs1;
            const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
            obs1 << kpUn1.pt.x, kpUn1.pt.y;

            // 步骤2.3 添加两个顶点（3D点）到相机投影的边
            g2o::EdgeSim3ProjectXYZ *e12 = new g2o::EdgeSim3ProjectXYZ();
            e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
            e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e12->setMeasurement(obs1);
            const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
            e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

            g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
            e12->setRobustKernel(rk1);
            rk1->setDelta(deltaHuber);
            optimizer.addEdge(e12);

            // Set edge x2 = S21*X1
            Eigen::Matrix<double, 2, 1> obs2;
            const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
            obs2 << kpUn2.pt.x, kpUn2.pt.y;

            g2o::EdgeInverseSim3ProjectXYZ *e21 = new g2o::EdgeInverseSim3ProjectXYZ();

            e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
            e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e21->setMeasurement(obs2);
            float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
            e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

            g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
            e21->setRobustKernel(rk2);
            rk2->setDelta(deltaHuber);
            optimizer.addEdge(e21);

            vpEdges12.push_back(e12);
            vpEdges21.push_back(e21);
            vnIndexEdge.push_back(i);
        }

        // Optimize!
        // 步骤3：g2o开始优化，先迭代5次
        optimizer.initializeOptimization();
        optimizer.optimize(5);

        // 步骤4：剔除一些误差大的边
        // Check inliers
        // 进行卡方检验，大于阈值的边剔除
        int nBad = 0;
        for (size_t i = 0; i < vpEdges12.size(); i++)
        {
            g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21)
                continue;

            if (e12->chi2() > th2 || e21->chi2() > th2)
            {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = static_cast<MapPoint *>(NULL);
                optimizer.removeEdge(e12);
                optimizer.removeEdge(e21);
                vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ *>(NULL);
                vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ *>(NULL);
                nBad++;
            }
        }

        int nMoreIterations;
        if (nBad > 0)
            nMoreIterations = 10;
        else
            nMoreIterations = 5;

        if (nCorrespondences - nBad < 10)
            return 0;

        // Optimize again only with inliers
        // 步骤5：再次g2o优化剔除后剩下的边
        optimizer.initializeOptimization();
        optimizer.optimize(nMoreIterations);

        int nIn = 0;
        for (size_t i = 0; i < vpEdges12.size(); i++)
        {
            g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21)
                continue;

            if (e12->chi2() > th2 || e21->chi2() > th2)
            {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = static_cast<MapPoint *>(NULL);
            }
            else
                nIn++;
        }

        // Recover optimized Sim3
        // 步骤6：得到优化后的结果
        g2o::VertexSim3Expmap *vSim3_recov = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
        g2oS12 = vSim3_recov->estimate();

        return nIn;
    }
} // namespace Camvox