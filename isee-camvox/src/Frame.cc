//初始化帧，包括相机参数等，计算畸变，并且计算特征点描述子放入网格
#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace Camvox
{
    //定义并初始化静态成员变量，对于所有该类，以下变量共享
    long unsigned int Frame::nNextId = 0;
    bool Frame::mbInitialComputations = true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

    Frame::Frame()
    {
        
    }

    //Copy Constructor
    Frame::Frame(const Frame &frame)
        : mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
          mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
          mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
          mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn), mvuRight(frame.mvuRight),
          mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
          mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
          mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
          mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
          mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
          mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
          mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
    {
        for (int i = 0; i < FRAME_GRID_COLS; i++)
            for (int j = 0; j < FRAME_GRID_ROWS; j++)
                mGrid[i][j] = frame.mGrid[i][j]; //vector变量类型，std::size_t

        if (!frame.mTcw.empty())
            SetPose(frame.mTcw);
    }

    // RGBD初始化
    Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
        : mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
          mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info                                                                    //! get scale 
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        ExtractORB(0, imGray); //计算特征点及描述子                                                                              //!  ORB Extractor  

        N = mvKeys.size();

        if (mvKeys.empty())
            return;
        // 调用OpenCV的矫正函数矫正orb提取的特征点
        UndistortKeyPoints();
        //计算特征点深度
        ComputeStereoFromRGBD(imDepth);                                                          //!  compute the keypoints depth
        //定义N维向量，并赋值为后面的数
        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL)); //NULL强制转化为MapPoint*类型
        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the Calibratingtion)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX); // 1/横向个数
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;                                                                               //! Why need get baseline 

        AssignFeaturesToGrid(); //当前点放入格
    }

    void Frame::AssignFeaturesToGrid()
    {
        int nReserve = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
                mGrid[i][j].reserve(nReserve);            //声明容量
                                                          // 在mGrid中记录了各特征点
        for (int i = 0; i < N; i++)
        {
            const cv::KeyPoint &kp = mvKeysUn[i];

            int nGridPosX, nGridPosY;
            if (PosInGrid(kp, nGridPosX, nGridPosY))      //判断点是否在网格中
                mGrid[nGridPosX][nGridPosY].push_back(i); //当前格放入点
        }
    }

    void Frame::ExtractORB(int flag, const cv::Mat &im)
    {
        if (flag == 0)
            (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors); //括号重载
        else
            (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight);
    }

    void Frame::SetPose(cv::Mat Tcw)
    {
        mTcw = Tcw.clone();                                // 相机姿态 世界坐标系到相机坐标坐标系的变换矩阵
        UpdatePoseMatrices();
    }

    void Frame::UpdatePoseMatrices()
    {
        mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
        mRwc = mRcw.t();
        mtcw = mTcw.rowRange(0, 3).col(3);
        mOw = -mRcw.t() * mtcw; //==mtwc,世界坐标系原点在当前相机坐标系下的坐标
        // mtcw, 即相机坐标系下相机坐标系到世界坐标系间的向量, 向量方向由相机坐标系指向世界坐标系
        // mOw, 即世界坐标系下世界坐标系到相机坐标系间的向量, 向量方向由世界坐标系指向相机坐标系
    }

    /**
     * @brief 判断一个点是否在视野内
     *
     * 计算了重投影坐标，观测方向夹角，预测在当前帧的尺度
     * @param  pMP             MapPoint
     * @param  viewingCosLimit 视角和平均视角的方向阈值
     * @return                 true if is in view
     * @see SearchLocalPoints()
     */
    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
    {
        pMP->mbTrackInView = false;

        // 3D in absolute coordinates
        cv::Mat P = pMP->GetWorldPos();

        // 3D in camera coordinates
        // 3D点P在相机坐标系下的坐标
        const cv::Mat Pc = mRcw * P + mtcw; // 这里的Rt是经过初步的优化后的
        const float &PcX = Pc.at<float>(0);
        const float &PcY = Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if (PcZ < 0.0f) //如果深度是负的表明这个点在图像后面，肯定不是能够观测到的点
            return false;

        // Project in image and check it is not outside
        //投影至像素坐标～并查看是否在图像内
        const float invz = 1.0f / PcZ;
        const float u = fx * PcX * invz + cx;
        const float v = fy * PcY * invz + cy;

        if (u < mnMinX || u > mnMaxX)
            return false;
        if (v < mnMinY || v > mnMaxY)
            return false;

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const cv::Mat PO = P - mOw;      //RtX1-RtT = p / -RtT=mOw 所以PO是RtX1 也就是三维点在当前帧坐标下的坐标左乘一个mRwc
        const float dist = cv::norm(PO); //矩阵2范数

        if (dist < minDistance || dist > maxDistance) //距离太近太远都不符合                    //!  minDistance maxDistance
            return false;

        // Check viewing angle
        // V-D 2) 计算当前视角和平均视角夹角的余弦值, 若小于cos(60), 即夹角大于60度则返回
        cv::Mat Pn = pMP->GetNormal(); //获取平均观测方向

        const float viewCos = PO.dot(Pn) / dist; //对应元素相乘相加

        if (viewCos < viewingCosLimit)
            return false;

        // Predict scale in the image
        // V-D 4) 根据深度预测尺度（对应特征点在一层）
        const int nPredictedLevel = pMP->PredictScale(dist, this); //this返回当前类也就是Frame   //! 根据深度预测尺度（对应特征点在一层）

        // Data used by the tracking
        // 标记该点将来要被投影
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = u;
        pMP->mTrackProjXR = u - mbf * invz; //该3D点投影到双目右侧相机上的横坐标                    //! this code need to be changed
        pMP->mTrackProjY = v;
        pMP->mnTrackScaleLevel = nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        return true;
    }                                            

    /**
     * @brief 找到在 以x,y为中心,边长为2r的方形内且在[minLevel, maxLevel]的特征点   
     * @param x        图像坐标u
     * @param y        图像坐标v                                                               
     * @param r        边长
     * @param minLevel 最小尺度
     * @param maxLevel 最大尺度
     * @return         满足条件的特征点的序号
     *说白了，任意给定一个正方形框，返回一个带有特征点编号的向量，第一步计算这个框覆盖了多少范围的grid
     ×第二步计算每一个被覆盖的GRID中的特征点是否在框中
     */
    vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv)); //floor  5.4 取5，此命令计算x点在哪个格子，跟0取最大
        if (nMinCellX >= FRAME_GRID_COLS)                                                   //超过则返回0个下面同理
            return vIndices;

        const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + r) * mfGridElementWidthInv)); //ceil与floor相反
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy]; //将该格里面的特征点的标号存入vCell
                if (vCell.empty())
                    continue;

                for (size_t j = 0, jend = vCell.size(); j < jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    if (bCheckLevels)
                    {
                        if (kpUn.octave < minLevel) //代表是从金字塔哪一层提取的得到的数据。                //! there has define the levels 
                            continue;
                        if (maxLevel >= 0)
                            if (kpUn.octave > maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r) //求浮点数x的绝对值
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) //引用，计算点在第几个grid内，判断点是否在grid内
    {
        posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
        posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
            return false;

        return true;
    }

    void Frame::ComputeBoW()
    {
        if (mBowVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }
    // 调用OpenCV的矫正函数矫正orb提取的特征点           
    void Frame::UndistortKeyPoints()
    {
        // 如果没有图像是矫正过的，没有失真
        if (mDistCoef.at<float>(0) == 0.0)
        {
            mvKeysUn = mvKeys;
            return;
        }

        // Fill matrix with points
        cv::Mat mat(N, 2, CV_32F);
        for (int i = 0; i < N; i++)
        {
            mat.at<float>(i, 0) = mvKeys[i].pt.x;
            mat.at<float>(i, 1) = mvKeys[i].pt.y;
        }

        // Undistort points
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
        mat = mat.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for (int i = 0; i < N; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x = mat.at<float>(i, 0);
            kp.pt.y = mat.at<float>(i, 1);
            mvKeysUn[i] = kp;
        }
    }

    void Frame::ComputeImageBounds(const cv::Mat &imLeft) //计算图像边界
    {
        if (mDistCoef.at<float>(0) != 0.0)
        {
            cv::Mat mat(4, 2, CV_32F);
            mat.at<float>(0, 0) = 0.0;
            mat.at<float>(0, 1) = 0.0;
            mat.at<float>(1, 0) = imLeft.cols;
            mat.at<float>(1, 1) = 0.0;
            mat.at<float>(2, 0) = 0.0;
            mat.at<float>(2, 1) = imLeft.rows;
            mat.at<float>(3, 0) = imLeft.cols;
            mat.at<float>(3, 1) = imLeft.rows;

            // Undistort corners
            mat = mat.reshape(2);
            cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK); //计算图中经过畸变计算的二维点位置
            mat = mat.reshape(1);

            mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
            mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
            mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
            mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
        }
        else
        {
            mnMinX = 0.0f;
            mnMaxX = imLeft.cols;
            mnMinY = 0.0f;
            mnMaxY = imLeft.rows;
        }
    }

    void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
    {
        // mvDepth直接由depth图像读取
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        for (int i = 0; i < N; i++)
        {
            const cv::KeyPoint &kp = mvKeys[i];
            const cv::KeyPoint &kpU = mvKeysUn[i];

            const float &v = kp.pt.y;
            const float &u = kp.pt.x;

            const float d = imDepth.at<float>(v, u);

            if (d > 0)
            {
                mvDepth[i] = d;
                mvuRight[i] = kpU.pt.x - mbf / d;                     //! bf是为了用来根据左目x坐标还原关联的右目x坐标，然后将右目x坐标关联到关键点  this code need to be changed!
            }
        }
    }

    /**
     * @brief Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
     * @param  i 第i个keypoint
     * @return   3D点（相对于世界坐标系）
     */
    cv::Mat Frame::UnprojectStereo(const int &i)                                    
    {
        // KeyFrame::UnprojectStereo
        // mvDepth是在ComputeStereoMatches函数中求取的
        // mvDepth对应的校正前的特征点，可这里却是对校正后特征点反投影
        // KeyFrame::UnprojectStereo中是对校正前的特征点mvKeys反投影            
        // 在ComputeStereoMatches函数中应该对校正后的特征点求深度？？
        const float z = mvDepth[i];
        if (z > 0)
        {
            const float u = mvKeysUn[i].pt.x; 
            const float v = mvKeysUn[i].pt.y; 
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
            return mRwc * x3Dc + mOw; 
        }
        else
            return cv::Mat();
    }

} // namespace Camvox
