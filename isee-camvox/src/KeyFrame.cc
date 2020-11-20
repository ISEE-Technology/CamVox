#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <mutex>

namespace Camvox
{
    long unsigned int KeyFrame::nNextId = 0;

    KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB) : mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
                                                                       mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
                                                                       mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
                                                                       mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
                                                                       fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
                                                                       mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
                                                                       mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
                                                                       mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
                                                                       mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
                                                                       mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
                                                                       mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
                                                                       mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
                                                                       mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb / 2), mpMap(pMap)
    {
        mnId = nNextId++;

        mGrid.resize(mnGridCols);
        for (int i = 0; i < mnGridCols; i++) //读取帧中grid，转化到kf中的grid中
        {
            mGrid[i].resize(mnGridRows);
            for (int j = 0; j < mnGridRows; j++)
                mGrid[i][j] = F.mGrid[i][j];
        }

        SetPose(F.mTcw);
    }

    void KeyFrame::ComputeBoW() //计算BOW与frame类成员函数一样
    {
        if (mBowVec.empty() || mFeatVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            // Feature vector associate features with nodes in the 4th level (from leaves up)
            // We assume the vocabulary tree has 6 levels, change the 4 otherwise
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    void KeyFrame::SetPose(const cv::Mat &Tcw_)
    {
        unique_lock<mutex> lock(mMutexPose);
        Tcw_.copyTo(Tcw);
        cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
        cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
        cv::Mat Rwc = Rcw.t();
        Ow = -Rwc * tcw;

        Twc = cv::Mat::eye(4, 4, Tcw.type());
        Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
        Ow.copyTo(Twc.rowRange(0, 3).col(3));
        // center为相机坐标系（左目）下，立体相机中心的坐标                                                            //! 立体相机坐标的确定
        // 立体相机中心点坐标与左目相机坐标之间只是在x轴上相差mHalfBaseline,
        // 因此可以看出，立体相机中两个摄像头的连线为x轴，正方向为左目相机指向右目相机
        cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
        // 世界坐标系下，左目相机中心到立体相机中心的向量，方向由左目相机指向立体相机中心
        Cw = Twc * center; //两个相机中间坐标
    }

    cv::Mat KeyFrame::GetPose()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.clone();
    }

    cv::Mat KeyFrame::GetPoseInverse()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Twc.clone();
    }

    cv::Mat KeyFrame::GetCameraCenter()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Ow.clone();
    }

    cv::Mat KeyFrame::GetStereoCenter()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Cw.clone();
    }

    cv::Mat KeyFrame::GetRotation()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rowRange(0, 3).colRange(0, 3).clone();
    }

    cv::Mat KeyFrame::GetTranslation()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rowRange(0, 3).col(3).clone();
    }

    /**
     * @brief 为关键帧之间添加连接
     * 
     * 更新了mConnectedKeyFrameWeights
     * @param pKF    关键帧
     * @param weight 权重，该关键帧与pKF共同观测到的3d点数量
     */
    void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
    {
        {
            unique_lock<mutex> lock(mMutexConnections);
            // std::map::count函数只可能返回0或1两种情况,
            //mConnectedKeyFrameWeights与该关键帧连接的关键帧与权重
            if (!mConnectedKeyFrameWeights.count(pKF)) // count函数返回0，mConnectedKeyFrameWeights中没有pKF，之前没有连接
                mConnectedKeyFrameWeights[pKF] = weight;
            else if (mConnectedKeyFrameWeights[pKF] != weight) // 之前连接的权重不一样
                mConnectedKeyFrameWeights[pKF] = weight;
            else
                return;
        }

        UpdateBestCovisibles();
    }

    /**
     * @brief 按照权重对连接的关键帧进行排序
     * 
     * 更新后的变量存储在mvpOrderedConnectedKeyFrames和mvOrderedWeights中
     */
    void KeyFrame::UpdateBestCovisibles()
    {
        unique_lock<mutex> lock(mMutexConnections);
        // http://stackoverflow.com/questions/3389648/difference-between-stdliststdpair-and-stdmap-in-c-stl
        vector<pair<int, KeyFrame *>> vPairs;
        vPairs.reserve(mConnectedKeyFrameWeights.size());
        // 取出所有连接的关键帧，mConnectedKeyFrameWeights的类型为std::map<KeyFrame*,int>，而vPairs变量将共视的3D点数放在前面，利于排序
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
            vPairs.push_back(make_pair(mit->second, mit->first));
        // 按照权重进行排序
        sort(vPairs.begin(), vPairs.end());
        /*1.vector数据结构
        vector和数组类似，拥有一段连续的内存空间，并且起始地址不变。
        因此能高效的进行随机存取，时间复杂度为o(1);
        但因为内存空间是连续的，所以在进行插入和删除操作时，会造成内存块的拷贝，时间复杂度为o(n)。
        另外，当数组中内存空间不够时，会重新申请一块内存空间并进行内存拷贝。

        2.list数据结构
        list是由双向链表实现的，因此内存空间是不连续的。
        只能通过指针访问数据，所以list的随机存取非常没有效率，时间复杂度为o(n);
        但由于链表的特点，能高效地进行插入和删除。
        */
        list<KeyFrame *> lKFs; // keyframe
        list<int> lWs;         // weight
        for (size_t i = 0, iend = vPairs.size(); i < iend; i++)
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }
        // 权重从大到小
        mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
    }

    /**
     * @brief 得到与该关键帧连接的关键帧
     * @return 连接的关键帧
     */
    set<KeyFrame *> KeyFrame::GetConnectedKeyFrames()
    {
        unique_lock<mutex> lock(mMutexConnections);
        set<KeyFrame *> s;
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(); mit != mConnectedKeyFrameWeights.end(); mit++)
            s.insert(mit->first);
        return s;
    }
    
    /**
     * @brief 得到与该关键帧连接的关键帧(已按权值排序)
     * @return 连接的关键帧
     */
    vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames()
    {
        unique_lock<mutex> lock(mMutexConnections);
        return mvpOrderedConnectedKeyFrames;
    }

    /**
     * @brief 得到与该关键帧连接的前N个关键帧(已按权值排序)
     * 
     * 如果连接的关键帧少于N，则返回所有连接的关键帧
     * @param N 前N个
     * @return 连接的关键帧
     */
    vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
    {
        unique_lock<mutex> lock(mMutexConnections);
        if ((int)mvpOrderedConnectedKeyFrames.size() < N)
            return mvpOrderedConnectedKeyFrames;
        else
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);
    }

    /**
     * @brief 得到与该关键帧连接的权重大于等于w的关键帧
     * @param w 权重
     * @return 连接的关键帧
     */
    vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w)
    {
        unique_lock<mutex> lock(mMutexConnections);

        if (mvpOrderedConnectedKeyFrames.empty())
            return vector<KeyFrame *>();

        vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w, KeyFrame::weightComp); //第一个参数是w，第二个是向量中的数
        if (it == mvOrderedWeights.end())
            return vector<KeyFrame *>();
        else
        {
            int n = it - mvOrderedWeights.begin();
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
        }
    }

    /**
     * @brief 得到该关键帧与pKF的权重
     * @param  pKF 关键帧
     * @return     权重
     */
    int KeyFrame::GetWeight(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (mConnectedKeyFrameWeights.count(pKF))
            return mConnectedKeyFrameWeights[pKF];
        else
            return 0;
    }

    /**
     * @brief Add MapPoint to KeyFrame
     * @param pMP MapPoint
     * @param idx MapPoint在KeyFrame中的索引
     */
    void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = pMP;
    }

    void KeyFrame::EraseMapPointMatch(const size_t &idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
    }

    void KeyFrame::EraseMapPointMatch(MapPoint *pMP)
    {
        int idx = pMP->GetIndexInKeyFrame(this);
        if (idx >= 0)
            mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
    }

    void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP)
    {
        mvpMapPoints[idx] = pMP;
    }

    set<MapPoint *> KeyFrame::GetMapPoints()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        set<MapPoint *> s;
        for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++)
        {
            if (!mvpMapPoints[i])
                continue;
            MapPoint *pMP = mvpMapPoints[i];
            if (!pMP->isBad())
                s.insert(pMP);
        }
        return s;
    }

    /**
     * @brief 关键帧中，大于等于minObs的MapPoints的数量
     * minObs就是一个阈值，大于minObs就表示该MapPoint是一个高质量的MapPoint
     * 一个高质量的MapPoint会被多个KeyFrame观测到，minObs小于等于0
     * @param  minObs 最小观测
     */
    int KeyFrame::TrackedMapPoints(const int &minObs)
    {
        unique_lock<mutex> lock(mMutexFeatures);

        int nPoints = 0;
        const bool bCheckObs = minObs > 0;
        for (int i = 0; i < N; i++)
        {
            MapPoint *pMP = mvpMapPoints[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    if (bCheckObs)
                    {
                        // 该MapPoint是一个高质量的MapPoint
                        if (mvpMapPoints[i]->Observations() >= minObs)
                            nPoints++;
                    }
                    else
                        nPoints++;
                }
            }
        }

        return nPoints;
    }

    /**
     * @brief Get MapPoint Matches
     *
     * 获取该关键帧的MapPoints
     */
    vector<MapPoint *> KeyFrame::GetMapPointMatches()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints;
    }

    MapPoint *KeyFrame::GetMapPoint(const size_t &idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints[idx];
    }

    /**
     * @brief 更新图的连接
     * 
     * 1. 首先获得该关键帧的所有MapPoint点，统计观测到这些3d点的每个关键与其它所有关键帧之间的共视程度
     *    对每一个找到的关键帧，建立一条边，边的权重是该关键帧与当前关键帧公共3d点的个数。
     * 2. 并且该权重必须大于一个阈值，如果没有超过该阈值的权重，那么就只保留权重最大的边（与其它关键帧的共视程度比较高）
     * 3. 对这些连接按照权重从大到小进行排序，以方便将来的处理
     *    更新完covisibility图之后，如果没有初始化过，则初始化为连接权重最大的边（与其它关键帧共视程度最高的那个关键帧），类似于最大生成树 spantree
     */
    void KeyFrame::UpdateConnections()
    {
        // 在没有执行这个函数前，关键帧只和MapPoints之间有连接关系，这个函数可以更新关键帧之间的连接关系

        //===============1==================================
        map<KeyFrame *, int> KFcounter; // 关键帧-权重，权重为其它关键帧与当前关键帧共视3d点的个数

        vector<MapPoint *> vpMP;

        {
            // 获得该关键帧的所有3D点
            unique_lock<mutex> lockMPs(mMutexFeatures);
            vpMP = mvpMapPoints;
        }

        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        // 通过3D点间接统计可以观测到这些3D点的所有关键帧之间的共视程度
        // 即统计每一个关键帧都有多少关键帧与它存在共视关系，统计结果放在KFcounter
        for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;

            if (!pMP)
                continue;

            if (pMP->isBad())
                continue;
            // 对于每一个MapPoint点，observations记录了可以观测到该MapPoint的所有关键帧以及该点在该关键帧的索引
            map<KeyFrame *, size_t> observations = pMP->GetObservations();

            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                // 除去自身，自己与自己不算共视
                if (mit->first->mnId == mnId)
                    continue;
                KFcounter[mit->first]++; //标签对应的整数+1，没有则创建mit，第一个数为关键帧，第二个数为它们具有共识点数
            }
        }

        // This should not happen
        if (KFcounter.empty())
            return;
        //===============2==================================
        //If the counter is greater than threshold add connection
        //In case no keyframe counter is over threshold add the one with maximum counter
        int nmax = 0;
        KeyFrame *pKFmax = NULL;
        int th = 15;

        // vPairs记录与其它关键帧共视帧数大于th的关键帧
        // pair<int,KeyFrame*>将关键帧的权重写在前面，关键帧写在后面方便后面排序
        vector<pair<int, KeyFrame *>> vPairs;
        vPairs.reserve(KFcounter.size());
        for (map<KeyFrame *, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++)
        {
            if (mit->second > nmax)
            {
                nmax = mit->second;
                // 找到对应权重最大的关键帧（共视程度最高的关键帧）
                pKFmax = mit->first;
            }
            if (mit->second >= th)
            {
                // 对应权重需要大于阈值，对这些关键帧建立连接
                vPairs.push_back(make_pair(mit->second, mit->first));
                // 更新KFcounter中该关键帧的mConnectedKeyFrameWeights
                // 更新其它KeyFrame的mConnectedKeyFrameWeights，更新其它关键帧与当前帧的连接权重
                (mit->first)->AddConnection(this, mit->second);
            }
        }

        // 如果没有超过阈值的权重，则对权重最大的关键帧建立连接
        if (vPairs.empty())
        {
            // 如果每个关键帧与它共视的关键帧的个数都少于th，
            // 那就只更新与其它关键帧共视程度最高的关键帧的mConnectedKeyFrameWeights
            // 这是对之前th这个阈值可能过高的一个补丁
            vPairs.push_back(make_pair(nmax, pKFmax));
            pKFmax->AddConnection(this, nmax);
        }
        // vPairs里存的都是相互共视程度比较高的关键帧和共视权重，由大到小
        sort(vPairs.begin(), vPairs.end());
        list<KeyFrame *> lKFs;
        list<int> lWs;
        for (size_t i = 0; i < vPairs.size(); i++)
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }
        //===============3==================================
        {
            unique_lock<mutex> lockCon(mMutexConnections);

            // mspConnectedKeyFrames = spConnectedKeyFrames;
            // 更新图的连接(权重)
            mConnectedKeyFrameWeights = KFcounter;
            mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
            // 更新生成树的连接
            if (mbFirstConnection && mnId != 0)
            {
                // 初始化该关键帧的父关键帧为共视程度最高的那个关键帧
                mpParent = mvpOrderedConnectedKeyFrames.front();
                // 建立双向连接关系
                mpParent->AddChild(this); //共识度最高的关键帧添加此关键帧为子类
                mbFirstConnection = false;
            }
        }
    }

    void KeyFrame::AddChild(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.insert(pKF);
    }

    void KeyFrame::EraseChild(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.erase(pKF);
    }

    void KeyFrame::ChangeParent(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mpParent = pKF;
        pKF->AddChild(this);
    }

    set<KeyFrame *> KeyFrame::GetChilds()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens;
    }

    KeyFrame *KeyFrame::GetParent()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mpParent;
    }

    bool KeyFrame::hasChild(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens.count(pKF);
    }

    void KeyFrame::AddLoopEdge(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mbNotErase = true;
        mspLoopEdges.insert(pKF);
    }

    set<KeyFrame *> KeyFrame::GetLoopEdges()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspLoopEdges;
    }

    void KeyFrame::SetNotErase()
    {
        unique_lock<mutex> lock(mMutexConnections);
        mbNotErase = true;
    }

    void KeyFrame::SetErase()
    {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mspLoopEdges.empty())
            {
                mbNotErase = false;
            }
        }
        // 这个地方是不是应该：(!mbToBeErased)，(wubo???)
        // SetBadFlag函数就是将mbToBeErased置为true，mbToBeErased就表示该KeyFrame被擦除了
        if (mbToBeErased)
        {
            SetBadFlag();
        }
    }

    void KeyFrame::SetBadFlag()
    {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mnId == 0)
                return;
            else if (mbNotErase) // mbNotErase表示不应该擦除该KeyFrame，于是把mbToBeErased置为true，表示已经擦除了，其实没有擦除
            {
                mbToBeErased = true;
                return;
            }
        }

        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
            mit->first->EraseConnection(this); // 让其它的KeyFrame删除与自己的联系

        for (size_t i = 0; i < mvpMapPoints.size(); i++)
            if (mvpMapPoints[i])
                mvpMapPoints[i]->EraseObservation(this); // 让与自己有联系的MapPoint删除与自己的联系
        {
            unique_lock<mutex> lock(mMutexConnections);
            unique_lock<mutex> lock1(mMutexFeatures);

            //清空自己与其它关键帧之间的联系
            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();

            // Update Spanning Tree
            set<KeyFrame *> sParentCandidates;
            sParentCandidates.insert(mpParent);

            // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
            // Include that children as new parent candidate for the rest
            // 如果这个关键帧有自己的孩子关键帧，告诉这些子关键帧，它们的父关键帧不行了，赶紧找新的父关键帧
            while (!mspChildrens.empty())
            {
                bool bContinue = false;

                int max = -1;
                KeyFrame *pC;
                KeyFrame *pP;
                // 遍历每一个子关键帧，让它们更新它们指向的父关键帧

                for (set<KeyFrame *>::iterator sit = mspChildrens.begin(), send = mspChildrens.end(); sit != send; sit++)
                {
                    KeyFrame *pKF = *sit;
                    if (pKF->isBad())
                        continue;

                    // Check if a parent candidate is connected to the keyframe
                    // 子关键帧遍历每一个与它相连的关键帧（共视关键帧）
                    vector<KeyFrame *> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                    for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) //遍历每一个子关键帧的每个共视帧
                    {
                        for (set<KeyFrame *>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end(); spcit != spcend; spcit++) //遍历候选父关键帧
                        {
                            // 如果该帧的子节点和父节点（祖孙节点）之间存在连接关系（共视）
                            // 举例：B-->A（B的父节点是A） C-->B（C的父节点是B） D--C（D与C相连） E--C（E与C相连） F--C（F与C相连） D-->A（D的父节点是A） E-->A（E的父节点是A）
                            //      现在B挂了，于是C在与自己相连的D、E、F节点中找到父节点指向A的D
                            //      此过程就是为了找到可以替换B的那个节点。
                            // 上面例子中，B为当前要设置为SetBadFlag的关键帧
                            //           A为spcit，也即sParentCandidates
                            //           C为pKF,pC，也即mspChildrens中的一个
                            //           D、E、F为vpConnected中的变量，由于C与D间的权重 比 C与E间的权重大，因此D为pP，这种说法有错误
                            if (vpConnected[i]->mnId == (*spcit)->mnId) //如果子帧的共视帧在父帧候选集中，目前就一个A，获取其为父帧
                            {
                                int w = pKF->GetWeight(vpConnected[i]);
                                if (w > max)
                                {
                                    pC = pKF;            //该挂点的子节点
                                    pP = vpConnected[i]; //在这里就是A，并且A点是与该挂点的子节点的该共视帧
                                    max = w;
                                    bContinue = true;
                                }
                            }
                        }
                    }
                }

                if (bContinue)
                {
                    //之前的操作虽然可能每一个子节点的共视关键帧都出现A但一次操作一个，即所有子节点中最像A的，后面候选父节点会增加
                    // 因为父节点死了，并且子节点找到了新的父节点，子节点更新自己的父节点
                    pC->ChangeParent(pP);
                    // 因为子节点找到了新的父节点并更新了父节点，那么该子节点升级，作为其它子节点的备选父节点，该做法增加候选量，即候选父节点互相的共视程度很高
                    sParentCandidates.insert(pC);
                    // 该子节点处理完毕
                    mspChildrens.erase(pC); //在子节点中去掉，返回while重新进行，直到continue为false
                }
                else
                    break;
            }

            // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
            // 如果还有子节点没有找到新的父节点
            if (!mspChildrens.empty())
                for (set<KeyFrame *>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++)
                {
                    // 直接把父节点的父节点作为自己的父节点
                    (*sit)->ChangeParent(mpParent);
                }

            mpParent->EraseChild(this);
            mTcp = Tcw * mpParent->GetPoseInverse(); //mTcp * 父节点位姿 = 该被剔除关键帧的位姿
            mbBad = true;
        }

        mpMap->EraseKeyFrame(this);
        mpKeyFrameDB->erase(this);
    }

    bool KeyFrame::isBad()
    {
        unique_lock<mutex> lock(mMutexConnections);
        return mbBad;
    }

    void KeyFrame::EraseConnection(KeyFrame *pKF)
    {
        bool bUpdate = false;
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mConnectedKeyFrameWeights.count(pKF))
            {
                mConnectedKeyFrameWeights.erase(pKF);
                bUpdate = true;
            }
        }

        if (bUpdate)
            UpdateBestCovisibles();
    }

    // r为边长（半径）
    vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv));
        if (nMinCellX >= mnGridCols)
            return vIndices;

        const int nMaxCellX = min((int)mnGridCols - 1, (int)ceil((x - mnMinX + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= mnGridRows)
            return vIndices;

        const int nMaxCellY = min((int)mnGridRows - 1, (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy];
                for (size_t j = 0, jend = vCell.size(); j < jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool KeyFrame::IsInImage(const float &x, const float &y) const
    {
        return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
    }

    /**
     * @brief Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
     * @param  i 第i个keypoint
     * @return   3D点（相对于世界坐标系）
     */
    cv::Mat KeyFrame::UnprojectStereo(int i) //反投影到三维空间中
    {
        const float z = mvDepth[i];
        if (z > 0)
        {
            // 由2维图像反投影到相机坐标系
            // mvDepth是在ComputeStereoMatches函数中求取的
            // mvDepth对应的校正前的特征点，因此这里对校正前特征点反投影
            // 可在Frame::UnprojectStereo中却是对校正后的特征点mvKeysUn反投影
            // 在ComputeStereoMatches函数中应该对校正后的特征点求深度？？ (wubo???)
            const float u = mvKeys[i].pt.x;
            const float v = mvKeys[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

            unique_lock<mutex> lock(mMutexPose);
            // 由相机坐标系转换到世界坐标系
            // Twc为相机坐标系到世界坐标系的变换矩阵
            // Twc.rosRange(0,3).colRange(0,3)取Twc矩阵的前3行与前3列
            return Twc.rowRange(0, 3).colRange(0, 3) * x3Dc + Twc.rowRange(0, 3).col(3);                   //! 将 R|t 中的R和t取出来
        }
        else
            return cv::Mat();
    }
} // namespace Camvox
