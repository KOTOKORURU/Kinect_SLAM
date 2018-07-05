#include "KeyFrame.h"
#include "Convert.h"
#include "ORBmatcher.h"
#include <QMutex>
namespace KINECT_SLAM
{
long unsigned int KeyFrame::nNextId = 0;
KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
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
    mbToBeErased(false), mbBad(false), /*mHalfBaseline(F.mb/2)*/ mpMap(pMap),cam_info(F.cam_info),pc_col(F.pc_col)
{
    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = toDescriptorVector(mDescriptors);
        //assume the features with nodes are in the 4th level(6th total,from levels up)
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

//set pose
void KeyFrame::SetPose(const cv::Mat& _Tcw)
{
    QMutexLocker locker(&mMutexPose);
    _Tcw.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc * tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));

   // cv::Mat center = (cv::Mat_<float>(4,1)<<mHalfBaseline, 0, 0, 1);
   // Cw = Twc * center;

}

cv::Mat KeyFrame::GetPose()
{
    QMutexLocker locker(&mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    QMutexLocker locker(&mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    QMutexLocker locker(&mMutexPose);
    return Ow.clone();
}

//cv::Mat KeyFrame::GetStereoCenter()
//{
 // QMutexLocker locker(&mMutexPose);
 // return Cw.clone

//}

//diff
cv::Mat KeyFrame::GetRotation()
{
    QMutexLocker locker(&mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}
//diff
cv::Mat KeyFrame::GetTranslation()
{
    QMutexLocker locker(&mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame* pKF,const int& weight)
{
    {
        QMutexLocker locker(&mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF] = weight;
        else if(mConnectedKeyFrameWeights[pKF] != weight)
            mConnectedKeyFrameWeights[pKF] = weight;
        else
            return;

    }
    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    QMutexLocker locker(&mMutexConnections);

    vector<pair<int,KeyFrame*>> vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(); mit !=mConnectedKeyFrameWeights.end(); mit++)
        vPairs.push_back(make_pair(mit->second,mit->first));

        sort(vPairs.begin(),vPairs.end());
        list<KeyFrame*> lKFs;//keyframe
        list<int> lWs;//weight

        for(size_t i = 0; i < vPairs.size(); i++)
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(),lWs.end());

}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    QMutexLocker locker(&mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin();mit != mConnectedKeyFrameWeights.end(); mit++)
        s.insert(mit->first);
    return s;
}

//get covisible keyframes ordered by the weights
vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    QMutexLocker locker(&mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    QMutexLocker locker(&mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*> (mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);


}

// get the keyframes whose weight bigger than w
vector<KeyFrame*> KeyFrame::GetCovisibilesByWeight(const int& w)
{
    QMutexLocker locker(&mMutexConnections);
    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

        vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,[=](int a,int b)->bool{return a>b;});//or use KeyFrame::weightComp()

        if( it == mvOrderedWeights.end())
            return vector<KeyFrame*>();
        else
        {
            int n = it - mvOrderedWeights.begin();
            return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+n);
        }
}

int KeyFrame::GetWeight(KeyFrame* pKF)
{
    QMutexLocker lokcer(&mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
      return mConnectedKeyFrameWeights[pKF];
    else
      return 0;
}

void KeyFrame::AddMapPoint(MapPoint* pMP,const size_t& idx)
{
    QMutexLocker locker(&mMutexFeatures);
    mvpMapPoints[idx] = pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t& idx)
{
    QMutexLocker locker(&mMutexFeatures);
    mvpMapPoints[idx] = static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx >= 0)
        mvpMapPoints[idx] = static_cast<MapPoint*>(NULL);
}

void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP)
{
    mvpMapPoints[idx] = pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    QMutexLocker locker(&mMutexFeatures);
    //diff
    set<MapPoint*> sp;
    for(size_t i = 0; i<mvpMapPoints.size(); i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            sp.insert(pMP);
    }
    return sp;
}

//return the number of the mappoints whose Observation value bigger than the threshold
int KeyFrame::TrackedMapPoints(const int &minObs)
{
    QMutexLocker locker(&mMutexFeatures);

    int nPoints = 0;
    const bool bCheckObs = minObs > 0;
    for(int i=0; i< N ;i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {   //diff
                if(bCheckObs)
                {
                    if(pMP->Observations() >= minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }
    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    QMutexLocker locker(&mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t& idx)
{
    QMutexLocker locker(&mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections()
{
    //====1====
    map<KeyFrame*,int> KFcounter;
    vector<MapPoint*> vpMP;
    {
        QMutexLocker locker(&mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit = vpMP.begin(); vit != vpMP.end(); vit++)
    {
        MapPoint* pMP = *vit;
        if(!pMP)
            continue;
        if(pMP->isBad())
            continue;
        map<KeyFrame*,size_t> observations = pMP->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit = observations.begin(); mit != observations.end(); mit++)
        {
            if(mit->first->mnId == mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }
    //not even a covisible keyframe exsit
    if(KFcounter.empty())
        return;

    //=========2=======
    int nmax = 0;
    KeyFrame* pKFmax = NULL;
    int th = 15;//see the same mappoint above 15 times

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit = KFcounter.begin(); mit != KFcounter.end(); mit++)
    {
        if(mit->second > nmax)
        {
            nmax = mit->second;
            pKFmax = mit ->first;
        }
        if(mit->second >= th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }
    //not even a keyframe see above 15 same points with current keyframe
    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }
    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i = 0; i<vPairs.size(); i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    //update connection with other keyframes,add the connection with keyframes
    //======3======
    {
        QMutexLocker locker(&mMutexConnections);

        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(),lWs.end());

        //update the spanning tree
        if(mbFirstConnection && mnId!=0)
        {
            //initial the most covisible keyframe to be the parent of current keyframe
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }
    }

}

void KeyFrame::AddChild(KeyFrame* pKF)
{
    QMutexLocker locker(&mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame* pKF)
{
    QMutexLocker locker(&mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame* pKF)
{
    QMutexLocker locker(&mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    QMutexLocker locker(&mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    QMutexLocker locker(&mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame* pKF)
{
   QMutexLocker locker(&mMutexConnections);
   return mspChildrens.count(pKF);
}
//should add a edge between pKF and current keyframe
void KeyFrame::ADDLoopEdge(KeyFrame *pKF)
{
    QMutexLocker locker(&mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    QMutexLocker locker(&mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    QMutexLocker locker(&mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        QMutexLocker locker(&mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }
    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{
    {
        QMutexLocker locker(&mMutexConnections);
        if(mnId == 0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }
    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(); mit != mConnectedKeyFrameWeights.end(); mit++)
    {
        mit->first->EraseConnection(this);
        for(size_t i=0; i<mvpMapPoints.size(); i++)
            if(mvpMapPoints[i])
                mvpMapPoints[i]->EraseObservation(this);
        {
            QMutexLocker locker1(&mMutexConnections);
            QMutexLocker locker2(&mMutexFeatures);

            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();

            //update spanning tree
            set<KeyFrame*> sParentCandidates;
            sParentCandidates.insert(mpParent);

            while(!mspChildrens.empty())
            {
                bool bContinue = false;
                int max = -1;
                KeyFrame* pC;
                KeyFrame* pP;

                for(set<KeyFrame*>::iterator sit = mspChildrens.begin(); sit!= mspChildrens.end(); sit++)
                {
                    KeyFrame* pKF =*sit;
                    if(pKF->isBad())
                        continue;
                    vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                    for(size_t i=0; i< vpConnected.size(); i++)
                    {
                        for(set<KeyFrame*>::iterator sit = sParentCandidates.begin();sit!=sParentCandidates.end();sit++)
                        {
                            if(vpConnected[i]->mnId == (*sit)->mnId)
                            {
                              int w =pKF->GetWeight(vpConnected[i]);
                              if(w>max)
                              {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                               }
                            }
                    }
                }
            }
                if(bContinue)
                {
                    pC->ChangeParent(pP);
                    sParentCandidates.insert(pC);
                    mspChildrens.erase(pC);
                }
                else
                    break;
            }
            if(!mspChildrens.empty())
                for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end();sit++)
                {
                    (*sit)->ChangeParent(mpParent);
                }
            mpParent->EraseChild(this);
            mTcp = Tcw*mpParent->GetPoseInverse();
            mbBad = true;
        }
    }
    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);

}

bool KeyFrame::isBad()
{
    QMutexLocker locker(&mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        QMutexLocker locker(&mMutexConnections);
        if(mConnectedKeyFrameWeights.count((pKF)))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate = true;
        }
    }
    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float& x,const float& y,const float& r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX >= mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinX-r)*mfGridElementHeightInv));
    if(nMinCellY >= mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY < 0)
        return vIndices;
    for(int ix = nMinCellX; ix<= nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy <= nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j = 0; j<vCell.size();j++)
            {
              const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
              const float distx = kpUn.pt.x - x;
              const float disty = kpUn.pt.y - y;
              if(fabs(distx)<r && fabs(disty)<r)
                  vIndices.push_back(vCell[j]);
            }
        }
    }
    return vIndices;
}

bool KeyFrame::IsInImage(const float &x,const float &y) const
{
    return (x >= mnMinX && x<mnMaxX && y >= mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
        if(z>0)
        {
            const float u = mvKeys[i].pt.x;
            const float v = mvKeys[i].pt.y;
            const float x = (u-cx)*z*invfx;
            const float y = (v-cy)*z*invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

            QMutexLocker locker(&mMutexPose);
            return Twc.rowRange(0,3).colRange(0,3) * x3Dc + Twc.rowRange(0,3).col(3);
        }
        else
            return cv::Mat();

}
}
