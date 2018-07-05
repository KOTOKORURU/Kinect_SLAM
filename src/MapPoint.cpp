#include "MapPoint.h"
#include "ORBmatcher.h"

#include <QMutex>
namespace KINECT_SLAM
{
long unsigned int MapPoint::nNextId = 0;
QMutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    QMutexLocker lock(&mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;


    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];


    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);


    QMutexLocker lock(&mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    QMutexLocker lock2(&mGlobalMutex);
    QMutexLocker lock3(&mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    QMutexLocker lock(&mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    QMutexLocker lock(&mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    QMutexLocker lock(&mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF,size_t idx)
{
    QMutexLocker lock(&mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    //record the index of the point in the keyframe by which this point seen
    mObservations[pKF] = idx;

    if(pKF->mvuRight[idx] >= 0)
        nObs += 2;

}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad = false;
    {
        QMutexLocker lock(&mMutexFeatures);
        if(mObservations.count(pKF))
         {
           int idx = mObservations[pKF];
           if(pKF->mvuRight[idx] >= 0)
             nObs -= 2;
           mObservations.erase(pKF);
           if(mpRefKF == pKF)
              mpRefKF = mObservations.begin()->first;
          if(nObs<=2)
              bBad = true;
          }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*,size_t> MapPoint::GetObservations()
{
    QMutexLocker locker(&mMutexFeatures);
    return mObservations;
}
int MapPoint::Observations()
{
    QMutexLocker locker(&mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        QMutexLocker locker1(&mMutexFeatures);
        QMutexLocker locker2(&mMutexPos);
        mbBad = true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit = obs.begin();mit != obs.end();mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }
    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    QMutexLocker locker1(&mMutexFeatures);
    QMutexLocker locker2(&mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
 if(pMP->mnId == this->mnId)
     return;
 int nvisible, nfound;
 map<KeyFrame*,size_t> obs;
 {
     QMutexLocker locker1(&mMutexFeatures);
     QMutexLocker locker2(&mMutexPos);
     obs = mObservations;
     mbBad = true;
     nvisible = mnVisible;
     mpReplaced = pMP;
 }
 for(map<KeyFrame*,size_t>::iterator mit = obs.begin(); mit != obs.end(); mit++)
 {
     KeyFrame *pKF = mit->first;
     if(!pMP->IsInKeyFrame(pKF))
     {
         pKF->ReplaceMapPointMatch(mit->second,pMP);
         pMP->AddObservation(pKF,mit->second);
     }
     else
     {
         pKF->EraseMapPointMatch(mit->second);//if this point is not in the keyframe,this keyframe should delete this being replaced point
     }
 }
 pMP->IncreaseFound(nfound);
 pMP->IncreaseVisible(nvisible);
 pMP->ComputeDistinctiveDescriptors();

 mpMap->EraseMapPoint(this);

}

bool MapPoint::isBad()
{
    QMutexLocker locker1(&mMutexFeatures);
    QMutexLocker locker2(&mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    QMutexLocker locker(&mMutexFeatures);
    mnVisible += n;
}

void MapPoint::IncreaseFound(int n)
{
    QMutexLocker locker(&mMutexFeatures);
    mnFound += n;
}

float MapPoint::GetFoundRatio()
{
    QMutexLocker locker(&mMutexFeatures);
    return static_cast<float>(mnFound) / mnVisible;
}

//an mappoint will be seen in many different keyframe,so it would be descripted many times.
//choose the beset descriptor correspond to this mappoint.
//the best descriptor should have the lest median distance between the others' descriptors
void MapPoint::ComputeDistinctiveDescriptors()
{
    vector<cv::Mat> vDescriptors;
    map<KeyFrame*,size_t> observations;

    {
        QMutexLocker locker(&mMutexFeatures);
        if(mbBad)
            return;
        observations = mObservations;
    }

    if(observations.empty())
        return;
    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit = observations.begin(); mit != observations.end(); mit++)
    {
        KeyFrame* pKF = mit->first;
        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));//get the descriptor correspond to this mappoint
    }

    if(vDescriptors.empty())
            return;
    const size_t N = vDescriptors.size();

    std::vector<std::vector<float>> Distances;
    Distances.resize(N,vector<float>(N,0));
    for(size_t i = 0; i < N ; i++)
    {
        Distances[i][i] = 0;
        for(size_t j = i+1; j < N; j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j] = distij;
            Distances[j][i] = distij;
        }
    }

    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i =0; i<N; i++)
    {
        vector<int> vDists(Distances[i].begin(),Distances[i].end());
        sort(vDists.begin(),vDists.end());

        int median = vDists[0.5 * (N - 1)];
        if(median < BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        QMutexLocker locker(&mMutexFeatures);

        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
  QMutexLocker locker(&mMutexFeatures);
  return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    QMutexLocker locker(&mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    QMutexLocker locker(&mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        QMutexLocker locker1(&mMutexFeatures);
        QMutexLocker locker2(&mMutexPos);
        if(mbBad)
            return;
        observations = mObservations;
        pRefKF = mpRefKF;
        Pos = mWorldPos.clone();
    }
    if(observations.empty())
        return;
    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n = 0;
    for(map<KeyFrame*,size_t>::iterator mit = observations.begin(); mit != observations.end(); mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normali + normali/cv::norm(normali);//unit vector
        n++;
    }
    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor = pRefKF->mvScaleFactors[level];//pymird scale factor
    const int nLevels = pRefKF->mnScaleLevels;

    {
        QMutexLocker locker3(&mMutexPos);
        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance /pRefKF->mvScaleFactors[nLevels-1];//the top,largest scale factor
        mNormalVector = normal/n;//median vector
    }


}

float MapPoint::GetMinDistanceInvariance()
{
    QMutexLocker locker(&mMutexPos);
    return 0.8f * mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    QMutexLocker locker(&mMutexPos);
    return 1.2 * mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, const float &logScaleFactor)
{
    float ratio;
    {
        QMutexLocker locker3(&mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }
    return ceil(log(ratio)/logScaleFactor);
}
}
