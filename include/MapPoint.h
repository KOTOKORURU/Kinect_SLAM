#ifndef MAPPOINT_H
#define MAPPOINT_H

#include<QMutex>
#include"Frame.h"
#include"Map.h"
#include<opencv2/core/core.hpp>
namespace KINECT_SLAM
{
class MapPoint{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();//normalize the scale to 1
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
       void IncreaseFound(int n=1);
       float GetFoundRatio();
       inline int GetFound(){
           return mnFound;
       }
       //compute the most Distinctive descriptor to discribe this mappoint
       void ComputeDistinctiveDescriptors();

       cv::Mat GetDescriptor();

       void UpdateNormalAndDepth();

       float GetMinDistanceInvariance();
       float GetMaxDistanceInvariance();
       int PredictScale(const float &currentDist, const float &logScaleFactor);
public:
    long unsigned int mnId; ///< Global ID for MapPoint
    static long unsigned int nNextId;
    const long int mnFirstKFid; ///< ID of the keyframe who create this mappoint
    const long int mnFirstFrame; ///<ID of the frame who create this mappoint
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    int mnTrackScaleLevel;
    float mTrackViewCos;

    bool mbTrackInView;

    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static QMutex mGlobalMutex;
protected:
    cv::Mat mWorldPos;
    std::map<KeyFrame*,size_t> mObservations;
    cv::Mat mNormalVector;
    cv::Mat mDescriptor;
    //Reference keyframe
    KeyFrame* mpRefKF;
    //tracking counters
    int mnVisible;
    int mnFound;

    //bad flag
    bool mbBad;
    MapPoint* mpReplaced;

    // Scale invariance distances
    float mfMinDistance;
    float mfMaxDistance;

    Map* mpMap;

    QMutex mMutexPos;
    QMutex mMutexFeatures;

};
}

#endif
