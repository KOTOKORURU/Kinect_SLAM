#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDataBase.h"
#include <QThread>
#include <QtConcurrentMap>
#include <qtconcurrentrun.h>
#include <QMutex>

namespace KINECT_SLAM
{
 class Tracking;
 class LoopClosing;
 class Map;

class LocalMapping
{
public:
    LocalMapping(Map* pMap);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    //main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    //Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStop();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();
    void RequestFinish();

    int KeyFrameInQueue(){
        QMutexLocker loker(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoint();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat& v);

    void ResetIfRequested();
    bool mbResetRequestedl;
    QMutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    QMutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    //tracking thread insert a keyframe into the list
    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    QMutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;

    QMutex mMutexStop;

    bool mbAcceptKeyFrames;
    QMutex mMutexAccept;
};

}//namespace

#endif
