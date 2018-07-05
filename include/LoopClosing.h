#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapper.h"
#include "Map.h"
#include "ORBVocabulary.h"
//#include "Tracking.h"
#include "KeyFrameDataBase.h"

//#include <QThread>
#include <QMutex>
#include <QtConcurrent>
//#include <thread>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace KINECT_SLAM
{
//class Tracking
//class LocalMapping
class KeyFrameDatabase;

class LoopClosing
{
public:
    typedef pair<set<KeyFrame*>,int> ConsistentGroup;
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
            Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

    public:

        LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

        void SetTracker(Tracking* pTracker);

        void SetLocalMapper(LocalMapping* pLocalMapper);

        // Main function
        void Run();

        void InsertKeyFrame(KeyFrame *pKF);

        void RequestReset();

        // This function will run in a separate thread
        void RunGlobalBundleAdjustment(unsigned long nLoopKF);

        bool isRunningGBA(){
            QMutexLocker lock(&mMutexGBA);
            return mbRunningGBA;
        }
        bool isFinishedGBA(){
            QMutexLocker lock(&mMutexGBA);
            return mbFinishedGBA;
        }

        void RequestFinish();

        bool isFinished();

    protected:

        bool CheckNewKeyFrames();

        bool DetectLoop();

        bool ComputeSim3();

        void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

        void CorrectLoop();

        void ResetIfRequested();
        bool mbResetRequested;
        QMutex mMutexReset;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        QMutex mMutexFinish;

        Map* mpMap;
        Tracking* mpTracker;

        KeyFrameDatabase* mpKeyFrameDB;
        ORBVocabulary* mpORBVocabulary;

        LocalMapping *mpLocalMapper;

        std::list<KeyFrame*> mlpLoopKeyFrameQueue;

        std::mutex mMutexLoopQueue;

        // Loop detector parameters
        float mnCovisibilityConsistencyTh;

        // Loop detector variables
        KeyFrame* mpCurrentKF;
        KeyFrame* mpMatchedKF;
        std::vector<ConsistentGroup> mvConsistentGroups;
        std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
        std::vector<KeyFrame*> mvpCurrentConnectedKFs;
        std::vector<MapPoint*> mvpCurrentMatchedPoints;
        std::vector<MapPoint*> mvpLoopMapPoints;
        cv::Mat mScw;
        g2o::Sim3 mg2oScw;

        long unsigned int mLastLoopKFid;

        // Variables related to Global Bundle Adjustment
        bool mbRunningGBA;
        bool mbFinishedGBA;
        bool mbStopGBA;
        QMutex mMutexGBA;
        QFuture<void>* mpThreadGBA;

        // Fix scale in the stereo/RGB-D case
        bool mbFixScale;
};
}
#endif
