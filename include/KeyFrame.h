#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDataBase.h"
#include<QMutex>

class KeyFrame {
public:
    KeyFrame(Frame& F,Map* pMap, KeyFrameDatebase* pKFDB);

    void setPose(const cv::Mat& Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    void ComputeBoW();

    //Covisibility graph
    void AddConnection(KeyFrame* pKF, const int& weight);
    void EraseConnection(KeyFrame* pKf);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame*> GetConnectedKeyFrames();
    std::vector<KeyFrame*> GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int& N);
    std::vector<KeyFrame*> GetCovisibilesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    //spanning tree
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    //loop Edges
    void ADDLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    //MapPoint observation
    void AddMapPoint(MapPoint* pMP,const size_t& idx);
    void EraseMapPointMatch(const size_t& idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t& idx,MapPoint& pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int& minObs);
    MapPoint* GetMapPoint(const size_t& idx);

    //KeyPoints
    std::vector<size_t> GetFeaturesInArea(const float& x,const float& y,const float& r)const;
    cv::Mat UnprojectStereo(int i);

    //IMage
    bool IsInImage(const float& x, const float& y) const;

    //Bad flag,whether to disable this KeyFrame
    void SetNotErase();
    void SetErase();

    //check/set bad flag
    void SetBadFlag();
    bool isBad();
    //float ComputeSceneMedianDepth(const int q);
    static bool weightComp(int a, int b)
    {
        return a>b;
    }
    static bool lId(KeyFrame* pKF1,KeyFrame* pKF2)
    {
        return pKF1->mnId < pKF2->mnId;
    }

public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;
    //Grid
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    //tracking Variables
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

   //local mapping Variables
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    //keyframe database variables
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    //loop closing Variables
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    //Calibration parameters
    const float fx,fy,cy,cy,invfx,invfy,mbf,mb,mThDepth;

    const int N;
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvu;
    const std::vector<float> mvDepth;
    const cv::Mat mDescriptors;

    //BoW
    DBow2::BowVector mBowVec;
    DBow2::FeatureVector mFeatVec;

    //// Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    //Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    //Image bounds and calibration
    const int mnMinX;
    const int mnMaxX;
    const int mnMinY;
    const int mnMaxY;
    const cv::Mat mK;
    sensor_msgs::CameraInfoConstPtr cam_info;
    pointcloud_type::Ptr pc_col;


protected:
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    std::vector<MapPoint*> mvpMapPoints;

    //BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    //grid image
    std:vector< std::vector<std::vector<size_t> > > mGrid;

    //Covisibility Graph
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    //Spanning tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    //Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    //float mHalfBaseline;

    Map* mpMap;

    QMutex mMutexPose;
    QMutex mMutexConnections;
    QMutex mMutexFeatures;

};
#endif
