#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <QMutex>
class MapPoint;
class KeyFrame;
namespace KINECT_SLAM
{
class Map{
  public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void setReferenceMapPoints(const std::vector<MapPoint*>& vpMPs);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();//how many mappoints
    long unsigned KeyFramesInMap();//how many keyframes

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;//best keyframes
    long unsigned int GetMaxKFid();//last keyframe id

    QMutex mQMutexMapUpdate;
    QMutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints; //store all the mappoints
    std::set<KeyFrame*> mspKeyFrames; //store all the keyframes

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    QMutex mMutexMap;


};
}
#endif
