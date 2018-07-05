#include "Map.h"
namespace KINECT_SLAM
{
Map::Map():mnMaxKFid(0)
{

}

void Map::AddKeyFrame(KeyFrame *pKF){
    QMutexLocker locker(&mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid = pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP){
    QMutexLocker locker (&mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP){

    QMutexLocker locker(&mMutexMap);
    mspMapPoints.erase(pMP);
}

void Map::EraseKeyFrame(KeyFrame *pKF){
    QMutexLocker locker(&mMutexMap);
    mspKeyFrames.erase(pKF);
}

void Map::setReferenceMapPoints(const std::vector<MapPoint *> &vpMPs){
    QMutexLocker locker(&mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<KeyFrame*> Map::GetAllKeyFrames(){
    QMutexLocker locker(&mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    QMutexLocker locker(&mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap(){
    QMutexLocker locker(&mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap(){
    QMutexLocker locker(&mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints(){
    QMutexLocker locker(&mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid(){
    QMutexLocker locker(&mMutexMap);
    return mnMaxKFid;
}

void Map::clear(){
    for(set<MapPoint*>::iterator it = mspMapPoints.begin();it!=mspMapPoints.end();it++){
        delete *it;
    }
    for(set<KeyFrame*>::iterator kit = mspKeyFrames.begin();kit!=mspKeyFrames.end();kit++){
        delete *kit;
    }
    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}
}
