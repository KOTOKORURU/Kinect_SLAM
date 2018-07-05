#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include "parameter_setting.h"
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
namespace KINECT_SLAM
{
class MapPoint;
class KeyFrame;

class Frame{
  public:
    Frame();
    //Copy constructor.
    Frame(const Frame &frame);
    Frame(const cv::Mat &imGray, cv::Mat&imRGB,const cv::Mat &imDepth, ORBextractor* extractor,ORBVocabulary* voc, const float &thDepth,const sensor_msgs::CameraInfoConstPtr cam_info);
    //Extract the keypoints
    void ExtractORB(const cv::Mat &im);
    //convert the keypoints to bagwords vector and features vector
    void ComputeBoW();
    void SetPose(cv::Mat Tcw);
    void UpdatePoseMatrices();
    cv::Mat GetCameraCenter()
    {
      return mOw.clone();
    }

    cv::Mat GetRotationInverse()
    {
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking

    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);
    //Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
    //to gain the keypoints which in xy center with the radius r area
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    void ComputeStereoFromKinect(const cv::Mat &imDepth);
    cv::Mat UnprojectStereo(const int &i);

public:
    ORBVocabulary* mpORBvocabulary;
    ORBextractor* mpORBextractor;
    ros::Time mTimeStamp;
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;
    sensor_msgs::CameraInfo cam_info;
    float mbf;//base_line
    float mb;

    float mThDepth;

    int N;//numbers of keypoints

    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<float> mvDepth;
    std::vector<float> mvuRight;

    //Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    cv::Mat mDescriptors;
    std::vector<MapPoint*> mvpMapPoints;
    std::vector<bool> mvbOutlier;

    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    //64X48
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
    //camera pose
    cv::Mat mTcw;
    static long unsigned int nNextId;
    long unsigned int mnId;//id

    KeyFrame* mpReferenceKF;//Point to the kf constructed by this frame

    pointcloud_type::Ptr pc_col;
    int mnScaleLevels;//levels
    float mfScaleFactor;//sacale factor
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;//ScaleFactor *ScaleFactor
    vector<float> mvInvLevelSigma2;

    static bool mbInitialComputations;
    // Undistorted Image Bounds (computed once).
       static float mnMinX;
       static float mnMaxX;
       static float mnMinY;
       static float mnMaxY;

private:
    //undistort keypoints given OpenCV distortion parameters
    void UndistortKeyPoints();
    //Compute Image Bounds for undistorted image
    void ComputeImageBounds(const cv::Mat& im);
    //Assign the keypoints to the grid for speed up feature matching
    void AssignFeaturesToGrid();

    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw;

};
}

#endif
