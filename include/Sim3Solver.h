#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "KeyFrame.h"

namespace KINECT_SLAM
{
 class Sim3Solver
{
 public:
     Sim3Solver(KeyFrame* pKF1,KeyFrame* pKF2, const std::vector<MapPoint*>& vpMatched12, const bool bFixScale = true);

     void SetRansacParameters(double probability = 0.99, int minInliers = 6, int maxIterations = 300);
     cv::Mat find(std::vector<bool>& vbInliers12, int& nInliers);
     cv::Mat iterate(int nIterations,bool &bNoMore, std::vector<bool>& vbInliers, int& nInliers);

     cv::Mat GetEstimatedRotation();
     cv::Mat GetEstimatedTranslation();
     float GetScale();

 protected:

     void ComputeCentroid(cv::Mat& P, cv::Mat& Pr, cv::Mat& C);//geometry center
     void ComputeSim3(cv::Mat& P1, cv::Mat& P2);

     void CheckInliers();

     void Project(const std::vector<cv::Mat>& vP3Dw, std::vector<cv::Mat>& vP2D, cv::Mat Tcw, cv::Mat K);
     void FromCameraToImage(const std::vector<cv::Mat>& vP3Dc, std::vector<cv::Mat>& vP2D, cv::Mat K);

 protected:
     //keyframe and matches keypoint
     KeyFrame* mpKF1;
     KeyFrame* mpKF2;

     std::vector<cv::Mat> mvX3Dc1;
     std::vector<cv::Mat> mvX3Dc2;

     std::vector<MapPoint*> mvpMapPoints1;
     std::vector<MapPoint*> mvpMapPoints2;
     std::vector<MapPoint*> mvpMatches12;

     std::vector<size_t> mvnIndices1;
     std::vector<size_t> mvSigmaSquare1;
     std::vector<size_t> mvSigmaSquare2;
     std::vector<size_t> mvnMaxError1;
     std::vector<size_t> mvnMaxError2;

     int N;
     int mN1;

     //Current Estimation
     cv::Mat mR12i;
     cv::Mat mt12i;
     float ms12i;
     cv::Mat mT12i;
     cv::Mat mT21i;
     std::vector<bool> mvbInliersi;
     int mnInliersi;

     //Current Ransac State
     int mnIterations;
     std::vector<bool> mvbBestInliers;
     int mnBestInliers;
     cv::Mat mBestT12;
     cv::Mat mBestRotation;
     cv::Mat mBestTranslation;
     float mBestScale;
     //Set the scale 1 to the kinect camera
     bool mbFixScale;

     //Indices for random selection
     std::vector<size_t> mvAllIndices;

     //Projections
     std::vector<cv::Mat> mvP1im1;
     std::vector<cv::Mat> mvP2im2;

     //Ransac probability
     double mRansacProb;

     //Ransac min inliers
     int mRansacMinInliers;

     //Ransac max iterations
     int mRansacMaxIts;

     //Threshold inlier/outlier. e=dist(Pi,T_ij*Pj)^2 < 5.991 * mSigma2 reproject error < 6 * sigma
     float mTh;
     float mSigma2;

     //Calibration
     cv::Mat mK1;
     cv::Mat mK2;


 };
}

#endif
