#ifndef _MODEL_H
#define _MODEL_H
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <QString>
#include <QMatrix4x4>

#include <ctime>
#include <limits>
#include <algorithm>
#include <cv.h>
#include "scoped_timer.h"
#include "header.h"

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/vertex_se3.h"

#include <pcl_ros/transforms.h>
#include "pcl/common/io.h"
#include "pcl/common/distances.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <omp.h>
#include "parameter_setting.h"


//for the observability
#include <boost/math/distributions/chi_squared.hpp>
#include <numeric>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

///noisy Model
//from the paper <Accuracy and Resolution of Kinect Depth Data for IndoorMapping Applications>
//modify the factor 1/2*2.85e-5 to 0.001
inline double depth_std_dev_1(double depth)
{
    double depth_std_dev = ParameterSetting::instance()->get<double>("depth_sigma");

    return depth_std_dev * std::pow(depth,2);
}

//from the paper <Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking>
//from bewteen the angle 10~60
inline double depth_std_dev_2(double depth)
{
    double depth_std_dev = 0.0012+0.0019*std::pow((depth-0.4),2);

    return depth_std_dev;
}

inline double depth_std_mean(double depth)
{
    static double depth_std_mean = ( depth_std_dev_1(depth) + depth_std_dev_2(depth) ) / 2;

    return depth_std_mean;
}
inline double depth_convariance_1(double depth)
{
   static double stddev_1 = depth_std_dev_1(depth);
   static double cov_1 = stddev_1 * stddev_1;
   return cov_1;
}

inline double depth_convariance_2(double depth)
{
    static double stddev_2 = depth_std_dev_2(depth);
    static double cov_2 = stddev_2 * stddev_2;
    return cov_2;
}
inline double depth_convariance_mean(double depth)
{
    static double stddev_mean = depth_std_mean(depth);
    static double cov_mean = stddev_mean * stddev_mean;
    return cov_mean;
}

inline Eigen::Matrix3d pixel_point_info_matrix(double d)
{
    Eigen::Matrix3d inf_mat = Eigen::Matrix3d::Identity();

    double depth_cov_1 = depth_convariance_1(d);
    double depth_cov_2 =depth_convariance_2(d);

    double mean = (depth_cov_1 + depth_cov_2)/2;

    inf_mat(2,2) = 1.0 / mean;

    return inf_mat;
}

//2d->3d
inline void backproject(const float fx_inv, const float fy_inv,
                        const float cx, const float cy,
                        const float u,const float v, const float z,
                        float& out_x, float& out_y, float& out_z)
{
    out_x = (u - cx) * z * fx_inv;
    out_y = (v - cy) * z * fy_inv;
    out_z = z;
}
inline int round(float d)
{
    return static_cast<int>(floor(d + 0.5));
}

///create pointcloud from rgb images
//pointcloud_type* createXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
pointcloud_type* createXYZRGBPointCloud (const cv::Mat& depth_msg, const cv::Mat& rgb_msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
void transformAndAppendPointCloud (const pointcloud_type &cloud_in, pointcloud_type &cloud_to_append_to,
                                   const tf::Transform transformation, float Max_Depth, int idx=0);

geometry_msgs::Point pointInWorldFrame(const Eigen::Vector4f& point3d, const g2o::VertexSE3::EstimateType& transf);

///if motion is too intense translation > 10cm or largest euler-angle>5 deg
bool isBigTrafo(const Eigen::Isometry3d& t);
bool isBigTrafo(const g2o::SE3Quat& t);
/// if motion is too gentle
bool isSmallTrafo(const g2o::SE3Quat& t, double seconds = 1.0);
bool isSmallTrafo(const Eigen::Isometry3d& t, double seconds = 1.0);
void transfSize(const Eigen::Isometry3d& t,double& angle, double& dist);


///Model function
//to compute the mahalanobis distance  between the two points
double mahalanobis_distance(const Eigen::Vector4f& p1,
                      const Eigen::Vector4f& p2,
                      const Eigen::Matrix4d& tf_1_to_2);

float getMinDepthInNeighborhood(const cv::Mat& depth, cv::Point2f center, float diameter);

//emm model
void observationLikelihood(const Eigen::Matrix4f& proposed_transformation,//new to old
                             pointcloud_type::Ptr new_pc,
                             pointcloud_type::Ptr old_pc,//point_cloud
                             const sensor_msgs::CameraInfo& old_cam_info,
                             double& likelihood,
                             double& confidence,
                             unsigned int& inliers,
                             unsigned int& outliers,
                             unsigned int& occluded,
                             unsigned int& all);

//wether the points in a Keyframe meet the criterion
bool observation_criterion_met(unsigned int inliers, unsigned int outliers, unsigned int all, double& quality);

void getColor(const point_type& p, unsigned char& r, unsigned char& g, unsigned char& b);

#endif
