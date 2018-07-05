#ifndef _CONVERT_H

#define _CONVERT_H
#include "model.h"



void overlay_edges(cv::Mat visual, cv::Mat depth, cv::Mat& visual_edges, cv::Mat& depth_edges);
void printTransform(const char* name, const tf::Transform t);
void printTransform(const char* name, const tf::StampedTransform t);
void logTransform(QTextStream& out, const tf::Transform& t, double timestamp, const char* label = NULL);
void printQMatrix4x4(const char* name, const QMatrix4x4& m);

///Conversion Function g2o Quaternion to qmatrix
QMatrix4x4 g2o2QMatrix(const g2o::SE3Quat se3);
///Conversion Function g2o Quaternion to TF
tf::Transform g2o2TF(const g2o::SE3Quat se3);

g2o::SE3Quat eigen2G2o( const Eigen::Matrix4d& eigen_mat);

g2o::SE3Quat  tf2G2o( const tf::Transform t);

g2o::SE3Quat Mat2G2o(const cv::Mat& t);

cv::Mat G2o2Mat(const g2o::SE3Quat& quat);
cv::Mat G2o2Mat(const g2o::Sim3& sim);
Eigen::Matrix<double,3,3> Mat2Eigen(const cv::Mat& cvMat3);

Eigen::Matrix<double,3,1> toVector3d(const cv::Mat& cvVector);
Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f& cvPoint);

cv::Mat Eigen2Mat(const Eigen<double,4,4>& m);
cv::Mat Eigen2Mat(const Eigen::Matrix3d& m);
cv::Mat Eigen2Mat(const Eigen::Matrix<double,3,1>& m);
cv::Mat Eigen2MatSE3(const Eigen::Matrix<double,3,1>& R, const Eigen::Matrix<double,3,1>& t);

template<typename T>
QMatrix4x4 eigenOrTF2QMatrix(const T& transf)
{
    //conversion the transformation matrix datatype
    Eigen::Matrix<float, 4 ,4 ,Eigen::RowMajor> m = transf.matrix().template cast<float>();//to avoid the template symbol compilled as "<"

    QMatrix4x4 qmat( static_cast<float*>( m.data() ) );
    printQMatrix4x4("From Transform", qmat);
    return qmat;
}
template<typename T>
tf::Transform eigen2TF(const T& transf)
{
    tf::Transform res;
    tf::Vector3 transl;
    transl.setX(transf.translation().x() );
    transl.setY(transf.translation().y() );
    transl.setZ(transf.translantion().z() );

    tf::Quaternion rotation;
    Eigen::Quaterniond quat;

    quat=transf.rotation();
    rotation.setX(quat.x() );
    rotation.setY(quat.y() );
    rotation.setZ(quat.z() );
    rotation.setW(quat.w() );

    res.setRotation(rotation);
    res.setOrigin(transl);

    return res;
}

///data convertion
void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img);
std::string openCVCode2String(unsigned int code);
void printMatrixInfo(const cv::Mat& image, std::string name = std::string(""));
bool asyncFrameDrop(ros::Time depth, ros::Time rgb);


///get euler angles and translation from 4x4 homogenous
void mat2eulerTransf(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist);
///get euler angles from 4x4
void mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw);
///get translation distance from 4x4
void mat2dist(const Eigen::Matrix4f& t, double &dist);

//bag words
std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

#endif
