#include "Convert.h"
//convert the rgba data


void  printQMatrix4x4(const char* name, const QMatrix4x4& m)
{
    ROS_DEBUG("QMATRIX %s:", name);
    ROS_DEBUG("%f\t%f\t%f\t%f", m(0,0), m(0,1) ,m(0,2) ,m(0,3));
    ROS_DEBUG("%f\t%f\t%f\t%f", m(1,0), m(1,1) ,m(1,2) ,m(1,3));
    ROS_DEBUG("%f\t%f\t%f\t%f", m(2,0), m(2,1) ,m(2,2) ,m(2,3));
    ROS_DEBUG("%f\t%f\t%f\t%f", m(3,0), m(3,1) ,m(3,2) ,m(3,3));
}

void printTransform(const char* name, const tf::StampedTransform t)
{

    ROS_INFO_STREAM(name<<": Translation "<<t.getOrigin().x() << " "<<t.getOrigin().y() << " " <<t.getOrigin().z());
    ROS_INFO_STREAM(name<<": Rotation "<< t.getRotation().getX()<< " "<< " " << t.getRotation().getY() << " " << t.getRotation().getZ());
    ROS_INFO_STREAM(name<<": from " << t.frame_id_ << " to " <<t.child_frame_id_ << " at time: " <<t.stamp_.sec);

}

void printTransform(const char* name, const tf::Transform t)
{
    ROS_INFO_STREAM(name << ": Translation " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z());
    ROS_INFO_STREAM(name << ": Rotation " << t.getRotation().getX() << " " << t.getRotation().getY() << " " <<    t.getRotation().getZ() << " " << t.getRotation().getW());

}

QMatrix4x4 g2o2QMatrix(const g2o::SE3Quat se3)
{
  Eigen::Matrix<float, 4, 4> tmp =se3.to_homogeneous_matrix().cast<float>();

  QMatrix4x4 qmat(static_cast<float*>( tmp.data()), 4, 4);

  printQMatrix4x4("conversion", qmat.transposed());

  return qmat.transposed();//cause g2o use different cols and rows
}

tf::Transform g2o2TF(const g2o::SE3Quat se3)
{
    tf::Transform res;
    tf::Vector3 trans;
    trans.setX(se3.translation().x() );
    trans.setY(se3.translation().y() );
    trans.setZ(se3.translation().z() );

    tf::Quaternion rot;
    rot.setX(se3.rotation().x() );
    rot.setY(se3.rotation().y() );
    rot.setZ(se3.rotation().z() );
    rot.setW(se3.rotation().w() );

    res.setOrigin(trans);
    res.setRotation(rot);

    return res;
}

void mat2dist(const Eigen::Matrix4f& t, double &dist)
{
    dist = sqrt(std::pow(t(0,3),2 ) + std::pow(t(1,3),2) + std::pow(t(2,3),2));

}

void mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw)
{
    roll = atan2(t(2,1),t(2,2));//tan(x)=zz/zy

    pitch = atan2(-t(2,0), sqrt(std::pow(t(2,1),2)) + std::pow(t(2,2),2)) ;

    yaw = atan2(t(1,0), t(0,0));
}

void mat2eulerTransf(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist)
{
    mat2RPY(t,roll,pitch,yaw);
    mat2dist(t,dist);

    roll=roll/M_PI * 180;
}

g2o::SE3Quat  tf2G2o( const tf::Transform t)
{
  Eigen::Quaterniond quat(t.getRotation().getW(), t.getRotation().getX(), t.getRotation().getY(), t.getRotation().getZ());
  Eigen::Vector3d  trans(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z());
  g2o::SE3Quat res(quat,trans);
  return res;
}

g2o::SE3Quat eigen2G2o( const Eigen::Matrix4d& eigen_mat)
{
    Eigen::Isometry3d transf(eigen_mat);
    Eigen::Quaterniond quat(transf.rotation());
    Eigen::Vector3d trans(eigen_mat(0,3), eigen_mat(1,3) , eigen_mat(2,3));

    g2o::SE3Quat res(quat,trans);

    return res;
}

g2o::SE3Quat Mat2G2o(const cv::Mat &t)
{
  Eigen::Matrix<double,3,3> R;
  R << t.at<float>(0,0), t.at<float>(0,1), t.at<float>(0,2),
       t.at<float>(1,0), t.at<float>(1,1), t.at<float>(1,2),
       t.at<float>(2,0), t.at<float>(2,1), t.at<float>(2,2);

  Eigen::Matrix<double,3,1> t(t.at<float>(0,3), t.at<float>(1,3), t.at<float>(2,3));
  return g2o::SE3Quat(R,t);
}

cv::Mat G2o2Mat(const g2o::SE3Quat& quat)
{
    Eigen::Matrix<double,4,4> eigMat = quat.to_homogeneous_matrix();
    return Eigen2Mat(eigMat);
}

cv::Mat G2o2Mat(const g2o::Sim3& sim)
{
    Eigen::Matrix3d R = sim.rotation().toRotationMatrix();
    Eigen::Vector3d t = sim.translation();
    double s=sim.scale();

    return Eigen2MatSE3(s*R,t);
}

Eigen::Matrix<double,3,3> Mat2Eigen(const cv::Mat& cvMat3)
{
    Eigen::Matrix<double,3,3> M;
    M<< cvMat3.at<float>(0,0),cvMat3.at<float>(0,1),cvMat.at<float>(0,2),
        cvMat3.at<float>(1,0),cvMat3.at<float>(1,1),cvMat.at<float>(1,2),
        cvMat3.at<float>(2,0),cvMat3.at<float>(2,1),cvMat.at<float>(2,2);

   return M;
}

Eigen::Matrix<double,3,1> toVector3d(const cv::Mat& cvVector)
{
 Eigen::Matrix<double,3,1> vector;
 vector << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);
 return vector;
}

Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f& cvPoint)
{
    Eigen::Matrix<double,3,1> vector;
    vector << cvPoint.x, cvPoint.y, cvPoint.z;
    return vector;
}

cv::Mat Eigen2Mat(const Eigen<double,4,4>& m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Eigen2Mat(const Eigen::Matrix3d& m)
{
    cv::Mat mat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        mat.at<float>(i,j) = m(i,j);

    return mat.clone();
}

cv::Mat Eigen2Mat(const Eigen::Matrix<double,3,1>& m)
{
    cv::Mat mat(3,1,CV_32F);
    for(int i=0;i<3;i++)
        mat.at<float>(i) = m(i);
}

cv::Mat Eigen2MatSE3(const Eigen::Matrix<double,3,1>& R, const Eigen::Matrix<double,3,1>& t)
{
    cv::Mat mat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            mat.at<float>(i,j) = R(i,j);
        }

    }
    for(int i=0;i<3;i++)
    {
        mat.at<float>(i,3) = t(i);
    }
    return mat.clone();
}

void printMatrixInfo(const cv::Mat& image, std::string name)
{
   ROS_INFO_STREAM("T theMatrix " << name << " - Type:" << openCVCode2String(image.type()) <<  " rows: " <<  image.rows  <<  " cols: " <<  image.cols);
}

//code = image.type();
std::string openCVCode2String(unsigned int code)
{
    switch(code){
        case 0 : return std::string("CV_8UC1" );
        case 8 : return std::string("CV_8UC2" );
        case 16: return std::string("CV_8UC3" );
        case 24: return std::string("CV_8UC4" );
        case 2 : return std::string("CV_16UC1");
        case 10: return std::string("CV_16UC2");
        case 18: return std::string("CV_16UC3");
        case 26: return std::string("CV_16UC4");
        case 5 : return std::string("CV_32FC1");
        case 13: return std::string("CV_32FC2");
        case 21: return std::string("CV_32FC3");
        case 29: return std::string("CV_32FC4");
      }
      return std::string("Unknown");
}
//convert the depth image to the mono type
void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img)
{
   if(depth_img.type() == CV_32F){
       depth_img.convertTo(mono8_img, CV_8UC1, 100.0);
   }
   else if(depth_img.type() == CV_16UC1){
       mono8_img = cv::Mat(depth_img.size(), CV_8UC1);
       cv::Mat tmp;
       depth_img.convertTo(mono8_img, CV_8UC1, 0.05 ,-25 );//scale 2cm 16=>8
       depth_img.convertTo(tmp, CV_32FC1, 0.001 , 0);//16f's depth scale=5000
       depth_img = tmp;//depth=>to the 32bit
   }
   else{
       printMatrixInfo(depth_img, "Depth Image");
       ROS_ERROR_STREAM("sorry,the type of depth image is:"<< openCVCode2String(depth_img.type()));
   }
}

bool asyncFrameDrop(ros::Time depth, ros::Time rgb)
{
    long rgb_time=abs(static_cast<long>(rgb.nsec) - static_cast<long>(depth.nsec));
    if(rgb_time > 33333333){// difference > 1/30s

        ROS_INFO("Depth and RGB image off more than 1/30sec: %li (nsec)", rgb_time);
        if(ParameterSetting::instance() -> get<bool>("drop_async_frames")){
            ROS_WARN("Asynchronous frames ignored.");
            return true;
        }
    }
     else{
            ROS_DEBUG("Depth image time: %d - %d", depth.sec,depth.nsec);
            ROS_DEBUG("RGB   image time: %d - %d", rgb.sec,rgb.nsec);
         }

    return false;
}

std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}
