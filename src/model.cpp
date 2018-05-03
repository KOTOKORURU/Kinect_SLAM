#include "model.h"
typedef union
{
    struct
    {
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;

    };
    float float_value;
    long long_value;

}RGBValue;

#define SQRT_2_PI 2.5066283
#define SQRT_2 1.41421
#define LN_SQRT_2_PI 0.9189385332

//get camera intrinsics
static void getIntrinsics(float& fx, float& fy, float& cx, float& cy, const sensor_msgs::CameraInfo& cam_info)
{
    ParameterSetting* ps=ParameterSetting::instance();
    fx = ps->get<double>("camera_fx") == 0 ? cam_info.K[0]:ps->get<double>("camera_fx");
    fy = ps->get<double>("camera_fy") == 0 ? cam_info.K[4]:ps->get<double>("camera_fy");
    cx = ps->get<double>("camera_cx") == 0 ? cam_info.K[2]:ps->get<double>("camera_cx");
    cy = ps->get<double>("camera_cy") == 0 ? cam_info.K[5]:ps->get<double>("camera_cy");
}
static void getDistortions(float& k1, float& k2, float& k3, float& p1, float& p3, const sensor_msgs::CameraInfo& cam_info)
{
    k1 = cam_info.D[0];
    k2 = cam_info.D[1];
    k3 = cam_info.D[4];
    p1 = cam_info.D[2];
    p2 = cam_info.D[3];
}

//get inv focal length inv
static void getForcalLengthInv(float& fxinv, float& fyinv, float& cx, float& cy,const sensor_msgs::CameraInfo& cam_info)
{
    getIntrinsics(fxinv, fyinv, cx, cy , cam_info);

    fxinv = 1. / fxinv;
    fyinv = 1. / fyinv;
}
//point in world frame
geometry_msgs::Point pointInWorldFrame(const Eigen::Vector4f& point3d, const g2o::VertexSE3::EstimateType& transf)
{
    Eigen::Vector3d tmp(point3d[0], point3d[1], point3d[2]);

    tmp = transf * tmp; // transform the point in worldframe

    geometry_msgs::Point p;

    p.x = tmp.x();
    p.y = tmp.y();
    p.z = tmp.z();

    return p;
}
void transfSize(const Eigen::Isometry3d& t,double& angle, double& dist)
{
    angle = acos( (t.rotation().trace()-1) / 2) * 180 /M_PI;
    dist = t.translation().norm();
    ROS_INFO("Rotation: %4.2f , Distance: % 4.3fm",angle,dist);
}

bool isBigTrafo(const Eigen::Isometry3d& t)
{
  double angle, dist;
  transfSize(t, angle, dist);
  return (dist>ParameterSetting::instance()->get<double>("min_translation_meter")||
          angle>ParameterSetting::instance()->get<double>("min_rotation_degree"));
}
bool isBigTrafo(const g2o::SE3Quat& t)
{
    float angle = 2.0 * acos(t.rotation().w()) *180 / M_PI;
    float dist = t.translation().norm();
    QString infostring;
    ROS_INFO("Rotation: %4.2f , Distance: % 4.3fm",angle,dist);
    infostring.sprintf("Rotation: %4.2f , Distance: % 4.3fm",angle,dist);
    return (dist>ParameterSetting::instance()->get<double>("min_translation_meter")||
            angle>ParameterSetting::instance()->get<double>("min_rotation_degree"));

}

bool isSmallTrafo(const g2o::SE3Quat& t, double seconds )
{
    if( seconds <= 0.0) {
        ROS_WARN("Time delta invalid: %f. Skipping test for small transformation", seconds);
        return true;
    }
    float angle = 2.0 * acos(t.rotation().w()) *180 / M_PI;
    float dist = t.translation().norm();
    QString infostring;
    ROS_INFO("Rotation: %4.2f , Distance: % 4.3fm",angle,dist);
    infostring.sprintf("Rotation: %4.2f , Distance: % 4.3fm",angle,dist);
    return (dist/ seconds < ParameterSetting::instance()->get<double>("max_translation_meter") &&
            angle / seconds < ParameterSetting::instance()->get<double>("max_rotation_degree"));
}
bool isSmallTrafo(const Eigen::Isometry3d& t, double seconds)
{
    if( seconds <= 0.0) {
        ROS_WARN("Time delta invalid: %f. Skipping test for small transformation", seconds);
        return true;
    }
    double angle, dist;
    transfSize(t, angle, dist);

    return (dist/ seconds < ParameterSetting::instance()->get<double>("max_translation_meter") &&
            angle / seconds < ParameterSetting::instance()->get<double>("max_rotation_degree"));
}
pointcloud_type* createXYZRGBPointCloud (const cv::Mat& depth_msg, const cv::Mat& rgb_msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    ScopedTimer s(__FUNCTION__);

    pointcloud_type* cloud(new pointcloud_type());
    cloud-> is_dense = false;//sparse


    float fxinv,fyinv,cx,cy;
    getForcalLengthInv(fxinv, fyinv, cx, cy, *cam_info);
    ParameterSetting* ps = ParameterSetting::instance();
    int data_skip_step = ps->get<int>("cloud_creation_skip_step");
    if(depth_msg.rows % data_skip_step != 0 || depth_msg.cols % data_skip_step != 0){
        ROS_WARN("The parameter cloud_creation_skip_step is not a divisor of the depth image dimensions.");
      }
    //just a zoom the point cloud
    cloud->height = ceil(depth_msg.rows / static_cast<float>(data_skip_step));
    cloud->width = ceil(depth_msg.cols / static_cast<float>(data_skip_step));
    cloud->points.resize (cloud->height * cloud->width);

    //rgb image
    char red_idx = 0, green_idx = 1, blue_idx = 2;
    int pixel_data_size = 0;
    if(rgb_msg.type() == CV_8UC1)
         pixel_data_size = 1;
    else
        pixel_data_size = 3;
    if(ps->get<bool>("encoding_bgr")){
        red_idx = 2;
        blue_idx = 0;
    }

    unsigned int color_row_step, color_pix_step, depth_pix_step, depth_row_step;
    //construct the rgb image's skip step to create sparse,cols: 3 channel * (rgbsize/cloudsize) 0 3 6 7 9...rows skip less a pix than cols
    color_pix_step = pixel_data_size * (rgb_msg.cols / cloud->width);
    color_row_step = pixel_data_size * (rgb_msg.rows / cloud->height -1 ) * rgb_msg.cols;

    depth_pix_step = (depth_msg.cols / cloud->width);
    depth_row_step = (depth_msg.rows / cloud->height -1 ) * depth_msg.cols;

    int color_idx = 0 , depth_idx = 0;
    double depth_scaling = ps->get<double>("depth_scaling_factor");
    float max_depth = ps->get<double>("maximum_depth");
    float min_depth = ps->get<double>("minimum_depth");
    if(max_depth < 0.0) max_depth = std::numeric_limits<float>::infinity();

    //creation
    pointcloud_type::iterator ptr = cloud->begin();
    for(int v = 0; v < (int)rgb_msg.rows; v += data_skip_step, color_idx += color_row_step, depth_idx += depth_row_step)
    {
        for(int u = 0; u < (int)rgb_msg.cols; u += data_skip_step, color_idx += color_pix_step, depth_idx +=depth_pix_step, ++ptr)
        {
            if(ptr == cloud->end())
                break;

            point_type& pt= *ptr;
            if(u < 0 || v < 0 || u >= depth_msg.cols || v >= depth_msg.rows){
                pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                continue;
            }
            float z = depth_msg.at<float>(depth_idx) * depth_scaling;

            if(!(z >= min_depth))
            {
                pt.x = (u - cx) * 1.0 * fxinv;
                pt.y = (v - cy) * 1.0 * fyinv;
                pt.z=std::numeric_limits<float>::quiet_NaN();
            }
            else
            {
                backproject(fxinv,fyinv, cx, cy, u, v , z , pt.x ,pt.y ,pt.z);
            }
            RGBValue color;
            if(color_idx > 0 && color_idx < rgb_msg.total()*color_pix_step){ //Only necessary because of the color_idx offset
                   if(pixel_data_size == 3){
                     color.Red   = rgb_msg.at<uint8_t>(color_idx + red_idx);
                     color.Green = rgb_msg.at<uint8_t>(color_idx + green_idx);
                     color.Blue  = rgb_msg.at<uint8_t>(color_idx + blue_idx);
                   } else {
                     color.Red   = color.Green = color.Blue  = rgb_msg.at<uint8_t>(color_idx);
                   }

                   color.Alpha = 0;
                   pt.data[3] = color.float_value;
        }
     }
  }
    return cloud;
}
void transformAndAppendPointCloud (const pointcloud_type &cloud_in, pointcloud_type &cloud_to_append_to,
                                   const tf::Transform transformation, float Max_Depth, int idx)
{
    bool compact = !ParameterSetting::instance()->get<bool>("preserve_raster_on_save");
    Eigen::Matrix4f eigen_transf;
    pcl_ros::transformAsMatrix(transformation,eigen_transf);
    unsigned int cloud_to_append_to_original_size = cloud_to_append_to.size();
    if(cloud_to_append_to.points.size() == 0){
        cloud_to_append_to.header = cloud_in.header;
        cloud_to_append_to.width = 0;
        cloud_to_append_to.height = 0;
        cloud_to_append_to.is_dense = false;
    }
    ROS_DEBUG("max_Depth = %f", Max_Depth);
    ROS_DEBUG("cloud_to_append_to_original_size = %i", cloud_to_append_to_original_size);

    cloud_to_append_to += cloud_in;

    Eigen::Matrix3f rot = eigen_transf.block<3,3>(0,0);
    Eigen::Vector3f trans = eigen_transf.block<3,1>(0,3);
    point_type origin = point_type();
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    int index = 0;
    for(size_t i = 0; i < cloud_in.points.size(); i++)
    {
        Eigen::Map<Eigen::Vector3f> p_in(const_cast<float*>(&cloud_in.points[i].x),3,1);
        Eigen::Map<Eigen::Vector3f> p_out(&cloud_to_append_to[index+cloud_to_append_to_original_size].x,3,1);
        if(compact){ cloud_to_append_to.points[index+cloud_to_append_to_original_size] = cloud_in.points[i]; }
        if(Max_Depth >= 0){
            if(pcl::squaredEuclideanDistance(cloud_in.points[i],origin)>Max_Depth * Max_Depth){
                p_out[0]=std::numeric_limits<float>::quiet_NaN();
                p_out[1]=std::numeric_limits<float>::quiet_NaN();
                p_out[2]=std::numeric_limits<float>::quiet_NaN();
                if(!compact) index++;
                continue;
            }
        }
        if (pcl_isnan (cloud_in.points[i].x) || pcl_isnan (cloud_in.points[i].y) || pcl_isnan (cloud_in.points[i].z)){
            if(!compact) index++;
            continue;
        }
        p_out = rot * p_in + trans;
    }
    if(compact){
          cloud_to_append_to.points.resize(index+cloud_to_append_to_original_size);
          cloud_to_append_to.width    = 1;
          cloud_to_append_to.height   = index+cloud_to_append_to_original_size;
        }

}

double mahalanobis_distance(const Eigen::Vector4f& p1,
                      const Eigen::Vector4f& p2,
                      const Eigen::Matrix4d& tf_1_to_2)
{
    const double cam_angle_x = 57.5/180*M_PI;
    const double cam_angle_y = 44.0/180*M_PI;//View angle of camera
    const double cam_resol_x = 640.0;
    const double cam_resol_y = 480.0;
    const double stddev_x = 2.5 * tan(cam_angle_x / cam_resol_x);// view_angle/pixel set the std range = 3pix
    const double stddev_y = 2.5 * tan(cam_angle_y / cam_resol_y);
    const double cov_x = std::pow(stddev_x,2);
    const double cov_y = std::pow(stddev_y,2);

    bool nan1 = std::isnan(p1(2));
    bool nan2 = std::isnan(p2(2));

    if(nan1||nan2){

        return std::numeric_limits<double>::max();
    }
    Eigen::Vector4d p_1 = p1.cast<double>();
    Eigen::Vector4d p_2 = p2.cast<double>();

    Eigen::Matrix4d tf_12 = tf_1_to_2;
    Eigen::Vector3d x_1 = p_1.head<3>();
    Eigen::Vector3d x_2 = p_2.head<3>();
    Eigen::Vector3d x_1_in_frame_x_2 = (tf_12 * p_1).head<3>();//point1 in the frame of point2;
    //short cut to judge if the distance between two point is two lagre
    double delta_sq_norm = (x_1_in_frame_x_2 - x_2).squaredNorm();
    double sig_1 = std::max(cov_x, depth_convariance_mean(x_1(2)));
    double sig_2 = std::max(cov_y, depth_convariance_mean(x_2(2)));
    if(delta_sq_norm > 2 * (sig_1+sig_2))
        return std::numeric_limits<double>::max();

    Eigen::Matrix3d rotation = tf_12.block(0,0,3,3);
    //COV_MATRIX OF POINT1
    Eigen::Matrix3d cov1 = Eigen::Matrix3d::Zero();
    cov1(0,0) = cov_x * x_1(2) * x_1(2);//x_cov in meter
    cov1(1,1) = cov_y * x_1(2) * x_1(2);
    cov1(2,2) = depth_convariance_mean(x_1(2));
    //COV_MATRIX OF POINT2
    Eigen::Matrix3d cov2 = Eigen::Matrix3d::Zero();
    cov2(0,0) = cov_x * x_2(2) * x_2(2);//x_cov in meter
    cov2(1,1) = cov_y * x_2(2) * x_2(2);
    cov2(2,2) = depth_convariance_mean(x_2(2));

    Eigen::Matrix3d cov1_in_frame_cov2 = rotation.transpose() * cov1 * rotation;
    // Δμ⁽²⁾ =  μ₁⁽²⁾ - μ₂⁽²⁾  measurement error
    Eigen::Vector3d delta_x_1_in_frame_x2 = x_1_in_frame_x_2 - x_2;

   if(std::isnan(delta_x_1_in_frame_x2(2))){
       ROS_ERROR("Unexpected NaN");
      return std::numeric_limits<double>::max();

   }
   Eigen::Matrix3d cov_sum_in_fram_x2 = cov1_in_frame_cov2 + cov2;
   double sqrd_mahalanobis_distance = delta_x_1_in_frame_x2.transpose() * cov_sum_in_fram_x2.llt().solve(delta_x_1_in_frame_x2);

   if(!(sqrd_mahalanobis_distance >= 0.0))
     {
       return std::numeric_limits<double>::max();
     }
     return sqrd_mahalanobis_distance;
}

float getMinDepthInNeighborhood(const cv::Mat& depth, cv::Point2f center, float diameter)
{
    int radius = (diameter-1)/2;
    int top = center.y - radius; top = top <0 ? 0:top;
    int bottom = center.y +radius; bottom = bottom > depth.rows ? depth.rows:bottom;
    int left = center.x - radius; left = left <0 ? 0:left;
    int right = center.x + radius; right = right > depth.cols ? depth.cols:right;

    cv::Mat neigborhood(depth, cv::Range(top,bottom),cv::Range(left,right));
    double minz = std::numeric_limits<float>::quiet_NaN();
    //to gain the min value in the mono image
    cv::minMaxLoc(neigborhood,&minz);
    if(minz == 0.0){
        ROS_INFO_THROTTLE(1,"Caught feature with zero in depth neighbourhood");
        minz = std::numeric_limits<float>::quiet_NaN();
    }
    return static_cast<float>(minz);
}

//probability of [-inf,x] of a gaussian CDF
double gaussian_cdf(double x, double mean, double sigma)
{
    return 0.5 * (1+erf((x- mean ) / (sigma * SQRT_2)));
}
//EM MODEL
void observationLikelihood(const Eigen::Matrix4f& proposed_transformation,//new to old
                             pointcloud_type::Ptr new_pc,
                             pointcloud_type::Ptr old_pc,//point_cloud
                             const sensor_msgs::CameraInfo& old_cam_info,
                             double& likelihood,
                             double& confidence,
                             unsigned int& inliers,
                             unsigned int& outliers,
                             unsigned int& occluded,
                             unsigned int& all)
{
    ScopedTimer s(__FUNCTION__);
    ParameterSetting* ps = ParameterSetting::instance();
    int skip_step = ps->get<int>("Subsample_skip_step");
    const bool mark_outliers = ps->get<bool>("mark_outliers");
    double observability_threshold = ps->get<double>("observability_threshold");
    if(skip_step<0 || observability_threshold <= 0.0)
    {
        inliers = all =1;
        return;
    }
    if(old_pc->width <=1 || old_pc->height <= 1){
        ROS_ERROR("Point not  be constructed.");
        if(ParameterSetting::instance()->get<double>("voxelfilter_size") > 0){
             ROS_ERROR(" voxelfilter_size is set. This is incompatible with the environment measurement model");
           }
           inliers = all = 1;
           return;
    }
    if(old_pc->width != new_pc->width){
       ROS_ERROR("Differing cloud dimensions. Skipping observationLikelihood.");
       return;
     }
    pointcloud_type transformed_new_pc;
    //to transform the new_pc to the old_pc's frame
    pcl::transformPointCloud(*new_pc, transformed_new_pc, proposed_transformation);
    float fx,fy,cx,cy;
    getIntrinsics(fx,fy,cx,cx,old_cam_info);
    //downsampled
    int cloud_skip_step = ps->get<int>("cloud_creation_skip_step");
    fx = fx / cloud_skip_step;
    fy = fy / cloud_skip_step;
    cx = cx / cloud_skip_step;
    cy = cy / cloud_skip_step;
    double sumloglikehood = 0.0, observation_count = 0.0;
    unsigned int bad_points = 0, good_points =0, occluded_points = 0;
    //mark point in the pointcloud
    uint8_t r1=rand() % 32, g1=128+rand()% 128 , b1 = 128 +rand() % 128;
    uint8_t cyan = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);//cyan
    uint8_t r2 = 128 + rand() % 128, g2 = rand() % 32,  b2 = 128+rand() % 128;
    uint32_t megent = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);//magenta
    for(int new_y = 0; new_y< (int)new_pc->height; new_y+= skip_step){
        for(int new_x = 0; new_x < (int)new_pc->width; new_x+= skip_step)
        {//Backproject transformed new 3D point to 2d frame of old image
            point_type& p = transformed_new_pc.at(new_x,new_y);
            if(std::isnan(p.z))continue;
            if(p.z < 0)continue;
            //2d point
            int old_x_center = round((p.x / p.z) * fx + cx);
            int old_y_center = round((p.y / p.z) * fx + cy);
            if(old_x_center >= (int) old_pc-> width || old_y_center >= old_pc->height ||
               old_x_center < 0 || old_y_center <0 )
            {
               ROS_DEBUG("New is not in the old image, skipping");
               continue;
            }
            int r = 2; // 3x3 radius
            bool good_point = false, occluded_point =false, bad_point =false;
            int startx = std::max(0, old_x_center - r);
            int endx = std::min(static_cast<int>(old_pc->width),old_x_center + r + 1);
            int starty = std::max(0, old_y_center - r);
            int endy = std::min(static_cast<int>(old_pc->height),old_y_center+ r + 1);
            int skip_neib_step = 2;
            for(int old_y = starty; old_y < endy; old_y += skip_neib_step){
                for(int old_x = startx; old_x < endx; old_x += skip_neib_step){

                    const point_type& old_p =old_pc->at(old_x,old_y);
                    if(std::isnan(old_p.z)) continue;

                    double sigma_1 = cloud_skip_step * depth_convariance_mean(old_p.z);
                    double sigma_2 = cloud_skip_step * depth_convariance_mean(p.z);
                    double joint_sigma = sigma_1+sigma_2;
                    //the probability that the the point's depth value in old frame is smaller than the its in the new frame
                    double probability_not_be_occluded_in_old_frame = gaussian_cdf(old_p.z, p.z, sqrt(joint_sigma));
                    if(probability_not_be_occluded_in_old_frame < 0.001)
                    {
                        occluded_point = true;
                    }
                    else if(probability_not_be_occluded_in_old_frame < 0.999)
                    {
                        good_point = true;
                    }
                    else
                    {
                        bad_point = true;
                    }
                }
            }
            if(good_point == true)
            {
                good_points++;
            }
            else if(bad_point = true)
            {
                bad_points++;
                if(mark_outliers == true)
                {
                    new_pc->at(new_x, new_y).data[3] = *reinterpret_cast<float*>(&megent);
                    old_pc->at(old_x_center).data[3] = *reinterpret_cast<float*>(&megent);
                }
            }
            else
            {
                occluded_points++;
                if(mark_outliers == true)
                {
                    new_pc->at(new_x, new_y).data[3] = *reinterpret_cast<float*>(&cyan);
                    old_pc->at(old_x_center).data[3] = *reinterpret_cast<float*>(&cyan);
                }
            }

        }
    }
    likelihood = sumloglikehood / observation_count;//more readable
    confidence = observation_count;
    inliers = good_points;
    outliers = bad_points;
    occluded = occluded_points;
}
bool observation_criterion_met(unsigned int inliers, unsigned int outliers, unsigned int all, double& quality)
{
    double obs_met_th = ParameterSetting::instance()-> get<double>("observability_threshold");
    if(obs_met_th < 0) return true;
    quality = inliers / static_cast<double>(inliers + outliers);
    double certainty = inliers / static_cast<double>(all);
    bool met_criterion1 = quality > obs_met_th;
    bool met_criterion2 = certainty > 0.25;
    if(!(met_criterion1 && met_criterion2))
    {
        ROS_WARN("Transformation does not meet observation_criterion");
    }
    return met_criterion1 && met_criterion2;
}
void getColor(const point_type& p, unsigned char& r, unsigned char& g, unsigned char& b)
{
    b = *(0 + (unsigned char*)(&p.data[3]));
    g = *(1 + (unsigned char*)(&p.data[3]));
    r = *(2 + (unsigned char*)(&p.data[3]));
}
