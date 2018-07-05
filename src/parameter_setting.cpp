#include "parameter_setting.h"


ParameterSetting* ParameterSetting:: _instance = NULL;//global setting
Config_input* Config_input::_Config_Setting = NULL;

void ParameterSetting::defaultConfig(){
    Config_input* ci = Config_input::Config_Setting();

    double inf=std::numeric_limits<double>::infinity();
    //input data settings
    addSetting("image_topic", std::string("/camera/rgb/image_color"));
    //addSetting("image_topic",ci->getParameter<std::string>("image_topic"));
    addSetting("image_topic_depth", std::string("/camera/depth_registered/sw_registered/image_rect_raw"));
    addSetting("camera_info_topic", std::string("/camera/rgb/camera_info"));
    addSetting("bagfile_name", std::string(" "));
    addSetting("subscriber_queue_size", static_cast<int> (3));
    addSetting("drop_async_frames", static_cast<bool> (false));
    addSetting("depth_scale_factor", static_cast<double> (1.0));
    addSetting("data_skip_step", static_cast<int> (1));
    addSetting("cloud_creation_skip_step", static_cast<int> (2));
    addSetting("create_cloud_every_nth_node", static_cast<int> (1));
    addSetting("maximum_depth", static_cast<double> (inf));
    addSetting("minimum_depth", static_cast<double> (0.1));
    addSetting("encoding_bgr", static_cast<bool> (true));
    addSetting("base_line", static_cast<double>(40.0));
    //Camera intrinsic
    addSetting("camera_fx", static_cast<double> (0.0));
    addSetting("camera_fy", static_cast<double> (0.0));
    addSetting("camera_cx", static_cast<double> (0.0));
    addSetting("camera_cy", static_cast<double> (0.0));
    addSetting("depth_sigma", static_cast<double> (0.001));

    //output data settings
    addSetting("store_pointclouds", static_cast<bool> (true));
    addSetting("individual_cloud_out_topic", std::string("/kinectslam/batch_clouds"));
    addSetting("aggregate_cloud_out_topic",  std::string("/kinectslam/aggregate_clouds"));
    addSetting("online_cloud_out_topic",  std::string("/kinectslam/online_clouds"));
    addSetting("send_clouds_rate", static_cast<double> (5));
    addSetting("publisher_queue_size", static_cast<int> (5));

    //Octomap settings
    addSetting("octomap_resolution", static_cast<double> (0.05));
    addSetting("octomap_autosave_step", static_cast<int> (50));
    addSetting("octomap_clear_after_save", static_cast<bool> (false));
    addSetting("octomap_clear_raycasted_clouds", static_cast<bool> (false));
    addSetting("octomap_occupancy_threshold", static_cast<double> (0.5));
    addSetting("octomap_clamping_max", static_cast<double> (0.999));
    addSetting("octomap_clamping_min", static_cast<double> (0.001));
    addSetting("octomap_prob_hit", static_cast<double> (0.9));
    addSetting("octomap_prob_miss", static_cast<double> (0.4));
    addSetting("octomap_online_creation", static_cast<bool> (false));
    addSetting("screencast_path_prefix", std::string(""));
    addSetting("transform_individual_clouds", static_cast<bool> (false));
    addSetting("compress_output_bagfile", static_cast<bool> (true));
    addSetting("occupancy_filter_threshold", static_cast<double> (0.9));

    //TF transformation settings
    addSetting("base_frame_name", std::string("/openni_rgb_optical_frame"));
    addSetting("fixed_camera", static_cast<bool> (true));
    addSetting("fixed_camera", static_cast<bool> (true));

    //Features extractor and descriptor
    addSetting("feature_detector_type", std::string("ORB"));
    addSetting("feature_extractor_type", std::string("ORB"));
    addSetting("matcher", std::string("BRUTEFORCE"));
    addSetting("max_keypoints", static_cast<int> (600));
    addSetting("min_keypoints", static_cast<int> (0));
    addSetting("max_matches", static_cast<int> (300));
    addSetting("min_matches", static_cast<int> (20));
    addSetting("sufficient_matches", static_cast<int> (1e9));
    addSetting("use_feature_min_depth", static_cast<bool>(false));
    addSetting("use_feature_mask", static_cast<bool>(false));
    addSetting("use_root_sift",    static_cast<bool>(true));

    // Front-end settings
    addSetting("max_translation_meter", static_cast<double> (1e10));
    addSetting("max_rotation_degree",   static_cast<double> (360));
    addSetting("min_translation_meter", static_cast<double> (0.0));
    addSetting("min_rotation_degree",   static_cast<double> (0.0));
    addSetting("max_dist_for_inliers",  static_cast<double> (3));
    addSetting("ransac_iterations",     static_cast<int> (200));
    addSetting("ransac_termination_inlier_Percentage", static_cast<double> (60.0));
    addSetting("g2o_transformation_refinement", static_cast<int> (0));
    addSetting("max_connections", static_cast<int> (-1));
    addSetting("geodesic_depth",  static_cast<int> (3));
    addSetting("predecessor_candidates", static_cast<int> (4));
    addSetting("neighbor_candidates",    static_cast<int> (4));
    addSetting("min_sampled_candidates", static_cast<int> (4));
    addSetting("Subsample_skip_step", static_cast<int> (8));
    addSetting("mark_outliers", static_cast<bool> (false));
    addSetting("observability_threshold", static_cast<double> (0));
    addSetting("allow_features_without_depth", static_cast<bool> (false));

    //Back-end settings
    addSetting("pose_relative_to", std::string("first"));
    addSetting("optimizer_iterations", static_cast<double> (0.01));
    addSetting("optimizer_skip_step",  static_cast<int> (1));
    addSetting("optimize_landmarks",   static_cast<bool> (false));
    addSetting("concurrent_optimization", static_cast<bool> (true));
    addSetting("backend_solver",  std::string("pcg"));//or csparse , cholmod

    // Visualization Settings
    addSetting("use_glwidget", static_cast<bool> (true));
    addSetting("use_gui", static_cast<bool> (true));
    addSetting("show_2d_display", static_cast<bool> (true));
    addSetting("glwidget_without_clouds", static_cast<bool> (false));
    addSetting("visualize_mono_depth_overlay", static_cast<bool> (false));
    addSetting("visualization_skip_step", static_cast<int> (1));
    addSetting("visualize_keyframes_only", static_cast<bool> (false));
    addSetting("fast_rendering_step",  static_cast<int> (1));
    addSetting("octomap_display_level",  static_cast<int> (16));
    addSetting("gl_point_size", static_cast<double> (1.0));
    addSetting("gl_grid_size_xy", static_cast<int> (0));
    addSetting("gl_grid_size_xz", static_cast<int> (20));
    addSetting("gl_grid_size_yz", static_cast<int> (0));
    addSetting("gl_cell_size",    static_cast<double> (1.0));
    addSetting("squared_meshing_threshold", static_cast<double> (0.0009));
    addSetting("show_axis", static_cast<bool> (true));
    addSetting("scalable_2d_display", static_cast<bool> (false));
    addSetting("cloud_display_type",  static_cast<std::string>("POINTS"));

    //Other Settings
    addSetting("start_paused", static_cast<bool> (false));
    addSetting("batch_processing", static_cast<bool> (false));
    addSetting("concurrent_node_construction",  static_cast<bool> (true));
    addSetting("concurrent_edge_construction",  static_cast<bool> (true));
    addSetting("concurrent_io", static_cast<bool> (true));
    addSetting("voxelfilter_size", static_cast<double> (-1.0));
    addSetting("nn_distance_ratio", static_cast<double> (0.95));
    addSetting("keep_nomotion_nodes", static_cast<bool> (false));
    addSetting("keep_good_nodes", static_cast<bool> (false));
    addSetting("clear_non_keyframes", static_cast<bool> (false));
    addSetting("min_time_reported", static_cast<double> (-1.0));
    addSetting("preserve_raster_on_save", static_cast<bool> (false));
   // addSetting("skip_first_n_frames", static_cast<int> (0));
    addSetting("segment_to_optimize", static_cast<int> (-1));
    addSetting("send_clouds_delay",   static_cast<double> (-10.0));
    addSetting("save_octomap_delay",  static_cast<double> (10.0));
    //Debug
    addSetting("show_cloud_with_id",  static_cast<int> (-1));
    addSetting("use_error_shortcut",  static_cast<bool> (true));

}
//constructor
ParameterSetting::ParameterSetting(){
    node_name_config += ros::this_node::getName();
    node_name_config += "/config/";

    defaultConfig();

    getValues();
}

ParameterSetting* ParameterSetting::instance(){
    if(_instance == NULL){
        _instance = new ParameterSetting();
    }
    return _instance;
}

void ParameterSetting::addSetting(std::string name, boost::any value){
    config[name] = value;
}

void ParameterSetting::getValues(){
    std::map<std::string,boost::any>::const_iterator it;
    for(it = config.begin();it != config.end();it++){
        std::string name = it->first;
        if(it->second.type() == typeid(std::string)){
           config[name] = getFromParameterSetting<std::string>(node_name_config+name,
                boost::any_cast<std::string>(it->second));
           ROS_DEBUG_STREAM("Value for " << name << ":             " << config[name]);
        }
        else if(it->second.type() == typeid(int)){
            config[name] = getFromParameterSetting<int>(node_name_config+name,
                 boost::any_cast<int>(it->second));
            ROS_DEBUG_STREAM("Value for " << name << ":            " <<config[name]);
        }
        else if(it->second.type() == typeid(double)){
            config[name] = getFromParameterSetting<double>(node_name_config+name,
                 boost::any_cast<double>(it->second));
            ROS_DEBUG_STREAM("Value for"<<name<<":             "<<config[name]);
        }
        else if(it->second.type() == typeid(bool)){
                config[name] = getFromParameterSetting<bool>(node_name_config+name,
                     boost::any_cast<bool>(it->second));
                 ROS_DEBUG_STREAM("Value for"<<name<<":             "<<config[name]);
            }

        }
         checkValues();
}
void ParameterSetting::checkValues(){
    if (get<double>("voxelfilter_size") > 0 && get<double>("observability_threshold") > 0) {
           ROS_ERROR("You cannot use the voxelfilter (param: voxelfilter_size) in combination with the environment measurement model (param: observability_threshold)");
       }
    if (get<double>("max_translation_meter") <= 0){
         double Inf = std::numeric_limits<double>::infinity();
         set("max_translation_meter", Inf);
       }
       if (get<double>("max_rotation_degree") <= 0){
         double Inf = std::numeric_limits<double>::infinity();
         set("max_rotation_degree",Inf);
       }
}
Config_input* Config_input::Config_Setting()
{
    if(_Config_Setting == NULL){
        _Config_Setting = new Config_input();
    }
}
Config_input::Config_input()
{
   setParameter();
}
