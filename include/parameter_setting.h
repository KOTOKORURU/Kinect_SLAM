#ifndef PARAMETER_SETTING_H
#define PARAMETER_SETTING_H
#include<string>
#include <ros/ros.h>
#include <boost/any.hpp>
#include <opencv2/core/core.hpp>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

typedef pcl::PointXYZRGB point_type;
typedef pcl::PointXYZRGBNormal point_normal_type;

//point cloud
typedef pcl::PointCloud<point_type> pointcloud_type;
typedef pcl::PointCloud<point_normal_type> pointcloud_normal_type;

#define RESCONSOLE_SEVERITY_INFO 1

class ParameterSetting{
public:

    //singleton pattern
    static  ParameterSetting* instance();

    template<typename T>
    void set(const std::string param,T value){// set param value
        config[param] = value;
        setOnParameterSetting(node_name_config+param,value);
    }

    template<typename T>
    T get(std::string name){//get param value
        boost::any value = config[name];
        try{
            return boost::any_cast<T>(value);
        }
        catch(boost::bad_any_cast bad){

            std::cerr << "its bad cast:Requested data type is:"<<typeid(T).name() << "for the param " << name;
            throw bad;
        }
    }

    std::map<std::string,boost::any>& getConfigData(){
        return config;
    }
    void getValues();

private:
    static ParameterSetting* _instance;
    std::string node_name_config;
    ros::NodeHandle n;
    std::map<std::string,boost::any> config;


    //default constructor
    ParameterSetting();

    void addSetting(std::string name,boost::any value);

    void defaultConfig();

    void checkValues();

    template<typename T>
    T getFromParameterSetting(const std::string& param,T def){
        T res;
        //void 	param (const std::string &param_name, T &param_val, const T &default_val) const
        //Assign value from parameter server, with default.
        n.param(param,res,def);
        return res;
    }
    template<typename T>
    void setOnParameterSetting(const std::string& param,T new_val){
        n.setParam(param,new_val);
    }

};
//read yaml file
class Config_input
{
public:
    static Config_input* Config_Setting();
    template<typename T>
    T getParameter(const std::string& name){
        return Setting[name];
    }

private:
    cv::FileStorage Setting;
    static Config_input* _Config_Setting;
    Config_input();


    void setParameter(const std::string& name ="../Config/Setting.yaml"){
        Setting = cv::FileStorage(name, cv::FileStorage::READ);
    }
};

#endif
