#ifdef KINECT_LISTENER_H
#define KINECT_LISTENER_H
#include"ros/ros.h"

//message fliters
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
//senor_msgs
#include<sensor_msgs/Image.h>
#include<sensor_msgs/CameraInfo.h>
#include<sensor_msgs/PointCloud2.h>
//opencv
#include<opencv2/core/core.hpp>
#include<opencv2/features2d.hpp>

//graph_optimizer

//qt
#include<QImage>
#include<QStringList>
#include<qt5/QtGui/QImage>
//rosbag
#include<rosbag/bag.h>

class kinect_listener
{
public:
    kinect_listener() {}
};
