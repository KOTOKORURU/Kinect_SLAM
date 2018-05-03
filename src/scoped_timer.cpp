#include "scoped_timer.h"
#include<ros/console.h>
#include"parameter_setting.h"

ScopedTimer::ScopedTimer(const char* name,bool only_for_logging,bool unconditional_logging)
    :name(name), triggering(unconditional_logging)
{

}

double ScopedTimer::elapsed(){
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) * 1e-9;
}

ScopedTimer::~ScopedTimer(){
  //No output anyway if above INFO level - ergo do nothing

  double min_time = ParameterSetting::instance()->get<double>("min_time_reported");
  if(triggering || min_time > 0){
    double runtime = elapsed();
    if(triggering || runtime > min_time){
      ROS_INFO_STREAM_NAMED("timings", name << " runtime: "<< runtime <<" s");
    }
  }
}

