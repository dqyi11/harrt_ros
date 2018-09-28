#ifndef HARRT_SERVICE_H_
#define HARRT_SERVICE_H_

#include <ros/ros.h>
#include "harrt_ros/harrt_initialize.h"
#include "harrt_ros/harrt_continue.h"
#include "topologyPathPlanning/harrts/BiRRTstar.hpp"

namespace harrt_ros {

class HARRTService {
public:
  HARRTService();
  virtual ~HARRTService();


  bool getPaths( harrt_ros::harrt_initialize::Request& req,
                  harrt_ros::harrt_initialize::Response& res);

  bool refinePaths( harrt_ros::harrt_continue::Request& req,
                     harrt_ros::harrt_continue::Response& res);

protected:
  void deleteHARRT();

  ros::NodeHandle mNh;
  ros::ServiceServer mHarrtInitSrv;
  ros::ServiceServer mHarrtContSrv;

  topologyPathPlanning::harrts::COST_FUNC_PTR mpFunc;
  topologyPathPlanning::harrts::BIRRTstar* mpHARRTs;
  topologyPathPlanning::homotopy::ReferenceFrameSet* mpReferenceFrameSet;
  double** mpFitnessDistribution;
  int** mpObstacle;
};

} // harrt_ros

#endif // HARRT_SERVICE_H_
