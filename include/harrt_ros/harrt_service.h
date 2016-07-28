#ifndef HARRT_SERVICE_H_
#define HARRT_SERVICE_H_

#include <ros/ros.h>
#include "harrt_ros/harrt_initialize.h"
#include "harrt_ros/harrt_continue.h"
#include "tpp/harrt/birrtstar.h"

class HARRTService {
public:
  HARRTService();
  virtual ~HARRTService();

  birrts::BIRRTstar*           mp_harrt;
  homotopy::ReferenceFrameSet* mp_reference_frame_set;

  bool get_paths( harrt_ros::harrt_initialize::Request& req,
                  harrt_ros::harrt_initialize::Response& res);

  bool refine_paths( harrt_ros::harrt_continue::Request& req,
                     harrt_ros::harrt_continue::Response& res);

  void delete_harrt();

  birrts::COST_FUNC_PTR func;
  double**              fitness_distribution;

  ros::NodeHandle m_nh;
  ros::NodeHandle m_cont;

  ros::ServiceServer m_harrt_init_srv;
  ros::ServiceServer m_harrt_cont_srv;
};

#endif // HARRT_SERVICE_H_
