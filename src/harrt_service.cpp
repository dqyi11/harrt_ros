#include <iostream>
#include "harrt_ros/harrt_service.h"

#define HARRT_INIT_SERVICE_NAME "/harrt/get_paths"
#define HARRT_CONT_SERVICE_NAME "/harrt/refine_paths"

HARRTService::HARRTService() {
  m_harrt_init_srv = m_nh.advertiseService( HARRT_INIT_SERVICE_NAME, &HARRTService::get_paths, this);
  m_harrt_cont_srv = m_cont.advertiseService( HARRT_CONT_SERVICE_NAME, &HARRTService::refine_paths, this);
}

HARRTService::~HARRTService() {

}

bool HARRTService::get_paths( harrt_ros::harrt_initialize::Request& req,
                              harrt_ros::harrt_initialize::Response& res) {

  return true;
}

bool HARRTService::refine_paths( harrt_ros::harrt_continue::Request& req,
                                 harrt_ros::harrt_continue::Response& res) 
{
  
  return false;
}
