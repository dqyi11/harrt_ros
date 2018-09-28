#include <ros/ros.h>
#include "harrt_ros/harrt_service.h"

int main( int argc, char** argv ) {
  ros::init( argc, argv, "harrt" );
  harrt_ros::HARRTService harrt_service;
  ros::spin();
  return 0;
}
