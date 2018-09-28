#include <iostream>
#include <topologyPathPlanning/homotopy/ImgLoadUtil.hpp>
#include <topologyPathPlanning/homotopy/ReferenceFrames.hpp>
#include "harrt_ros/harrt_service.h"

#define HARRT_INIT_SERVICE_NAME "/harrt/get_paths"
#define HARRT_CONT_SERVICE_NAME "/harrt/refine_paths"

using namespace std;
using namespace topologyPathPlanning;

namespace harrt_ros {

static double calc_dist( harrts::POS2D pos_a, harrts::POS2D pos_b, 
                         double** pp_distribution, void* tree ) 
{
  double dist = 0.0;
  if (pos_a == pos_b) {
    return dist;
  }
  double delta_x = fabs(pos_a[0]-pos_b[0]);
  double delta_y = fabs(pos_a[1]-pos_b[1]);
  dist = sqrt(delta_x*delta_x+delta_y*delta_y);

  if(dist < 0.0) {
    cout << "Dist negative " << dist << endl;
  }
  return dist;
}

static double calc_cost( harrts::POS2D pos_a, harrts::POS2D pos_b, 
                         double** pp_distribution, void* tree ) 
{
  double cost = 0.0;
  harrts::BIRRTstar* rrts = (harrts::BIRRTstar*)tree;
  if ( pos_a == pos_b ) {
    return cost;
  }
  if( pp_distribution == NULL ) {
    return cost;
  }

  float x1 = pos_a[0];
  float y1 = pos_a[1];
  float x2 = pos_b[0];
  float y2 = pos_b[1];

  const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
  if (steep) {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }

  if (x1 > x2) {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  const float dx = x2 - x1;
  const float dy = fabs(y2 - y1);

  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y = (int)y1;

  const int maxX = (int)x2;

  for(int x=(int)x1; x<maxX; x++) {
    if(steep) {
      if (y>=0 && y<rrts->getSamplingWidth() && x>=0 && x<rrts->getSamplingHeight()) {
        cost += pp_distribution[y][x];
      }
    }
    else {
      if (x>=0 && x<rrts->getSamplingWidth() && y>=0 && y<rrts->getSamplingHeight()) {
        cost += pp_distribution[x][y];
      }
    }

    error -= dy;
    if(error < 0) {
      y += ystep;
      error += dx;
    }
  }
  return cost;
}


HARRTService::HARRTService() 
  : mpHARRTs(NULL)
{
  mHarrtInitSrv = mNh.advertiseService( HARRT_INIT_SERVICE_NAME, &HARRTService::getPaths, this);
  mHarrtContSrv = mNh.advertiseService( HARRT_CONT_SERVICE_NAME, &HARRTService::refinePaths, this);
}

HARRTService::~HARRTService() 
{

}

bool HARRTService::getPaths( harrt_ros::harrt_initialize::Request& req,
                              harrt_ros::harrt_initialize::Response& res) 
{
  // init service
  std::cout << "Starting HARRT..." << std::endl;
  deleteHARRT();
   
  mpObstacle = new int*[req.init.width];
  for(int w=0; w<req.init.width; w++)
  {
    mpObstacle[w] = new int[req.init.height];
    for(int h=0; h<req.init.height; h++)
    {
      mpObstacle[w][h] = req.init.collision_map.data[ h*req.init.width+w ];
    }
  }

  std::vector< std::vector<homotopy::Point2D> > obstacles;
  homotopy::loadMapInfo( mpObstacle, req.init.width, req.init.height, obstacles );
  std::cout << "Num of obstacles = " << obstacles.size() << std::endl;

  homotopy::grammar_type_t grammarType = homotopy::STRING_GRAMMAR_TYPE;
  mpReferenceFrameSet = new homotopy::ReferenceFrameSet();
  mpReferenceFrameSet->init(req.init.width, req.init.height, obstacles);
  if(req.init.sketched_topology.size()>0)
  {
    std::vector<homotopy::Point2D> refPoints;
    for(std::size_t i=0; i<req.init.sketched_topology.size(); i++)
    {
      geometry_msgs::Point p = req.init.sketched_topology[i];
      refPoints.push_back( homotopy::Point2D( p.x, p.y ) );
    }
    if(mpReferenceFrameSet)
    {
      mpReferenceFrameSet->importStringConstraint(refPoints, grammarType);
    }
  } 

  if(req.init.minimum_distance_enabled == true)
  {
    mpFunc = calc_dist;
  }
  else
  {
    mpFitnessDistribution = new double*[req.init.cost_map.width];
    for(int w=0; w<req.init.cost_map.width; w++)
    {
      mpFitnessDistribution[w] = new double[req.init.cost_map.height];
      for(int h=0; h<req.init.cost_map.height; h++)
      {
        mpFitnessDistribution[w][h] = req.init.cost_map.int_array[w+req.init.cost_map.width*h];
      }
    }
    mpFunc = calc_cost;
  }

  mpHARRTs = new harrts::BIRRTstar( req.init.width, req.init.height, req.init.segment_length );
  mpHARRTs->setReferenceFrames( mpReferenceFrameSet );
  harrts::POS2D start( req.init.start.x, req.init.start.y );
  harrts::POS2D goal( req.init.goal.x, req.init.goal.y );
  mpHARRTs->init(start, goal, mpFunc, mpFitnessDistribution);
  
  while(mpHARRTs->getCurrentIteration() <= req.init.number_of_iterations)
  {
    mpHARRTs->extend();  
  }

  mpHARRTs->getStringClassMgr()->merge();
  std::vector<harrts::Path*> paths = mpHARRTs->getPaths();

  // export paths to response
  for(std::size_t i=0; i<paths.size(); i++)
  {
    harrts::Path* p = paths[i];
    if(p)
    {
      harrt_ros::single_objective_path pp;
      for(std::size_t j=0; j<p->mWaypoints.size(); j++)
      {
        geometry_msgs::Pose pose;
        pose.position.x = p->mWaypoints[j][0];
        pose.position.y = p->mWaypoints[j][1];
        pp.waypoints.poses.push_back(pose);
      }
      pp.cost.data = p->mCost;
      res.paths.push_back(pp);
    }
  }

  return true;
}

bool HARRTService::refinePaths( harrt_ros::harrt_continue::Request& req,
                                harrt_ros::harrt_continue::Response& res) 
{
  std::cout << "Refine paths of HARRT ... " << std::endl;
  
  if( mpHARRTs ) 
  {
    size_t new_iterations = mpHARRTs->getCurrentIteration() + req.iterations;
    while(mpHARRTs->getCurrentIteration() <= new_iterations) {
      mpHARRTs->extend();
    }

    mpHARRTs->getStringClassMgr()->merge();
    std::vector<harrts::Path*> paths = mpHARRTs->getPaths();

    for(unsigned int i=0; i<paths.size(); i++) {
      harrts::Path* p = paths[i];
      if(p) {
        harrt_ros::single_objective_path pp;
        for(unsigned int j=0; j<p->mWaypoints.size(); j++) {
          geometry_msgs::Pose pose;
          pose.position.x = p->mWaypoints[j][0];
          pose.position.y = p->mWaypoints[j][1];
          pp.waypoints.poses.push_back(pose); 
        }
        pp.cost.data = p->mCost;
        res.paths.push_back(pp);
      }
    }
  }
  std::cout << "Refine paths of HARRT finished " << std::endl;
 
  return true;  
}

void HARRTService::deleteHARRT()
{
  if(mpHARRTs) 
  {
    delete mpHARRTs;
    mpHARRTs = NULL;
  }
  if(mpReferenceFrameSet) 
  {
    delete mpReferenceFrameSet;
    mpReferenceFrameSet = NULL;
  }
  if(mpFitnessDistribution) 
  {
    size_t array_size = sizeof(mpFitnessDistribution)/sizeof(double*);
    for(size_t w=0; w < array_size; w++) 
    {
      delete [] mpFitnessDistribution[w];
    }
    delete [] mpFitnessDistribution;
    mpFitnessDistribution = NULL;
  }

  if(mpObstacle) 
  {
    size_t array_size = sizeof(mpObstacle)/sizeof(int*);
    for(size_t w=0; w < array_size; w++) 
    {
      delete [] mpObstacle[w];
    }
    delete [] mpObstacle;
    mpObstacle = NULL;
  }
  mpFunc = NULL;
}

} // harrt_ros
