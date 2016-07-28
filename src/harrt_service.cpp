#include <iostream>
#include "tpp/homotopy/img_load_util.h"
#include "harrt_ros/harrt_service.h"

using namespace std;
using namespace homotopy;
using namespace birrts;

#define HARRT_INIT_SERVICE_NAME "/harrt/get_paths"
#define HARRT_CONT_SERVICE_NAME "/harrt/refine_paths"

static double calc_dist( POS2D pos_a, POS2D pos_b, double** pp_distribution, void* tree ) {
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

static double calc_cost( POS2D pos_a, POS2D pos_b, double** pp_distribution, void* tree ) {
  double cost = 0.0;
  BIRRTstar* rrts = (BIRRTstar*)tree;
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
      if (y>=0 && y<rrts->get_sampling_width() && x>=0 && x<rrts->get_sampling_height()) {
        cost += pp_distribution[y][x];
      }
    }
    else {
      if (x>=0 && x<rrts->get_sampling_width() && y>=0 && y<rrts->get_sampling_height()) {
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

HARRTService::HARRTService() {
  mp_harrt = NULL;
  m_harrt_init_srv = m_nh.advertiseService( HARRT_INIT_SERVICE_NAME, &HARRTService::get_paths, this);
  m_harrt_cont_srv = m_cont.advertiseService( HARRT_CONT_SERVICE_NAME, &HARRTService::refine_paths, this);
}

HARRTService::~HARRTService() {

}

bool HARRTService::get_paths( harrt_ros::harrt_initialize::Request& req,
                              harrt_ros::harrt_initialize::Response& res) {

  std::cout << "Starting HARRT..." << std::endl;
  delete_harrt();

  int** pp_obstacle = new int*[req.init.map.width];
  for(unsigned int w=0; w < req.init.map.width; w++) {

    pp_obstacle[w] = new int[req.init.map.height];
    for(unsigned int h=0; h < req.init.map.height; h++) {
      pp_obstacle[w][h] = req.init.map.int_array[w+req.init.map.width*h];
    }
  }

  std::vector< std::vector<Point2D> > obstacles;
  load_map_info( pp_obstacle, req.init.map.width, req.init.map.height, obstacles );
  std::cout << "NUM OF OBS = " << obstacles.size() << std::endl;

  grammar_type_t grammar_type = STRING_GRAMMAR_TYPE;
  mp_reference_frame_set = new ReferenceFrameSet();
  mp_reference_frame_set->init(req.init.width, req.init.height, obstacles);
  if(req.init.sketched_topology.size() > 0) {
    std::vector< Point2D > ref_points;
    for( unsigned int i = 0; i < req.init.sketched_topology.size(); i ++ ) {
      geometry_msgs::Point p = req.init.sketched_topology[i];
      ref_points.push_back( Point2D( p.x, p.y ) );
    }
    if( mp_reference_frame_set ) {
      mp_reference_frame_set->import_string_constraint( ref_points, grammar_type );
    }
  }

  mp_harrt = new BIRRTstar(req.init.width, req.init.height, req.init.segment_length);
  mp_harrt->set_reference_frames( mp_reference_frame_set );
  POS2D start(req.init.start.x, req.init.start.y);
  POS2D goal(req.init.goal.x, req.init.goal.y);
  mp_harrt->init(start, goal, func, fitness_distribution, grammar_type );  

  while(mp_harrt->get_current_iteration() <= req.init.number_of_iterations) {
    mp_harrt->extend();
  }

  mp_harrt->get_string_class_mgr()->merge();
  std::vector<Path*> paths = mp_harrt->get_paths();

  for(unsigned int i=0; i<paths.size(); i++) {
    Path* p = paths[i];
    if(p) {
      harrt_ros::single_objective_path pp;
      for(unsigned int j=0; j<p->m_way_points.size(); j++) {
        geometry_msgs::Pose2D point;
        point.x = p->m_way_points[j][0];
        point.y = p->m_way_points[j][1];
        pp.waypoints.push_back(point); 
      }
      res.paths.push_back(pp);
    }
  }

  for(unsigned int w=0; w < req.init.map.width; w++) {
    delete [] pp_obstacle[w];
    pp_obstacle[w] = NULL;
  }
  delete [] pp_obstacle;

  std::cout << "HARRT finished!" << std::endl;

  return true;
}

bool HARRTService::refine_paths( harrt_ros::harrt_continue::Request& req,
                                 harrt_ros::harrt_continue::Response& res) {
  
  std::cout << "Refine paths of HARRT ... " << std::endl;
  
  if( mp_harrt ) {
    unsigned int new_iterations = mp_harrt->get_current_iteration() + req.iterations;
    while(mp_harrt->get_current_iteration() <= new_iterations) {
      mp_harrt->extend();
    }

    mp_harrt->get_string_class_mgr()->merge();
    std::vector<Path*> paths = mp_harrt->get_paths();

    for(unsigned int i=0; i<paths.size(); i++) {
      Path* p = paths[i];
      if(p) {
        harrt_ros::single_objective_path pp;
        for(unsigned int j=0; j<p->m_way_points.size(); j++) {
          geometry_msgs::Pose2D point;
          point.x = p->m_way_points[j][0];
          point.y = p->m_way_points[j][1];
          pp.waypoints.push_back(point); 
        }
        res.paths.push_back(pp);
      }
    }
  }
 
  std::cout << "Refine paths of HARRT finished " << std::endl;
 
  return true;
}

void HARRTService::delete_harrt() {

  if(mp_harrt) {
    delete mp_harrt;
    mp_harrt = NULL;
  }
  if(mp_reference_frame_set) {
    delete mp_reference_frame_set;
    mp_reference_frame_set = NULL;
  }
  func = NULL;
  if(fitness_distribution) {
    unsigned int array_size = sizeof(fitness_distribution)/sizeof(int*);

    for(unsigned int w=0; w < array_size; w++) {
      delete [] fitness_distribution[w];
    }

    delete [] fitness_distribution;
    fitness_distribution = NULL;
  }
}
