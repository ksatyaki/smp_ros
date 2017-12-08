// Standard header files
#include <iostream>
#include <memory>

// ROS headers
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

// SMP HEADER FILES ------
#include <smp/components/collision_checkers/multiple_circles_mrpt.hpp>
#include <smp/components/distance_evaluators/kdtree.hpp>
#include <smp/components/extenders/double_integrator.hpp>
#include <smp/components/multipurpose/minimum_time_reachability.hpp>
#include <smp/components/samplers/uniform.hpp>
#include <smp/planners/rrtstar.hpp>

#include <smp/planner_utils/trajectory.hpp>
#include <smp/planner_utils/vertex_edge.hpp>

// SMP TYPE DEFINITIONS -------
// State, input, vertex_data, and edge_data definitions
typedef smp::state_double_integrator<2> StateDoubleIntegrator;
typedef smp::input_double_integrator<2> InputDoubleIntegrator;
typedef smp::minimum_time_reachability_vertex_data vertex_data_t;
typedef smp::minimum_time_reachability_edge_data edge_data_t;

// Create the typeparams structure
typedef struct _typeparams {
  typedef StateDoubleIntegrator state;
  typedef InputDoubleIntegrator input;
  typedef vertex_data_t vertex_data;
  typedef edge_data_t edge_data;
} typeparams;

// Define the trajectory type
typedef smp::trajectory<typeparams> trajectory_t;

// Define all planner component types
typedef smp::sampler_uniform<typeparams, 4> UniformSampler;
typedef smp::distance_evaluator_kdtree<typeparams, 4> KDTreeDistanceEvaluator;
typedef smp::extender_double_integrator<typeparams, 2> ExtenderDoubleIntegrator;
typedef smp::collision_checker_mc_mrpt<typeparams> CollisionCheckerMCMRPT;
typedef smp::minimum_time_reachability<typeparams, 4> MinimumTimeReachability;

// Define all algorithm types
typedef smp::rrtstar<typeparams> RRTStar;

geometry_msgs::PoseArray graph;

template <typename T> void graphToMsg(smp::vertex<T> *root) {
  geometry_msgs::Pose p;
  p.position.x = root->state->state_vars[0];
  p.position.y = root->state->state_vars[1];
  p.orientation.w = cos(root->state->state_vars[2] / 2);
  p.orientation.z = sin(root->state->state_vars[2] / 2);

  graph.poses.push_back(p);
  for (auto another_root : root->outgoing_edges) {
    graphToMsg(another_root->vertex_dst);
  }
}

void mrptMapFromROSMsg(
    const std::shared_ptr<mrpt::maps::COccupancyGridMap2D> &map,
    const nav_msgs::OccupancyGrid &rosMap) {

  map->setSize(rosMap.info.origin.position.x,
               rosMap.info.origin.position.x +
                   (rosMap.info.width * rosMap.info.resolution),
               rosMap.info.origin.position.y,
               rosMap.info.origin.position.y +
                   (rosMap.info.height * rosMap.info.resolution),
               rosMap.info.resolution);

  // std::cout << "X: " << map->getXMin() << std::endl
  //           << "Y: " << map->getYMin() << std::endl
  //           << "Resolution: " << map->getResolution() << std::endl
  //           << "XMax: " << map->getXMax() << std::endl
  //           << "YMax: " << map->getYMax() << std::endl;

  for (int h = 0; h < rosMap.info.height; h++) {
    for (int w = 0; w < rosMap.info.width; w++) {
      float value = -1.0f;
      const int8_t &occ_map_value = rosMap.data[w + h * rosMap.info.width];
      if (occ_map_value <= 100 || occ_map_value >= 0) {
        value = 1.0f - (float)occ_map_value / 100.0f;
      } else if (occ_map_value == -1) {
        value = -1.0f;
      } else {
        std::cerr << "[ERROR]: Converting nav_msgs::OccupancyGrid to "
                     "mrpt::maps::COccupancyGridMap2D. Saw an unknown value in "
                     "data: "
                  << occ_map_value << "\n";
      }
      map->setCell(w, h, value);
    }
  }
}

int main(int argn, char *args[]) {

  ros::init(argn, args, "test_smp_ros");

  ros::NodeHandle nh;

  ros::Publisher path_pub = nh.advertise<geometry_msgs::PoseArray>("/path", 1);
  ros::Publisher graph_pub =
      nh.advertise<geometry_msgs::PoseArray>("/graph", 1);

  ros::ServiceClient map_client =
      nh.serviceClient<nav_msgs::GetMap>("static_map");
  map_client.waitForExistence();
  nav_msgs::GetMap get_map_;
  if (!map_client.call(get_map_)) {
    ROS_FATAL("Failed to call the map server for map!");
  }
  ROS_INFO("Got a map!");

  std::shared_ptr<mrpt::maps::COccupancyGridMap2D> map =
      std::make_shared<mrpt::maps::COccupancyGridMap2D>();
  mrptMapFromROSMsg(map, get_map_.response.map);

  std::shared_ptr<mrpt::math::CPolygon> footprint =
      std::make_shared<mrpt::math::CPolygon>();
  footprint->AddVertex(0.1, 0.1);
  footprint->AddVertex(0.1, -0.1);
  footprint->AddVertex(-0.1, 0.1);
  footprint->AddVertex(-0.1, -0.1);

  // 1. CREATE PLANNING OBJECTS

  // 1.a Create the components
  UniformSampler sampler;
  KDTreeDistanceEvaluator distance_evaluator;
  ExtenderDoubleIntegrator extender;
  CollisionCheckerMCMRPT collision_checker(map, 0.05, footprint);
  MinimumTimeReachability min_time_reachability;

  // 1.b Create the planner algorithm -- Note that the min_time_reachability
  // variable acts both
  //                                       as a model checker and a cost
  //                                       evaluator.
  RRTStar planner(sampler, distance_evaluator, extender, collision_checker,
                  min_time_reachability, min_time_reachability);

  // The phase parameter can be used to run the algorithm as an RRT,
  // See the documentation of the RRG algorithm for more information.

  // Set this parameter should be set at least to the side length of
  //   the (bounded) state space. E.g., if the state space is a box
  //   with side length L, then this parameter should be set to at
  //   least L for rapid and efficient convergence in trajectory space.
  planner.parameters.set_phase(2);
  planner.parameters.set_gamma(35.0);

  planner.parameters.set_dimension(4);
  planner.parameters.set_max_radius(10.0);

  // 2. INITALIZE PLANNING OBJECTS
  // 2.a Initialize the sampler
  smp::region<4> sampler_support;

  sampler_support.center[0] = map->getXMax() / 2.0;
  sampler_support.size[0] = map->getXMax();

  sampler_support.center[1] = map->getYMax() / 2.0;
  sampler_support.size[1] = map->getYMax();

  sampler_support.center[2] = 0.0;
  sampler_support.size[2] = 3.14;

  sampler_support.center[3] = 0.0;
  sampler_support.size[3] = 1.0;

  sampler.set_support(sampler_support);

  // 2.b Initialize the distance evaluator
  //     Nothing to initialize. One could change the kdtree weights.

  // 2.e Initialize the model checker and the cost evaluator (with
  // minimum_time_reachability).
  double goalX, goalY;
  ros::param::param("goal_x", goalX, 6.0);
  ros::param::param("goal_y", goalY, 2.0);

  ROS_INFO("Going to goal: (%lf,%lf)", goalX, goalY);
  smp::region<4> region_goal;
  region_goal.center[0] = goalX;
  region_goal.size[0] = 2.0;

  region_goal.center[1] = goalY;
  region_goal.size[1] = 2.0;

  region_goal.center[2] = 0.0;
  region_goal.size[2] = 3.14;

  region_goal.center[3] = 0.0;
  region_goal.size[3] = 1.0;

  min_time_reachability.set_goal_region(region_goal);

  // 2.f Initialize the planner
  StateDoubleIntegrator *state_initial = new StateDoubleIntegrator;

  state_initial->state_vars[0] = 3.0;
  state_initial->state_vars[1] = 2.0;
  state_initial->state_vars[2] = 0.0;
  state_initial->state_vars[3] = 0.0;

  if (collision_checker.check_collision_state(state_initial) == 0) {
    ROS_INFO("Start state is in collision.");
  } else
    ROS_INFO("Start state is not in collision.");

  planner.initialize(state_initial);

  ros::Time t = ros::Time::now();
  // 3. RUN THE PLANNER
  int i = 0;
  double planning_time = std::atof(args[1]);

  graph.header.frame_id = "map";
  while (ros::ok()) {
    ++i;
    if (ros::Time::now() - t > ros::Duration(planning_time)) {
      ROS_INFO("Planning time of %lf sec. elapsed.", planning_time);
      break;
    }

    planner.iteration();

    graph.poses.clear();

    graphToMsg(planner.get_root_vertex());
    graph.header.stamp = ros::Time::now();

    //graph_pub.publish(graph);
    ROS_INFO_THROTTLE(1.0, "Iteration : %d", i);
  }

  // 4. GET THE RESULTS
  trajectory_t trajectory_final;
  min_time_reachability.get_solution(trajectory_final);

  geometry_msgs::PoseArray poses;
  poses.header.frame_id = "map";
  poses.header.stamp = ros::Time::now();
  for (const auto &state : trajectory_final.list_states) {
    geometry_msgs::Pose pose;
    pose.position.x = (*state)[0];
    pose.position.y = (*state)[1];
    pose.orientation.z = sin((*state)[2] / 2);
    pose.orientation.w = cos((*state)[2] / 2);
    poses.poses.push_back(pose);
  }

  path_pub.publish(poses);

  while (ros::ok()) {
    ros::spinOnce();
    path_pub.publish(poses);
  }

  return 0;
}
