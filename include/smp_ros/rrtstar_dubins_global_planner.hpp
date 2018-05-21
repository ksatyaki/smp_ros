// Standard header files
#include <iostream>
#include <memory>

// SMP HEADER FILES ------
#include <smp/components/collision_checkers/multiple_circles_mrpt.hpp>
#include <smp/components/distance_evaluators/kdtree.hpp>
#include <smp/components/extenders/dubins.hpp>
#include <smp/components/extenders/single_integrator.hpp>
#include <smp/components/multipurpose/minimum_time_reachability.hpp>
#include <smp/components/samplers/uniform.hpp>
#include <smp/planners/rrtstar.hpp>

#include <smp/planner_utils/trajectory.hpp>
#include <smp/planner_utils/vertex_edge.hpp>

// ROS headers
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

// SMP TYPE DEFINITIONS -------
// State, input, vertex_data, and edge_data definitions
typedef smp::state_dubins StateDubins;
typedef smp::input_dubins InputDubins;
typedef smp::minimum_time_reachability_vertex_data vertex_data_t;
typedef smp::minimum_time_reachability_edge_data edge_data_t;

// Create the typeparams structure
typedef struct _typeparams {
  typedef StateDubins state;
  typedef InputDubins input;
  typedef vertex_data_t vertex_data;
  typedef edge_data_t edge_data;
} typeparams;

// Define the trajectory type
typedef smp::trajectory<typeparams> trajectory_t;

// Define all planner component types
typedef smp::sampler_uniform<typeparams, 3> UniformSampler;
typedef smp::distance_evaluator_kdtree<typeparams, 3> KDTreeDistanceEvaluator;
typedef smp::extender_dubins<typeparams> ExtenderDubins;
typedef smp::collision_checker_mc_mrpt<typeparams> CollisionCheckerMCMRPT;
typedef smp::minimum_time_reachability<typeparams, 3> MinimumTimeReachability;

// Define all algorithm types
typedef smp::rrtstar<typeparams> RRTStar;

namespace smp_ros {

class RRTStarDubinsGlobalPlanner : public nav_core::BaseGlobalPlanner {

private:
  ros::NodeHandle nh;
  UniformSampler sampler;
  ExtenderDubins extender;
  std::shared_ptr<CollisionCheckerMCMRPT> collision_checker;

  ros::Publisher graph_pub;

  std::shared_ptr<mrpt::maps::COccupancyGridMap2D> map;
  std::shared_ptr<mrpt::math::CPolygon> footprint;

  //Debugging
  geometry_msgs::PoseArray graph;

protected:
  virtual void initialize(std::string name,
                          costmap_2d::Costmap2DROS *costmap_ros);
  virtual bool makePlan(const geometry_msgs::PoseStamped &start,
                        const geometry_msgs::PoseStamped &goal,
                        std::vector<geometry_msgs::PoseStamped> &plan);

public:
  inline RRTStarDubinsGlobalPlanner() {}
  inline virtual ~RRTStarDubinsGlobalPlanner() {}
};

} // namespace smp_ros

// Given a state and goal array, find the distance between them.
// Angles have a different distance formula. That is why we need this one.

std::array<double, 3> distanceBetweenStates(const std::array<double, 3> &state,
                                            const std::array<double, 3> &goal);

template <typename T> void graphToMsg(ros::NodeHandle& nh, geometry_msgs::PoseArray& graph, smp::vertex<T> *root) {
  geometry_msgs::Pose p;
  p.position.x = root->state->state_vars[0];
  p.position.y = root->state->state_vars[1];
  p.orientation.w = cos(root->state->state_vars[2] / 2);
  p.orientation.z = sin(root->state->state_vars[2] / 2);

  graph.poses.push_back(p);
  for (auto another_root : root->outgoing_edges) {
    graphToMsg(nh, graph, another_root->vertex_dst);
  }
}

template <typename T> void freeGraph(smp::vertex<T> *root) {
  for (auto another_root : root->outgoing_edges) {
    freeGraph(another_root->vertex_dst);
  }
  delete root;
}
