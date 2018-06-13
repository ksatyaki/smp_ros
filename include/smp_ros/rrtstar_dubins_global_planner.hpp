/*
 * Copyright (C) 2018 Sertac Karaman
 * Copyright (C) 2018 Chittaranjan Srinivas Swaminathan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 */

// Standard header files
#include <iostream>
#include <memory>

// SMP HEADER FILES ------
#include <smp/collision_checkers/multiple_circles_mrpt.hpp>
#include <smp/distance_evaluators/kdtree.hpp>
#include <smp/extenders/dubins.hpp>
#include <smp/multipurpose/minimum_time_reachability.hpp>
#include <smp/planners/rrtstar.hpp>
#include <smp/samplers/uniform.hpp>

#include <smp/trajectory.hpp>
#include <smp/vertex_edge.hpp>

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
using State = smp::StateDubins;
using Input = smp::InputDubins;
using Trajectory = smp::Trajectory<State, Input>;

namespace smp_ros {

class RRTStarDubinsGlobalPlanner : public nav_core::BaseGlobalPlanner {

private:
  ros::NodeHandle nh;
  smp::samplers::Uniform<State, 3> sampler;
  smp::extenders::Dubins extender;
  std::shared_ptr<smp::collision_checkers::MultipleCirclesMRPT<State>>
      collision_checker;

  ros::Publisher graph_pub;

  std::shared_ptr<mrpt::maps::COccupancyGridMap2D> map;
  std::shared_ptr<mrpt::math::CPolygon> footprint;

  // Debugging
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

template <class State, class Input>
void graphToMsg(ros::NodeHandle &nh, geometry_msgs::PoseArray &graph,
                smp::Vertex<State, Input> *root) {
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
