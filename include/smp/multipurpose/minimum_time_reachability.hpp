/*! \file components/multipurpose/minimum_time_reachability.h
  \brief An implementation of the vertex and edge components in the graph.

  Provides an implementation of the vertex and edge components in the graph.
  Both classes
  are defined as templates that take the types of the state, input, and the data
  stored in
  the vertices as well as the type of the data that is stored in the edges as an
  argument.

  * Copyright (C) 2018 Sertac Karaman
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

#pragma once

#include <smp/cost_evaluators/base.hpp>
#include <smp/model_checkers/base.hpp>
#include <smp/planners/rrtstar.hpp>
#include <smp/region.hpp>

#include <array>
#include <functional>

namespace smp {
namespace multipurpose {

//! Vertex data for minimum-time reachability.
/*! This data structure is attached to each vertex in the graph maintained by
  the planner algorithm. The data structure includes two variables. One variable
  indicates whether the associated vertex lies inside the goal region. Another
  variables keeps track of the cost to reach this particular vertex starting
  from the root vertex. The latter variable is particularly created to work with
  teh RRT* algorithm.
*/
class MTRVertexData : public RRTStarVertexData {

public:
  //! Reachability of the goal region.
  /*!
    This variable that indicates whether the associated vertex
    state is inside the goal region.
  */
  bool reaches_goal;
};

//! Edge data for minimum-time reachability.
/*!
  This empty class is implemented for the sake of completeness.
*/
class MTREdgeData : public RRTStarEdgeData {};
} // namespace multipurpose

//! A combination of the minimum-time cost evaluator and the reachability model
//! checker
/*!
  Combining the minimum-time cost evaluator and the reachability model
  checker, this class is able to keep track of the minimum-time that reaches the
  goal region. The class constitutes a good example of multiple-purpose
  algorithm component made possible with mutliple inheritance.

  \ingroup model_checkers
  \ingroup cost_evaluators
*/

template <class State, class Input, int NUM_DIMENSIONS>
class MinimumTimeReachability
    : public model_checker_base<State, Input, MTRVertexData, MTREdgeData>,
      public cost_evaluator_base<State, Input, MTRVertexData, MTREdgeData> {

  using vertex_data_t = MTRVertexData;
  using edge_data_t = MTREdgeData;

  using edge_t = Edge<State, Input, MTRVertexData, MTREdgeData>;
  using vertex_t = Vertex<State, Input, MTRVertexData, MTREdgeData>;

  using trajectory_t = Trajectory<State, Input>;
  using region_t = Region<NUM_DIMENSIONS>;

  using update_function_t = std::function<int(trajectory_t *)>;
  //  typedef int (*update_func_t)(trajectory_t *);

  using distance_function_t = std::function<std::array<double, NUM_DIMENSIONS>(
      const std::array<double, NUM_DIMENSIONS> &,
      const std::array<double, NUM_DIMENSIONS> &)>;

  using cost_function_t = std::function<double(state_t *state_initial_in,
                                               trajectory_t *trajectory_in,
                                               state_t *state_final_in)>;
  distance_function_t distance_function;

  cost_function_t cost_function;

  // A list of functions that will be called in the event of updating the
  // minimum cost trajectory.
  std::list<update_func_t> list_update_functions;

  vertex_t *min_cost_vertex; // A pointer to the minimum cost vertex in the tree
  trajectory_t min_cost_trajectory; // A copy of the mininum cost trajectory

  region_t region_goal;

public:
  MinimumTimeReachability();
  ~MinimumTimeReachability();

  /**
   * \brief Constructor that initializes the goal region.
   *
   * This constructor initializes the goal region. Note that the
   * there is a constructor with no arguments. If initiated that
   * constructor will initialize the goal region to its default
   * values derived from the region class, which amounts
   * to a point in the origin.
   *
   * @param region_goal New goal region.
   */
  MinimumTimeReachability(const region_t &region_goal);

  /**
   * \brief Modifies the goal region.
   *
   * This function sets the goal region to its new value given
   * as an argument.
   *
   * @param region_goal New goal region.
   *
   * @returns Returns 1 if succcess, and a non-positive value to indicate error.
   */
  int set_goal_region(const region_t &region_goal);

  int ce_update_vertex_cost(vertex_t *vertex_in);

  int ce_update_edge_cost(edge_t *edge_in);

  int mc_update_insert_vertex(vertex_t *vertex_in);

  bool reaches_goal(vertex_t *vertex_in);

  inline void set_distance_function(distance_function_t func) {
    distance_function = func;
  }

  inline void set_cost_function(cost_function_t func) { cost_function = func; }

  int mc_update_insert_edge(edge_t *edge_in);

  int mc_update_delete_vertex(vertex_t *vertex_in);

  int mc_update_delete_edge(edge_t *edge_in);

  int get_solution(trajectory_t &trajectory_out);

  double evaluate_cost_trajectory(state_t *state_initial_in,
                                  trajectory_t *trajectory_in,
                                  state_t *state_final_in = 0);

  double default_cost_function(state_t *state_initial_in,
                               trajectory_t *trajectory_in,
                               state_t *state_final_in);

  /**
   * \brief Returns the cost of the best trajectory.
   *
   *  This function returns the cost of the minimum cost trajectory that reaches
   * the goal,
   *  if such a trajectory exists. Otherwise, it returns -1.0.
   *
   * @returns Returns the cost of the minimum cost trajectory, or -1.0 to
   * indicate no such trajectory exists.
   */
  double get_best_cost();

  /**
   * \brief Clears the update function list for minimum cost update.
   *
   * Whenever an optimizing motion planning algorithm using this component finds
   * a better trajectory, this component calls a list of functions that are
   * registered
   * for this call back. This method clears this list of functions.
   *
   * @returns Returns 1 if succcess, and a non-positive value to indicate error.
   */
  int clear_update_function_list();

  /**
   * \brief Clears the update function list for minimum cost update.
   *
   * Whenever an optimizing motion planning algorithm using this component finds
   * a better trajectory, this component calls a list of functions that are
   * registered
   * for this call back. This method registers a new update function, i.e., adds
   * the function given in the argument to the appropriate list.
   *
   * @returns Returns 1 if succcess, and a non-positive value to indicate error.
   */
  int register_new_update_function(update_func_t update_function);
};

template <int NUM_DIMENSIONS>
std::array<double, NUM_DIMENSIONS>
default_distance_function(const std::array<double, NUM_DIMENSIONS> &state,
                          const std::array<double, NUM_DIMENSIONS> &goal);
} // namespace smp
} // namespace smp

#include <smp/multipurpose/minimum_time_reachability_impl.hpp>
