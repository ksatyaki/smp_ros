/*! \file planners/rrtstar.h
  \brief An implementation of a RRT* algorithm.

  Provides an implementation of the RRT* algorithm. Inherits from the generic
  incremental sampling-based motion planner, overriding the iteration function.

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
#include <smp/planners/base_incremental.hpp>
#include <smp/planners/parameters.hpp>

#include <chrono>

namespace smp {

namespace planners {
//! RRT* algorithm
/*!
  Provides an implementation of the RRT* algorithm. Inherits from the generic
  incremental sampling-based motion planner, overriding the iteration function.

  \ingroup planners
*/
template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
class RRTStar : public BaseIncremental<State, Input, VertexData, EdgeData,
                                       NUM_DIMENSIONS> {

  using vertex_t = Vertex<State, Input, VertexData, EdgeData>;
  using edge_t = Edge<State, Input, VertexData, EdgeData>;

  using trajectory_t = Trajectory<State, Input>;
  using sampler_t = samplers::Base<State>;
  using distance_evaluator_t =
      distance_evaluators::Base<State, Input, VertexData, EdgeData>;
  using extender_t = extenders::Base<State, Input>;
  using collision_checker_t = collision_checkers::Base<State, Input>;
  using model_checker_t =
      model_checkers::Base<State, Input, VertexData, EdgeData>;
  using cost_evaluator_t =
      cost_evaluators::Base<State, Input, VertexData, EdgeData>;

private:
  // This function adds the given state to the beginning of the tracjetory and
  // calls the collision checker.
  int check_extended_trajectory_for_collision(State *state,
                                              trajectory_t *trajectory) {

    trajectory->list_states.push_front(state);
    int collision_check =
        this->collision_checker.check_collision_trajectory(trajectory);
    trajectory->list_states.pop_front();

    return collision_check;
  }

  // The radius that was used in the previous iteration
  double radius_last;

protected:
  /**
   * Total planning time.
   */
  float planning_time;

  /**
   * A steady clock
   */
  std::chrono::steady_clock clock;
  /**
   * @name Components
   */
  //@{

  //! A pointer to the cost evaluator component
  /*!
    The cost evaluator component evaluates the cost of a given trajectory.
  */
  cost_evaluator_t &cost_evaluator;

  //@}

  /**
   * \brief A function call the propagate the new cost down the edges of the
   * tree structure.
   *
   * Modifies the cost of the vertex stored in the vertex_in argument to the
   * cost stored in the total_cost_new argument. And propagates the new cost
   * along the outgoing edges of vertex_in.
   *
   * @param vertex_in The vertex the cost of which will be modified.
   * @param total_cost_new The new cost of the vertex_in variable.
   *
   * @return Returns 1 for success, and a non-positive number for failure.
   */
  int propagate_cost(vertex_t *vertex_in, double total_cost_new);

public:
  //! Algorithm parameters
  /*!
    This class stores the parameters used by the algorithm. These parameters
    can be modified by the user using the methods provided by the class
    planner_parameters.
  */
  Parameters parameters;

  RRTStar();
  ~RRTStar();

  /**
   * \brief A constructor that initializes all components.
   *
   * This is the recommended constructor that initializes all components all at
   * once.
   * It calls the corresponding constructor of the base class
   * planner_incremental<typeparams> with its first five arguments. The last
   * argument, i.e., cost_evaluator_in, is the new cost evaluator component,
   * a reference to which is stored in this class (not the base class
   * planner_incremental<typeparams>).
   *
   * @param sampler_in New sampler component.
   * @param distance_evaluator_in New distance evaluator component.
   * @param extender_in New extension function component.
   * @param collision_checker_in New collision checker component.
   * @param model_checker_in New model checker component.
   * @param cost_evaluator_in New cost evaluator component.
   */
  RRTStar(sampler_t &sampler_in, distance_evaluator_t &distance_evaluator_in,
          extender_t &extender_in, collision_checker_t &collision_checker_in,
          model_checker_t &model_checker_in,
          cost_evaluator_t &cost_evaluator_in);

  /**
   * \brief A function call to initialize the incremental sampling-based
planner.
   *
   * First it calls the planner_incremental::itinialize function, which deletes
   * the current graph stored by the planner, and If the initial_state_in
argument
   * is non-NULL, creates a new vertex that with the state stored in the
initial_state_in
   * argument.
x         *
   * @param initial_state_in The state that the root_vertex will include. If
this argument
   * is NULL, then no root vertex is created (But, the graph stored in the
planner is
   * deleted.
   *
   * @returns Returns 1 for success, and a non-positive number for failure.
   */
  int initialize(State *initial_state_in = 0);

  float get_planning_time();

  /**
   * @name Component initializer functions
   */
  //@{

  /**
   * \brief Initializes the cost evaluator component.
   *
   * @param cost_evaluator_in The cost evalutor component.
   *
   * @return Returns 1 for success, a non-positive number for failure (see the
   * source code for failure modes).
   */
  int init_cost_evaluator(cost_evaluator_t &cost_evaluator_in);

  //@}

  /**
   * \brief Returns the radius of the ball that the connections are sought
   * within.
   *
   * @return Returns the radius of the ball that the connections are sought
   * within.
   */
  double get_ball_radius_last() { return radius_last; }

  /**
   * \brief Initiate one iteration of the RRT* algorithm.
   *
   * Runs one iteration of the RRT* algorithm which includes the following
   * steps:
   * - get one sample state (using the sampler component)
   * - find the vertex in the graph that is nearest to the sample state
   * (using the distance evaluator component)
   * - generate a trajectory that starts from the state stored in the nearest
   * vertex and reaches exactly or approximately to the sample state (using
   * the extension function component)
   * - check whether the new trajectory satsifies the conditions for being
   * collision free (using the collision checker component).
   * - if the new trajectory is collision free, then
   *   - set the minimum cost vertex to the nearest vertex and the minimum cost
   *     trajectory to the current trajectory.
   *   - compute the set of near vertices (using the distance evaluator
   * component).
   *   - for all vertecies in the near set
   *     - generate a new trajectory from the near vertex to the extended vertex
   *       (using the extension function component).
   *     - if the new trajectory is collision free (check using the collision
   * checker
   *       component) and exactly connects the two vertices, then compute the
   * cost
   *       of the new trajectory (using the cost evaluator component).
   *     - if the cost to get to the near node plus the cost of the new
   * trajectory
   *       is less than the mininimum cost solution, then
   *       - set the minimum cost vertex to the current near vertex and set the
   *         minimum cost trajectory to the current trajectory.
   *   - add the new vertex to the graph and add an edge from the min cost
   * vertex to
   *     the new vertex connecting them with the minimum cost trajectory.
   *   - for all vertecies in the near set (// rewiring step)
   *     - generate a new trajectory from the extended vertex to the near vertex
   *       (using the extension function component).
   *     - if the new trajectory is collision free (check using the collision
   * checker
   *       component) and exactly connects the two vertices,
   *       then add the new trajectory to the graph as an edge from the extended
   * vertex to
   *       the near vertex.
   *     - incrementally check whether the graph includes a trajectory that
   *       satisfies the termination requirement (using the model checker
   * component).
   *
   * @returns Returns 1 for success, and a non-positive number for failure.
   */
  int iteration();
};
} // namespace planners
} // namespace smp

#include <smp/planners/rrtstar_impl.hpp>
