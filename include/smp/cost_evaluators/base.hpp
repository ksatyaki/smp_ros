/*! \file components/cost_evaluators/base.h
  \brief The abstract cost evaluator

  This file provides an implementation of the abstract class for the
  generic cost evaluator.

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

#ifndef _SMP_COST_EVALUATOR_BASE_H_
#define _SMP_COST_EVALUATOR_BASE_H_

#include <smp/trajectory.hpp>
#include <smp/vertex_edge.hpp>

namespace smp {
namespace cost_evaluators {

//! The abstract class that specifies the structure of the cost evalutor
//! component.
/*! This class implements the abstract cost evaluator class, which provides one
  main method that returns the cost of a trajectory starting from a given
  initial state a reaching a given final vertex.

  \ingroup cost_evaluators
*/
template <class State, class Input>
class Base {

  using edge_t = Edge<State, Input>;
  using vertex_t = Vertex<State, Input>;
  using trajectory_t = Trajectory<State, Input>;

public:
  /**
   * \brief Update function for vertex cost modification
   *
   * This function is called by the planner whenever a the cost associated
   * with a vertex is changed by the optimizing (incremental) planning
   * algorithm.
   *
   * @param vertex_in A pointer to the vertex with modified cost.
   *
   * @returns Return 1 if success, a non-positive value to indiacate error.
   */
  virtual int ce_update_vertex_cost(vertex_t *vertex_in) = 0;

  /**
   * \brief Update function for edge cost modification
   *
   * This function is called by the planner whenever the cost associated
   * with an edge is changed by the optimizing (incremental) planning algorithm.
   *
   * @param edge_in A pointer to the edge with modified cost.
   *
   * @returns Return 1 if success, a non-positive value to indiacate error.
   */
  virtual int ce_update_edge_cost(edge_t *edge_in) = 0;

  /**
   * \brief Evaluates the cost of a trajectory.
   *
   * This function returns the cost of a given trajectory that starts from
   * state_initial_in and reaches state_final_in. Sometimes the final state
   * is embedded in the trajectory itself, in which case state_final_in argument
   * can be set to NULL.
   *
   * @param state_initial_in Initial state that the trajectory starts from
   * @param trajectory_in Trajectory
   * @param state_final_in Final state that the trajectory reaches
   *
   * @returns Returns 1 for success, and a non-positive number to indicate
   * error.
   */
  virtual double evaluate_cost_trajectory(State *state_initial_in,
                                          trajectory_t *trajectory_in,
                                          State *state_final_in = 0) = 0;
};
} // namespace cost_evaluators
} // namespace smp
#endif
