/*! \file planners/base_incremental.h
  \brief Generic incremental sampling-based motion planner definition

  The generic incremental sampling-based motion planner inherits from the
  generic sampling-based motion planner and provides the virtual iteration()
  function, which is overloaded by the inheriting incremental sampling-based
  algorithm.

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

#ifndef _SMP_PLANNER_INCREMENTAL_H_
#define _SMP_PLANNER_INCREMENTAL_H_

#include <smp/planners/base.h>

namespace smp {

//! Generic incremental sampling-based motion planner
/*!
  The generic incremental sampling-based motion planner inherits from the
  generic sampling-based motion planner and provides the virtual iteration()
  function, which is overloaded by the inheriting incremental sampling-based
  algorithm.

  \ingroup planners_base
*/
template <class typeparams>
class planner_incremental : public planner<typeparams> {

  typedef typename typeparams::state state_t;

  typedef vertex<typeparams> vertex_t;
  typedef planner<typeparams> planner_t;

  typedef trajectory<typeparams> trajectory_t;

  typedef sampler_base<typeparams> sampler_t;
  typedef distance_evaluator_base<typeparams> distance_evaluator_t;
  typedef extender_base<typeparams> extender_t;
  typedef collision_checker_base<typeparams> collision_checker_t;
  typedef model_checker_base<typeparams> model_checker_t;

public:
  //! A pointer to the root vertex of the incremental algorithm
  /*!
    Any incremental algorithm is assumed to generate a graph of trajectories of
    a
    dynamical system such that each trajectory starts from a given initial
    condition.
    This variable is a pointer to the vertex that stores the initial state.
  */
  vertex_t *root_vertex;

  planner_incremental();
  ~planner_incremental();

  /**
   * \brief A constructor that initializes all components.
   *
   * This is the recommended constructor that initializes all components all at
   * once.
   *
   * @param sampler_in New sampler component.
   * @param distance_evaluator_in New distance evaluator component.
   * @param extender_in New extension function component.
   * @param collision_checker_in New collision checker component.
   * @param model_checker_in New model checker component.
   */
  planner_incremental(sampler_t &sampler_in,
                      distance_evaluator_t &distance_evaluator_in,
                      extender_t &extender_in,
                      collision_checker_t &collision_checker_in,
                      model_checker_t &model_checker_in);

  /**
   * \brief A function call to initialize the incremental sampling-based
   * planner.
   *
   * Deletes the current graph stored by the planner. If the initial_state_in
   * argument
   * is non-NULL, creates a new vertex that with the state stored in the
   * initial_state_in
   * argument.
   *
   * @param initial_state_in The state that the root_vertex will include. If
   * this argument
   * is NULL, then no root vertex is created (But, the graph stored in the
   * planner is
   * deleted.
   *
   * @returns Returns 1 for success, and a non-positive number for failure.
   */
  int initialize(state_t *initial_state_in = 0);

  /**
   * \brief Returns a pointer to the root vertex
   *
   * An incremental planner might (optionally) have a root vertex, a pointer to
   * which
   * can be obtained using this function.
   *
   * @returns Returns a pointer to the root vertex, and NULL if the root vertex
   * is not set.
   */
  vertex_t *get_root_vertex() { return root_vertex; }

  /**
   * \brief A virtual call to initiate one iteration of the algorithm.
   *
   * Runs one iteration of the inheriting incremental sampling-based algorithm,
   * which
   * overloads this function.
   *
   * @returns Returns 1 for success, and a non-positive number for failure.
   */
  virtual int iteration() = 0;
};
}

#endif
