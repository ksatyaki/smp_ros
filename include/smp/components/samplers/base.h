/*! \file components/samplers/base.h
  \brief The abstract sampler

  The sampler provides random or quasi-random sample states.
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

#ifndef _SMP_SAMPLER_BASE_H_
#define _SMP_SAMPLER_BASE_H_

#include <smp/planner_utils/vertex_edge.h>

namespace smp {

//! The abstract class that specifies the structure of a sampler component.
/*!
  A sampler component provides random or quasi-random samples of states.
  This abstract class specifies how the sample function should be
  implemented in any derived class.

  \ingroup samplers_base
*/
template <class typeparams> class sampler_base {

  typedef typename typeparams::state state_t;
  typedef typename typeparams::vertex_data vertex_data_t;
  typedef typename typeparams::edge_data edge_data_t;

  typedef vertex<typeparams> vertex_t;
  typedef edge<typeparams> edge_t;

public:
  virtual ~sampler_base(){};

  /**
   * \brief Update function for vertex insertion
   *
   * This function is called by the planner whenever a new vertex is
   * added to the graph. A pointer to the new vertex is given as an argument.
   *
   * @param vertex_in A pointer to the new vertex.
   *
   * @returns Return 1 if success, a non-positive value to indiacate error.
   */
  virtual int sm_update_insert_vertex(vertex_t *vertex_in) = 0;

  /**
   * \brief Update function for edge insertion
   *
   * This function is called by the planner whenever a new edge is
   * added to the graph. A pointer to the new edge is given as an argument.
   *
   * @param edge_in A pointer to the new edge.
   *
   * @returns Return 1 for success, a non-positive value to indiacate error.
   */
  virtual int sm_update_insert_edge(edge_t *edge_in) = 0;

  /**
   * \brief Update function for vertex deletion
   *
   * This function is called by the planner whenever a vertex is deleted
   * from the graph. A pointer to the vertex is given as an argument.
   *
   * @param vertex_in A pointer to deleted vertex.
   *
   * @returns Return 1 if success, a non-positive value to indiacate error.
   */
  virtual int sm_update_delete_vertex(vertex_t *vertex_in) = 0;

  /**
   * \brief Update function for edge insertion
   *
   * This function is called by the planner whenever an edge is delete
   * from the graph. A pointer to the edge is given as an argument.
   *
   * @param edge_in A pointer to deleted edge.
   *
   * @returns Return 1 for success, a non-positive value to indiacate error.
   */
  virtual int sm_update_delete_edge(edge_t *edge_in) = 0;

  /**
   * \brief Provides a sample state from the state space.
   *
   * This function creates (by allocating the memory) for a new state
   * that is sampled (randomly or quasi-randomly) from the state space.
   * It returns a pointer to the new state.
   *
   * @param state_sample_out A pointer to the state that will be returned.
   *                         This variable can be set to, e.g., the address
   *                         of a null pointer.
   *
   * @returns Returns 1 for success, a non-positive number for failure.
   */
  virtual int sample(state_t **state_sample_out) = 0;
};
}

#endif
