/*! \file components/collision_checkers/base.h
  \brief The abstract collision checker.

  The collision checker function provides the following two functions: check
  whether
  a given state is collision-free, and check whether a given trajectory is
  collision free.
  The collision check function can be thought of as a generic check as to
  whether or not
  the new trajectory is suitable to be included in the graph.

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

#ifndef _SMP_COLLISION_CHECKER_BASE_H_
#define _SMP_COLLISION_CHECKER_BASE_H_

#include <smp/planner_utils/trajectory.h>
#include <smp/planner_utils/vertex_edge.h>

namespace smp {

//! An abstract collision checker.
/*! The collision checker function provides the following two functions: check
  whether a given state is collision-free, and check whether a given trajectory
  is collision free. The collision check function can be thought of as a generic
  check as to whether or not the new trajectory is suitable to be included in
  the graph.

  \ingroup collision_checkers_base
*/
template <class typeparams> class collision_checker_base {

  typedef typename typeparams::state state_t;
  typedef typename typeparams::input input_t;
  typedef typename typeparams::vertex_data vertex_data_t;
  typedef typename typeparams::edge_data edge_data_t;

  typedef vertex<typeparams> vertex_t;
  typedef edge<typeparams> edge_t;

  typedef trajectory<typeparams> trajectory_t;

public:
  virtual ~collision_checker_base(){};

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
  virtual int cc_update_insert_vertex(vertex_t *vertex_in) = 0;

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
  virtual int cc_update_insert_edge(edge_t *edge_in) = 0;

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
  virtual int cc_update_delete_vertex(vertex_t *vertex_in) = 0;

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
  virtual int cc_update_delete_edge(edge_t *edge_in) = 0;

  /**
   * \brief Checks whether a given state is collision free
   *
   * @param state_in The state that will be checked for collision.
   *
   * @return Returns 1 if the trajectory is collision-free, 0 if the
   *         trajectory collides with an obstacle, and a non-positive
   *         if error.
   */
  virtual int check_collision_state(state_t *state_in) = 0;

  /**
   * \brief Checks whether a given trajectory is collision free
   *
   * @param trajectory_in The trajectory that will be checked for collision.
   *
   * @return Returns 1 if the trajectory is collision-free, 0 if the
   *         trajectory collides with an obstacle, and a non-positive
   *         if error.
   */
  virtual int check_collision_trajectory(trajectory_t *trajectory_in) = 0;
};
}

#endif
