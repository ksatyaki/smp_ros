/*! \file branch_and_bound_euclidean.h
  \brief Branch and bound with the Euclidean distance heuristic

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

#ifndef _SMP_BRANCH_AND_BOUND_EUCLIDEAN_H_
#define _SMP_BRANCH_AND_BOUND_EUCLIDEAN_H_

#include <smp/utils/branch_and_bound_base.h>

#include <smp/common/region.h>
#include <smp/planner_utils/vertex_edge.h>

namespace smp {

//! Branch and bound with the Euclidean distance admissible heuristic (not
//! considering obstacles).
/*!
  Currently, the heuristic is not computing the exact distance. The current
  heuristic is not even
  admissible. It should be employed with care.

  \ingroup bnb
 */
template <class typeparams, int NUM_DIMENSIONS>
class branch_and_bound_euclidean : public branch_and_bound_base<typeparams> {

  typedef typename typeparams::state state_t;
  typedef typename typeparams::input input_t;
  typedef typename typeparams::vertex_data vertex_data_t;
  typedef typename typeparams::edge_data edge_data_t;

  typedef region<NUM_DIMENSIONS> region_t;

  typedef vertex<typeparams> vertex_t;
  typedef edge<typeparams> edge_t;

  typedef planner<typeparams> planner_t;

  region_t region_goal;

  vertex_t *root_vertex;

  int add_children_to_list(list<vertex_t *> &list_vertices_in,
                           vertex_t *vertex_in);

public:
  branch_and_bound_euclidean();
  ~branch_and_bound_euclidean();

  int run_branch_and_bound();

  /**
   * \brief Sets the goal region to which the Euclidean distance will be
   * computed.
   *
   * The branch and bound with the Euclidean distance heuristic considers the
   * Euclidean
   * distance neglegting the obstacles. It is assumed that the goal region is a
   * box, which
   * can be set or modified using this function.
   *
   * @param region_goal_in New goal region.
   *
   * @returns Returns 1 for success, and a non-positive value to indicate error.
   */
  int set_goal_region(region_t region_goal_in);

  /**
   * \brief Sets the root vertex of the planner.
   *
   * This function can be used to provide the heuristic with the knowledge of
   * the root vertex.
   * Since the heuristic is not admissible, it has the danger of deleting the
   * root vertex, which
   * can be avoided if a pointer to the root vertex is provided to the heuristic
   * using
   * this function.
   *
   * @param root_vertex_in A pointer to the root vertex of the incremental
   * planner.
   *
   * @returns Returns 1 for success, and a non-positive value to indicate error.
   */
  int set_root_vertex(vertex_t *root_vertex_in);
};
}

#endif
