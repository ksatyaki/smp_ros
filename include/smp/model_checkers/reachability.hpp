/*
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

#ifndef _SMP_MODEL_CHECKER_REACHABILITY_HPP_
#define _SMP_MODEL_CHECKER_REACHABILITY_HPP_

#include <smp/components/model_checkers/reachability.h>

#include <smp/common/region.hpp>
#include <smp/components/model_checkers/base.hpp>

template <class typeparams, int NUM_DIMENSIONS>
smp::model_checker_reachability<typeparams,
                                NUM_DIMENSIONS>::model_checker_reachability() {

  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    region_goal.center[i] = 0.0;
    region_goal.size[i] = 0.0;
  }
}

template <class typeparams, int NUM_DIMENSIONS>
smp::model_checker_reachability<typeparams,
                                NUM_DIMENSIONS>::~model_checker_reachability() {

}

template <class typeparams, int NUM_DIMENSIONS>
smp::model_checker_reachability<typeparams, NUM_DIMENSIONS>::
    model_checker_reachability(const region_t &region_in) {

  region_goal = region_in;
}

template <class typeparams, int NUM_DIMENSIONS>
int smp::model_checker_reachability<
    typeparams, NUM_DIMENSIONS>::set_goal_region(const region_t &region_in) {

  region_goal = region_in;

  return 1;
}

template <class typeparams, int NUM_DIMENSIONS>
int smp::model_checker_reachability<
    typeparams, NUM_DIMENSIONS>::mc_update_insert_vertex(vertex_t *vertex_in) {

  for (int i = 0; i < NUM_DIMENSIONS; i++) {

    // double state_var_curr = (*vertex_in->state)[i];

    if (fabs((*vertex_in->state)[i] - region_goal.center[i]) >
        region_goal.size[i]) {
      vertex_in->data.reaches_goal = 0;
      return 1;
    }
  }

  vertex_in->data.reaches_goal = 1;

  return 1;
}

template <class typeparams, int NUM_DIMENSIONS>
int smp::model_checker_reachability<
    typeparams, NUM_DIMENSIONS>::mc_update_insert_edge(edge_t *edge_in) {

  return 1;
}

template <class typeparams, int NUM_DIMENSIONS>
int smp::model_checker_reachability<
    typeparams, NUM_DIMENSIONS>::mc_update_delete_vertex(vertex_t *vertex_in) {

  return 1;
}

template <class typeparams, int NUM_DIMENSIONS>
int smp::model_checker_reachability<
    typeparams, NUM_DIMENSIONS>::mc_update_delete_edge(edge_t *edge_in) {

  return 1;
}

template <class typeparams, int NUM_DIMENSIONS>
int smp::model_checker_reachability<typeparams, NUM_DIMENSIONS>::get_solution(
    trajectory_t &trajectory_out) {

  trajectory_out.clear();

  return 1;
}

#endif
