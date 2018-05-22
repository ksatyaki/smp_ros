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

#ifndef _SMP_SYSTEM_SINGLE_INTEGRATOR_HPP_
#define _SMP_SYSTEM_SINGLE_INTEGRATOR_HPP_

#include <smp/components/extenders/single_integrator.h>

#include <smp/components/extenders/base.hpp>
#include <smp/components/extenders/input_array_double.hpp>
#include <smp/components/extenders/state_array_double.hpp>

#include <iostream>
#include <math.h>

template <class typeparams, int NUM_DIMENSIONS>
int smp::extender_single_integrator<
    typeparams, NUM_DIMENSIONS>::ex_update_insert_vertex(vertex_t *vertex_in) {

  return 1;
}

template <class typeparams, int NUM_DIMENSIONS>
int smp::extender_single_integrator<
    typeparams, NUM_DIMENSIONS>::ex_update_insert_edge(edge_t *edge_in) {

  return 1;
}

template <class typeparams, int NUM_DIMENSIONS>
int smp::extender_single_integrator<
    typeparams, NUM_DIMENSIONS>::ex_update_delete_vertex(vertex_t *vertex_in) {

  return 1;
}

template <class typeparams, int NUM_DIMENSIONS>
int smp::extender_single_integrator<
    typeparams, NUM_DIMENSIONS>::ex_update_delete_edge(edge_t *edge_in) {

  return 1;
}

template <class typeparams, int NUM_DIMENSIONS>
smp::extender_single_integrator<typeparams,
                                NUM_DIMENSIONS>::extender_single_integrator() {

  max_length = 1.0;
}

template <class typeparams, int NUM_DIMENSIONS>
smp::extender_single_integrator<typeparams,
                                NUM_DIMENSIONS>::~extender_single_integrator() {

}

template <class typeparams, int NUM_DIMENSIONS>
int smp::extender_single_integrator<typeparams, NUM_DIMENSIONS>::set_max_length(
    double max_length_in) {

  if (max_length_in <= 0.0)
    return 0;

  max_length = max_length_in;

  return 1;
}

template <class typeparams, int NUM_DIMENSIONS>
int smp::extender_single_integrator<typeparams, NUM_DIMENSIONS>::extend(
    state_t *state_from_in, state_t *state_towards_in,
    int *exact_connection_out, trajectory_t *trajectory_out,
    std::list<state_t *> *intermediate_vertices_out) {

  if (max_length <= 0.0)
    return 0;

  trajectory_out->list_states.clear();
  trajectory_out->list_inputs.clear();
  intermediate_vertices_out->clear();

  double dists[NUM_DIMENSIONS];
  double dist = 0.0;

  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    dists[i] = (*state_towards_in)[i] - (*state_from_in)[i];
    dist += dists[i] * dists[i];
  }
  dist = sqrt(dist);

  state_t *state_new;
  input_t *input_new = new input_t;

  if (dist < max_length) {

    state_new = new state_t(*state_towards_in);
    (*input_new)[0] = dist;
    *exact_connection_out = 1;
  } else {

    state_new = new state_t;
    for (int i = 0; i < NUM_DIMENSIONS; i++)
      (*state_new)[i] = (*state_from_in)[i] + dists[i] / dist * max_length;
    (*input_new)[0] = max_length;
    *exact_connection_out = 0;
  }

  trajectory_out->list_states.push_back(state_new);
  trajectory_out->list_inputs.push_back(input_new);

  return 1;
}

#endif
