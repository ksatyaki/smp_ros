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

#ifndef _SMP_COST_EVALUATOR_TIME_HPP_
#define _SMP_COST_EVALUATOR_TIME_HPP_

#include <iostream>

#include <smp/components/cost_evaluators/time.h>

#include <smp/components/cost_evaluators/base.hpp>

template <class typeparams>
int smp::cost_evaluator_time<typeparams>::ce_update_vertex_cost(
    vertex_t *vertex_in) {

  return 1;
}

template <class typeparams>
int smp::cost_evaluator_time<typeparams>::ce_update_edge_cost(edge_t *edge_in) {

  return 1;
}

template <class typeparams>
double smp::cost_evaluator_time<typeparams>::evaluate_cost_trajectory(
    state_t *state_initial_in, trajectory_t *trajectory_in,
    state_t *state_final_in) {

  double total_time = 0.0;
  for (typename std::list<input_t *>::iterator iter =
           trajectory_in->list_inputs.begin();
       iter != trajectory_in->list_inputs.end(); iter++) {

    input_t *input_curr = *iter;

    total_time += (*input_curr)[0];
  }

  return total_time;
};

#endif
