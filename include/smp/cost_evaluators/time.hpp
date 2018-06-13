/*! \file components/cost_evaluators/time.h
  \brief The cost evaluator based on the execution time

  This file implements the smp_cost_evaluator_time class that computes the
  cost of the trajectory based on the time it takes to execute it.

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

namespace smp {
namespace cost_evaluators {

//! The cost evaluator class based on the trajectory execution time
/*!
  This class computes the cost of a trajectory according to the time
  it takes to execute that particular trajectory.

  \ingroup cost_evaluators
*/
template <class State, class Input>
class Time : public Base<State, Input> {

  using vertex_t = Vertex<State, Input>;
  using edge_t = Edge<State, Input>;
  using trajectory_t = Trajectory<State, Input>;

public:
  int ce_update_vertex_cost(vertex_t *vertex_in) { return 1; }

  int ce_update_edge_cost(edge_t *edge_in) { return 1; }

  double evaluate_cost_trajectory(State *state_initial_in,
                                  trajectory_t *trajectory_in,
                                  State *state_final_in = 0) {
    double total_time = 0.0;
    for (typename std::list<input_t *>::iterator iter =
             trajectory_in->list_inputs.begin();
         iter != trajectory_in->list_inputs.end(); iter++) {

      input_t *input_curr = *iter;

      total_time += (*input_curr)[0];
    }

    return total_time;
  }
};
} // namespace cost_evaluators
} // namespace smp
