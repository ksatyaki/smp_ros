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

#ifndef _SMP_TRAJECTORY_HPP_
#define _SMP_TRAJECTORY_HPP_

#include <smp/planner_utils/trajectory.h>

template <class typeparams> smp::trajectory<typeparams>::trajectory() {}

template <class typeparams> smp::trajectory<typeparams>::~trajectory() {

  this->clear_delete();
}

template <class typeparams> int smp::trajectory<typeparams>::clear() {

  // Clear the list of states and the list of inputs.
  list_states.clear();
  list_inputs.clear();

  return 1;
}

template <class typeparams> int smp::trajectory<typeparams>::clear_delete() {

  // Free all the memory occupied by the states in the list.
  for (typename std::list<state_t *>::iterator iter_state = list_states.begin();
       iter_state != list_states.end(); iter_state++) {
    state_t *state_curr = *iter_state;
    delete state_curr;
  }

  // Free all the memory occupied by the inputs in the list.
  for (typename std::list<input_t *>::iterator iter_input = list_inputs.begin();
       iter_input != list_inputs.end(); iter_input++) {
    input_t *input_curr = *iter_input;
    delete input_curr;
  }

  // Clear the list of states and the list of inputs.
  this->clear();

  return 1;
}

#endif
