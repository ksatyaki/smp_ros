/*! \file trajectory.h
  \brief Definition of the trajectory class

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

#ifndef _SMP_TRAJECTORY_H_
#define _SMP_TRAJECTORY_H_

#include <list>

namespace smp {

//! Trajectory definition as a states with interleaving inputs.
/*!
  The Trajectory class, composed of a list of states and a list of inputs,
  is an implementation of the notion of a trajectory that connects two given
  states in the graph.

  \ingroup graphs
*/
template <class State, class Input> class Trajectory {

public:
  //! A list of the states in the trajectory.
  std::list<State *> list_states;

  //! A list of the inputs in the trajectory.
  std::list<Input *> list_inputs;

  Trajectory() {}
  ~Trajectory() { this->clear_delete(); }

  //! Clears the trajectory.
  /*! This function clears both the state list and the input list in the
    trajectory. But, it does NOT attempt to free the memory occupied by the said
    states and inputs.
  */
  int clear() {
    list_states.clear();
    list_inputs.clear();
    return 1;
  }

  //! Clears the trajectory and frees the memory.
  /*! This function clears both the state list and the input list in the
    trajectory. It also frees the memory occupied by the said states and the
    inputs, by calling the delete operator with each state and input present in
    the lists.
  */
  int clear_delete() {

    // Free all the memory occupied by the states in the list.
    for (typename std::list<State *>::iterator iter_state = list_states.begin();
         iter_state != list_states.end(); iter_state++) {
      State *state_curr = *iter_state;
      delete state_curr;
    }

    // Free all the memory occupied by the inputs in the list.
    for (typename std::list<Input *>::iterator iter_input = list_inputs.begin();
         iter_input != list_inputs.end(); iter_input++) {
      Input *input_curr = *iter_input;
      delete input_curr;
    }

    // Clear the list of states and the list of inputs.
    this->clear();

    return 1;
  }

};
} // namespace smp

#endif
