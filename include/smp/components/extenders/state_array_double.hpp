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

#ifndef _SMP_STATE_ARRAY_DOUBLE_HPP_
#define _SMP_STATE_ARRAY_DOUBLE_HPP_

#include <smp/components/extenders/state_array_double.h>

#include <iostream>

template <int NUM_STATES>
smp::StateArrayDouble<NUM_STATES>::StateArrayDouble() {

  for (int i = 0; i < NUM_STATES; i++)
    state_vars[i] = 0.0;
}

template <int NUM_STATES>
smp::StateArrayDouble<NUM_STATES>::StateArrayDouble(
    const smp::StateArrayDouble<NUM_STATES> &state_in) {

  for (int i = 0; i < NUM_STATES; i++)
    state_vars[i] = state_in.state_vars[i];
}

template <int NUM_STATES>
smp::StateArrayDouble<NUM_STATES>::~StateArrayDouble() {}

template <int NUM_STATES>
const smp::StateArrayDouble<NUM_STATES> &smp::StateArrayDouble<NUM_STATES>::
operator=(const smp::StateArrayDouble<NUM_STATES> &state_in) {

  if (&state_in != this) {
    for (int i = 0; i < NUM_STATES; i++)
      state_vars[i] = state_in.state_vars[i];
  }

  return *this;
}

template <int NUM_STATES>
double &smp::StateArrayDouble<NUM_STATES>::operator[](int index_in) {

  if ((index_in < NUM_STATES) && (index_in >= 0))
    return state_vars[index_in];

  // TODO: ideally throw an exception also.
  return state_vars[0];
}

#endif
