/*! \file components/extenders/double_integrator.h
  \brief The double integrator system components. State, input, and extender
  definitions.

  This file implements the state, input, and extender classes for a
  d-dimensional double integrator system, where d is a template parameter
  when appropriate. Currently, the implementation supports only d = 2.
  The author is working on the case when d > 2.

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

#ifndef _SMP_SYSTEM_DOUBLE_INTEGRATOR_H_
#define _SMP_SYSTEM_DOUBLE_INTEGRATOR_H_

#include <smp/components/extenders/base.h>
#include <smp/components/extenders/input_array_double.h>
#include <smp/components/extenders/state_array_double.h>

#include <list>

namespace smp {

//! Implementation of the state data structure for the double integrator
//! dynamics
/*! This class implements the state data structure for the double integrator
  dynamics. The number of state variables is twice number of dimensions, since
  for each dimension both position and velocity has to be stored. The positions
  are stored in the usual order first, and then the all velocities are stored in
  their usual order, in the array.
*/
template <int NUM_DIMENSIONS>
class StateDoubleIntegrator : public StateArrayDouble<NUM_DIMENSIONS * 2> {};

//! Implementation of the input data structure for the double integrator
//! dynamics
/*! This class implements the input data structure for the double integrator
  dynamics. The number of input variables is one plus the dimensions. The extra
  input variable, placed in the beginning of the array, is used to store the
  time it takes to execute the trajectory segment.
*/
template <int NUM_DIMENSIONS>
class InputDoubleIntegrator : public InputArrayDouble<NUM_DIMENSIONS + 1> {};

//! Extender function with double integrator dynamics.
/*! This class implements an extender with double integrator dynamics. It is
  intended that the number of dimensions of the state space is a template
  argument for the class. However, this feature is not implemented yet.
  Currently, the double integrator system works only in two dimensions
  (NUM_DIMENSIONS = 2), i.e., two positions and their two velocities.

  \ingroup extenders
*/
template <int NUM_DIMENSIONS>
class ExtenderDoubleIntegrator
    : public ExtenderBase<StateDoubleIntegrator<NUM_DIMENSIONS>,
                          InputDoubleIntegrator<NUM_DIMENSIONS>> {

  using state_t = StateDoubleIntegrator<NUM_DIMENSIONS>;
  using input_t = InputDoubleIntegrator<NUM_DIMENSIONS>;
  using trajectory_t = Trajectory<StateDoubleIntegrator<NUM_DIMENSIONS>,
                                  InputDoubleIntegrator<NUM_DIMENSIONS>>;

  // // TODO: get back to appropriate velocity constraints.
  // double velocity_constraint_min[NUM_DIMENSIONS];
  // double velocity_constraint_max[NUM_DIMENSIONS];

  int extend_with_optimal_control(state_t *state_ini, state_t *state_fin,
                                  std::list<state_t *> *list_states_out,
                                  std::list<input_t *> *list_inputs_out);

public:
  ExtenderDoubleIntegrator();
  ~ExtenderDoubleIntegrator();

  int extend(state_t *state_from_in, state_t *state_towards_in,
             int *exact_connection_out, trajectory_t *trajectory_out,
             std::list<state_t *> *intermediate_vertices_out);
};
} // namespace smp

#endif
