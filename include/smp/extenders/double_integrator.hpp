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

// TODO: Generalize to multiple dimensions, make these configurable variables
// (already started in the .h file)
#define VELOCITY_CONSTRAINT_RANGE 2.0
#define VELOCITY_CONSTRAINT_RANGE_2 1.0
#define VELOCITY_CONSTRAINT_MAX 1.0
#define VELOCITY_CONSTRAINT_MIN -1.0
#define VELOCITY_CONSTRAINT_SQ 1.0
//
#define INPUT_CONSTRAINT_MAX 1.0
#define INPUT_CONSTRAINT_MIN -1.0
//
#define DELTA_T 0.1 // The time interval of integration and node placement

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <list>
#include <smp/extenders/base.hpp>
#include <smp/input_array_double.hpp>
#include <smp/state_array_double.hpp>

namespace smp {

double extend_with_time_optimal_control_one_axis(
    double s_ini[2], double s_fin[2], double u_max, int *direction,
    int *traj_saturated, double *x_intersect_beg, double *x_intersect_end,
    double *v_intersect);

int extend_with_effort_optimal_control_one_axis(
    double s_ini[2], double s_fin[2], double u_max, double t_min, double t_goal,
    double t_eps, int *dir, int *traj_saturated, double *max_control,
    double *x_intersect_beg, double *x_intersect_end, double *v_intersect);

//! Implementation of the state data structure for the double integrator
//! dynamics
/*! This class implements the state data structure for the double integrator
  dynamics. The number of state variables is twice number of dimensions,
  since for each dimension both position and velocity has to be stored. The
  positions are stored in the usual order first, and then the all velocities
  are stored in their usual order, in the array.
*/
class StateDoubleIntegrator : public StateArrayDouble<4> {};

//! Implementation of the input data structure for the double integrator
//! dynamics
/*! This class implements the input data structure for the double integrator
  dynamics. The number of input variables is one plus the dimensions. The extra
  input variable, placed in the beginning of the array, is used to store the
  time it takes to execute the trajectory segment.
*/
class InputDoubleIntegrator : public InputArrayDouble<3> {};

namespace extenders {
//! Extender function with double integrator dynamics.
/*! This class implements an extender with double integrator dynamics. It is
  intended that the number of dimensions of the state space is a template
  argument for the class. However, this feature is not implemented yet.
  Currently, the double integrator system works only in two dimensions
  (NUM_DIMENSIONS = 2), i.e., two positions and their two velocities.

  \ingroup extenders
*/
class DoubleIntegrator
    : public Base<StateDoubleIntegrator, InputDoubleIntegrator> {

  using state_t = StateDoubleIntegrator;
  using input_t = InputDoubleIntegrator;
  using trajectory_t = Trajectory<StateDoubleIntegrator, InputDoubleIntegrator>;

  // // TODO: get back to appropriate velocity constraints.
  // double velocity_constraint_min[NUM_DIMENSIONS];
  // double velocity_constraint_max[NUM_DIMENSIONS];

  int extend_with_optimal_control(
      StateDoubleIntegrator *state_ini, StateDoubleIntegrator *state_fin,
      std::list<StateDoubleIntegrator *> *list_states_out,
      std::list<input_t *> *list_inputs_out);

public:
  inline DoubleIntegrator() {}
  inline ~DoubleIntegrator() {}

  inline int
  extend(StateDoubleIntegrator *state_from_in,
         StateDoubleIntegrator *state_towards_in, int *exact_connection_out,
         trajectory_t *trajectory_out,
         std::list<StateDoubleIntegrator *> *intermediate_vertices_out);
}; // namespace extenders
} // namespace extenders
} // namespace smp

#endif
