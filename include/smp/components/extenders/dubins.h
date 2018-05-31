/*! \file components/extenders/dubins.h
  \brief The extend function component that implements a dubins car.

  The extender that this file implements an extender based on the
  dubins car dynamical system (the dimensionality of the state space
  is a template parameter) that exactly or approximately connects two given
  states.

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

#ifndef _SMP_SYSTEM_DUBINS_H_
#define _SMP_SYSTEM_DUBINS_H_

#include <smp/components/extenders/base.h>
#include <smp/components/extenders/input_array_double.h>
#include <smp/components/extenders/state_array_double.h>

#include <list>

namespace smp {

//! Implementation of the state data structure for the Dubins car dynamics
/*!
  This class implements the state data structure for the Dubins car dynamics.
  The number of state variables is three. The state variables indicate position
  in the x and y coordinates and the orientation, in this order.

  \ingroup states
*/
class StateDubins : public StateArrayDouble<3> {};

//! Implementation of the input data structure for the Dubins car dynamics.
/*!
  This class implements the input data structure for teh Dubins car dynamics.
  The number of input variables is exactly two. The first input variable
  stores the time it takes to execute the trajectory segment, while the second
  variable stores the steering input required.

  \ingroup inputs
*/
class InputDubins : public InputArrayDouble<2> {};

//! Implements the extender function with Dubins car dynamics.
/*!
  This class implements an extender with the Dubins car dynamics.

  \ingroup extenders
*/
class ExtenderDubins : public ExtenderBase<StateDubins, InputDubins> {

  using TrajectoryDubins = Trajectory<StateDubins, InputDubins>;

  double turning_radius{1.0};

  int extend_dubins_spheres(double x_s1, double y_s1, double t_s1, double x_s2,
                            double y_s2, double t_s2, int comb_no,
                            int *fully_extends,
                            std::list<StateDubins *> *list_states,
                            std::list<InputDubins *> *list_inputs);

  double extend_dubins_all(StateDubins *state_ini, StateDubins *state_fin,
                           int *fully_extends,
                           std::list<StateDubins *> *list_states_out,
                           std::list<InputDubins *> *list_inputs_out);

public:
  ExtenderDubins();
  ~ExtenderDubins();

  inline void set_turning_radius(double radius) { turning_radius = radius; }

  int extend(StateDubins *state_from_in, StateDubins *state_towards_in,
             int *exact_connection_out, TrajectoryDubins *trajectory_out,
             std::list<StateDubins *> *intermediate_vertices_out);
};
}

#endif
