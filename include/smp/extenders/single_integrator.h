/*! \file components/extenders/single_integrator.h
  \brief The single integrator system components. State, input, and extender
  definitions.

  This file implements the state, input, and extender classes for a
  d-dimensional single integrator system, where d is a template parameter
  when appropriate.

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

#ifndef _SMP_SYSTEM_SINGLE_INTEGRATOR_H_
#define _SMP_SYSTEM_SINGLE_INTEGRATOR_H_

#include <cmath>
#include <smp/components/extenders/base.h>
#include <smp/components/extenders/input_array_double.h>
#include <smp/components/extenders/state_array_double.h>

namespace smp {

//! State data structure for the single integrator dynamics
/*!
  This class implements the state data structure for the single integrator
  dynamics.
  The number of state variables is twice number of dimensions, since for each
  dimension
  both position and velocity has to be stored. The positions are stored in the
  usual order
  first, and then the all velocities are stored in their usual order, in the
  array.

  \ingroup states
*/
template <int NUM_DIMENSIONS>
class state_single_integrator : public state_array_double<NUM_DIMENSIONS> {};

//! Input data structure for the single integrator dynamics
/*!
  This class implements the input data structure for the single integrator
  dynamics.
  The number of input variables is one plus the dimensions. The extra input
  variable,
  placed in the beginning of the array, is used to store the time it takes to
  execute the trajectory segment.

  \ingroup inputs
*/
class input_single_integrator : public input_array_double<1> {};

//! Extender function with single integrator dynamics.
/*!
  This class implements an extender with single integrator dynamics, which can
  be used for planning in configuration spaces. Note that the set_max_length
  method of the class of must be called, before any other method can be called.
  The number of dimensions of the state space is a template argument for the
  class.

  \ingroup extenders
*/
template <class typeparams, int NUM_DIMENSIONS>
class extender_single_integrator : public extender_base<typeparams> {

  typedef typename typeparams::state state_t;
  typedef typename typeparams::input input_t;
  typedef typename typeparams::vertex_data vertex_data_t;
  typedef typename typeparams::edge_data edge_data_t;

  typedef vertex<typeparams> vertex_t;
  typedef edge<typeparams> edge_t;

  typedef trajectory<typeparams> trajectory_t;

  double max_length;

public:
  extender_single_integrator();
  ~extender_single_integrator();

  int ex_update_insert_vertex(vertex_t *vertex_in);

  int ex_update_insert_edge(edge_t *edge_in);

  int ex_update_delete_vertex(vertex_t *vertex_in);

  int ex_update_delete_edge(edge_t *edge_in);

  int extend(state_t *state_from_in, state_t *state_towards_in,
             int *exact_connection_out, trajectory_t *trajectory_out,
             std::list<state_t *> *intermediate_vertices_out);

  /**
   * \brief Sets the maximum length of the trajectory returned by the algorithm.
   *
   * This method sets the length of the longest trajectory returned by the
   * algorithm.
   * If the trajectory connecting two given states is longer than the value
   * specified
   * by the argument of this function then the only the maximum-length prefix of
   * this trajectory is returned. By default, the max_length paramter is set to
   * 1.0.
   *
   * @param max_length_in Maximum length of a trajectory.
   *
   * @returns Returns 1 for success, and a non-positive number to indicate
   * error.
   */
  int set_max_length(double max_length_in);
};
}

#endif
