/*! \file components/extenders/base.h
  \brief The abstract extender

  The extender (aka, the extension function) generates that exactly or
  approximately
  connects two given states.

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

#ifndef _SMP_EXTENDER_BASE_H_
#define _SMP_EXTENDER_BASE_H_

#include <smp/trajectory.hpp>

#include <list>

namespace smp {
namespace extenders {
//! The abstract class that specifies the structure of the extender component.
/*! An extender provides the function to generate a trajectory that connects two
  given states. The extender can also provide a list of designated states, which
  become vertices of their own when added to the graph maintained by the
  planning algorithm.

  \ingroup extenders_base
*/
template <class State, class Input> class Base {

  using trajectory_t = Trajectory<State, Input>;

public:
  /**
   * \brief Abstract function that generates a trajectory connecting two given
   * states.
   *
   * Generates a trajectory, returned in the trajectory_out argument, that
   * connects two given states, provided with the state_from_in and
   * state_towards_in arguments. If the connection is exact, i.e., the
   * trajectory reaches state_towards_in exactly, then the output variable
   * exact_connection_out is set to one. If, on the other hand, the connection
   * is approximate, then the same variable is set to zero.
   *
   * @param state_from_in The state that the new trajectory starts from. @param
   * state_towards_in The state that the new trajectory is shooted towards.
   * @param exact_connection_out Set to one if the connection is exact,
   * otherwise this variable is set to zero by this function. @param
   * trajectory_out The output variable that contians the resulting trajectory.
   * @param intermediate_vertices_out The list of states that will be individual
   * vertices.
   *
   * @returns Returns 1 for success, a non-positive number to indicate error.
   */
  virtual int extend(State *state_from_in, State *state_towards_in,
                     int *exact_connection_out, trajectory_t *trajectory_out,
                     std::list<State *> *intermediate_vertices_out) = 0;
};
}
} // namespace smp

#endif
