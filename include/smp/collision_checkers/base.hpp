/*! \file components/collision_checkers/base.h
  \brief The abstract collision checker.

  The collision checker function provides the following two functions: check
  whether
  a given state is collision-free, and check whether a given trajectory is
  collision free.
  The collision check function can be thought of as a generic check as to
  whether or not
  the new trajectory is suitable to be included in the graph.

 * Copyright (C) 2018 Sertac Karaman
 * Copyright (C) 2018 Chittaranjan Srinivas Swaminathan
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

#ifndef _SMP_COLLISION_CHECKER_BASE_H_
#define _SMP_COLLISION_CHECKER_BASE_H_

#include <smp/trajectory.hpp>
#include <smp/vertex_edge.hpp>

namespace smp {
namespace collision_checkers {
//! An abstract collision checker.
/*! The collision checker function provides the following two functions: check
  whether a given state is collision-free, and check whether a given trajectory
  is collision free. The collision check function can be thought of as a generic
  check as to whether or not the new trajectory is suitable to be included in
  the graph.

  \ingroup collision_checkers_base
*/
template <class State> class Base {

public:
  virtual ~Base(){};
  /**
   * \brief Checks whether a given state is collision free
   *
   * @param state_in The state that will be checked for collision.
   *
   * @return Returns 1 if the trajectory is collision-free, 0 if the
   *         trajectory collides with an obstacle, and a non-positive
   *         if error.
   */
  virtual int check_collision(State *state_in) = 0;

  /**
   * \brief Checks whether a given trajectory is collision free
   *
   * @param trajectory_in The trajectory that will be checked for collision.
   *
   * @return Returns 1 if the trajectory is collision-free, 0 if the
   *         trajectory collides with an obstacle, and a non-positive
   *         if error.
   */
  virtual int check_collision(const std::list<State *> &list_states) = 0;
};
} // namespace collision_checkers
} // namespace smp

#endif
