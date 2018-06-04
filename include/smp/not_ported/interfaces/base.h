/*! \file interfaces/base.h
  \brief The abstract interfacer

  This file provides the necessary classes and function to interface
  certain robotics libraries, e.g., libbot, Microsoft Robotics, and the Robot
  Operating System (ROS), and their visualization tools.

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

#ifndef _SMP_INTERFACE_BASE_H_
#define _SMP_INTERFACE_BASE_H_

#include <smp/planners/base.h>

namespace smp {

//! The abstract class that specifies the structure of a interfacing component.
/*!
  The interfacing component provides a function to publish the graph that is
  stored in the planner. It also provides a function to visualize a certain
  trajectory (for instance, this trajectory can be the optimal trajectory in
  the graph).

  \ingroup interfaces
*/
template <class typeparams> class interface_base {

  typedef trajectory<typeparams> trajectory_t;
  typedef planner<typeparams> planner_t;

protected:
public:
  /**
   * \brief Publishes the graph maintained by the planner
   *
   * This function publishes the graph maintained by the planner.
   *
   * @returns Returns 1 for success, and a non-positive value to indicate an
   * error.
   */
  virtual int publish_data() = 0;

  /**
   * \brief Publishes a given trajectory
   *
   * This function will send out the messages (of the development environment)
   * containing the trajectory given as an argument.
   *
   * @param trajectory The trajectory that will be published.
   *
   * @returns Returns 1 for success, and a non-positive value to indicate an
   * error.
   */
  virtual int publish_trajectory(trajectory_t &trajectory) = 0;
};
}

#endif
