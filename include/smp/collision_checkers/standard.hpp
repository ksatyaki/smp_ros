/*! \file components/collision_checkers/standard.h
  \brief The standard brute-force collision checker

  This file implements the standard collision checker class. The region
  class, which is used to describe rectangular obstacles in the Euclidean
  space is defined in region.h

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

#pragma once

#include <smp/collision_checkers/base.hpp>
#include <smp/region.hpp>

#include <cmath>
#include <list>

namespace smp {
namespace collision_checkers {

//! Standard collision checker
/*!
  This class implements the standard collision checker. Standard collision
  checking procedure discretizes the trajectories connecting consecutive
  states. The said trajectory is obtained by a linear interpolation between
  the said states. Each interpolated state is, then, checked for collisioon
  with all the obstacles. This procedure is continued for all the states in
  the trajectory. A single states is checked for collision by merely going
  through the list of obstacles to check whether the query state resides
  inside any of the obstacles.

  \ingroup collision_checkers
*/
template <class State, class Input, int NUM_DIMENSIONS>
class Standard : public Base<State, Input> {

  using trajectory_t = Trajectory<State, Input>;
  using region_t = Region<NUM_DIMENSIONS>;

  int num_discretization_steps;
  double discretization_length;

  // 0: no discretization
  // 1: use steps discretization
  // 2: use length discretization
  int discretization_method;

  std::list<region_t *> list_obstacles;

public:
  Standard() {

    num_discretization_steps = 20;
    discretization_length = 0.1;
    discretization_method = 2;
  }

  ~Standard() {
    for (typename std::list<region_t *>::iterator iter = list_obstacles.begin();
         iter != list_obstacles.end(); iter++) {

      region_t *region_curr = *iter;

      delete region_curr;
    }
  }

  int check_collision(State *state_in) {
    if (list_obstacles.size() == 0)
      return 1;

    for (typename std::list<region_t *>::iterator iter = list_obstacles.begin();
         iter != list_obstacles.end(); iter++) {
      region_t *region_curr = *iter;

      bool collision = true;

      for (int i = 0; i < NUM_DIMENSIONS; i++) {

        if (fabs((*state_in)[i] - region_curr->center[i]) >=
            region_curr->size[i] / 2.0)
          collision = false;
      }

      if (collision) {
        return 0;
      }
    }

    return 1;
  }

  int check_collision(const std::list<State *> &list_states) {

    if (list_obstacles.size() == 0)
      return 1;

    if (list_states.size() == 0)
      return 1;

    typename std::list<State *>::iterator iter = list_states.begin();

    State *state_prev = *iter;

    if (this->check_collision(state_prev) == 0)
      return 0;

    iter++;

    for (; iter != list_states.end(); iter++) {

      State *state_curr = *iter;

      if (discretization_method != 0) {
        // Compute the increments
        double dist_total = 0.0;
        double increments[NUM_DIMENSIONS];
        for (int i = 0; i < NUM_DIMENSIONS; i++) {
          double increment_curr = (*state_curr)[i] - (*state_prev)[i];
          dist_total += increment_curr * increment_curr;
          increments[i] = increment_curr;
        }
        dist_total = sqrt(dist_total);

        // Compute the number of increments
        int num_increments;
        if (discretization_method == 1) {
          num_increments = num_discretization_steps;
        } else if (discretization_method == 2) {
          num_increments = (int)floor(dist_total / discretization_length);
        }

        // Execute the remaining only if the discretization is required.
        if (num_increments > 0) {

          for (int i = 0; i < NUM_DIMENSIONS; i++) // Normalize the increments.
            increments[i] = increments[i] / ((double)(num_increments + 1));

          for (typename std::list<region_t *>::iterator iter =
                   list_obstacles.begin();
               iter != list_obstacles.end(); iter++) {

            region_t *region_curr = *iter;

            for (int idx_state = 1; idx_state <= num_increments; idx_state++) {
              bool collision = true;

              for (int i = 0; i < NUM_DIMENSIONS; i++) {
                if (fabs((*state_prev)[i] + increments[i] * idx_state -
                         region_curr->center[i]) >=
                    region_curr->size[i] / 2.0) {
                  collision = false;
                }
              }
              if (collision == true) {
                return 0;
              }
            }
          }
        }
      }

      if (check_collision(state_curr) == 0) {
        return 0;
      }

      state_prev = state_curr;
    }

    return 1;
  }

  /**
   * \brief Sets the number of discretization steps.
   *
   * This function can be used to set the number of intermediate states
   * in the discretization process. In this case, the trajectory between
   * two consecutive states is approximated by a straight line. And this
   * line is discretized in such a way that the line includes
   * number of states exactly equal to that provided to this function.
   *
   * @param num_discretization_steps_in Number of discretization steps.
   *
   * @returns Returns 1 for success, a non-positive value to indicate error.
   */
  int set_discretization_steps(int num_discretization_steps_in) {
    if (num_discretization_steps <= 0) {
      num_discretization_steps = 0;
      discretization_length = 0;
      discretization_method = 0;
    } else {
      num_discretization_steps = num_discretization_steps_in;
      discretization_method = 1;
    }

    return 1;
  }

  /**
   * \brief Sets the length for the discretization.
   *
   * This function can be used to set the length of the discretization.
   * In this case, the trajectory between two states is approximated by a line
   * connecting them, and discretized in such a way that the maximum length
   * of any segment is at most the parameter provided to this function.
   *
   * @param discretization_length_in Length of the discretization.
   *
   * @returns Returns 1 for success, a non-positive value to indicate error.
   */
  int set_discretization_length(double discretization_length_in) {

    if (discretization_length <= 0.0) {
      num_discretization_steps = 0;
      discretization_length = 0.05;
      discretization_method = 0;
    } else {
      discretization_length = discretization_length_in;
      discretization_method = 2;
    }

    return 1;
  }

  /**
   * \brief Adds a new obstacle to the list of obstacles.
   *
   * This function adds a new obstacle to the list of obstacle, which
   * must be a type of region<NUM_DIMENSIONS>. Note that the
   * NUM_DIMENSIONS template argument of the region and this class
   * must match. Otherwise, compuilation errors will occur.
   *
   * @param obstacle_in The pointer to the new obstacle
   *
   * @returns Returns 1 for success, a non-positive value to indicate error.
   */
  int add_obstacle(region_t &obstacle_in) {

    list_obstacles.push_back(new region_t(obstacle_in));

    return 1;
  }
};
} // namespace collision_checkers
} // namespace smp
