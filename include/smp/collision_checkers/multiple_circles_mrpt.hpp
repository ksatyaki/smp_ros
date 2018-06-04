/*! \file components/collision_checkers/multiple_circles_mrpt.h
  \brief A collisiion checker using multiple circles for mrpt 2D maps

  This file implements a collision checker for MRPT Occupancy Maps. Code
  borrowed from Federico Pecora.

  * Copyright (C) 2018 Chittaranjan Srinivas Swaminathan
  * Copyright (C) 2018 Federico Pecora
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

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/math/CPolygon.h>
#include <smp/collision_checkers/base.hpp>

#include <cmath>
#include <list>
#include <memory>

namespace smp {
namespace collision_checkers {

namespace mm = mrpt::maps;

//! MRPT Occupancy map collision checker using a robot footprint.
/*!
  Checks for collisions between a robot footprint and the MRPT occupancy map.
  \ingroup collision_checkers
*/
template <class State, class Input>
class MultipleCirclesMRPT : public Base<State, Input> {

  using trajectory_t = Trajectory<State, Input>;

  std::shared_ptr<mm::COccupancyGridMap2D> map;
  double inflation_radius;
  std::shared_ptr<mrpt::math::CPolygon> robot_footprint;

public:
  inline MultipleCirclesMRPT() : inflation_radius(1.0) {}
  inline MultipleCirclesMRPT(
      const std::shared_ptr<mm::COccupancyGridMap2D> &_map, double radius,
      const std::shared_ptr<mrpt::math::CPolygon> &footprint)
      : map(_map), inflation_radius(radius), robot_footprint(footprint) {}

  inline ~MultipleCirclesMRPT() {}

  int check_collision_state(State *state_in) {

    if (!map) {
      std::cerr << "[check_collision_state]: NO MAP!\n";
      return 1;
    }

    if (!robot_footprint) {
      std::cerr << "[check_collision_state]: NO FOOTPRINT!\n";
      return 1;
    }

    std::vector<double> xCoords, yCoords;
    robot_footprint->getAllVertices(xCoords, yCoords);

    double theta = state_in->state_vars[2];

    for (size_t i = 0; i < xCoords.size(); i++) {

      double x = xCoords[i] * cos(theta) - yCoords[i] * sin(theta);
      double y = xCoords[i] * sin(theta) + yCoords[i] * cos(theta);

      x = state_in->state_vars[0];
      y = state_in->state_vars[1];

      double value = map->computeClearance(x, y, inflation_radius);
      // std::cout << "State: (" << x << "," << y << "," << theta << ")\n";

      if (value < inflation_radius) {
        return 0;
      }
    }
    return 1;
  }

  int check_collision_trajectory(trajectory_t *trajectory_in) {

    if (!map) {
      std::cerr << "[check_collision_trajectory]: NO MAP!\n";
      return 1;
    }

    if (!robot_footprint) {
      std::cerr << "[check_collision_trajectory]: NO FOOTPRINT!\n";
      return 1;
    }

    if (trajectory_in->list_states.size() == 0)
      return 1;

    // This might be a problem with very thin obstacles. We ignore that for now.
    for (const auto &iter : trajectory_in->list_states) {
      if (check_collision_state(iter) == 0) {
        // std::cout << "()()()()\n";
        return 0;
      }
    }
    return 1;
  }

  inline void
  set_robot_footprint(const std::shared_ptr<mrpt::math::CPolygon> &footprint) {
    robot_footprint = footprint;
  }
  inline void set_map(const std::shared_ptr<mm::COccupancyGridMap2D> &_map) {
    map = _map;
  }
  inline void set_inflation_radius(double radius) { inflation_radius = radius; }
};
} // namespace collision_checkers
} // namespace smp
