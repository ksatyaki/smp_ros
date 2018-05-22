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
#include <smp/components/collision_checkers/base.h>

#include <cmath>
#include <list>
#include <memory>

namespace mm = mrpt::maps;

namespace smp {

//! MRPT Occupancy map collision checker using a robot footprint.
/*!
  Checks for collisions between a robot footprint and the MRPT occupancy map.
  \ingroup collision_checkers
*/
template <class typeparams>
class collision_checker_mc_mrpt : public collision_checker_base<typeparams> {

  typedef typename typeparams::state state_t;
  typedef typename typeparams::input input_t;
  typedef typename typeparams::vertex_data vertex_data_t;
  typedef typename typeparams::edge_data edge_data_t;

  typedef vertex<typeparams> vertex_t;
  typedef edge<typeparams> edge_t;
  typedef trajectory<typeparams> trajectory_t;

  std::shared_ptr<mm::COccupancyGridMap2D> map;
  double inflation_radius;
  std::shared_ptr<mrpt::math::CPolygon> robot_footprint;

public:
  inline collision_checker_mc_mrpt() : inflation_radius(1.0) {}
  inline collision_checker_mc_mrpt(
      const std::shared_ptr<mm::COccupancyGridMap2D> &_map, double radius,
      const std::shared_ptr<mrpt::math::CPolygon> &footprint)
      : map(_map), inflation_radius(radius), robot_footprint(footprint) {}

  inline ~collision_checker_mc_mrpt() {}

  int cc_update_insert_vertex(vertex_t *vertex_in);

  int cc_update_insert_edge(edge_t *edge_in);

  int cc_update_delete_vertex(vertex_t *vertex_in);

  int cc_update_delete_edge(edge_t *edge_in);

  int check_collision_state(state_t *state_in);

  int check_collision_trajectory(trajectory_t *trajectory_in);

  inline void
  set_robot_footprint(const std::shared_ptr<mrpt::math::CPolygon> &footprint) {
    robot_footprint = footprint;
  }
  inline void set_map(const std::shared_ptr<mm::COccupancyGridMap2D> &_map) {
    map = _map;
  }
  inline void set_inflation_radius(double radius) { inflation_radius = radius; }
};
}
