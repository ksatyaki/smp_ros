/*
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

#ifndef _SMP_VERTEX_EDGE_HPP_
#define _SMP_VERTEX_EDGE_HPP_

#include <smp/planner_utils/vertex_edge.h>

#include <smp/planner_utils/trajectory.hpp>

template <class typeparams> smp::edge<typeparams>::edge() {

  vertex_src = 0;
  vertex_dst = 0;
  trajectory_edge = 0;
}

template <class typeparams> smp::edge<typeparams>::~edge() {

  delete trajectory_edge;
}

template <class typeparams> smp::vertex<typeparams>::vertex() {

  incoming_edges.clear();
  outgoing_edges.clear();
}

template <class typeparams> smp::vertex<typeparams>::~vertex() {

  if (state)
    delete state;

  incoming_edges.clear();
  outgoing_edges.clear();
}

#endif
