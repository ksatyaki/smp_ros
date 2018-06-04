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

#ifndef _SMP_PLANNER_INCREMENTAL_HPP_
#define _SMP_PLANNER_INCREMENTAL_HPP_

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
smp::planners::BaseIncremental<State, Input, VertexData, EdgeData,
                               NUM_DIMENSIONS>::BaseIncremental() {

  root_vertex = 0;
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
smp::planners::BaseIncremental<State, Input, VertexData, EdgeData,
                               NUM_DIMENSIONS>::~BaseIncremental() {

  // Note that the root vertex is deleted by the smp_planner class
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
smp::planners::BaseIncremental<State, Input, VertexData, EdgeData,
                               NUM_DIMENSIONS>::
    BaseIncremental(sampler_t &sampler_in,
                    distance_evaluator_t &distance_evaluator_in,
                    extender_t &extender_in,
                    collision_checker_t &collision_checker_in,
                    model_checker_t &model_checker_in)
    : Base<State, Input, VertexData, EdgeData, NUM_DIMENSIONS>(
          sampler_in, distance_evaluator_in, extender_in, collision_checker_in,
          model_checker_in) {}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
int smp::planners::BaseIncremental<
    State, Input, VertexData, EdgeData,
    NUM_DIMENSIONS>::initialize(state_t *initial_state_in) {

  // This function deletes all existing vertices
  // in the graph, including the root vertex.
  this->Base<State, Input, VertexData, EdgeData, NUM_DIMENSIONS>::initialize();

  if (initial_state_in == 0) {
    root_vertex = 0;
    return 1;
  }

  root_vertex = new vertex_t;
  root_vertex->state = initial_state_in;

  this->insert_vertex(root_vertex);

  return 1;
}

#endif
