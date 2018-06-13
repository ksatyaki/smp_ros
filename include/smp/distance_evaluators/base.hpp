/*! \file components/distance_evaluators/base.h
  \brief The abstract distance evaluator

  The distance evaluator provides services related to the relative distance
  computation
  among the states.

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

#include <smp/vertex_edge.hpp>
#include <list>

namespace smp {
namespace distance_evaluators {

//! The abstract class that specifies the structure of a distance evaluator
//! component.
/*!
  A distance evaluator component provides functions for computing the nearest
  and
  near vertices. There two ways to compute the nearest vertices, the radius and
  the
  k-nearest methods, both of which should be implemented by a derived class.

  \ingroup distance_evaluators_base
*/
template <class State, class Input>
class Base {

  using vertex_t = Vertex<State, Input>;
  using edge_t = Edge<State, Input>;

public:
  virtual ~Base(){};

  /**
   * \brief Update function for vertex insertion
   *
   * This function is called by the planner whenever a new vertex is
   * added to the graph. A pointer to the new vertex is given as an argument.
   *
   * @param vertex_in A pointer to the new vertex.
   *
   * @returns Return 1 if success, a non-positive value to indiacate error.
   */
  virtual int de_update_insert_vertex(vertex_t *vertex_in) = 0;

  /**
   * \brief Update function for edge insertion
   *
   * This function is called by the planner whenever a new edge is
   * added to the graph. A pointer to the new edge is given as an argument.
   *
   * @param edge_in A pointer to the new edge.
   *
   * @returns Return 1 for success, a non-positive value to indiacate error.
   */
  virtual int de_update_insert_edge(edge_t *edge_in) = 0;

  /**
   * \brief Update function for vertex deletion
   *
   * This function is called by the planner whenever a vertex is deleted
   * from the graph. A pointer to the vertex is given as an argument.
   *
   * @param vertex_in A pointer to deleted vertex.
   *
   * @returns Return 1 if success, a non-positive value to indiacate error.
   */
  virtual int de_update_delete_vertex(vertex_t *vertex_in) = 0;

  /**
   * \brief Update function for edge insertion
   *
   * This function is called by the planner whenever an edge is delete
   * from the graph. A pointer to the edge is given as an argument.
   *
   * @param edge_in A pointer to deleted edge.
   *
   * @returns Return 1 for success, a non-positive value to indiacate error.
   */
  virtual int de_update_delete_edge(edge_t *edge_in) = 0;

  /**
   * \brief Abstract function that provides the nearest vertex.
   *
   * Returns the vertex with state that is closest to the query state given
   * by the state_in argument. The data associated with the nearest vertex
   * is output with the data_out argument.
   *
   * @param state_in The query state.
   * @param data_out Data that is associated with the nearest vertex (usually
   *                  this data is basically a pointer to the nearest vertex
   * itself.
   *
   * @returns Returns 1 for success, a non-positive number for failure.
   */
  virtual int find_nearest_vertex(State *state_in, void **data_out) = 0;

  /**
   * \brief Abstract function that provides the set of near vertices within
   *        a certain ball.
   *
   * Returns the set of all vertices that lie within the Euclidean ball of
   * radius
   * given by the radius_in argument and centered at the state given by the
   * state_in
   * argument.
   *
   * @param state_in The query state.
   * @param radius_in The radius of the ball.
   * @param list_data_out Data that is associated with each vertex in the near
   * set
   *                       organized into a list.
   *
   * @returns Returns 1 for success, a non-positive number for failure.
   */
  virtual int find_near_vertices_r(State *state_in, double radius_in,
                                   std::list<void *> *list_data_out) = 0;

  /**
   * \brief Abstract function that provides the set of near vertices that are
   * the
   *        k nearest to the query state.
   *
   * Returns the set of k-nearest vertices to the query state.
   *
   * @param state_in The query state.
   * @param k_in The number k.
   * @param list_data_out Data that is associated with each vertex in the near
   * set
   *                       organized into a list.
   *
   * @returns Returns 1 for success, a non-positive number for failure.
   */
  virtual int find_near_vertices_k(State *state_in, int k_in,
                                   std::list<void *> *list_data_out) = 0;
};
} // namespace distance_evaluators
} // namespace smp

