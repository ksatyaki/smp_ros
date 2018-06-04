/*! \file components/distance_evaluators/kdtree.h
  \brief The abstract sampler

  The sampler provides random or quasi-random sample states.

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

#include <iostream>
#include <smp/distance_evaluators/base.hpp>
#include <smp/external_libraries/kdtree/kdtree.h>

namespace smp {
namespace distance_evaluators {

//! Distance evalutor that employs a kd-tree structure.
/*!
  This class implements a distance evaluator by storing the states in the
  Euclidean space in a kd-tree structure. It implements nearest neighbor
  computation and the computation of near states that reside in a ball of
  given radius. However, it does NOT implement the k-nearest states.

  Note that the class has an initialization function which must be called
  with an appropriate argument, before any other method of the class can
  be called.

  \ingroup distance_evaluators
*/
template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
class KDTree
    : public Base<State, Input, VertexData, EdgeData> {

  using vertex_t = Vertex<State, Input, VertexData, EdgeData>;
  using edge_t = Edge<State, Input, VertexData, EdgeData>;

  typedef struct kdtree kdtree_t;
  typedef struct kdres kdres_t;

  kdtree_t *kdtree;

  std::list<vertex_t *> *list_vertices;
  bool vertex_deleted;

  double weights[NUM_DIMENSIONS];

public:
  KDTree();
  ~KDTree();

  int de_update_insert_vertex(vertex_t *vertex_in);

  int de_update_insert_edge(edge_t *edge_in);

  int de_update_delete_vertex(vertex_t *vertex_in);

  int de_update_delete_edge(edge_t *edge_in);

  int find_nearest_vertex(State *state_in, void **data_out);

  int find_near_vertices_r(State *state_in, double radius_in,
                           std::list<void *> *list_data_out);

  int find_near_vertices_k(State *state_in, int k_in,
                           std::list<void *> *list_data_out);

  /**
   * \brief Sets the list of vertices used to rebuild the kdtree
   *
   * If the user desires to rebuild the kdtree from a list vertices.
   * The appropriate list of vertices can be initiliazsed using this
   * function and the reconstruct_kdtree_from_vertex_list method of this
   * class can be called to rebuild the tree. The distance_kdtree class
   * also reconstructs the tree whenever a vertex is deleted. For the
   * reconstruction to succeed, this method must be called a priori.
   *
   * @param list_vertices_in A pointer to the list of vertices
   *
   * @returns Returns 1 for success, and a non-positive value to indicate error.
   */
  int set_list_vertices(std::list<vertex_t *> *list_vertices_in);

  /**
   * \brief Reconstructs the tree from its vertex list.
   *
   * This method clears all the points in the kdtree and then calls
   * the de_update_insert_vertex method of this class for each vertex in the
   * vertex list initialized using the set_list_vertices method of this class.
   *
   * @returns Returns 1 for success, and a non-positive value to indicate error.
   */
  int reconstruct_kdtree_from_vertex_list();

  /**
   * \brief Sets the weights in the kdtree.
   *
   * The kdtree structure stores the vertices of the kdtree in an Euclidean
   * space,
   * each axis of which is scaled with certain weights. This function can be
   * used
   * to set those weights. By default, all weights are set to one.
   *
   * @param weights_in Weight for each dimension.
   *
   * @returns Returns 1 for success, and a non-positive value to indicate error.
   */
  int set_weights(double weights_in[NUM_DIMENSIONS]);
};
} // namespace distance_evaluators
}

#include <smp/distance_evaluators/kdtree_impl.hpp>

