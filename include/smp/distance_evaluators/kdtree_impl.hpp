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

#ifndef _SMP_DISTANCE_EVALUATOR_KDTREE_HPP_
#define _SMP_DISTANCE_EVALUATOR_KDTREE_HPP_

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
smp::distance_evaluators::KDTree<State, Input, VertexData, EdgeData,
                                 NUM_DIMENSIONS>::KDTree() {

  kdtree = kd_create(NUM_DIMENSIONS);

  list_vertices = NULL;
  vertex_deleted = false;

  for (int i = 0; i < NUM_DIMENSIONS; i++)
    weights[i] = 1.0;
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
smp::distance_evaluators::KDTree<State, Input, VertexData, EdgeData,
                                 NUM_DIMENSIONS>::~KDTree() {

  kd_free(kdtree);
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
int smp::distance_evaluators::KDTree<
    State, Input, VertexData, EdgeData,
    NUM_DIMENSIONS>::de_update_insert_vertex(vertex_t *vertex_in) {

  State *state_in = vertex_in->state;

  // Create the state key
  double state_key[NUM_DIMENSIONS];
  for (int i = 0; i < NUM_DIMENSIONS; i++)
    state_key[i] = (*state_in)[i] * weights[i];

  // Insert the state into the kd-tree with the vertex pointer as the data
  kd_insert(kdtree, (double *)&state_key, (void *)vertex_in);

  return 1;
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
int smp::distance_evaluators::KDTree<
    State, Input, VertexData, EdgeData,
    NUM_DIMENSIONS>::de_update_insert_edge(edge_t *edge_in) {

  return 1;
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
int smp::distance_evaluators::KDTree<
    State, Input, VertexData, EdgeData,
    NUM_DIMENSIONS>::de_update_delete_vertex(vertex_t *vertex_in) {

  vertex_deleted = true;

  return 1;
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
int smp::distance_evaluators::KDTree<
    State, Input, VertexData, EdgeData,
    NUM_DIMENSIONS>::de_update_delete_edge(edge_t *edge_in) {

  return 1;
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
int smp::distance_evaluators::KDTree<
    State, Input, VertexData, EdgeData,
    NUM_DIMENSIONS>::find_nearest_vertex(State *state_in, void **data_out) {

  if (vertex_deleted) {
    if (list_vertices) {
      reconstruct_kdtree_from_vertex_list();
      vertex_deleted = false;
    } else
      return 0;
  }

  // Create the state key
  double *state_key = new double[NUM_DIMENSIONS];
  for (int i = 0; i < NUM_DIMENSIONS; i++)
    state_key[i] = (*state_in)[i];

  // Query the nearest state
  kdres_t *kdres = kd_nearest(kdtree, state_key);
  if (kd_res_end(kdres)) {
    std::cout << "ERROR: No nearest vertex" << std::endl;
    kd_res_free(kdres);
    delete[] state_key;
    return -2;
  }

  // Set the return variables
  *data_out = kd_res_item_data(kdres);

  // Free temporary memory
  kd_res_free(kdres);
  delete[] state_key;

  return 1;
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
int smp::distance_evaluators::KDTree<
    State, Input, VertexData, EdgeData,
    NUM_DIMENSIONS>::find_near_vertices_r(State *state_in, double radius_in,
                                          std::list<void *> *list_data_out) {

  if (vertex_deleted) {
    if (list_vertices) {
      reconstruct_kdtree_from_vertex_list();
      vertex_deleted = false;
    } else
      return 0;
  }

  // Create the state key
  double *state_key = new double[NUM_DIMENSIONS];
  for (int i = 0; i < NUM_DIMENSIONS; i++)
    state_key[i] = (*state_in)[i];

  // Query the near states
  kdres_t *kdres = kd_nearest_range(kdtree, state_key, radius_in);

  // Set the return variables
  kd_res_rewind(kdres);
  while (!kd_res_end(kdres)) {
    list_data_out->push_back(kd_res_item_data(kdres));
    kd_res_next(kdres);
  }

  // Free temporary memory
  kd_res_free(kdres);
  delete[] state_key;

  return 1;
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
int smp::distance_evaluators::KDTree<
    State, Input, VertexData, EdgeData,
    NUM_DIMENSIONS>::find_near_vertices_k(State *state_in, int k_in,
                                          std::list<void *> *list_data_out) {

  return 0;
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
int smp::distance_evaluators::KDTree<State, Input, VertexData, EdgeData,
                                     NUM_DIMENSIONS>::
    set_list_vertices(std::list<vertex_t *> *list_vertices_in) {

  list_vertices = list_vertices_in;

  return 1;
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
int smp::distance_evaluators::KDTree<
    State, Input, VertexData, EdgeData,
    NUM_DIMENSIONS>::reconstruct_kdtree_from_vertex_list() {

  // std::cout << "Reconstructing the kdtree" << std::endl;

  kd_free(kdtree);
  kdtree = kd_create(NUM_DIMENSIONS);

  if (list_vertices) {
    for (typename std::list<vertex_t *>::iterator it_vertex =
             list_vertices->begin();
         it_vertex != list_vertices->end(); it_vertex++) {

      vertex_t *vertex_curr = *it_vertex;

      // std::cout << "Vertex : " << vertex_curr->state->state_vars[0] << ", "
      // << vertex_curr->state->state_vars[1] << std::endl;

      this->de_update_insert_vertex(vertex_curr);
    }
  } else {
    std::cout << "ERROR:distance_evaluators:kdtree: No list of vertices to "
                 "reconstruct the tree"
              << std::endl;
    return 0;
  }

  // std::cout << "Reconstruction of the kdtree is complete" << std::endl;

  return 1;
}

template <class State, class Input, class VertexData, class EdgeData,
          int NUM_DIMENSIONS>
int smp::distance_evaluators::KDTree<
    State, Input, VertexData, EdgeData,
    NUM_DIMENSIONS>::set_weights(double weights_in[NUM_DIMENSIONS]) {

  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    if (weights_in[i] >= 0.0) {
      weights[i] = weights_in[i];
    } else
      weights[i] = 0.0;
  }

  return 1;
}

#endif
