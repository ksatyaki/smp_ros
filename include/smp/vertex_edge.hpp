/*! \file vertex_edge.h
  \brief An implementation of the vertex and edge components in the graph.

  Provides an implementation of the vertex and edge components in the graph.
  Both classes are defined as templates that take the types of the state, input,
  and the data stored in the vertices as well as the type of the data that is
  stored in the edges as an argument.

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

#ifndef _SMP_VERTEX_EDGE_H_
#define _SMP_VERTEX_EDGE_H_

#include <smp/trajectory.hpp>

#include <list>

//! This parameter can be set to one for fast vertex deletion.
/*! The planner maintains a list of all the vertices present in the graph. If
  this parameter is set to one, then each vertex includes a pointer to its
  location in the list, which makes vertex deletion faster. However, it
  introduces the overhead of maintaining this variable, which some users may not
  like. This variable should be set to one when there is intense vertex
  deletion, e.g., when using a branch and bound heuristic.
 */
#define _SMP_FAST_VERTEX_DELETE 1

namespace smp {

template <class State, class Input, class VertexData, class EdgeData>
class Vertex;
template <class State, class Input, class VertexData, class EdgeData>
class Edge;

//! Vertex data structure of the graph maintained by a planner algorithm
/*!
  This class provides a generic vertex structure that takes the types of the
  state, input, and the data stored in the vertices as a template argument (for
  technical reasons it takes the edge data as an argument as well). The main
  components
  of the vertex class are the state and the data that the vertex stores. Also
  for effective search, lists of incoming and outgoing edges are also stored.

  \ingroup graphs
*/
template <class State, class Input, class VertexData, class EdgeData>
class Vertex {

  using edge_t = Edge<State, Input, VertexData, EdgeData>;
  using vertex_t = Vertex<State, Input, VertexData, EdgeData>;

public:
  //! The data that is stored in this vertex
  /*!
    The data that is stored in every vertex of the graph. The type for this
    data is given as a template argument, and in principle it can be any type/
  */
  VertexData data;

  //! A pointer to the state stored in this vertex
  /*!
    The state that is associated with this vertex. The type for this state
    is as a template argument, and it can be of any type.
  */
  State *state;

  //! A list of incoming edges
  /*!
    The list of all edges that point to this vertex.
  */
  std::list<edge_t *> incoming_edges;

  //! A list of outgoing edges
  /*!
    This list of all edges that point out from this vertex.
  */
  std::list<edge_t *> outgoing_edges;

#if _SMP_FAST_VERTEX_DELETE

  //! A pointer to the location of this vertex in the list of vertices
  //! maintained by the planner
  /*!
    The planner maintains a list of vertices present in the graph. This variable
    is a pointer to the location of this vertex in that list. The variable is
    used to quickly remove the vertex from list, without having to traverse the
    whole list.
   */
  typename std::list<vertex_t *>::iterator it_vertex_list;
#endif

  Vertex() {
    incoming_edges.clear();
    outgoing_edges.clear();
  }

  ~Vertex() {
    if (state)
      delete state;
    incoming_edges.clear();
    outgoing_edges.clear();
  }
};

//! Edge data structure of the graph maintained by a planner algorithm
/*!
  \ingroup graphs
*/
template <class State, class Input, class VertexData, class EdgeData>
class Edge {

  using trajectory_t = Trajectory<State, Input>;
  using vertex_t = Vertex<State, Input, VertexData, EdgeData>;

public:
  //! The data that is stored in this vertex.
  /*! The data that is stored in every vertex of the graph. The type for the
    data is given as a template argument
  */
  EdgeData data;

  //! A pointer to the state stored in this vertex.
  /*! The trajectory along this edge. The types for the state and the input
    for this trajectory are taken as template arguments.
  */
  trajectory_t *trajectory_edge;

  //! A pointer to the source vertex.
  /*! The source vertex that this edge starts from.
   */
  vertex_t *vertex_src;

  //! A pointer to the destination vertex.
  /*!
    The destination vertex that this edge ends at.
  */
  vertex_t *vertex_dst;

  Edge() {
    vertex_src = 0;
    vertex_dst = 0;
    trajectory_edge = 0;
  }

  ~Edge() { delete trajectory_edge; }
};
} // namespace smp

#endif
