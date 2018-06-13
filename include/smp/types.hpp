//! Vertex data for minimum-time reachability.
/*! This data structure is attached to each vertex in the graph maintained by
  the planner algorithm. The data structure includes two variables. One variable
  indicates whether the associated vertex lies inside the goal region. Another
  variables keeps track of the cost to reach this particular vertex starting
  from the root vertex. The latter variable is particularly created to work with
  teh RRT* algorithm.
*/

#pragma once

namespace smp {

//! Vertex data structure for the RRT* algorithm.
/*!
  The RRT* algorithm requires the vertex data to include a variable
  that stores the cost to get the vertex from the root node.This class
  implements such a data structure .The user can directly use this
  data structure for implementation either as is, or a derived class
  that inherits from this class. Alternatively, the user can generate
  another edge data class that includes the edge_cost variable.
*/
class VertexData {

public:
  //! Total cost to get to this particular vertex.
  double total_cost;
  bool reaches_goal;
};

//! Edge data for the RRT* algorithm.
/*!
  The RRT* algorithm requires the edge data to include a variable
  that stores the cost to traverse that particular edge. This class
  implements such a data structure. The user can directly use this
  data structure for implementation either as is, or a derived class
  that inherits from this class. Alternatively, the user can generate
another edge data class that includes the edge_cost variable.
*/
class EdgeData {

public:
  //! The cost to traverse this particular trajectory.
  double edge_cost;
};

} // namespace smp
