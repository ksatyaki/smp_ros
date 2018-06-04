//! Vertex data for minimum-time reachability.
/*! This data structure is attached to each vertex in the graph maintained by
  the planner algorithm. The data structure includes two variables. One variable
  indicates whether the associated vertex lies inside the goal region. Another
  variables keeps track of the cost to reach this particular vertex starting
  from the root vertex. The latter variable is particularly created to work with
  teh RRT* algorithm.
*/
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
class RRTStarVertexData {

public:
  //! Total cost to get to this particular vertex.
  double total_cost;
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
class RRTStarEdgeData {

public:
  //! The cost to traverse this particular trajectory.
  double edge_cost;
};

class MTRVertexData : public RRTStarVertexData {

public:
  //! Reachability of the goal region.
  /*!
    This variable that indicates whether the associated vertex
    state is inside the goal region.
  */
  bool reaches_goal;
};

//! Edge data for minimum-time reachability.
/*!
  This empty class is implemented for the sake of completeness.
*/
class MTREdgeData : public RRTStarEdgeData {};

} // namespace smp
