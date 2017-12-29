/*! \file components/extenders/dubins.h
  \brief The extend function component that implements a dubins car.

  The extender that this file implements an extender based on the
  dubins car dynamical system (the dimensionality of the state space
  is a template parameter) that exactly or approximately connects two given
  states.
*/

#ifndef _SMP_SYSTEM_DUBINS_H_
#define _SMP_SYSTEM_DUBINS_H_

#include <smp/components/extenders/base.h>
#include <smp/components/extenders/input_array_double.h>
#include <smp/components/extenders/state_array_double.h>

#include <list>

namespace smp {

//! Implementation of the state data structure for the Dubins car dynamics
/*!
  This class implements the state data structure for the Dubins car dynamics.
  The number of state variables is three. The state variables indicate position
  in the x and y coordinates and the orientation, in this order.

  \ingroup states
*/
class state_dubins : public state_array_double<3> {};

//! Implementation of the input data structure for the Dubins car dynamics.
/*!
  This class implements the input data structure for teh Dubins car dynamics.
  The number of input variables is exactly two. The first input variable
  stores the time it takes to execute the trajectory segment, while the second
  variable stores the steering input required.

  \ingroup inputs
*/
class input_dubins : public input_array_double<2> {};

//! Implements the extender function with Dubins car dynamics.
/*!
  This class implements an extender with the Dubins car dynamics.

  \ingroup extenders
*/
template <class typeparams>
class extender_dubins : public extender_base<typeparams> {

  typedef typename typeparams::state state_t;
  typedef typename typeparams::input input_t;
  typedef typename typeparams::vertex_data vertex_data_t;
  typedef typename typeparams::edge_data edge_data_t;

  typedef vertex<typeparams> vertex_t;
  typedef edge<typeparams> edge_t;

  typedef trajectory<typeparams> trajectory_t;

  double turning_radius{1.0};

  int extend_dubins_spheres(double x_s1, double y_s1, double t_s1, double x_s2,
                            double y_s2, double t_s2, int comb_no,
                            int *fully_extends,
                            std::list<state_t *> *list_states,
                            std::list<input_t *> *list_inputs);

  double extend_dubins_all(state_t *state_ini, state_t *state_fin,
                           int *fully_extends,
                           std::list<state_t *> *list_states_out,
                           std::list<input_t *> *list_inputs_out);

public:
  extender_dubins();
  ~extender_dubins();

  int ex_update_insert_vertex(vertex_t *vertex_in);

  int ex_update_insert_edge(edge_t *edge_in);

  int ex_update_delete_vertex(vertex_t *vertex_in);

  int ex_update_delete_edge(edge_t *edge_in);

  inline void set_turning_radius(double radius) { turning_radius = radius; }

  int extend(state_t *state_from_in, state_t *state_towards_in,
             int *exact_connection_out, trajectory_t *trajectory_out,
             std::list<state_t *> *intermediate_vertices_out);
};
}

#endif
