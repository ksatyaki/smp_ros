/*
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

#ifndef _SMP_MINIMUM_TIME_REACHABILITY_HPP_
#define _SMP_MINIMUM_TIME_REACHABILITY_HPP_

template <int NUM_DIMENSIONS>
std::array<double, NUM_DIMENSIONS>
smp::default_distance_function(const std::array<double, NUM_DIMENSIONS> &state,
                               const std::array<double, NUM_DIMENSIONS> &goal) {
  std::array<double, NUM_DIMENSIONS> result;
  for (int i = 0; i < state.size(); i++) {
    result[i] = state[i] - goal[i];
  }
  return result;
}

template <class State, class Input, int NUM_DIMENSIONS>
smp::multipurpose::MinimumTimeReachability<
    State, Input, NUM_DIMENSIONS>::MinimumTimeReachability() {

  min_cost_vertex = NULL;

  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    region_goal.center[i] = 0.0;
    region_goal.size[i] = 0.0;
  }
}

template <class State, class Input, int NUM_DIMENSIONS>
smp::multipurpose::MinimumTimeReachability<
    State, Input, NUM_DIMENSIONS>::~MinimumTimeReachability() {}

template <class State, class Input, int NUM_DIMENSIONS>
smp::multipurpose::MinimumTimeReachability<State, Input, NUM_DIMENSIONS>::
    MinimumTimeReachability(const region_t &region_in) {

  region_goal = region_in;
}

template <class State, class Input, int NUM_DIMENSIONS>
int smp::multipurpose::MinimumTimeReachability<
    State, Input, NUM_DIMENSIONS>::set_goal_region(const region_t &region_in) {

  region_goal = region_in;

  return 1;
}

template <class State, class Input, int NUM_DIMENSIONS>
int smp::multipurpose::MinimumTimeReachability<
    State, Input, NUM_DIMENSIONS>::ce_update_vertex_cost(vertex_t *vertex_in) {

  if (vertex_in->data.reaches_goal == true) {

    bool update_trajectory = false;

    if (min_cost_vertex == NULL) {
      min_cost_vertex = vertex_in;
      update_trajectory = true;
    }

    else if ((vertex_in->data.total_cost <= min_cost_vertex->data.total_cost)) {
      min_cost_vertex = vertex_in;
      update_trajectory = true;
    }

    if (update_trajectory == true) {

      std::cout << "UPDATING TRAJECTORY. NEW LOWEST COST -- : "
                << vertex_in->data.total_cost << std::endl;
      fflush(stdout);

      min_cost_trajectory.clear_delete();

      vertex_t *vertex_ptr = min_cost_vertex;
      while (1) {

        if (vertex_ptr->incoming_edges.size() == 0)
          break;

        edge_t *edge_curr = vertex_ptr->incoming_edges.back();

        trajectory_t *trajectory_curr = edge_curr->trajectory_edge;
        min_cost_trajectory.list_states.push_front(
            new State(*(vertex_ptr->state)));

        for (typename std::list<State *>::reverse_iterator it_state =
                 trajectory_curr->list_states.rbegin();
             it_state != trajectory_curr->list_states.rend(); it_state++) {
          min_cost_trajectory.list_states.push_front(new State(**it_state));
        }

        for (typename std::list<Input *>::reverse_iterator it_input =
                 trajectory_curr->list_inputs.rbegin();
             it_input != trajectory_curr->list_inputs.rend(); it_input++) {
          min_cost_trajectory.list_inputs.push_front(new Input(**it_input));
        }

        vertex_ptr = edge_curr->vertex_src;
      }

      // std::cout << "Min Cost Traj contains: " <<
      // min_cost_trajectory.list_states.size() << " states";
      // Call all the update functions
      for (typename std::list<update_func_t>::iterator it_func =
               list_update_functions.begin();
           it_func != list_update_functions.end(); it_func++) {

        (*it_func)(&min_cost_trajectory);
      }
    }
  }

  return 1;
}

template <class State, class Input, int NUM_DIMENSIONS>
int smp::multipurpose::MinimumTimeReachability<
    State, Input, NUM_DIMENSIONS>::ce_update_edge_cost(edge_t *edge_in) {

  return 1;
}

template <class State, class Input, int NUM_DIMENSIONS>
int smp::multipurpose::MinimumTimeReachability<State, Input, NUM_DIMENSIONS>::
    mc_update_insert_vertex(vertex_t *vertex_in) {

  vertex_in->data.reaches_goal = reaches_goal(vertex_in);

  return 1;
}

template <class State, class Input, int NUM_DIMENSIONS>
bool smp::multipurpose::MinimumTimeReachability<
    State, Input, NUM_DIMENSIONS>::reaches_goal(vertex_t *vertex_in) {

  std::array<double, NUM_DIMENSIONS> state;
  std::array<double, NUM_DIMENSIONS> goal;

  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    state[i] = vertex_in->state->state_vars[i];
    goal[i] = region_goal.center[i];
  }

  std::array<double, NUM_DIMENSIONS> distance;
  if (distance_function) {
    distance = distance_function(state, goal);
  } else {
    distance = default_distance_function<NUM_DIMENSIONS>(state, goal);
  }

  // THIS SHOULD NEVER HAPPEN
  if (NUM_DIMENSIONS != distance.size()) {
    std::cout << "FATAL ERROR: There is a mismatch between NUM_DIMENSIONS and "
                 "the dimensions of the distance. Check distance function!";
    exit(1);
  }

  // First two dimensions are always the x and y of the position.
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    if (fabs(distance[i]) > region_goal.size[i]) {
      return false;
    }
  }

  return true;
}

template <class State, class Input, int NUM_DIMENSIONS>
int smp::multipurpose::MinimumTimeReachability<
    State, Input, NUM_DIMENSIONS>::mc_update_insert_edge(edge_t *edge_in) {

  return 1;
}

template <class State, class Input, int NUM_DIMENSIONS>
int smp::multipurpose::MinimumTimeReachability<State, Input, NUM_DIMENSIONS>::
    mc_update_delete_vertex(vertex_t *vertex_in) {

  return 1;
}

template <class State, class Input, int NUM_DIMENSIONS>
int smp::multipurpose::MinimumTimeReachability<
    State, Input, NUM_DIMENSIONS>::mc_update_delete_edge(edge_t *edge_in) {

  return 1;
}

template <class State, class Input, int NUM_DIMENSIONS>
int smp::multipurpose::MinimumTimeReachability<
    State, Input, NUM_DIMENSIONS>::get_solution(trajectory_t &trajectory_out) {

  if (!min_cost_vertex)
    return 1;

  trajectory_out.clear();

  for (typename std::list<State *>::iterator it_state =
           min_cost_trajectory.list_states.begin();
       it_state != min_cost_trajectory.list_states.end(); it_state++) {

    trajectory_out.list_states.push_back(new State(**it_state));
  }

  for (typename std::list<Input *>::iterator it_input =
           min_cost_trajectory.list_inputs.begin();
       it_input != min_cost_trajectory.list_inputs.end(); it_input++) {
    trajectory_out.list_inputs.push_back(new Input(**it_input));
  }

  return 1;
}

template <class State, class Input, int NUM_DIMENSIONS>
double
smp::multipurpose::MinimumTimeReachability<State, Input, NUM_DIMENSIONS>::
    evaluate_cost_trajectory(State *state_initial_in,
                             trajectory_t *trajectory_in,
                             State *state_final_in) {

  if (cost_function) {
    return cost_function(state_initial_in, trajectory_in, state_final_in);
  } else {
    return default_cost_function(state_initial_in, trajectory_in,
                                 state_final_in);
  }
}

template <class State, class Input, int NUM_DIMENSIONS>
double
smp::multipurpose::MinimumTimeReachability<State, Input, NUM_DIMENSIONS>::
    default_cost_function(State *state_initial_in, trajectory_t *trajectory_in,
                          State *state_final_in) {

  double total_time = 0.0;
  double total_distance = 0.0;

  for (typename std::list<Input *>::iterator iter =
           trajectory_in->list_inputs.begin();
       iter != trajectory_in->list_inputs.end(); iter++) {
    Input *input_curr = *iter;
    total_time += (*input_curr)[0];
  }
  return total_time;
}

template <class State, class Input, int NUM_DIMENSIONS>
double
smp::multipurpose::MinimumTimeReachability<State, Input,
                                           NUM_DIMENSIONS>::get_best_cost() {

  if (min_cost_vertex == NULL)
    return -1.0;
  else
    return (double)(min_cost_vertex->data.total_cost);
}

template <class State, class Input, int NUM_DIMENSIONS>
int smp::multipurpose::MinimumTimeReachability<
    State, Input, NUM_DIMENSIONS>::clear_update_function_list() {

  list_update_functions.clear();

  return 1;
}

template <class State, class Input, int NUM_DIMENSIONS>
int smp::multipurpose::MinimumTimeReachability<State, Input, NUM_DIMENSIONS>::
    register_new_update_function(update_func_t update_function) {

  if (update_function == NULL)
    return 0;

  list_update_functions.push_back(update_function);

  return 1;
}

#endif
