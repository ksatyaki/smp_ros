#pragma once

#include <smp/components/collision_checkers/multiple_circles_mrpt.h>

#include <smp/components/collision_checkers/base.hpp>

template <class typeparams>
int smp::collision_checker_mc_mrpt<typeparams>::cc_update_insert_vertex(
    vertex_t *vertex_in) {

  return 1;
}

template <class typeparams>
int smp::collision_checker_mc_mrpt<typeparams>::cc_update_insert_edge(
    edge_t *edge_in) {

  return 1;
}

template <class typeparams>
int smp::collision_checker_mc_mrpt<typeparams>::cc_update_delete_vertex(
    vertex_t *vertex_in) {

  return 1;
}

template <class typeparams>
int smp::collision_checker_mc_mrpt<typeparams>::cc_update_delete_edge(
    edge_t *edge_in) {

  return 1;
}

// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision
template <class typeparams>
int smp::collision_checker_mc_mrpt<typeparams>::check_collision_state(
    state_t *state_in) {

  if (!map) {
    std::cerr << "[check_collision_state]: NO MAP!\n";
    return 1;
  }

  if (!robot_footprint) {
    std::cerr << "[check_collision_state]: NO FOOTPRINT!\n";
    return 1;
  }

  std::vector<double> xCoords, yCoords;
  robot_footprint->getAllVertices(xCoords, yCoords);

  double theta = state_in->state_vars[2];

  for (size_t i = 0; i < xCoords.size(); i++) {

    double x = xCoords[i] * cos(theta) - yCoords[i] * sin(theta);
    double y = xCoords[i] * sin(theta) + yCoords[i] * cos(theta);

    x = state_in->state_vars[0];
    y = state_in->state_vars[1];

    double value = map->computeClearance(x, y, inflation_radius);
    // std::cout << "State: (" << x << "," << y << "," << theta << ")\n";

    if (value < inflation_radius) {
      return 0;
    }
  }
  return 1;
}

// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision
template <class typeparams>
int smp::collision_checker_mc_mrpt<typeparams>::check_collision_trajectory(
    trajectory_t *trajectory_in) {

  if (!map) {
    std::cerr << "[check_collision_trajectory]: NO MAP!\n";
    return 1;
  }

  if (!robot_footprint) {
    std::cerr << "[check_collision_trajectory]: NO FOOTPRINT!\n";
    return 1;
  }

  if (trajectory_in->list_states.size() == 0)
    return 1;

  // This might be a problem with very thin obstacles. We ignore that for now.
  for (const auto &iter : trajectory_in->list_states) {
    if (check_collision_state(iter) == 0) {
      // std::cout << "()()()()\n";
      return 0;
    }
  }
  return 1;
}
