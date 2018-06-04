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

#define DISTANCE_LIMIT 100.0
#define DELTA_DISTANCE 0.25

#ifndef DBL_MAX
#define DBL_MAX 10000000000000000.0
#endif

#include <smp/extenders/dubins.hpp>

#include <cmath>
#include <cstdlib>

namespace smp {
namespace extenders {

int Dubins::extend_dubins_spheres(
    double x_s1, double y_s1, double t_s1, double x_s2, double y_s2,
    double t_s2, int comb_no, int *fully_extends,
    std::list<StateDubins *> *list_states,
    std::list<InputDubins *> *list_inputs) {

  double x_tr = x_s2 - x_s1;
  double y_tr = y_s2 - y_s1;
  double t_tr = atan2(y_tr, x_tr);

  double distance = sqrt(x_tr * x_tr + y_tr * y_tr);

  double x_start;
  double y_start;
  double t_start;
  double x_end;
  double y_end;
  double t_end;

  if (distance > 2 * turning_radius) { // disks do not intersect

    double t_balls = acos(2 * turning_radius / distance);

    switch (comb_no) {
    case 1:
      t_start = t_tr - t_balls;
      t_end = t_tr + M_PI - t_balls;
      break;
    case 2:
      t_start = t_tr + t_balls;
      t_end = t_tr - M_PI + t_balls;
      break;
    case 3:
      t_start = t_tr - M_PI_2;
      t_end = t_tr - M_PI_2;
      break;
    case 4:
      t_start = t_tr + M_PI_2;
      t_end = t_tr + M_PI_2;
      break;
    default:
      return -1.0;
    }
  }

  else { // disks are intersecting

    switch (comb_no) {
    case 1:
    case 2:
      // No solution
      if (list_states) {
        list_states->clear();
        list_inputs->clear();
      }
      return -1.0;
      break;

    case 3:
      t_start = t_tr - M_PI_2;
      t_end = t_tr - M_PI_2;
      break;
    case 4:
      t_start = t_tr + M_PI_2;
      t_end = t_tr + M_PI_2;
      break;
    }
  }

  x_start = x_s1 + turning_radius * cos(t_start);
  y_start = y_s1 + turning_radius * sin(t_start);
  x_end = x_s2 + turning_radius * cos(t_end);
  y_end = y_s2 + turning_radius * sin(t_end);

  int direction_s1 = 1;
  if ((comb_no == 2) || (comb_no == 4)) {
    direction_s1 = -1;
  }
  int direction_s2 = 1;
  if ((comb_no == 1) || (comb_no == 4)) {
    direction_s2 = -1;
  }

  double t_increment_s1 = direction_s1 * (t_start - t_s1);
  double t_increment_s2 = direction_s2 * (t_s2 - t_end);

  while (t_increment_s1 < 0)
    t_increment_s1 += 2.0 * M_PI;
  while (t_increment_s1 > 2.0 * M_PI)
    t_increment_s1 -= 2.0 * M_PI;

  while (t_increment_s2 < 0)
    t_increment_s2 += 2.0 * M_PI;
  while (t_increment_s2 > 2.0 * M_PI)
    t_increment_s2 -= 2.0 * M_PI;

  if (((t_increment_s1 > M_PI) && (t_increment_s2 > M_PI)) ||
      ((t_increment_s1 > 3 * M_PI_2) || (t_increment_s2 > 3 * M_PI_2))) {
    return -1.0;
  }

  double total_distance_travel =
      (t_increment_s1 + t_increment_s2) * turning_radius + distance;

  double distance_limit = DISTANCE_LIMIT;

  if (fully_extends)
    *fully_extends = 0;

  if (list_states) {
    // Generate states/inputs

    double del_d = DELTA_DISTANCE;
    double del_t = del_d * turning_radius;

    double t_inc_curr = 0.0;

    while (t_inc_curr < t_increment_s1) {
      double t_inc_rel = del_t;
      t_inc_curr += del_t;
      if (t_inc_curr > t_increment_s1) {
        t_inc_rel -= t_inc_curr - t_increment_s1;
        t_inc_curr = t_increment_s1;
      }

      StateDubins *state_curr = new StateDubins;
      InputDubins *input_curr = new InputDubins;

      (*state_curr)[0] =
          x_s1 + turning_radius * cos(direction_s1 * t_inc_curr + t_s1);
      (*state_curr)[1] =
          y_s1 + turning_radius * sin(direction_s1 * t_inc_curr + t_s1);
      (*state_curr)[2] = direction_s1 * t_inc_curr + t_s1 +
                         ((direction_s1 == 1) ? M_PI_2 : 3.0 * M_PI_2);
      while ((*state_curr)[2] < 0)
        (*state_curr)[2] += 2 * M_PI;
      while ((*state_curr)[2] > 2 * M_PI)
        (*state_curr)[2] -= 2 * M_PI;

      (*input_curr)[0] = t_inc_rel * turning_radius;
      (*input_curr)[1] = ((comb_no == 1) || (comb_no == 3)) ? -1 : 1;

      list_states->push_back(state_curr);
      list_inputs->push_back(input_curr);

      if (t_inc_curr * turning_radius > distance_limit) {

        if (fully_extends)
          *fully_extends = 0;

        return total_distance_travel;
      }
    }

    double d_inc_curr = 0.0;
    while (d_inc_curr < distance) {
      double d_inc_rel = del_d;
      d_inc_curr += del_d;
      if (d_inc_curr > distance) {
        d_inc_rel -= d_inc_curr - distance;
        d_inc_curr = distance;
      }

      StateDubins *state_curr = new StateDubins;
      InputDubins *input_curr = new InputDubins;

      (*state_curr)[0] = (x_end - x_start) * d_inc_curr / distance + x_start;
      (*state_curr)[1] = (y_end - y_start) * d_inc_curr / distance + y_start;
      (*state_curr)[2] = direction_s1 * t_inc_curr + t_s1 +
                         ((direction_s1 == 1) ? M_PI_2 : 3.0 * M_PI_2);
      while ((*state_curr)[2] < 0)
        (*state_curr)[2] += 2 * M_PI;
      while ((*state_curr)[2] > 2 * M_PI)
        (*state_curr)[2] -= 2 * M_PI;

      (*input_curr)[0] = d_inc_rel;
      (*input_curr)[1] = 0.0;

      list_states->push_back(state_curr);
      list_inputs->push_back(input_curr);

      if (t_inc_curr * turning_radius + d_inc_curr > distance_limit) {

        if (fully_extends)
          *fully_extends = 0;

        return total_distance_travel;
      }
    }

    double t_inc_curr_prev = t_inc_curr;
    t_inc_curr = 0.0;
    while (t_inc_curr < t_increment_s2) {
      double t_inc_rel = del_t;
      t_inc_curr += del_t;
      if (t_inc_curr > t_increment_s2) {
        t_inc_rel -= t_inc_curr - t_increment_s2;
        t_inc_curr = t_increment_s2;
      }

      StateDubins *state_curr = new StateDubins;
      InputDubins *input_curr = new InputDubins;

      (*state_curr)[0] =
          x_s2 + turning_radius *
                     cos(direction_s2 * (t_inc_curr - t_increment_s2) + t_s2);
      (*state_curr)[1] =
          y_s2 + turning_radius *
                     sin(direction_s2 * (t_inc_curr - t_increment_s2) + t_s2);
      (*state_curr)[2] = direction_s2 * (t_inc_curr - t_increment_s2) + t_s2 +
                         ((direction_s2 == 1) ? M_PI_2 : 3.0 * M_PI_2);
      while ((*state_curr)[2] < 0)
        (*state_curr)[2] += 2 * M_PI;
      while ((*state_curr)[2] > 2 * M_PI)
        (*state_curr)[2] -= 2 * M_PI;

      (*input_curr)[0] = t_inc_rel * turning_radius;
      (*input_curr)[1] = ((comb_no == 2) || (comb_no == 3)) ? -1 : 1;

      list_states->push_back(state_curr);
      list_inputs->push_back(input_curr);

      if ((t_inc_curr_prev + t_inc_curr) * turning_radius + d_inc_curr >
          distance_limit) {

        if (fully_extends)
          *fully_extends = 0;

        return total_distance_travel;
      }
    }
    //         printf (".");

    if (fully_extends)
      *fully_extends = 1;
  }

  return total_distance_travel;
}

double
Dubins::extend_dubins_all(StateDubins *state_ini,
                                  StateDubins *state_fin, int *fully_extends,
                                  std::list<StateDubins *> *list_states_out,
                                  std::list<InputDubins *> *list_inputs_out) {

  // 1. Compute the centers of all four spheres
  double ti = (*state_ini)[2];
  double tf = (*state_fin)[2];
  double sin_ti = sin(-ti);
  double cos_ti = cos(-ti);
  double sin_tf = sin(-tf);
  double cos_tf = cos(-tf);

  double si_left[3] = {(*state_ini)[0] + turning_radius * sin_ti,
                       (*state_ini)[1] + turning_radius * cos_ti,
                       ti + 3 * M_PI_2};
  double si_right[3] = {(*state_ini)[0] - turning_radius * sin_ti,
                        (*state_ini)[1] - turning_radius * cos_ti, ti + M_PI_2};
  double sf_left[3] = {(*state_fin)[0] + turning_radius * sin_tf,
                       (*state_fin)[1] + turning_radius * cos_tf,
                       tf + 3 * M_PI_2};
  double sf_right[3] = {(*state_fin)[0] - turning_radius * sin_tf,
                        (*state_fin)[1] - turning_radius * cos_tf, tf + M_PI_2};

  // 2. extend all four spheres
  double times[4];

  times[0] =
      extend_dubins_spheres(si_left[0], si_left[1], si_left[2], sf_right[0],
                            sf_right[1], sf_right[2], 1, NULL, NULL, NULL);
  times[1] =
      extend_dubins_spheres(si_right[0], si_right[1], si_right[2], sf_left[0],
                            sf_left[1], sf_left[2], 2, NULL, NULL, NULL);
  times[2] =
      extend_dubins_spheres(si_left[0], si_left[1], si_left[2], sf_left[0],
                            sf_left[1], sf_left[2], 3, NULL, NULL, NULL);
  times[3] =
      extend_dubins_spheres(si_right[0], si_right[1], si_right[2], sf_right[0],
                            sf_right[1], sf_right[2], 4, NULL, NULL, NULL);

  double min_time = DBL_MAX;
  int comb_min = -1;
  for (int i = 0; i < 4; i++) {
    if ((times[i] >= 0.0) && (times[i] < min_time)) {
      comb_min = i + 1;
      min_time = times[i];
    }
  }

  //     printf ("min comb : %d \n", comb_min);

  int res;
  switch (comb_min) {
  case 1:
    res = extend_dubins_spheres(si_left[0], si_left[1], si_left[2], sf_right[0],
                                sf_right[1], sf_right[2], 1, fully_extends,
                                list_states_out, list_inputs_out);
    //         if (*fully_extends)
    //             printf (":");
    return res;

  case 2:
    res = extend_dubins_spheres(
        si_right[0], si_right[1], si_right[2], sf_left[0], sf_left[1],
        sf_left[2], 2, fully_extends, list_states_out, list_inputs_out);
    //         if (*fully_extends)
    //             printf (":");
    return res;

  case 3:
    res = extend_dubins_spheres(si_left[0], si_left[1], si_left[2], sf_left[0],
                                sf_left[1], sf_left[2], 3, fully_extends,
                                list_states_out, list_inputs_out);
    //         if (*fully_extends)
    //             printf (":");
    return res;

  case 4:
    res = extend_dubins_spheres(
        si_right[0], si_right[1], si_right[2], sf_right[0], sf_right[1],
        sf_right[2], 4, fully_extends, list_states_out, list_inputs_out);
    //         if (*fully_extends)
    //             printf (":");
    return res;
  case -1:
  default:
    if (list_states_out) {
      list_states_out->clear();
      list_inputs_out->clear();
    }
    return -1.0;
  }
}

Dubins::Dubins() {}

Dubins::~Dubins() {}

int Dubins::extend(
    StateDubins *state_from_in, StateDubins *state_towards_in,
    int *exact_connection_out, TrajectoryDubins *trajectory_out,
    std::list<StateDubins *> *intermediate_vertices_out) {

  if (extend_dubins_all(state_from_in, state_towards_in, exact_connection_out,
                        &(trajectory_out->list_states),
                        &(trajectory_out->list_inputs)) < 0.0) {

    return 0;
  }

  return 1;
}

} // namespace dubins
} // namespace smp
