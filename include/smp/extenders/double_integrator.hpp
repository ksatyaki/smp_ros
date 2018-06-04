/*! \file components/extenders/double_integrator.h
  \brief The double integrator system components. State, input, and extender
  definitions.

  This file implements the state, input, and extender classes for a
  d-dimensional double integrator system, where d is a template parameter
  when appropriate. Currently, the implementation supports only d = 2.
  The author is working on the case when d > 2.

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

#ifndef _SMP_SYSTEM_DOUBLE_INTEGRATOR_H_
#define _SMP_SYSTEM_DOUBLE_INTEGRATOR_H_

// TODO: Generalize to multiple dimensions, make these configurable variables
// (already started in the .h file)
#define VELOCITY_CONSTRAINT_RANGE 2.0
#define VELOCITY_CONSTRAINT_RANGE_2 1.0
#define VELOCITY_CONSTRAINT_MAX 1.0
#define VELOCITY_CONSTRAINT_MIN -1.0
#define VELOCITY_CONSTRAINT_SQ 1.0
//
#define INPUT_CONSTRAINT_MAX 1.0
#define INPUT_CONSTRAINT_MIN -1.0
//
#define DELTA_T 0.1 // The time interval of integration and node placement

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <list>
#include <smp/extenders/base.hpp>
#include <smp/input_array_double.hpp>
#include <smp/state_array_double.hpp>

namespace smp {

double extend_with_time_optimal_control_one_axis(
    double s_ini[2], double s_fin[2], double u_max, int *direction,
    int *traj_saturated, double *x_intersect_beg, double *x_intersect_end,
    double *v_intersect);

int extend_with_effort_optimal_control_one_axis(
    double s_ini[2], double s_fin[2], double u_max, double t_min, double t_goal,
    double t_eps, int *dir, int *traj_saturated, double *max_control,
    double *x_intersect_beg, double *x_intersect_end, double *v_intersect);

//! Implementation of the state data structure for the double integrator
//! dynamics
/*! This class implements the state data structure for the double integrator
  dynamics. The number of state variables is twice number of dimensions,
  since for each dimension both position and velocity has to be stored. The
  positions are stored in the usual order first, and then the all velocities
  are stored in their usual order, in the array.
*/
template <int NUM_DIMENSIONS>
class StateDoubleIntegrator : public StateArrayDouble<NUM_DIMENSIONS * 2> {};

//! Implementation of the input data structure for the double integrator
//! dynamics
/*! This class implements the input data structure for the double integrator
  dynamics. The number of input variables is one plus the dimensions. The extra
  input variable, placed in the beginning of the array, is used to store the
  time it takes to execute the trajectory segment.
*/
template <int NUM_DIMENSIONS>
class InputDoubleIntegrator : public InputArrayDouble<NUM_DIMENSIONS + 1> {};

namespace extenders {
//! Extender function with double integrator dynamics.
/*! This class implements an extender with double integrator dynamics. It is
  intended that the number of dimensions of the state space is a template
  argument for the class. However, this feature is not implemented yet.
  Currently, the double integrator system works only in two dimensions
  (NUM_DIMENSIONS = 2), i.e., two positions and their two velocities.

  \ingroup extenders
*/
template <int NUM_DIMENSIONS>
class ExtenderDoubleIntegrator
    : public Base<StateDoubleIntegrator<NUM_DIMENSIONS>,
                  InputDoubleIntegrator<NUM_DIMENSIONS>> {

  using state_t = StateDoubleIntegrator<NUM_DIMENSIONS>;
  using input_t = InputDoubleIntegrator<NUM_DIMENSIONS>;
  using trajectory_t = Trajectory<StateDoubleIntegrator<NUM_DIMENSIONS>,
                                  InputDoubleIntegrator<NUM_DIMENSIONS>>;

  // // TODO: get back to appropriate velocity constraints.
  // double velocity_constraint_min[NUM_DIMENSIONS];
  // double velocity_constraint_max[NUM_DIMENSIONS];

  int extend_with_optimal_control(state_t *state_ini, state_t *state_fin,
                                  std::list<state_t *> *list_states_out,
                                  std::list<input_t *> *list_inputs_out) {

    list_states_out->clear();
    list_inputs_out->clear();

    // 1. Extend both axes
    double s_ini_a1[2] = {(*state_ini)[0], (*state_ini)[2]};
    double s_fin_a1[2] = {(*state_fin)[0], (*state_fin)[2]};
    int direction_a1 = 0;
    int traj_saturated_a1;
    double x_intersect_beg_a1;
    double x_intersect_end_a1;
    double v_intersect_a1;
    double time_a1 = extend_with_time_optimal_control_one_axis(
        s_ini_a1, s_fin_a1, INPUT_CONSTRAINT_MAX, &direction_a1,
        &traj_saturated_a1, &x_intersect_beg_a1, &x_intersect_end_a1,
        &v_intersect_a1);

    double s_ini_a2[2] = {(*state_ini)[1], (*state_ini)[3]};
    double s_fin_a2[2] = {(*state_fin)[1], (*state_fin)[3]};
    int direction_a2 = 0;
    int traj_saturated_a2;
    double x_intersect_beg_a2;
    double x_intersect_end_a2;
    double v_intersect_a2;
    double time_a2 = extend_with_time_optimal_control_one_axis(
        s_ini_a2, s_fin_a2, INPUT_CONSTRAINT_MAX, &direction_a2,
        &traj_saturated_a2, &x_intersect_beg_a2, &x_intersect_end_a2,
        &v_intersect_a2);

    double max_control_a1 = INPUT_CONSTRAINT_MAX;
    double max_control_a2 = INPUT_CONSTRAINT_MAX;

    // std::cout << "time_a1 : " << time_a1 << " :::: time_a2 : "  << time_a2 <<
    // std::endl;

    if ((time_a1 < 0.0) || (time_a2 < 0.0)) {
      std::cout << "No traj feasible" << std::endl;
      return 0;
    }

    // 2. Compute the minimum effor control for the lagging axes
    if (time_a1 < time_a2 - 0.001) {

      // Compute the minimum effort control for a2
      if (!extend_with_effort_optimal_control_one_axis(
              s_ini_a1, s_fin_a1, INPUT_CONSTRAINT_MAX, time_a1, time_a2,
              0.0001, &direction_a1, &traj_saturated_a1, &max_control_a1,
              &x_intersect_beg_a1, &x_intersect_end_a1, &v_intersect_a1))
        return 0;

    } else if (time_a2 < time_a1 - 0.001) {

      // 2.a. Compute the minimum effort control for a1
      if (!extend_with_effort_optimal_control_one_axis(
              s_ini_a2, s_fin_a2, INPUT_CONSTRAINT_MAX, time_a2, time_a1,
              0.0001, &direction_a2, &traj_saturated_a2, &max_control_a2,
              &x_intersect_beg_a2, &x_intersect_end_a2, &v_intersect_a2))
        return 0;
    }

    // 3. Create the merged trajectories
    // printf ("Max controls : (%3.5lf, %3.5lf)\n", max_control_a1,
    // max_control_a2);

    // Reverse engineer the timing

    double t_intersect_beg_a1 =
        fabs((v_intersect_a1 - s_ini_a1[1]) / max_control_a1);
    double t_intersect_end_a1;
    if (traj_saturated_a1)
      t_intersect_end_a1 =
          t_intersect_beg_a1 +
          fabs((x_intersect_beg_a1 - x_intersect_end_a1) / v_intersect_a1);
    else
      t_intersect_end_a1 = t_intersect_beg_a1;
    double t_end_a1 = fabs((s_fin_a1[1] - v_intersect_a1) / max_control_a1) +
                      t_intersect_end_a1;

    // printf ("times_a1 : %3.5lf, %3.5lf, %3.5lf\n", t_intersect_beg_a1,
    // t_intersect_end_a1, t_end_a1);

    double t_intersect_beg_a2 =
        fabs((v_intersect_a2 - s_ini_a2[1]) / max_control_a2);
    double t_intersect_end_a2;
    if (traj_saturated_a2)
      t_intersect_end_a2 =
          t_intersect_beg_a2 +
          fabs((x_intersect_beg_a2 - x_intersect_end_a2) / v_intersect_a2);
    else
      t_intersect_end_a2 = t_intersect_beg_a2;
    double t_end_a2 = fabs((s_fin_a2[1] - v_intersect_a2) / max_control_a2) +
                      t_intersect_end_a2;

    // printf ("times_a2 : %3.5lf, %3.5lf, %3.5lf\n", t_intersect_beg_a2,
    // t_intersect_end_a2, t_end_a2);

    double times_a1[3] = {t_intersect_beg_a1, t_intersect_end_a1, t_end_a1};
    double times_a2[3] = {t_intersect_beg_a2, t_intersect_end_a2, t_end_a2};

    double times[6];
    int stages[6][2];
    int k_a1 = 0;
    int k_a2 = 0;
    for (int i = 0; i < 6; i++) {
      if (k_a1 == 2) {
        // printf ("-a2-");
        stages[i][0] = k_a1 + 1;
        stages[i][1] = k_a2 + 1;
        times[i] = times_a2[k_a2++];
        if (k_a2 > 2)
          k_a2 = 2;
      } else if (k_a2 == 2) {
        // printf ("-a1-");
        stages[i][0] = k_a1 + 1;
        stages[i][1] = k_a2 + 1;
        times[i] = times_a1[k_a1++];
        if (k_a1 > 2)
          k_a1 = 2;
      } else if (times_a1[k_a1] < times_a2[k_a2]) {
        // printf ("-s1-");
        stages[i][0] = k_a1 + 1;
        stages[i][1] = k_a2 + 1;
        times[i] = times_a1[k_a1++];
      } else {
        // printf ("-s2-");
        stages[i][0] = k_a1 + 1;
        stages[i][1] = k_a2 + 1;
        times[i] = times_a2[k_a2++];
      }
    }
    // printf ("\n");

    // Create the merging states/inputs
    double t_curr = 0.0;

    int times_counter = 0;

    double min_control_a1 = -max_control_a1;
    double min_control_a2 = -max_control_a2;

    double c0_1_a1 =
        s_ini_a1[0] - (s_ini_a1[1] * s_ini_a1[1]) / (2.0 * max_control_a1);
    double c1_1_a1 =
        s_fin_a1[0] - (s_fin_a1[1] * s_fin_a1[1]) / (2.0 * min_control_a1);
    double c0_2_a1 =
        s_ini_a1[0] - (s_ini_a1[1] * s_ini_a1[1]) / (2.0 * min_control_a1);
    double c1_2_a1 =
        s_fin_a1[0] - (s_fin_a1[1] * s_fin_a1[1]) / (2.0 * max_control_a1);

    double c0_1_a2 =
        s_ini_a2[0] - (s_ini_a2[1] * s_ini_a2[1]) / (2.0 * max_control_a2);
    double c1_1_a2 =
        s_fin_a2[0] - (s_fin_a2[1] * s_fin_a2[1]) / (2.0 * min_control_a2);
    double c0_2_a2 =
        s_ini_a2[0] - (s_ini_a2[1] * s_ini_a2[1]) / (2.0 * min_control_a2);
    double c1_2_a2 =
        s_fin_a2[0] - (s_fin_a2[1] * s_fin_a2[1]) / (2.0 * max_control_a2);

    // printf ("times: \n");
    // for (int i = 0; i < 6; i++)
    //   printf (" %3.5lf - (%d,%d) - (%3.5lf,%3.5lf)\n", times[i],
    //   stages[i][0], stages[i][1],
    // 	    times_a1[stages[i][0]-1], times_a2[stages[i][1]-1]);
    // printf ("\n");

    while (times_counter < 6) {

      int increment_times_counter = 0;

      // Determine the current time to act
      double del_t = DELTA_T;
      t_curr += DELTA_T;

      if (t_curr > times[times_counter]) {
        del_t -= t_curr - times[times_counter];
        t_curr = times[times_counter];
        increment_times_counter = 1;
      }

      // Calculate the states/inputs at the current time
      state_t *state_new = new state_t;
      input_t *input_new = new input_t;

      //      Determine the first axis at this time step
      double t_diff_curr;
      if (stages[times_counter][0] == 1) {
        (*state_new)[2] = s_ini_a1[1] + direction_a1 * max_control_a1 * t_curr;
        (*state_new)[0] = ((*state_new)[2] * (*state_new)[2]) /
                          (2 * direction_a1 * max_control_a1);
        if (direction_a1 == 1) {
          (*state_new)[0] += c0_1_a1;
        } else {
          (*state_new)[0] += c0_2_a1;
        }
        (*input_new)[1] = direction_a1 * max_control_a1;
      } else if (stages[times_counter][0] == 2) {
        (*state_new)[0] =
            x_intersect_beg_a1 + v_intersect_a1 * (t_curr - times_a1[0]);
        (*state_new)[2] = v_intersect_a1;
        (*input_new)[1] = 0.0;
      } else {
        t_diff_curr = times_a1[2] - t_curr;
        (*state_new)[2] =
            s_fin_a1[1] - direction_a1 * min_control_a1 * t_diff_curr;
        (*state_new)[0] = ((*state_new)[2] * (*state_new)[2]) /
                          (2 * direction_a1 * min_control_a1);
        if (direction_a1 == 1) {
          (*state_new)[0] += c1_1_a1;
        } else {
          (*state_new)[0] += c1_2_a1;
        }

        (*input_new)[1] = direction_a1 * min_control_a1;
      }

      //      Determine the second axis at this time step
      if (stages[times_counter][1] == 1) {
        (*state_new)[3] = s_ini_a2[1] + direction_a2 * max_control_a2 * t_curr;
        (*state_new)[1] = ((*state_new)[3] * (*state_new)[3]) /
                          (2 * direction_a2 * max_control_a2);
        if (direction_a2 == 1) {
          (*state_new)[1] += c0_1_a2;
        } else {
          (*state_new)[1] += c0_2_a2;
        }

        (*input_new)[2] = direction_a2 * max_control_a2;
      } else if (stages[times_counter][1] == 2) {
        (*state_new)[1] =
            x_intersect_beg_a2 + v_intersect_a2 * (t_curr - times_a2[0]);
        (*state_new)[3] = v_intersect_a2;
        (*input_new)[2] = 0.0;
      } else {
        t_diff_curr = times_a2[2] - t_curr;
        (*state_new)[3] =
            s_fin_a2[1] - direction_a2 * min_control_a2 * t_diff_curr;
        (*state_new)[1] = ((*state_new)[3] * (*state_new)[3]) /
                          (2 * direction_a2 * min_control_a2);
        if (direction_a2 == 1) {
          (*state_new)[1] += c1_1_a2;
        } else {
          (*state_new)[1] += c1_2_a2;
        }

        (*input_new)[2] = direction_a2 * min_control_a2;
      }

      (*input_new)[0] = del_t;

      // printf ("(%d, %d, %d) - t_curr: %2.5lf - a1: (%3.5lf , %3.5lf , %3.5lf)
      // - a2:  (%3.5lf , %3.5lf , %3.5lf)\n", 	    times_counter,
      // stages[times_counter][0], stages[times_counter][1], 	    t_curr,
      // state_new->state_vars[0], state_new->state_vars[2],
      // input_new->input_vars[0], state_new->state_vars[1],
      // state_new->state_vars[3], input_new->x[1]);

      // Store the states/inputs to the list
      list_states_out->push_back(state_new);
      list_inputs_out->push_back(input_new);

      // Check whether we are done
      if (increment_times_counter == 1) {
        while (times_counter < 6)
          if (fabs(times[times_counter] - times[times_counter + 1]) < 0.0001)
            times_counter++;
          else
            break;
        times_counter++;
      }
    }

    return 1;
  }

public:
  ExtenderDoubleIntegrator() {}
  ~ExtenderDoubleIntegrator() {}

  int extend(state_t *state_from_in, state_t *state_towards_in,
             int *exact_connection_out, trajectory_t *trajectory_out,
             std::list<state_t *> *intermediate_vertices_out) {

    // std::cout << "state_from";
    // for (int i = 0; i < 4; i++)
    //   std::cout << " : " << state_from_in->state_vars[i];
    // std::cout << std::endl;

    // std::cout << "state_towa";
    // for (int i = 0; i < 4; i++)
    //   std::cout << " : " << state_towards_in->state_vars[i];
    // std::cout << std::endl;

    intermediate_vertices_out->clear();

    if (extend_with_optimal_control(state_from_in, state_towards_in,
                                    &(trajectory_out->list_states),
                                    &(trajectory_out->list_inputs)) == 0) {

      return 0;
    }

    *exact_connection_out = 1;

    return 1;
  }
};
} // namespace extenders
} // namespace smp

#endif
