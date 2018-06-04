#include <smp/extenders/double_integrator.hpp>

double extend_with_time_optimal_control_one_axis(
    double s_ini[2], double s_fin[2], double u_max, int *direction,
    int *traj_saturated, double *x_intersect_beg, double *x_intersect_end,
    double *v_intersect) {

  double u_min = -u_max;

  // Define global variables traj_1
  bool traj_1_feasible = false;
  bool traj_1_saturated = false;
  double t_tot_1 = DBL_MAX;
  double x_intersect_beg_1;
  double x_intersect_end_1;
  double v_intersect_1;

  // Define global variables for traj_2
  bool traj_2_feasible = false;
  bool traj_2_saturated = false;
  double t_tot_2 = DBL_MAX;
  double x_intersect_beg_2;
  double x_intersect_end_2;
  double v_intersect_2;

  // 1. Calculate the traj (um, 0, -um)
  double c0_1 = s_ini[0] - (s_ini[1] * s_ini[1]) / (2.0 * u_max);
  double c1_1 = s_fin[0] - (s_fin[1] * s_fin[1]) / (2.0 * u_min);

  if (c0_1 < c1_1) {
    double x_intersect_1 = (c0_1 + c1_1) / 2.0;
    double v_intersect_pos_1 = sqrt((x_intersect_1 - c0_1) * 2 * u_max);
    double v_intersect_neg_1 = -v_intersect_pos_1;

    if ((s_ini[1] < v_intersect_pos_1) && (s_fin[1] < v_intersect_pos_1)) {

      traj_1_feasible = true;
      // std::cout << "traj 1 feasible" << std::endl;

      if ((s_ini[1] < v_intersect_neg_1) && (s_fin[1] < v_intersect_neg_1)) {
        v_intersect_1 = v_intersect_neg_1;
      } else {
        v_intersect_1 = v_intersect_pos_1;
      }

      /* ===== Consistency check === TODO: Remove this later ====
      if ((v_intersect_1 < VELOCITY_CONSTRAINT_MIN - 0.1)) {
        std::cout << "ERR: Velocity constraint is not met :" << v_intersect_1
             << std::endl;
        exit(1);
      }
      // ===== */

      if (v_intersect_1 > VELOCITY_CONSTRAINT_MAX) {
        traj_1_saturated = true;
        v_intersect_1 = VELOCITY_CONSTRAINT_MAX;
        x_intersect_beg_1 = VELOCITY_CONSTRAINT_SQ / (2.0 * u_max) + c0_1;
        x_intersect_end_1 = VELOCITY_CONSTRAINT_SQ / (2.0 * u_min) + c1_1;
      } else {
        x_intersect_beg_1 = x_intersect_1;
        x_intersect_end_1 = x_intersect_1;
      }

      double t0_1 = (v_intersect_1 - s_ini[1]) / u_max;
      double t1_1 = (s_fin[1] - v_intersect_1) / u_min;
      double ti_1 = 0.0;
      if (traj_1_saturated) {
        ti_1 = fabs(x_intersect_end_1 - x_intersect_beg_1) /
               VELOCITY_CONSTRAINT_MAX;
      }
      t_tot_1 = t0_1 + t1_1 + ti_1;

      // std::cout << "Times control 1 : " << t0_1 << " : " << t0_1 + ti_1 << "
      // : "
      // << t0_1 + ti_1 + t1_1 << std::endl << std::endl;
    }
  }

  // 2. Calculate the traj (um, 0, -um)
  double c0_2 = s_ini[0] - (s_ini[1] * s_ini[1]) / (2.0 * u_min);
  double c1_2 = s_fin[0] - (s_fin[1] * s_fin[1]) / (2.0 * u_max);
  if (c1_2 < c0_2) {
    double x_intersect_2 = (c0_2 + c1_2) / 2.0;
    double v_intersect_pos_2 = sqrt((x_intersect_2 - c1_2) * 2 * u_max);
    double v_intersect_neg_2 = -v_intersect_pos_2;

    if ((s_ini[1] > v_intersect_neg_2) && (s_fin[1] > v_intersect_neg_2)) {

      traj_2_feasible = true;
      // std::cout << "traj 2 feasible" << std::endl;

      if ((s_ini[1] > v_intersect_pos_2) && (s_fin[1] > v_intersect_pos_2)) {
        v_intersect_2 = v_intersect_pos_2;
      } else {
        v_intersect_2 = v_intersect_neg_2;
      }

      /* ===== Consistency check === TODO: Remove this later ====
      if ((v_intersect_2 > VELOCITY_CONSTRAINT_MAX + 0.1)) {
        std::cout << "ERR: Velocity constraint is not met :n" << v_intersect_2
             << std::endl;
        exit(1);
      }
      // ===== */

      if (v_intersect_2 < VELOCITY_CONSTRAINT_MIN) {
        traj_2_saturated = true;
        v_intersect_2 = VELOCITY_CONSTRAINT_MIN;
        x_intersect_beg_2 = VELOCITY_CONSTRAINT_SQ / (2.0 * u_min) + c0_2;
        x_intersect_end_2 = VELOCITY_CONSTRAINT_SQ / (2.0 * u_max) + c1_2;
      } else {
        x_intersect_beg_2 = x_intersect_2;
        x_intersect_end_2 = x_intersect_2;
      }

      double t0_2 = (v_intersect_2 - s_ini[1]) / u_min;
      double t1_2 = (s_fin[1] - v_intersect_2) / u_max;
      double ti_2 = 0.0;
      if (traj_2_saturated) {
        ti_2 = fabs(x_intersect_end_2 - x_intersect_beg_2) /
               VELOCITY_CONSTRAINT_MAX;
      }
      t_tot_2 = t0_2 + t1_2 + ti_2;

      // std::cout << std::endl;
      // std::cout << "Times control 2 : " << t0_2 << " : " << t0_2 + ti_2 << "
      // : "
      // << t0_2 + ti_2 + t1_2 << std::endl;
    }
  }

  // 3. Return the results
  if ((!traj_1_feasible) && (!traj_2_feasible)) { // This should never kick in.
    std::cout << "ERR: no traj feasible" << std::endl;
    // return -1.0;
    exit(1);
  }

  if (t_tot_1 < t_tot_2) {
    if (direction != NULL) {
      *direction = 1;
      *traj_saturated = traj_1_saturated ? 1 : 0;
      *x_intersect_beg = x_intersect_beg_1;
      *x_intersect_end = x_intersect_end_1;
      *v_intersect = v_intersect_1;
    } else {
      std::cout << "ERR: no direction controls 1" << std::endl;
      exit(1);
    }
    return t_tot_1;
  } else {
    if (direction != NULL) {
      *direction = -1;
      *traj_saturated = traj_2_saturated ? 1 : 0;
      *x_intersect_beg = x_intersect_beg_2;
      *x_intersect_end = x_intersect_end_2;
      *v_intersect = v_intersect_2;
    } else {
      std::cout << "ERR: no direction controls 2" << std::endl;
      exit(1);
    }
    return t_tot_2;
  }
}

int extend_with_effort_optimal_control_one_axis(
    double s_ini[2], double s_fin[2], double u_max, double t_min, double t_goal,
    double t_eps, int *dir, int *traj_saturated, double *max_control,
    double *x_intersect_beg, double *x_intersect_end, double *v_intersect) {

  // Use time_optima_control function with binary search to find the control
  // effort that achieves time t_goal -+ t_eps

  if (t_goal < t_min) {
    std::cout << "ERR: t_goal < t_min \n" << std::endl;
    exit(1);
  }

  double t_curr = t_min;
  double u_curr = u_max / 2.0;
  double u_diff = u_curr / 2.0; // next difference in input: u_max/4.0;
  int max_steps = 20;
  int i = 0;

  while ((fabs(t_curr - t_goal) > t_eps) && (i++ < max_steps)) {
    t_curr = extend_with_time_optimal_control_one_axis(
        s_ini, s_fin, u_curr, dir, traj_saturated, x_intersect_beg,
        x_intersect_end, v_intersect);

    if (t_curr < t_goal) { // decrease the control effort
      u_curr -= u_diff;
    } else { // increase the control effort
      u_curr += u_diff;
    }

    u_diff /= 2.0;
    // printf ("u_curr: %3.5lf, t_curr: %3.5lf, t_diff: %3.5lf\n", u_curr,
    // t_curr, t_curr- t_goal);
  }

  // printf ("time_min: %3.5lf, time_req: %3.5lf, time_diff: %3.5lf, t_eps:
  // %3.5lf num_steps: %d\n",
  // 	  t_min, t_goal, t_curr - t_goal, t_eps, i);

  *max_control = u_curr;

  if (i > max_steps) {
    // printf ("Search diverges: scanning the spectrum \n");
    // for (int i = 1; i < 100; i++) {
    //     printf ("u_curr: %3.5lf, t_curr: %3.5lf\n", 1.0/100.0 * ((double)i),
    //             optsystem_extend_with_time_optimal_control_one_axis (s_ini,
    //             s_fin, 1.0/100.0 * ((double)i),
    //                                                                  dir,
    //                                                                  traj_saturated,
    //                                                                  x_intersect_beg,
    //                                                                  x_intersect_end,
    //                                                                  v_intersect)
    //                                                                  );
    // }
    return 0;
  }

  return 1;
}
