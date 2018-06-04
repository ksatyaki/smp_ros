
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <boost/range.hpp>
#include <boost/shared_array.hpp>

#include <cmath>
#include <cstdlib>

#include <smp/extenders/posq.hpp>

namespace smp {
namespace extenders {

double PosQ::set_angle_to_range(double alpha, double min) {

  while (alpha >= min + 2.0 * M_PI) {
    alpha -= 2.0 * M_PI;
  }
  while (alpha < min) {
    alpha += 2.0 * M_PI;
  }
  return alpha;
}

double PosQ::diff_angle_unwrap(double alpha1, double alpha2) {
  double delta;

  // normalize angles alpha1 and alpha2
  alpha1 = set_angle_to_range(alpha1, 0);
  alpha2 = set_angle_to_range(alpha2, 0);

  // take difference and unwrap
  delta = alpha1 - alpha2;
  if (alpha1 > alpha2) {
    while (delta > M_PI) {
      delta -= 2.0 * M_PI;
    }
  } else if (alpha2 > alpha1) {
    while (delta < -M_PI) {
      delta += 2.0 * M_PI;
    }
  }
  return delta;
}

double PosQ::normangle(double a, double mina) {

  double ap, minap;
  ap = a;
  minap = mina;
  while (ap >= (minap + M_PI * 2)) {
    ap = ap - M_PI * 2;
  }
  while (ap < minap) {
    ap = ap + M_PI * 2;
  }

  return ap;
}

double PosQ::f(double rho) {

  double Kv;
  /// New implementation  Kv= max atanh(rho)/rho
  Kv = 1.15;
  //    Kv=1;

  return tanh(rho * Kv);
  /// German Paper Version

  //  return (2*v0/M_PI)*atan((M_PI/(2*v0))*rho);
}

double *PosQ::posctrlstep(double x_c, double y_c, double t_c, double x_end,
                          double y_end, double t_end, double ct, double b,
                          int dir) {

  /** This function will generate a vector of double as output:

   *  [0] Vl velocity of the left wheel;
   *  [1] Vr velocity of the right wheel;
   *  [2] V translational Velocity;
   *  [3] W Angular Velocity.
   *  [4] EOT End Of Trajectory

  **/

  static double oldBeta, controllerType;

  double Krho, Kalpha, Kbeta, Kphi, Vmax, RhoThreshold1, RhoThreshold2,
      RhoEndCondition, PhiEndCondition;
  // [1 3 -1 -1]
  Krho = 1;
  Kalpha = 3;
  Kbeta = -1;
  Kphi = -1;
  Vmax = Krho;
  //  RhoThreshold1   = 0.04;
  RhoThreshold1 = 0.005;

  RhoThreshold2 = 0.02;
  //  RhoThreshold2   = 0.2;
  /// The RRT* edges' lenght is related to the RhoEndCondition

  RhoEndCondition = 0.05;

  /// the PhiEndCondition has to be setted properly,
  /// small Value, local minima can occur, tested --> greater than 35*M_PI/180

  //  PhiEndCondition = 1*M_PI/180; [NOT IN THE ASTOLFI PAPER :)]
  PhiEndCondition = 50 * M_PI / 180;

  if (ct == 0) {
    oldBeta = 0;
    controllerType = 1;
  }

  double dx, dy, rho, fRho, alpha, phi, beta, v, w, vl, vr, eot, vi, di;
  vi = 0.1;
  // rho
  eot = 1;
  dx = x_end - x_c;
  dy = y_end - y_c;
  rho = sqrt(dx * dx + dy * dy);
  fRho = rho;

  if (fRho > (Vmax / Krho)) {
    fRho = Vmax / Krho;
  }

  // alpha

  alpha = atan2(dy, dx) - t_c;
  alpha = normangle(alpha, -M_PI);

  // direction

  if (dir == 0) {

    if (alpha > (M_PI / 2)) {
      fRho = -fRho;
      alpha = alpha - M_PI;
    } else if (alpha <= -M_PI / 2) {
      fRho = -fRho;
      alpha = alpha + M_PI;
    }
  } else if (dir == -1) {
    fRho = -fRho;
    alpha = alpha + M_PI;
    if (alpha > M_PI) {
      alpha = alpha - 2 * M_PI;
    }
  }

  // phi

  phi = t_end - t_c;
  phi = normangle(phi, -M_PI);

  beta = normangle(phi - alpha, -M_PI);

  if ((abs(oldBeta - beta) > M_PI)) {
    beta = oldBeta;
  }
  oldBeta = beta;

  // set speed

  v = Krho * f(fRho);
  w = (Kalpha * alpha + Kbeta * beta);

  if (rho < RhoEndCondition) {

    eot = 1;
  } else {
    eot = 0;
  }

  if (eot) {
    w = 0.;
  }

  // Convert speed to wheel speed

  vl = v - w * b / 2;

  if (abs(vl) > Vmax) {

    if (vl < 0) {
      vl = Vmax * -1;
    } else {
      vl = Vmax;
    }
  }

  vr = v + w * b / 2;

  if (abs(vr) > Vmax) {
    if (vr < 0) {
      vr = Vmax * -1;
    } else {
      vr = Vmax;
    }
  }

  result[0] = vl;
  result[1] = vr;
  result[2] = v;
  result[3] = w;
  result[4] = eot;

  return result;
}

double PosQ::posctrl(StatePosQ *state_ini, StatePosQ *state_fin, int dir,
                     double b, double dt,
                     std::list<StatePosQ *> *list_states_out,
                     std::list<InputPosQ *> *list_inputs_out) {

  double sl, sr, oldSl, oldSr, t, eot, dSl, dSr, dSm, dSd, vl, vr, enc_l, enc_r;

  enc_l = 0;
  enc_r = 0;
  sl = 0;
  sr = 0;
  oldSl = 0;
  oldSr = 0;
  eot = 0;
  t = 0;
  vl = 0;
  vr = 0;

  double x, y, th;
  x = (*state_ini)[0];
  y = (*state_ini)[1];
  th = (*state_ini)[2];

  this->xi = x;
  this->yi = y;

  double vv, ww, ths;
  vv = 0;
  ww = 0;
  ths = 0.1;

  double dist;
  dist = 0;

  if (list_states_out) {

    while (eot == 0) {
      // calculate distance for both wheels
      dSl = sl - oldSl;
      dSr = sr - oldSr;
      dSm = (dSl + dSr) / 2;

      dSd = (dSr - dSl) / b;
      StatePosQ *curr = new StatePosQ;
      InputPosQ *ve = new InputPosQ;

      (*curr)[0] = x + dSm * cos(th + dSd / 2);
      (*curr)[1] = y + dSm * sin(th + dSd / 2);
      (*curr)[2] = normangle(th + dSd, -M_PI);

      intRes = posctrlstep((*curr)[0], (*curr)[1], (*curr)[2], (*state_fin)[0],
                           (*state_fin)[1], (*state_fin)[2], t, b, dir);
      // Save the velocity commands,eot
      vv = intRes[2];
      ww = intRes[3];
      (*ve)[0] = intRes[2];
      (*ve)[1] = intRes[3];
      eot = intRes[4];
      vl = intRes[0];
      vr = intRes[1];

      // Increase the timer
      t = t + dt;

      // keep track of previous wheel position
      oldSl = sl;
      oldSr = sr;

      // increase encoder values
      enc_l = enc_l + dt * vl;
      enc_r = enc_r + dt * vr;

      sl = enc_l;
      sr = enc_r;

      //
      float dxl, dyl;
      dxl = (*state_fin)[0] - (*curr)[0];
      dyl = (*state_fin)[1] - (*curr)[1];
      dist = sqrt(dxl * dxl + dyl * dyl);

      // save the state for the next sample
      x = (*curr)[0];
      y = (*curr)[1];
      th = (*curr)[2];

      if (eot == 1) {

        /// save the last state!!!
        StatePosQ *save = new StatePosQ;
        InputPosQ *vesave = new InputPosQ;
        (*vesave)[0] = intRes[2];
        (*vesave)[1] = intRes[3];
        dSl = sl - oldSl;
        dSr = sr - oldSr;
        dSm = (dSl + dSr) / 2;
        dSd = (dSr - dSl) / b;
        (*save)[0] = x + dSm * cos(th + dSd / 2);
        (*save)[1] = y + dSm * sin(th + dSd / 2);
        (*save)[2] = normangle(th + dSd, -M_PI);
        // Add current values to the Trajectory
        list_states_out->push_back((curr));
        list_inputs_out->push_back((ve));

        list_states_out->push_back((save));
        list_inputs_out->push_back((vesave));

      } else

      {
        // Add current values to the Trajectory
        list_states_out->push_back((curr));
        list_inputs_out->push_back((ve));
      }
    }
  }
  return dist;
}

PosQ::PosQ() {
  result = (double *)malloc(sizeof(double) * 5);
  intRes = (double *)malloc(sizeof(double) * 5);
}

PosQ::~PosQ() {}

int PosQ::extend(StatePosQ *state_from_in, StatePosQ *state_towards_in,
                 int *exact_connection_out, trajectory_t *trajectory_out,
                 std::list<StatePosQ *> *intermediate_vertices_out) {

  int dir;
  dir = 1;

  double b, d, myEps;
  const double dt = 0.1;

  T = 0.31;
  /// Base
  b = 0.4;

  myEps = 0.50;

  //   cout<<"call posctrl /...."<<endl<<endl;
  intermediate_vertices_out->clear();
  trajectory_out->clear();
  d = posctrl(state_from_in, state_towards_in, dir, b, dt,
              &(trajectory_out->list_states), &(trajectory_out->list_inputs));

  if (d < myEps) {
    (*exact_connection_out) = 1;
    return 1;

  } else

  {

    (*exact_connection_out) = 0;
    return 0;
  }
}
} // namespace extenders
} // namespace smp
