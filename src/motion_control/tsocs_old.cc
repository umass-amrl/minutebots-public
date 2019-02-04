// Copyright 2018 dbalaban@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include <stdio.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/tsocs_old.h"
#include "motion_control/tsocs_templates.h"

using std::isnan;
using ntoc::ControlSequence1D;
using math_util::AngleMod;

namespace tsocs {

static const double kScale = 10.0;
static const bool kUseNewParams = false;
const bool kUsePolarParams = false;
const bool kUseDoublePolarParams = false;
static const bool kUseRegularization = false;
static const bool kUseParallelVelocityCost = false;
static const bool kConstantC = false;
static const bool kRescaleGuess = true;
const bool kUsePerturbationsFix = false;
const int kMaxPerturbations = 5;
const bool kWriteFailures = false;
static const bool kOpenLoop = false;
static const double kDeltaT = 1.0 / kTransmitFrequency;
static bool use_t_regularization = false;
static double t_expected;
static double v_cost_coef = 1.0;
static double x_cost_coef = 1.0;
static unsigned int seed = 0;

// static ScopedFile failure_fid("TSOCS-simulated-iterative-failures.txt", "a");

Eigen::Vector2d delta_pose;
Eigen::Vector2d delta_vel;
Eigen::Vector2d current_vel;

double t_max;
Eigen::Vector2d x0_prev, xf_prev, v0_prev, vf_prev;

double theta, a, b, c, d, t_tot;
double theta1, theta2, theta3;

template <typename T>
T V_end(const T a, const T b, const T c, const T d, const T t) {
  const T f1 = F1(a, b);
  const T f2 = F2(a, b, c, d);
  const T f3 = F3(a, b, c, d);
  const T f4 = F1(c, d);
  const T g1 = sqrt(f1 * f1 + T(2) * f2 + f4 * f4);
  const T ln = LogRatio(g1, f1, f2, f4);

  const T result =
      a * (g1 - f4) / (f1 * f1) + b * f3 * log(ln) / (f1 * f1 * f1);

  return result * t;
}

struct Xdist {
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const c,
                  const T* const d, const T* const t, T* residual) const {
    const double vInit1 = current_vel.x(), vInit2 = current_vel.y(),
                 deltaX1 = delta_pose.x(), deltaX2 = delta_pose.y();

    if (*t < T(0)) {
      return false;
    }

    T x1, x2;
    if (kUseNewParams) {
      x1 = X_end(*a, *b, *c, *d, *t) + T(vInit1) * t[0];
      x2 = X_end(*b, *a, *d, *c, *t) + T(vInit2) * t[0];
    } else {
      x1 = X(*a, *b, *c, *d, *t) + T(vInit1) * t[0];
      x2 = X(*b, *a, *d, *c, *t) + T(vInit2) * t[0];
    }

    if (ceres::IsNaN(x2)) {
      printf("x2 is nan in Xdist\n");
      cout << "a: " << *a << endl;
      cout << "b: " << *b << endl;
      cout << "c: " << *c << endl;
      cout << "d: " << *d << endl;
      cout << "T: " << *t << endl;
    }

    residual[0] = T(sqrt(x_cost_coef)) * (x1 - T(deltaX1));
    residual[1] = T(sqrt(x_cost_coef)) * (x2 - T(deltaX2));

    return true;
  }
};

struct Xdist_polar {
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const theta,
                  const T* const t, T* residual) const {
    const double vInit1 = current_vel.x(), vInit2 = current_vel.y(),
                 deltaX1 = delta_pose.x(), deltaX2 = delta_pose.y();

    if (*t < T(0)) {
      return false;
    }

    T x1, x2;
    x1 = X(*a, *b, cos(*theta), sin(*theta), *t) + T(vInit1) * t[0];
    x2 = X(*b, *a, sin(*theta), cos(*theta), *t) + T(vInit2) * t[0];

    if (ceres::IsNaN(x2)) {
      std::printf("x2 is nan in Xdist_polar\n");
    }

    residual[0] = x1 - T(deltaX1);
    residual[1] = x2 - T(deltaX2);

    return true;
  }
};

struct Xdist_double_polar {
  template <typename T>
  bool operator()(const T* const theta1, const T* const theta2,
                  const T* const theta3, const T* const t, T* residual) const {
    const double vInit1 = current_vel.x(), vInit2 = current_vel.y(),
                 deltaX1 = delta_pose.x(), deltaX2 = delta_pose.y();

    if (*t < T(0)) {
      return false;
    }

    T a, b;
    if (*t == T(0)) {
      printf("t is 0 in Xdist double polar\n");
      a = T(1);
      b = T(1);
    } else {
      a = (sin(*theta1) - tan(*theta1 + *theta3) * cos(*theta1)) /
          (tan(*theta2) - tan(*theta1 + *theta3) + T(kEpsilon));
      b = a * tan(*theta2);
      // at this point (a,b) hold the endpoints of the adjoint line.
      // Now I convert them to rates of change
      a = (a - cos(*theta1)) / *t;
      b = (b - sin(*theta1)) / *t;
    }
    T x1 = X(a, b, cos(*theta1), sin(*theta1), *t) + T(vInit1) * t[0];
    T x2 = X(b, a, sin(*theta1), cos(*theta1), *t) + T(vInit2) * t[0];

    if (ceres::IsNaN(x2)) {
      std::printf("x2 is nan in Xdist_double_polar\n");
    }

    residual[0] = x1 - T(deltaX1);
    residual[1] = x2 - T(deltaX2);

    return true;
  }
};

struct Xdist_constT {
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const c,
                  const T* const d, T* residual) const {
    const double vInit1 = current_vel.x(), vInit2 = current_vel.y(),
                 deltaX1 = delta_pose.x(), deltaX2 = delta_pose.y();

    T x1, x2;
    if (kUseNewParams) {
      x1 = X_end(*a, *b, *c, *d, T(t_max)) + T(vInit1 * t_max);
      x2 = X_end(*b, *a, *d, *c, T(t_max)) + T(vInit2 * t_max);
    } else {
      x1 = X(*a, *b, *c, *d, T(t_max)) + T(vInit1 * t_max);
      x2 = X(*b, *a, *d, *c, T(t_max)) + T(vInit2 * t_max);
    }

    residual[0] = x1 - T(deltaX1);
    residual[1] = x2 - T(deltaX2);

    return true;
  }
};

struct Xdist_polar_constT {
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const theta,
                  T* residual) const {
    const double vInit1 = current_vel.x(), vInit2 = current_vel.y(),
                 deltaX1 = delta_pose.x(), deltaX2 = delta_pose.y();

    T x1, x2;

    x1 = X(*a, *b, cos(*theta), sin(*theta), T(t_max)) + T(vInit1 * t_max);
    x2 = X(*b, *a, sin(*theta), cos(*theta), T(t_max)) + T(vInit2 * t_max);

    residual[0] = x1 - T(deltaX1);
    residual[1] = x2 - T(deltaX2);

    return true;
  }
};

struct Xdist_double_polar_constT {
  template <typename T>
  bool operator()(const T* const theta1, const T* const theta2,
                  const T* const theta3, T* residual) const {
    const double vInit1 = current_vel.x(), vInit2 = current_vel.y(),
                 deltaX1 = delta_pose.x(), deltaX2 = delta_pose.y();

    if (t_max < T(0)) {
      return false;
    }

    T a, b;
    a = (sin(*theta1) - tan(*theta1 + *theta3) * cos(*theta1)) /
        (tan(*theta2) - tan(*theta1 + *theta3) + T(kEpsilon));
    b = a * tan(*theta2);
    // at this point (a,b) hold the endpoints of the adjoint line.
    // Now I convert them to rates of change
    a = (a - cos(*theta1)) / t_max;
    b = (b - sin(*theta1)) / t_max;
    T x1 = X(a, b, cos(*theta1), sin(*theta1), T(t_max)) + T(vInit1) * t_max;
    T x2 = X(b, a, sin(*theta1), cos(*theta1), T(t_max)) + T(vInit2) * t_max;

    if (ceres::IsNaN(x2)) {
      std::printf("x2 is nan in Xdist_double_polar_constT\n");
    }

    residual[0] = x1 - T(deltaX1);
    residual[1] = x2 - T(deltaX2);

    return true;
  }
};

struct Vdist {
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const c,
                  const T* const d, const T* const t, T* residual) const {
    const double deltaV1 = delta_vel.x(), deltaV2 = delta_vel.y();

    if (*t < T(0)) {
      return false;
    }

    T v1, v2;
    if (kUseNewParams) {
      v1 = V_end(*a, *b, *c, *d, *t);
      v2 = V_end(*b, *a, *d, *c, *t);
    } else {
      v1 = V(*a, *b, *c, *d, *t);
      v2 = V(*b, *a, *d, *c, *t);
    }

    if (ceres::IsNaN(v1)) {
      std::printf("v1 is nan \n");
    }
    if (ceres::IsNaN(v2)) {
      std::printf("v2 is nan \n");
    }

    residual[0] = T(sqrt(v_cost_coef)) * (v1 - T(deltaV1));
    residual[1] = T(sqrt(v_cost_coef)) * (v2 - T(deltaV2));

    return true;
  }
};

struct Vdist_polar {
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const theta,
                  const T* const t, T* residual) const {
    const double deltaV1 = delta_vel.x(), deltaV2 = delta_vel.y();

    if (*t < T(0)) {
      return false;
    }

    T v1, v2;
    v1 = V(*a, *b, cos(*theta), sin(*theta), *t);
    v2 = V(*b, *a, sin(*theta), cos(*theta), *t);

    if (ceres::IsNaN(v1)) {
      std::printf("v1 is nan \n");
    }
    if (ceres::IsNaN(v2)) {
      std::printf("v2 is nan \n");
    }

    residual[0] = v1 - T(deltaV1);
    residual[1] = v2 - T(deltaV2);

    return true;
  }
};

struct Vdist_double_polar {
  template <typename T>
  bool operator()(const T* const theta1, const T* const theta2,
                  const T* const theta3, const T* const t, T* residual) const {
    const double deltaV1 = delta_vel.x(), deltaV2 = delta_vel.y();

    if (*t < T(0)) {
      return false;
    }

    T a, b;
    if (*t == T(0)) {
      printf("t is 0 in Vdist double polar\n");
      a = T(1);
      b = T(1);
    } else {
      a = (sin(*theta1) - tan(*theta1 + *theta3) * cos(*theta1)) /
          (tan(*theta2) - tan(*theta1 + *theta3) + T(kEpsilon));
      b = a * tan(*theta2);
      // at this point (a,b) hold the endpoints of the adjoint line.
      // Now I convert them to rates of change
      a = (a - cos(*theta1)) / *t;
      b = (b - sin(*theta1)) / *t;
    }
    T v1 = V(a, b, cos(*theta1), sin(*theta1), *t);
    T v2 = V(b, a, sin(*theta1), cos(*theta1), *t);

    residual[0] = v1 - T(deltaV1);
    residual[1] = v2 - T(deltaV2);

    return true;
  }
};

struct Vdist_constT {
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const c,
                  const T* const d, T* residual) const {
    const double deltaV1 = delta_vel.x(), deltaV2 = delta_vel.y();

    T v1, v2;
    const T t = T(t_max);
    if (kUseNewParams) {
      v1 = V_end(*a, *b, *c, *d, t);
      v2 = V_end(*b, *a, *d, *c, t);
    } else {
      v1 = V(*a, *b, *c, *d, t);
      v2 = V(*b, *a, *d, *c, t);
    }

    if (ceres::IsNaN(v1)) {
      std::printf("v1 is nan \n");
    }
    if (ceres::IsNaN(v2)) {
      std::printf("v2 is nan \n");
    }

    residual[0] = v1 - T(deltaV1);
    residual[1] = v2 - T(deltaV2);

    return true;
  }
};

struct Vdist_polar_constT {
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const theta,
                  T* residual) const {
    const double deltaV1 = delta_vel.x(), deltaV2 = delta_vel.y();

    T v1, v2;
    const T t = T(t_max);
    v1 = V(*a, *b, cos(*theta), sin(*theta), t);
    v2 = V(*b, *a, sin(*theta), cos(*theta), t);

    if (ceres::IsNaN(v1)) {
      std::printf("v1 is nan \n");
    }
    if (ceres::IsNaN(v2)) {
      std::printf("v2 is nan \n");
    }

    residual[0] = v1 - T(deltaV1);
    residual[1] = v2 - T(deltaV2);

    return true;
  }
};

struct Vdist_double_polar_constT {
  template <typename T>
  bool operator()(const T* const theta1, const T* const theta2,
                  const T* const theta3, T* residual) const {
    const double deltaV1 = delta_vel.x(), deltaV2 = delta_vel.y();

    if (t_max < T(0)) {
      return false;
    }

    T a, b;
    a = (sin(*theta1) - tan(*theta1 + *theta3) * cos(*theta1)) /
        (tan(*theta2) - tan(*theta1 + *theta3) + T(kEpsilon));
    b = a * tan(*theta2);
    // at this point (a,b) hold the endpoints of the adjoint line. Now I convert
    // them to rates of change
    a = (a - cos(*theta1)) / t_max;
    b = (b - sin(*theta1)) / t_max;

    T v1 = V(a, b, cos(*theta1), sin(*theta1), T(t_max));
    T v2 = V(b, a, sin(*theta1), cos(*theta1), T(t_max));

    residual[0] = v1 - T(deltaV1);
    residual[1] = v2 - T(deltaV2);

    return true;
  }
};

struct V_stage1 {
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const c,
                  const T* const d, T* residual) const {
    const double vInit1 = current_vel.x(), vInit2 = current_vel.y(),
                 deltaV1 = delta_vel.x(), deltaV2 = delta_vel.y(),
                 deltaX1 = delta_pose.x(), deltaX2 = delta_pose.y();

    T v1, v2;
    if (kUseNewParams) {
      v1 = V_end(*a, *b, *c, *d, T(t_max));
      v2 = V_end(*b, *a, *d, *c, T(t_max));
    } else {
      v1 = V(*a, *b, *c, *d, T(t_max));
      v2 = V(*b, *a, *d, *c, T(t_max));
    }
    T v1f = T(vInit1) + v1;
    T v2f = T(vInit2) + v2;
    T vf_mag = v1f * v1f + v2f * v2f;
    T target_v1f = T(deltaV1 + vInit1);
    T target_v2f = T(deltaV2 + vInit2);
    T target_v2f_mag = target_v1f * target_v1f + target_v2f * target_v2f;

    if (target_v2f_mag > T(kEpsilon)) {
      residual[0] =
          T(kScale) * ((v1f * target_v1f + v2f * target_v2f) /
                           (sqrt(target_v2f_mag * vf_mag) + T(kEpsilon)) -
                       T(1));
    } else {
      // time to reach rest (screeching halt)
      const T vInitMag = T(sqrt(vInit1 * vInit1 + vInit2 * vInit2));
      // displacement vector after reaching rest point
      T x1 = T(deltaX1) - T(.5 * vInit1) * vInitMag;
      T x2 = T(deltaX2) - T(.5 * vInit2) * vInitMag;
      const T norm = sqrt(x1 * x1 + x2 * x2);
      x1 = x1 / norm;
      x2 = x2 / norm;
      residual[0] = T(0);
      residual[1] = sqrt(vf_mag) / T(kScale);
      return true;
    }

    if (vf_mag < target_v2f_mag) {
      residual[1] = log(vf_mag / (T(kEpsilon) + target_v2f_mag));
    } else {
      residual[1] = T(0);
    }

    return true;
  }
};

struct V_polar_stage1 {
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const theta,
                  T* residual) const {
    const double vInit1 = current_vel.x(), vInit2 = current_vel.y(),
                 deltaV1 = delta_vel.x(), deltaV2 = delta_vel.y(),
                 deltaX1 = delta_pose.x(), deltaX2 = delta_pose.y();

    T v1, v2;
    v1 = V(*a, *b, cos(*theta), sin(*theta), T(t_max));
    v2 = V(*b, *a, sin(*theta), cos(*theta), T(t_max));
    T v1f = T(vInit1) + v1;
    T v2f = T(vInit2) + v2;
    T vf_mag = v1f * v1f + v2f * v2f;
    T target_v1f = T(deltaV1 + vInit1);
    T target_v2f = T(deltaV2 + vInit2);
    T target_v2f_mag = target_v1f * target_v1f + target_v2f * target_v2f;

    if (target_v2f_mag > T(kEpsilon)) {
      residual[0] =
          T(kScale) * ((v1f * target_v1f + v2f * target_v2f) /
                           (sqrt(target_v2f_mag * vf_mag) + T(kEpsilon)) -
                       T(1));
    } else {
      // time to reach rest (screeching halt)
      const T vInitMag = T(sqrt(vInit1 * vInit1 + vInit2 * vInit2));
      // displacement vector after reaching rest point
      T x1 = T(deltaX1) - T(.5 * vInit1) * vInitMag;
      T x2 = T(deltaX2) - T(.5 * vInit2) * vInitMag;
      const T norm = sqrt(x1 * x1 + x2 * x2);
      x1 = x1 / norm;
      x2 = x2 / norm;
      residual[0] = T(0);
      residual[1] = sqrt(vf_mag) / T(kScale);
      return true;
    }

    if (vf_mag < target_v2f_mag) {
      residual[1] = log(vf_mag / (T(kEpsilon) + target_v2f_mag));
    } else {
      residual[1] = T(0);
    }

    return true;
  }
};

struct Reg {
  template <typename T>
  bool operator()(const T* const c, const T* const d, T* residual) const {
    residual[0] = T(1) - (c[0] * c[0] + d[0] * d[0]);
    return true;
  }
};

struct Reg_T {
  template <typename T>
  bool operator()(const T* const t, T* residual) const {
    residual[0] = sqrt(kTSOCSThreshold) / 2 *
                  ceres::exp(T(10) * (*t / T(t_expected + kEpsilon) - 1.4));
    residual[0] = min(residual[0], T(100));
    return true;
  }
};

void GetTimeBound() {
  // time to accelerate to rest
  const double vInitMag = current_vel.norm();
  // position after accelerating to rest, first rest point
  const Eigen::Vector2d x_rest = current_vel * vInitMag / 2;

  // time of direct acceleration to end velocity:
  const double T_l2 = (delta_vel + current_vel).norm();

  // vector from second rest point to final point:
  const Eigen::Vector2d x_prime =
      delta_vel * (delta_vel + current_vel).norm() / 2;
  // dist between two rest points
  const double x_tilda = (delta_pose - x_rest - x_prime).norm();
  double T1 = 2 * sqrt(x_tilda);  // travel time between rest points
  // total time for 1D problems and acceleration to rest
  t_max = T_l2 + T1 + vInitMag;
}

struct Xdist x_dist;
struct Vdist v_dist;
struct Xdist_polar x_dist_polar;
struct Vdist_polar v_dist_polar;
struct Xdist_double_polar x_dist_double_polar;
struct Vdist_double_polar v_dist_double_polar;
struct Reg_T reg_t;

double GetFeasibleTime(double tp, double tm, int ai, double dX, double dV,
                       double vi) {
  double tpt = 2 * tp - ai * dV;
  double xp =
      vi * tpt + ai * tp * tpt - ai * (tp * tp + (tpt - tp) * (tpt - tp)) / 2;

  if (fabs(xp - dX) < kEpsilon) {
    return tp;
  }

  return tm;
}

double GetAxisSolution(double x, double v0, double vf, int* sign_u0) {
  // switching funciton at time 0
  double x_switch_init;
  double tp, tm;

  if (0 > vf - v0) {
    x_switch_init = x + (vf * vf - v0 * v0) / 2.0;
  } else {
    x_switch_init = x - (vf * vf - v0 * v0) / 2.0;
  }

  if (0 < x_switch_init) {
    tp = -v0 + sqrt(x + (vf * vf + v0 * v0) / 2);
    tm = -v0 - sqrt(x + (vf * vf + v0 * v0) / 2);
    (*sign_u0) = 1;
    return GetFeasibleTime(tp, tm, 1, x, vf - v0, v0);
  } else if (0 > x_switch_init) {
    tp = v0 + sqrt((vf * vf + v0 * v0) / 2 - x);
    tm = v0 - sqrt((vf * vf + v0 * v0) / 2 - x);
    (*sign_u0) = -1;
    return GetFeasibleTime(tp, tm, -1, x, vf - v0, v0);
  } else {
    (*sign_u0) = 0;
    return 0;
  }
}

SolutionParameters GetNonProjectionGuess() {
  const double theta_init =
      atan2(delta_pose.y() + kEpsilonSq, delta_pose.x() + kEpsilonSq);
  const double cos_theta0 = cos(theta_init);
  const double sin_theta0 = sin(theta_init);
  int sign_uo_x, sign_uo_y;

  const Eigen::Vector2d finalVel = current_vel + delta_vel;
  const double axis1_time =
      GetAxisSolution(delta_pose.x() / cos_theta0, current_vel.x() / cos_theta0,
                      finalVel.x() / cos_theta0, &sign_uo_x);
  const double axis2_time =
      GetAxisSolution(delta_pose.y() / sin_theta0, current_vel.y() / sin_theta0,
                      finalVel.y() / sin_theta0, &sign_uo_y);
  c = sign_uo_x * cos(theta_init);
  d = sign_uo_y * sin(theta_init);
  a = -c / (axis1_time + kEpsilon);
  b = -d / (axis2_time + kEpsilon);
  if (kUseNewParams) {
    a = a * t_max;
    b = b * t_max;
  }
  if (kUsePolarParams) {
    a /= sqrt(c * c + d * d);
    b /= sqrt(c * c + d * d);
    c /= sqrt(c * c + d * d);
    d /= sqrt(c * c + d * d);
    theta = std::atan2(d, c);
  } else if (kUseDoublePolarParams) {
    a /= sqrt(c * c + d * d);
    b /= sqrt(c * c + d * d);
    c /= sqrt(c * c + d * d);
    d /= sqrt(c * c + d * d);
    theta1 = std::atan2(d, c);
    theta2 = std::atan2(d + b * t_max, c + a * t_max);
    theta3 = std::atan2(b * t_max, c * t_max) - theta1;
  }

  return SolutionParameters(a, b, c, d, t_max);
}

SolutionParameters GetProjectionGuess() {
  // project starting and ending velocities onto displacement vector
  Eigen::Vector2d delta_pose_unit = delta_pose.normalized();

  ControlSequence1D control;
  TimeOptimalControlAnyFinal1D(
      0, delta_pose_unit.dot(current_vel), delta_pose.norm(),
      delta_pose_unit.dot(current_vel + delta_vel), 0, 1, 1e10, &control);
  if (control.num_phases == 0) {
    t_max = kEpsilon;
    a = kEpsilon;
    b = kEpsilon;
    c = kEpsilon;
    d = kEpsilon;
  } else if (control.num_phases == 1) {
    t_max = control.phases[0].duration;
    c = delta_pose.x() * control.phases[0].acceleration;
    d = delta_pose.y() * control.phases[0].acceleration;
    a = delta_pose.x() * control.phases[0].acceleration;
    b = delta_pose.y() * control.phases[0].acceleration;
  } else if (control.num_phases == 2) {
    t_max = control.phases[0].duration + control.phases[1].duration;
    c = delta_pose_unit.x() * control.phases[0].duration *
        control.phases[0].acceleration;
    d = delta_pose_unit.y() * control.phases[0].duration *
        control.phases[0].acceleration;
    a = -delta_pose_unit.x() * control.phases[0].acceleration;
    b = -delta_pose_unit.y() * control.phases[0].acceleration;
  } else {
    cout << "In GetGuess, the time optimal control along the displacement "
         << "vector had " << control.num_phases << " phases, should be <= 2."
         << endl;
    cout << "delta_pose = " << delta_pose << endl;
    cout << "delta_vel = " << delta_vel << endl;
    t_max = kEpsilon;
    a = kEpsilon;
    b = kEpsilon;
    c = kEpsilon;
    d = kEpsilon;
  }
  /*
  printf("%d control phases. Switching time = %f, total time =  %f\n",
    control.num_phases, control.phases[0].duration, t_max);
  */
  if (kUseNewParams) {
    a = a * t_max;
    b = b * t_max;
  }
  if (kUsePolarParams) {
    a /= sqrt(c * c + d * d);
    b /= sqrt(c * c + d * d);
    c /= sqrt(c * c + d * d);
    d /= sqrt(c * c + d * d);
    theta = std::atan2(d, c);
  } else if (kUseDoublePolarParams) {
    a /= sqrt(c * c + d * d);
    b /= sqrt(c * c + d * d);
    c /= sqrt(c * c + d * d);
    d /= sqrt(c * c + d * d);
    theta1 = std::atan2(d, c);
    theta2 = std::atan2(d + b * t_max, c + a * t_max);
    theta3 = std::atan2(b * t_max, c * t_max) - theta1;
  }
  return SolutionParameters(a, b, c, d, t_max);
}

SolutionParameters GetGuess() { return GetNonProjectionGuess(); }

double RunStage0() { return 0; }

double RunStage1() {
  if (kRescaleGuess) {
    // scale the mean of the parameters to have abs 1
    double mean = (a + b + c + d) / 4;
    double scaling_factor = 1.0 / std::abs(mean);
    a *= scaling_factor;
    b *= scaling_factor;
    c *= scaling_factor;
    d *= scaling_factor;
  }
  static const int kMaxIterations = 100;
  ceres::Problem stage1;
  if (kUsePolarParams) {
    stage1.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Xdist_polar_constT, 2, 1, 1, 1>(
            new Xdist_polar_constT),
        NULL, &a, &b, &theta);
    ///*
    if (kUseParallelVelocityCost) {
      stage1.AddResidualBlock(
          new ceres::AutoDiffCostFunction<V_polar_stage1, 2, 1, 1, 1>(
              new V_polar_stage1),
          NULL, &a, &b, &theta);
    } else {
      stage1.AddResidualBlock(
          new ceres::AutoDiffCostFunction<Vdist_polar_constT, 2, 1, 1, 1>(
              new Vdist_polar_constT),
          NULL, &a, &b, &theta);
    }
    //*/
  } else if (kUseDoublePolarParams) {
    // NOTE: I did not add the parallel velocity coss functionality to the
    // double polar parameterization
    stage1.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Xdist_double_polar_constT, 2, 1, 1, 1>(
            new Xdist_double_polar_constT),
        NULL, &theta1, &theta2, &theta3);
    stage1.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Vdist_double_polar_constT, 2, 1, 1, 1>(
            new Vdist_double_polar_constT),
        NULL, &theta1, &theta2, &theta3);
  } else {  // normal parameterization
    stage1.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Xdist_constT, 2, 1, 1, 1, 1>(
            new Xdist_constT),
        NULL, &a, &b, &c, &d);
    if (kUseParallelVelocityCost) {
      stage1.AddResidualBlock(
          new ceres::AutoDiffCostFunction<V_stage1, 2, 1, 1, 1, 1>(
              new V_stage1),
          NULL, &a, &b, &c, &d);
    } else {
      stage1.AddResidualBlock(
          new ceres::AutoDiffCostFunction<Vdist_constT, 2, 1, 1, 1, 1>(
              new Vdist_constT),
          NULL, &a, &b, &c, &d);
    }
    if (kUseRegularization) {
      stage1.AddResidualBlock(
          new ceres::AutoDiffCostFunction<Reg, 1, 1, 1>(new Reg), NULL, &c, &d);
    }
    if (kConstantC) {
      stage1.SetParameterBlockConstant(&c);
    }
  }
  ceres::Solver::Options options;

  options.max_num_iterations = kMaxIterations;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary1;

  Solve(options, &stage1, &summary1);

  t_tot = t_max;
  return summary1.final_cost;
}

class TsocsCallback : public ceres::IterationCallback {
 public:
  TsocsCallback() {}

  virtual ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary) {
    printf("%05d, %f, %f, %f, %f, %f, %f, %f\n", summary.iteration, a, b, c, d,
           t_tot, summary.cost, summary.gradient_norm);
    return ceres::SOLVER_CONTINUE;
  }
};

double RunStage2() {
  static const bool kUseCeresCallback = false;
  if (kRescaleGuess) {
    // scale the mean of the parameters to have abs 1
    double mean = (a + b + c + d) / 4;
    double scaling_factor = 1.0 / std::abs(mean);
    a *= scaling_factor;
    b *= scaling_factor;
    c *= scaling_factor;
    d *= scaling_factor;
  }
  static const int kMaxIterations = 100;

  ceres::Solver::Options options;

  options.max_num_iterations = kMaxIterations;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;
  options.update_state_every_iteration = true;

  TsocsCallback callback;
  if (kUseCeresCallback) {
    options.callbacks.push_back(&callback);
  }
  ceres::Problem stage2;
  if (kUsePolarParams) {
    stage2.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Xdist_polar, 2, 1, 1, 1, 1>(
            new Xdist_polar),
        NULL, &a, &b, &theta, &t_tot);
    stage2.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Vdist_polar, 2, 1, 1, 1, 1>(
            new Vdist_polar),
        NULL, &a, &b, &theta, &t_tot);
  } else if (kUseDoublePolarParams) {
    stage2.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Xdist_double_polar, 2, 1, 1, 1, 1>(
            new Xdist_double_polar),
        NULL, &theta1, &theta2, &theta3, &t_tot);
    stage2.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Vdist_double_polar, 2, 1, 1, 1, 1>(
            new Vdist_double_polar),
        NULL, &theta1, &theta2, &theta3, &t_tot);
  } else {
    stage2.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Xdist, 2, 1, 1, 1, 1, 1>(new Xdist),
        NULL, &a, &b, &c, &d, &t_tot);
    stage2.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Vdist, 2, 1, 1, 1, 1, 1>(new Vdist),
        NULL, &a, &b, &c, &d, &t_tot);
    if (use_t_regularization) {
      stage2.AddResidualBlock(
          new ceres::AutoDiffCostFunction<Reg_T, 1, 1>(new Reg_T), NULL,
          &t_tot);
    }
    if (kConstantC) {
      stage2.SetParameterBlockConstant(&c);
    }
    if (kUseRegularization) {
      stage2.AddResidualBlock(
          new ceres::AutoDiffCostFunction<Reg, 1, 1, 1>(new Reg), NULL, &c, &d);
    }
  }

  stage2.SetParameterLowerBound(&t_tot, 0, 0);

  // options.log_to_stderr = true;

  ceres::Solver::Summary summary2;

  Solve(options, &stage2, &summary2);

  //           << summary2.total_time_in_seconds << std::endl;
  if (summary2.message == "Residual and Jacobian evaluation failed.") {
    /*
    std::cout << summary2.FullReport() << std::endl;
    printf("cost is %f\n", summary2.final_cost);
    */
    return kTSOCSThreshold * 2;
  }
  return summary2.final_cost;
}

// uniform distrubition between min and max, inclusive
double UniformRand(double min, double max) {
  return min + (max - min) * static_cast<double>(rand_r(&seed)) / RAND_MAX;
}

bool GetSolutionSetFromGuess(Eigen::Vector2d x0, Eigen::Vector2d v0,
                             Eigen::Vector2d xf, Eigen::Vector2d vf,
                             const double a_max,
                             std::vector<SolutionParameters>* params,
                             const SolutionParameters guess) {
  if (isnan(x0.x()) || isnan(x0.y()) || isnan(v0.x()) || isnan(v0.y()) ||
      isnan(xf.x()) || isnan(xf.y()) || isnan(vf.x()) || isnan(vf.y())) {
    printf("GetSolutionSetFromGuess was passed nan values\n");
    cout << "x0 = " << x0 << endl;
    cout << "v0 = " << v0 << endl;
    cout << "xf = " << xf << endl;
    cout << "vf = " << vf << endl;
  }
  delta_pose = (xf - x0) / a_max;
  delta_vel = (vf - v0) / a_max;
  current_vel = v0 / a_max;
  GetTimeBound();

  a = guess.a;
  b = guess.b;
  c = guess.c;
  d = guess.d;
  t_max = guess.T;
  t_tot = guess.T;

  params->push_back(SolutionParameters(a, b, c, d, t_max));
  double cost1 = RunStage1();
  SolutionParameters params_stage1;
  if (kUsePolarParams) {
    params_stage1 =
        SolutionParameters(a, b, cos(theta), sin(theta), t_tot, cost1);
  } else {
    params_stage1 = SolutionParameters(a, b, c, d, t_tot, cost1);
  }
  params->push_back(params_stage1);
  double cost2;
  if (kUsePerturbationsFix) {
    for (int i = 0; i < kMaxPerturbations; i++) {
      cost2 = RunStage2();
      if (cost2 < kTSOCSThreshold) {
        break;
      }
      // perturb parameters that resulted from stage 1
      a = params_stage1.a * exp(UniformRand(-1, 1));
      b = params_stage1.b * exp(UniformRand(-1, 1));
      c = params_stage1.c * exp(UniformRand(-1, 1));
      d = params_stage1.d * exp(UniformRand(-1, 1));
      t_tot = params_stage1.T * exp(UniformRand(-1, 1));
    }
  } else {
    cost2 = RunStage2();
  }
  SolutionParameters sol;
  if (kUsePolarParams) {
    sol = SolutionParameters(a, b, cos(theta), sin(theta), t_tot, cost2);
  } else {
    sol = SolutionParameters(a, b, c, d, t_tot, cost2);
  }
  if (cost2 < kTSOCSThreshold) {
    sol.isInitialized = true;
  } else {
    sol.isInitialized = false;
  }
  params->push_back(sol);
  if (cost2 < kTSOCSThreshold) {
    return true;
  }
  return false;
}

bool GetSolutionSet(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d xf,
                    Eigen::Vector2d vf, const double a_max,
                    std::vector<SolutionParameters>* params,
                    int* perturbations) {
  if (isnan(x0.x()) || isnan(x0.y()) || isnan(v0.x()) || isnan(v0.y()) ||
      isnan(xf.x()) || isnan(xf.y()) || isnan(vf.x()) || isnan(vf.y())) {
    printf("GetSolutionSet was passed nan values\n");
    cout << "x0 = " << x0 << endl;
    cout << "v0 = " << v0 << endl;
    cout << "xf = " << xf << endl;
    cout << "vf = " << vf << endl;
  }
  delta_pose = (xf - x0) / a_max;
  delta_vel = (vf - v0) / a_max;
  current_vel = v0 / a_max;
  t_max = GetMinimumUpperTimeBound(x0, v0, xf, vf, a_max, INFINITY);
  GetGuess();
  if (kUseDoublePolarParams) {
    params->push_back(SolutionParameters(theta1, theta2, theta3, 0, t_max));
  } else {
    params->push_back(SolutionParameters(a, b, c, d, t_max));
  }
  double cost1 = RunStage1();
  SolutionParameters params_stage1;
  if (kUsePolarParams) {
    // TODO(afischer)
    params_stage1 =
        SolutionParameters(a, b, cos(theta), sin(theta), t_tot, cost1);
  } else if (kUseDoublePolarParams) {
    params_stage1 = SolutionParameters(theta1, theta2, theta3, 0, t_tot);
  } else {
    params_stage1 = SolutionParameters(a, b, c, d, t_tot, cost1);
  }
  params_stage1.cost = cost1;
  params->push_back(params_stage1);
  double cost2;
  if (kUsePerturbationsFix) {
    int i;
    for (i = 0; i < kMaxPerturbations; i++) {
      cost2 = RunStage2();
      if (cost2 < kTSOCSThreshold && t_tot <= t_max) {
        break;
      }

      // perturb parameters that resulted from stage 1
      a = params_stage1.a * exp(UniformRand(-1, 1));
      b = params_stage1.b * exp(UniformRand(-1, 1));
      c = params_stage1.c * exp(UniformRand(-1, 1));
      d = params_stage1.d * exp(UniformRand(-1, 1));
      t_tot = params_stage1.T * exp(UniformRand(-1, 1));
    }
    if (perturbations != NULL) {
      *perturbations = i;
    }
  } else {
    cost2 = RunStage2();
  }
  SolutionParameters params_stage2;
  if (kUsePolarParams) {
    // TODO(afischer)
    params_stage2 =
        SolutionParameters(a, b, cos(theta), sin(theta), t_tot, cost2);
  } else if (kUseDoublePolarParams) {
    theta1 = AngleMod(theta1);
    theta2 = AngleMod(theta2);
    theta3 = AngleMod(theta3);
    params_stage2 = SolutionParameters(theta1, theta2, theta3, 0, t_tot);
    params_stage2.cost = cost2;
    printf("final params:\ntheta1=%f\ntheta2=%f\ntheta3=%f\nT=%f\n", theta1,
           theta2, theta3, t_tot);
  } else {
    params_stage2 = SolutionParameters(a, b, c, d, t_tot, cost2);
  }
  params_stage2.isInitialized = cost2 < kTSOCSThreshold;
  params->push_back(params_stage2);
  return cost2 < kTSOCSThreshold;
}

// Gets a solution using Ceres without 2 stages
bool GetSolutionNoStages(Eigen::Vector2d x0, Eigen::Vector2d v0,
                         Eigen::Vector2d xf, Eigen::Vector2d vf,
                         const double a_max, SolutionParameters* params) {
  if (isnan(x0.x()) || isnan(x0.y()) || isnan(v0.x()) || isnan(v0.y()) ||
      isnan(xf.x()) || isnan(xf.y()) || isnan(vf.x()) || isnan(vf.y())) {
    printf("GetSolutionNoStages was passed nan values\n");
    cout << "x0 = " << x0 << endl;
    cout << "v0 = " << v0 << endl;
    cout << "xf = " << xf << endl;
    cout << "vf = " << vf << endl;
  }
  delta_pose = (xf - x0) / a_max;
  delta_vel = (vf - v0) / a_max;
  current_vel = v0 / a_max;
  t_tot = params->T;
  if (kUsePolarParams) {
    // TODO(afischer)
  } else if (kUseDoublePolarParams) {
    a /= sqrt(c * c + d * d);
    b /= sqrt(c * c + d * d);
    c /= sqrt(c * c + d * d);
    d /= sqrt(c * c + d * d);
    theta1 = params->a;
    theta2 = params->b;
    theta3 = params->c;
  } else {
    a = params->a;
    b = params->b;
    c = params->c;
    d = params->d;
  }
  double cost = RunStage2();
  if (kUsePolarParams) {
    // TODO(afischer)
  } else if (kUseDoublePolarParams) {
    *params = SolutionParameters(theta1, theta2, theta3, 0, t_tot, cost);
    printf("GetSolutionNoStages got:\ntheta1=%f\ntheta2=%f\ntheta3=%f\nT=%f\n",
           theta1, theta2, theta3, t_tot);
  } else {
    *params = SolutionParameters(a, b, c, d, t_tot, cost);
  }
  params->isInitialized = cost < kTSOCSThreshold;
  return cost < kTSOCSThreshold;
}

bool GetSolution(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d xf,
                 Eigen::Vector2d vf, const double a_max,
                 SolutionParameters* params, logger::Logger* the_log) {
  if (isnan(x0.x()) || isnan(x0.y()) || isnan(v0.x()) || isnan(v0.y()) ||
      isnan(xf.x()) || isnan(xf.y()) || isnan(vf.x()) || isnan(vf.y())) {
    printf("GetSolution was passed nan values\n");
    cout << "x0 = " << x0 << endl;
    cout << "v0 = " << v0 << endl;
    cout << "xf = " << xf << endl;
    cout << "vf = " << vf << endl;
  }
  SolutionParameters params_old = *params;
  delta_pose = (xf - x0) / a_max;
  delta_vel = (vf - v0) / a_max;
  current_vel = v0 / a_max;

  // GetTimeBound();
  t_max = GetMinimumUpperTimeBound(x0, v0, xf, vf, a_max, INFINITY);
  SolutionParameters params_stage1(0, 0, 0, 0, 0);
  params_stage1.isInitialized = false;
  if (!params->isInitialized || params->cost >= kTSOCSThreshold) {
    if (the_log != NULL) {
      the_log->LogPrint("Initializing Parameters");
    }
    GetGuess();
    double cost1 = RunStage1();
    if (kUsePolarParams) {
      params_stage1 =
          SolutionParameters(a, b, cos(theta), sin(theta), t_tot, cost1);
    } else if (kUseDoublePolarParams) {
      params_stage1 = DoublePolarToStandard(theta1, theta2, theta3, t_tot);
    } else {
      params_stage1 = SolutionParameters(a, b, c, d, t_tot, cost1);
    }
    params_stage1.isInitialized = true;
  } else {
    if (kOpenLoop) {
      params->c += params->a * kDeltaT;
      params->d += params->b * kDeltaT;
      params->T -= kDeltaT;
      return true;
    } else {
      a = params->a;
      b = params->b;
      c = params->c;
      d = params->d;
      t_tot = params->T;
      c += a * kDeltaT;
      d += b * kDeltaT;
      t_tot -= kDeltaT;
      use_t_regularization = true;
      t_expected = params->T - kDeltaT;
      if (t_expected < 0) {
        // return solution with negative time to indicate that we are finished
        // ie, as close to the end point as we are going to get
        *params = SolutionParameters(0, 0, 0, 0, -1, 0);
        use_t_regularization = false;
        return false;
      }
      if (the_log != NULL) {
        the_log->LogPrint("Using Old Parameters as Initial Guess");
      }
    }
  }
  if (use_t_regularization) {
    double propAccel = static_cast<double>(delta_vel.norm()) / t_expected;
    propAccel = max(1.0, propAccel);

    v_cost_coef = max(pow(1 - propAccel, 1.0), min_v_cost_coef);

    if (the_log != NULL) {
      the_log->LogPrint("V prop: %f", propAccel);
      the_log->LogPrint("V cost coef: %f", v_cost_coef);
      the_log->LogPrint("X cost coef: %f", x_cost_coef);
    }
  } else {
    v_cost_coef = 1.0;
  }
  x_cost_coef = 1.0;
  double cost2 = RunStage2();
  if (cost2 >= kTSOCSThreshold) {
    if (the_log != NULL) {
      the_log->LogPrint("TSOCS failed, retrying");
      GetGuess();
      RunStage1();
      cost2 = RunStage2();
    }
  }
  if (the_log != NULL) {
    if (use_t_regularization) {
      double reg_t_cost;
      reg_t(&t_tot, &reg_t_cost);
      reg_t_cost *= reg_t_cost / 2;  // it's really 1/2 * cost^2
      the_log->LogPrint("T reg. log_10 cost: %f", log10(reg_t_cost));
      the_log->LogPrint("Expected t: %f", t_expected);
    }
    the_log->LogPrint("Actual t:   %f", t_tot);
    the_log->LogPrint("T_max:      %f", t_max);
    the_log->LogPrint("log_10 cost: %f", log10(cost2));
  }
  if (kUsePerturbationsFix && cost2 >= kTSOCSThreshold) {
    if (!params_stage1.isInitialized) {
      GetGuess();
      RunStage1();
      params_stage1 = SolutionParameters(a, b, c, d, t_tot);
      params_stage1.isInitialized = true;
    }
    int perturbations = 0;
    for (perturbations = 0;
         perturbations < kMaxPerturbations && cost2 < kTSOCSThreshold;
         perturbations++) {
      // perturb parameters that resulted from stage 1
      a = params_stage1.a * exp(UniformRand(-1, 1));
      b = params_stage1.b * exp(UniformRand(-1, 1));
      c = params_stage1.c * exp(UniformRand(-1, 1));
      d = params_stage1.d * exp(UniformRand(-1, 1));
      t_tot = params_stage1.T * exp(UniformRand(-1, 1));
      cost2 = RunStage2();
    }
    if (the_log != NULL && kUsePerturbationsFix) {
      the_log->LogPrint("Performed %d perturbations", perturbations);
    }
  }
  // fprintf(failure_fid, "%12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f\n",
  //         x0.x(), x0.y(), v0.x(), v0.y(), xf.x(), xf.y(), vf.x(), vf.y());
  // fflush(failure_fid);
  if (kWriteFailures && params->isInitialized && cost2 >= kTSOCSThreshold) {
    if (the_log != NULL) {
      the_log->LogPrint("Failed where previous problem succeeded");
    }
    // it succeeded last iteratiton but failed this iteration
//     fprintf(failure_fid, "%12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f\n",
//             x0_prev.x(), x0_prev.y(), v0_prev.x(), v0_prev.y(), xf_prev.x(),
//             xf_prev.y(), vf_prev.x(), vf_prev.y());
//     fprintf(failure_fid, "%12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f\n",
//             x0.x(), x0.y(), v0.x(), v0.y(), xf.x(), xf.y(), vf.x(), vf.y());
//     fflush(failure_fid);
  }
  if (cost2 < kTSOCSThreshold) {
    if (kUsePolarParams) {
      *params = SolutionParameters(a, b, cos(theta), sin(theta), t_tot, cost2);
    } else if (kUseDoublePolarParams) {
      *params = DoublePolarToStandard(theta1, theta2, theta3, t_tot);
    } else {
      *params = SolutionParameters(a, b, c, d, t_tot, cost2);
    }
  } else {
    *params = params_old;
    params->c += params->a * kDeltaT;
    params->d += params->b * kDeltaT;
    params->T -= kDeltaT;
  }

  if (kWriteFailures && cost2 < kTSOCSThreshold) {
    x0_prev = x0;
    v0_prev = v0;
    xf_prev = xf;
    vf_prev = vf;
  }
  if (kCollectTSOCSData) {
//     fprintf(tsocs_data_fid, "\tCOST COEFS: %f %f\n", x_cost_coef,
//             v_cost_coef);
//     fprintf(tsocs_data_fid, "\tT REGULARIZATION: %s\n",
//             use_t_regularization ? "TRUE" : "FALSE");
//     if (use_t_regularization) {
//       fprintf(tsocs_data_fid, "\tEXPECTED T: %f\n", t_expected);
//     }
  }
  use_t_regularization = false;
  x_cost_coef = 1.0;
  v_cost_coef = 1.0;
  // params->isInitialized = cost2 < kTSOCSThreshold;
  if (the_log != NULL) {
    the_log->LogPrint("Time = %f", params->T);
  }
  params->isInitialized = true;
  return (cost2 < kTSOCSThreshold);
}

void GetState(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d* xt,
              Eigen::Vector2d* vt, const double a_max, const double t,
              SolutionParameters params) {
  if (kUseNewParams) {
    a = params.a / params.T;
    b = params.b / params.T;
    c = params.c;
    d = params.d;
  } else if (kUsePolarParams) {
    // TODO(afischer)
  } else if (kUseDoublePolarParams) {
    theta1 = params.a;
    theta2 = params.b;
    theta3 = params.c;
    if (params.T == 0) {
      a = 1;
      b = 1;
    } else {
      a = (sin(theta1) - tan(theta1 + theta3) * cos(theta1)) /
          (tan(theta2) - tan(theta1 + theta3) + kEpsilon);
      b = a * tan(theta2);
      // at this point (a,b) hold the endpoints of the adjoint line.
      // Now I convert them to rates of change
      a = (a - cos(theta1)) / params.T;
      b = (b - sin(theta1)) / params.T;
    }
    c = cos(theta1);
    d = sin(theta1);
  } else {
    a = params.a;
    b = params.b;
    c = params.c;
    d = params.d;
  }
  (*xt)[0] = X(a, b, c, d, t) * a_max + v0.x() * t + x0.x();
  (*xt)[1] = X(b, a, d, c, t) * a_max + v0.y() * t + x0.y();

  (*vt)[0] = V(a, b, c, d, t) * a_max + v0.x();
  (*vt)[1] = V(b, a, d, c, t) * a_max + v0.y();
}

void GetState(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d* xt,
              Eigen::Vector2d* vt, const double a_max, const double t,
              SolutionParameters params, logger::Logger* the_log) {
  if (kUseNewParams) {
    a = params.a / params.T;
    b = params.b / params.T;
  } else {
    a = params.a;
    b = params.b;
  }
  c = params.c;
  d = params.d;

  (*xt)[0] = X(a, b, c, d, t) * a_max + v0.x() * t + x0.x();
  (*xt)[1] = X(b, a, d, c, t) * a_max + v0.y() * t + x0.y();
  the_log->LogPrint("Expected Pos: (%f, %f)", (*xt)[0], (*xt)[1]);

  (*vt)[0] = V(a, b, c, d, t) * a_max + v0.x();
  (*vt)[1] = V(b, a, d, c, t) * a_max + v0.y();
  the_log->LogPrint("Expected Vel: (%f, %f)", (*vt)[0], (*vt)[1]);
  the_log->LogPrint("Desired Delta Velocity: (%f, %f)",
                    V(a, b, c, d, t) * a_max, V(b, a, d, c, t) * a_max);
}

std::vector<Eigen::Vector2f> GetPath(Eigen::Vector2d x0, Eigen::Vector2d v0,
                                     const double a_max,
                                     const unsigned int npts,
                                     SolutionParameters params) {
  Eigen::Vector2f x_prev;
  Eigen::Vector2f v_prev;

  const double delta_t = params.T / npts;

  std::vector<Eigen::Vector2f> path_points;

  for (unsigned int i = 1; i <= npts; i++) {
    Eigen::Vector2d x_t;
    Eigen::Vector2d v_t;

    GetState(x0, v0, &x_t, &v_t, a_max, i * delta_t, params);
    path_points.push_back(x_t.cast<float>());
  }
  return path_points;
}

SolutionParameters DoublePolarToStandard(double theta1, double theta2,
                                         double theta3, double T) {
  double c = cos(theta1), d = sin(theta1);
  double a = (sin(theta1) - tan(theta1 + theta3) * cos(theta1)) /
             (tan(theta2) - tan(theta1 + theta3));
  double b = a * tan(theta2);
  // at this point (a,b) hold the endpoints of the adjoint line.
  // Now I convert them to rates of change
  a = (a - c) / T;
  b = (b - d) / T;
  return SolutionParameters(a, b, c, d, T, 0);
}

void StandardToDoublePolar(SolutionParameters params, double* theta1,
                           double* theta2, double* theta3) {
  double a = params.a, b = params.b, c = params.c, d = params.d, T = params.T;
  a /= sqrt(c * c + d * d);
  b /= sqrt(c * c + d * d);
  c /= sqrt(c * c + d * d);
  d /= sqrt(c * c + d * d);
  *theta1 = std::atan2(d, c);
  *theta2 = std::atan2(d + b * T, c + a * T);
  *theta3 = std::atan2(b * T, c * T) - *theta1;
}

double CostFunction(double a, double b, double c, double d, double T) {
  double residual1[2], residual2[2];
  // g++ complains if residual arrays aren't initialized
  residual1[0] = 0;
  residual1[1] = 0;
  residual2[0] = 0;
  residual2[1] = 0;
  x_dist(&a, &b, &c, &d, &T, residual1);
  v_dist(&a, &b, &c, &d, &T, residual2);
  return residual1[0] * residual1[0] + residual1[1] * residual1[1] +
         residual2[0] * residual2[0] + residual2[1] * residual2[1];
}

double CostFunctionPolar(double a, double b, double theta, double T) {
  double residual1[2], residual2[2];
  // g++ complains if residual arrays aren't initialized
  residual1[0] = 0;
  residual1[1] = 0;
  residual2[0] = 0;
  residual2[1] = 0;
  x_dist_polar(&a, &b, &theta, &T, residual1);
  v_dist_polar(&a, &b, &theta, &T, residual2);
  return residual1[0] * residual1[0] + residual1[1] * residual1[1] +
         residual2[0] * residual2[0] + residual2[1] * residual2[1];
}

double CostFunctionDoublePolar(double theta1, double theta2, double theta3,
                               double T) {
  double residual1[2] = {0, 0}, residual2[2] = {0, 0};
  x_dist_double_polar(&theta1, &theta2, &theta3, &T, residual1);
  v_dist_double_polar(&theta1, &theta2, &theta3, &T, residual2);
  return residual1[0] * residual1[0] + residual1[1] * residual1[1] +
         residual2[0] * residual2[0] + residual2[1] * residual2[1];
}

bool TSOCSFinished(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d xf,
              Eigen::Vector2d vf, const double a_max,
              SolutionParameters* params) {
  if ((x0 - xf).norm() < 5.0 && (v0 - vf).norm() < 50.0) {
    return true;
  }
  return params->T <= 0;
}

}  // namespace tsocs
