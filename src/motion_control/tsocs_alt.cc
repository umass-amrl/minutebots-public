// Copyright 2017 dbalaban@cs.umass.edu
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

#include <vector>

#include "eigen3/Eigen/Dense"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/tsocs_alt.h"

namespace temp {

// static const double kEpsilonSq = kEpsilon * kEpsilon;
static const double kScale = 10.0;
static const double kThreshold = 1e-6;

double t_max;
Eigen::Vector2d deltaPose;
Eigen::Vector2d deltaVel;
Eigen::Vector2d currentVel;

double theta_0, theta_T, r, T, R;

template <typename T>
T F1(const T theta_0, const T theta_T, const T r, const T total_time) {
  return (sqrt(r * r + T(1) - T(2) * r * cos(theta_T - theta_0)) / total_time);
}

template <typename T>
T F2(const T theta_0, const T theta_T, const T r, const T total_time) {
  return (r * cos(theta_T - theta_0) - T(1)) / total_time;
}

template <typename T>
T F3(const T theta_0, const T theta_T, const T r, const T total_time) {
  return r * sin(theta_T - theta_0) / total_time;
}

template <typename T>
T G1(const T f1, const T f2, const T t) {
  T g1 = sqrt(t * t * f1 * f1 + T(2) * t * f2 + T(1));
  return g1;
}

template <typename T>
T G2(const T g1, const T f1, const T f2, const T t) {
  T g2 = f1 * g1 + t * f1 * f1 + f2;
  return g2;
}

template <typename T, const int N>
T PowN(const T& x) {
  T result = T(1.0);
  for (int i = 0; i < N; ++i) {
    result = result * x;
  }
  return result;
}

template <typename T> T Abs(const T& x) {
  if (ceres::abs(x) > T(kEpsilon)) {
    return ceres::abs(x);
  } else {
    return 0.5 * x * x / T(kEpsilon);
  }
}

template <typename T> T Sign(const T& x) {
  if (x < T(0)) {
    return T(-1);
  } else if (x == T(0)) {
    return T(0);
  } else {
    return T(1);
  }
}

//  to convert to y axis, subtract pi/2 from each angle
template <typename T>
T X(const T theta_0, const T theta_T, const T r, const T total_time,
    const T t) {
  const T a = (r * cos(theta_T) - cos(theta_0)) / total_time;
  const T b = (r * sin(theta_T) - sin(theta_0)) / total_time;
  const T f1 = F1(theta_0, theta_T, r, total_time);
  const T f2 = F2(theta_0, theta_T, r, total_time);
  const T f3 = F3(theta_0, theta_T, r, total_time);
  const T g1 = G1(f1, f2, t);
  const T g2 = G2(g1, f1, f2, t);

  const T apart = a * (g1 * (f1 * f2 + t * PowN<T, 3>(f1)) +
                       f3 * f3 * log(g2 / (f1 + f2)) - f1 * f2) /
                  (T(2) * PowN<T, 5>(f1));
  const T bpart =
      (b * f3 / PowN<T, 3>(f1)) *
          ((t + (f2 / (f1 * f1))) * log(g2 / (f1 + f2)) - g1 / f1 + T(1) / f1) -
      a * t / (f1 * f1);

  const T sum = apart + bpart;
  return sum;
}

//  to convert to y axis, subtract pi/2 from each angle
template <typename T>
T V(const T theta_0, const T theta_T, const T r, const T total_time,
    const T t) {
  const T a = (r * cos(theta_T) - cos(theta_0)) / total_time;
  const T b = (r * sin(theta_T) - sin(theta_0)) / total_time;
  const T f1 = F1(theta_0, theta_T, r, total_time);
  const T f2 = F2(theta_0, theta_T, r, total_time);
  const T f3 = F3(theta_0, theta_T, r, total_time);
  const T g1 = G1(f1, f2, t);
  const T g2 = G2(g1, f1, f2, t);
  return (a * (g1 - T(1)) / (f1 * f1) +
          b * f3 * log(g2 / (f1 + f2)) / (f1 * f1 * f1));
}

struct Xdist {
  template <typename T>
  bool operator()(const T* theta_0, const T* theta_T,
                  const T* r, const T* R,
                  const T* total_time, T* residual) const {
    const double vInit1 = currentVel.x(), vInit2 = currentVel.y(),
                 deltaX1 = deltaPose.x(), deltaX2 = deltaPose.y();

    if (*total_time <= T(0)) {
      return false;
    }

    T angle_diff = theta_T[0] - theta_0[0];
    T length = sqrt(r[0] * r[0] + R[0] * R[0]
        - T(2) * r[0] * R[0] * cos(angle_diff));
    T cos_diff = r[0] * cos(theta_T[0]) - R[0] * cos(theta_0[0]);
    T sin_diff = r[0] * sin(theta_T[0]) - R[0] * sin(theta_0[0]);

    T x1 = cos_diff * (Abs<T>(r[0]) - Abs<T>(R[0]))
        * (length * (r[0] * cos(angle_diff) - Abs<T>(R[0]))
        + PowN<T, 3>(length)) / T(2);

    T x2 = -sin_diff * (Abs<T>(r[0]) - Abs<T>(R[0]))
        * (length * (r[0] * cos(angle_diff) - Abs<T>(R[0]))
        + PowN<T, 3>(length)) / T(2);

    bool didCalculateLog = false;
    T signed_cos_dist = ceres::abs(cos(angle_diff) - T(-1));
    T cos_phi = (R[0] * R[0] + length * length - r[0] * r[0])
        / (T(2) * Abs<T>(R[0]) * length);
    if (signed_cos_dist > T(kEpsilon) && cos_phi > T(-1)) {
      didCalculateLog = true;

      T numerator = Abs<T>(r[0]) + Abs<T>(R[0]) * cos_phi + length;
      T denominator = Abs<T>(R[0]) * (T(1) + cos_phi);

      T log_term = log(numerator / denominator);

      T sinSq = sin(angle_diff) * sin(angle_diff);
      x1 = x1 + cos_diff * log_term * r[0] * r[0] * sinSq / T(2)
          + r[0] * sin_diff * sin(angle_diff) * ((length * length
          + r[0] * cos(angle_diff) - T(1)) * log_term
          - length * (Abs<T>(r[0]) - T(1)));

      x2 = x2 + sin_diff * log_term * r[0] * r[0] * sinSq / T(2)
          - r[0] * cos_diff * sin(angle_diff) * ((length * length
          + r[0] * cos(angle_diff) - T(1)) * log_term
          - length * (Abs<T>(r[0]) - T(1)));
    }

    x1 = x1 * total_time[0] * total_time[0] / PowN<T, 5>(length)
        + T(vInit1) * total_time[0];
    x2 = x2 * total_time[0] * total_time[0] / PowN<T, 5>(length)
        + T(vInit2) * total_time[0];

    if (ceres::IsNaN(x2)) {
      T numerator = Abs<T>(r[0]) + Abs<T>(R[0]) * cos_phi + length;
      T denominator = Abs<T>(R[0]) * (T(1) + cos_phi);
      std::printf("\nx2 is nan \n");
      std::cout << "theta_0 = " << theta_0[0] << std::endl;
      std::cout << "theta_T = " << theta_T[0] << std::endl;
      std::cout << "r = " << r[0] << std::endl;
      std::cout << "R = " << R[0] << std::endl;
      std::cout << "T = " << total_time[0] << std::endl;
      std::cout << "angle_diff = " << angle_diff << std::endl;
      std::cout << "sin(angle_diff) = " << sin(angle_diff) << std::endl;
      std::cout << "cos(angle_diff) = " << cos(angle_diff) << std::endl;
      std::cout << "length = " << length << std::endl;
      std::cout << "cos_diff = " << cos_diff << std::endl;
      std::cout << "sin_diff = " << sin_diff << std::endl;
      std::cout << "cos_phi = " << cos_phi << std::endl;
      std::cout << "numerator = " << numerator << std::endl;
      std::cout << "denominator = " << denominator << std::endl;
      std::cout << "signed cosine distance: " << signed_cos_dist << std::endl;
      if (didCalculateLog) {
        std::cout << "calculated log term\n";
      }
    }

    residual[0] = x1 - T(deltaX1);
    residual[1] = x2 - T(deltaX2);

    return true;
  }
};

struct Xdist_constT {
  template <typename T>
  bool operator()(const T* theta_0, const T* theta_T,
                  const T* r, const T* R,
                  T* residual) const {
    const double vInit1 = currentVel.x(), vInit2 = currentVel.y(),
                 deltaX1 = deltaPose.x(), deltaX2 = deltaPose.y();
    T angle_diff = theta_T[0] - theta_0[0];
    T length = sqrt(r[0] * r[0] + R[0] * R[0]
        - T(2) * r[0] * R[0] * cos(angle_diff));
    T cos_diff = r[0] * cos(theta_T[0]) - R[0] * cos(theta_0[0]);
    T sin_diff = r[0] * sin(theta_T[0]) - R[0] * sin(theta_0[0]);
    T total_time = T(t_max);

    T x1 = cos_diff * (Abs<T>(r[0]) - T(1))
        * (length * (r[0] * cos(angle_diff) - T(1))
        + PowN<T, 3>(length)) / T(2);

    T x2 = -sin_diff * (Abs<T>(r[0]) - T(1))
        * (length * (r[0] * cos(angle_diff) - T(1))
        + PowN<T, 3>(length)) / T(2);

    T cos_phi = (R[0] * R[0] + length * length - r[0] * r[0])
        / (T(2) * Abs<T>(R[0]) * length);
    if (ceres::abs(cos(angle_diff) - T(-1)) > T(kEpsilon)
        && cos_phi > T(-1)) {
      T numerator = Abs<T>(r[0]) + Abs<T>(R[0]) * cos_phi + length;
      T denominator = Abs<T>(R[0]) * (T(1) + cos_phi);

      T log_term = log(numerator / denominator);

      T sinSq = sin(angle_diff) * sin(angle_diff);
      x1 = x1 + cos_diff * log_term * r[0] * r[0] * sinSq / T(2)
          + r[0] * sin_diff * sin(angle_diff) * ((length * length
          + r[0] * cos(angle_diff) - T(1)) * log_term
          - length * (Abs<T>(r[0]) - T(1)));

      x2 = x2 + sin_diff * log_term * r[0] * r[0] * sinSq / T(2)
          - r[0] * cos_diff * sin(angle_diff) * ((length * length
          + r[0] * cos(angle_diff) - T(1)) * log_term
          - length * (Abs<T>(r[0]) - T(1)));
    }

    x1 = x1 * total_time * total_time / PowN<T, 5>(length)
        + T(vInit1) * total_time;
    x2 = x2 * total_time * total_time / PowN<T, 5>(length)
        + T(vInit2) * total_time;

    residual[0] = x1 - T(deltaX1);
    residual[1] = x2 - T(deltaX2);

    return true;
  }
};

struct Vdist {
  template <typename T>
  bool operator()(const T* theta_0, const T* theta_T,
                  const T* r, const T* R,
                  const T* total_time, T* residual) const {
    const double deltaV1 = deltaVel.x(), deltaV2 = deltaVel.y();

    if (*total_time <= T(0)) {
      return false;
    }

    T angle_diff = theta_T[0] - theta_0[0];
    T length = sqrt(r[0] * r[0] + R[0] * R[0]
        - T(2) * r[0] * R[0] * cos(angle_diff));
    T cos_diff = r[0] * cos(theta_T[0]) - R[0] * cos(theta_0[0]);
    T sin_diff = r[0] * sin(theta_T[0]) - R[0] * sin(theta_0[0]);

    T v1 = (Abs<T>(r[0]) - Abs<T>(R[0])) * total_time[0] * cos_diff
        / (length * length);
    T v2 = (Abs<T>(r[0]) - Abs<T>(R[0])) * total_time[0] * sin_diff
        / (length * length);

    T cos_phi = (R[0] * R[0] + length * length - r[0] * r[0])
        / (T(2) * Abs<T>(R[0]) * length);
    if (ceres::abs(cos(angle_diff) - T(-1)) > T(kEpsilon)
        && cos_phi > T(-1)) {
      T numerator = Abs<T>(r[0]) + Abs<T>(R[0]) * cos_phi + length;
      T denominator = Abs<T>(R[0]) * (T(1) + cos_phi);

      v1 = v1 + r[0] * Abs<T>(R[0]) * total_time[0] * sin_diff * sin(angle_diff)
          * log(numerator / denominator) / PowN<T, 3>(length);

      v2 = v2 - r[0] * Abs<T>(R[0]) * total_time[0] * cos_diff * sin(angle_diff)
          * log(numerator / denominator) / PowN<T, 3>(length);
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

struct V_stage1 {
  template <typename T>
  bool operator()(const T* theta_0, const T* theta_T,
                  const T* r, const T* R,
                  T* residual) const {
    const double vInit1 = currentVel.x(), vInit2 = currentVel.y(),
                 deltaV1 = deltaVel.x(), deltaV2 = deltaVel.y(),
                 deltaX1 = deltaPose.x(), deltaX2 = deltaPose.y();

    T angle_diff = theta_T[0] - theta_0[0];
    T length = sqrt(r[0] * r[0] + R[0] * R[0]
        - T(2) * r[0] * R[0] * cos(angle_diff));
    T cos_diff = r[0] * cos(theta_T[0]) - R[0] * cos(theta_0[0]);
    T sin_diff = r[0] * sin(theta_T[0]) - R[0] * sin(theta_0[0]);
    T total_time = T(t_max);

    T v1 = (Abs<T>(r[0]) - Abs<T>(R[0])) * total_time * cos_diff
        / (length * length);
    T v2 = (Abs<T>(r[0]) - Abs<T>(R[0])) * total_time * sin_diff
        / (length * length);

    T cos_phi = (R[0] * R[0] + length * length - r[0] * r[0])
        / (T(2) * Abs<T>(R[0]) * length);
    if (ceres::abs(cos(angle_diff) - T(-1)) > T(kEpsilon)
        && cos_phi > T(-1)) {
      T numerator = Abs<T>(r[0]) + Abs<T>(R[0]) * cos_phi + length;
      T denominator = Abs<T>(R[0]) * (T(1) + cos_phi);

      v1 = v1 + r[0] * Abs<T>(R[0]) * total_time * sin_diff * sin(angle_diff)
          * log(numerator / denominator) / PowN<T, 3>(length);

      v2 = v2 - r[0] * Abs<T>(R[0]) * total_time * cos_diff * sin(angle_diff)
          * log(numerator / denominator) / PowN<T, 3>(length);
    }

    T v1f = T(vInit1) + v1;
    T v2f = T(vInit2) + v2;
    T vf_mag = v1f * v1f + v2f * v2f;
    T target_v1f = T(deltaV1 + vInit1);
    T target_v2f = T(deltaV2 + vInit2);
    T target_v2f_mag = target_v1f * target_v1f + target_v2f * target_v2f;

    if ( target_v2f_mag > T(kEpsilon) ) {
      residual[0] = T(kScale) * ((v1f * target_v1f + v2f * target_v2f) /
          (sqrt(target_v2f_mag * vf_mag) + T(kEpsilon)) - T(1));
    } else {
      // time to reach rest (screeching halt)
      const T vInitMag = T(sqrt(vInit1 * vInit1 + vInit2 * vInit2));
      // displacement vector after reaching rest point
      T x1 = T(deltaX1) -  T(.5 * vInit1) * vInitMag;
      T x2 = T(deltaX2) -  T(.5 * vInit2) * vInitMag;
      const T norm = sqrt(x1 * x1 + x2 * x2);
      x1 = x1 / norm;
      x2 = x2 / norm;
      residual[0] = T(0);
      residual[1] = sqrt(vf_mag) /  T(kScale);
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

void GetTimeBound_alt() {
  // time to accelerate to rest
  const double vInitMag = currentVel.norm();
  // position after accelerating to rest, first rest point
  const Eigen::Vector2d x_rest = currentVel * vInitMag / 2;

  // time of direct acceleration to end velocity:
  const double T_l2 = (deltaVel + currentVel).norm();

  // vector from second rest point to final point:
  const Eigen::Vector2d x_prime = deltaVel * (deltaVel + currentVel).norm() / 2;
  // dist between two rest points
  const double x_tilda = (deltaPose - x_rest - x_prime).norm();
  double T1 = 2 * sqrt(x_tilda);  // travel time between rest points
  // total time for 1D problems and acceleration to rest
  t_max = T_l2 + T1 + vInitMag;
}

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

double GetAxisSolution(double x, double v0, double vf) {
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
    return GetFeasibleTime(tp, tm, 1, x, vf - v0, v0);
  } else if (0 > x_switch_init) {
    tp = v0 + sqrt((vf * vf + v0 * v0) / 2 - x);
    tm = v0 - sqrt((vf * vf + v0 * v0) / 2 - x);
    return GetFeasibleTime(tp, tm, -1, x, vf - v0, v0);
  } else {
    return 0;
  }
}

void GetGuess() {
  const Eigen::Vector2d finalVel = currentVel + deltaVel;

  if (finalVel.norm() == 0 && currentVel.norm() == 0) {
    theta_0 = atan2(deltaPose.y(), deltaPose.x());
    theta_T = theta_0;
    r = 10.0;
    R = 1.0;
    return;
  }

  theta_0 = atan2(deltaVel.y(), deltaVel.x());
  theta_T = atan2(finalVel.y(), finalVel.x());
  r = 10.0;
  R = 1.0;
}

double RunStage0() { return 0; }

double RunStage1() {
  static const int kMaxIterations = 100;
  ceres::Problem stage1;
  stage1.AddResidualBlock(
      new ceres::AutoDiffCostFunction<Xdist_constT, 2, 1, 1, 1, 1>(
          new Xdist_constT),
      NULL, &theta_0, &theta_T, &r, &R);
  stage1.AddResidualBlock(
      new ceres::AutoDiffCostFunction<V_stage1, 2, 1, 1, 1, 1>(
          new V_stage1), NULL,
      &theta_0, &theta_T, &r, &R);

  ceres::Solver::Options options;

  options.max_num_iterations = kMaxIterations;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary1;

  Solve(options, &stage1, &summary1);

  T = t_max;
  return summary1.final_cost;
}

double RunStage2() {
  static const int kMaxIterations = 100;

  ceres::Solver::Options options;

  options.max_num_iterations = kMaxIterations;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;

  ceres::Problem stage2;
  stage2.AddResidualBlock(
      new ceres::AutoDiffCostFunction<Xdist, 2, 1, 1, 1, 1, 1>(new Xdist), NULL,
      &theta_0, &theta_T, &r, &R, &T);
  stage2.AddResidualBlock(
      new ceres::AutoDiffCostFunction<Vdist, 2, 1, 1, 1, 1, 1>(new Vdist), NULL,
      &theta_0, &theta_T, &r, &R, &T);

  stage2.SetParameterLowerBound(&T, 0, 0);
  // stage2.SetParameterLowerBound(&r, 0, 0);

  ceres::Solver::Summary summary2;

  Solve(options, &stage2, &summary2);

  return summary2.final_cost;
}

bool GetSolutionSet_alt(Eigen::Vector2d x0, Eigen::Vector2d v0,
                        Eigen::Vector2d xf, Eigen::Vector2d vf,
                        const double a_max,
                        std::vector<SolutionParameters_alt>* params) {
  deltaPose = (xf - x0) / a_max;
  deltaVel = (vf - v0) / a_max;
  currentVel = v0 / a_max;

  // GetTimeBound();
  t_max = GetMinimumUpperTimeBound(x0, v0, xf, vf, a_max, INFINITY);
  GetGuess();
  params->push_back(SolutionParameters_alt(theta_0, theta_T, r, t_max));

  double cost1 = RunStage1();

  params->push_back(SolutionParameters_alt(theta_0, theta_T, r, T));
  /**
  if(cost1 > kThreshold){
    const SolutionParameters p(a, b, c, d, t_tot);
    *params = p;
    return false;
  }
  */
  double cost2 = RunStage2();
  //             << c << ", " << d << ", " << t_tot << std::endl;

  // const SolutionParameters p(a, b, c, d, t_tot);
  params->push_back(SolutionParameters_alt(theta_0, theta_T, r, T));

  if (cost2 < kThreshold) {
    return true;
  }
  std::cout << " costs: (" << cost1 << ", " << cost2 << ")\n";
  return false;
}

bool GetSolution_alt(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d xf,
                     Eigen::Vector2d vf, const double a_max,
                     SolutionParameters_alt* params) {
  deltaPose = (xf - x0) / a_max;
  deltaVel = (vf - v0) / a_max;
  currentVel = v0 / a_max;

  // GetTimeBound();
  t_max = GetMinimumUpperTimeBound(x0, v0, xf, vf, a_max, INFINITY);

  double cost1 = 0;
  if (!params->isInitialized) {
    GetGuess();
    cost1 = RunStage1();
  } else {
    theta_0 = params->theta_0;
    theta_T = params->theta_T;
    r = params->r;
  }

  double cost2 = RunStage2();

  *params = SolutionParameters_alt(theta_0, theta_T, r, T);

  if (cost2 < kThreshold) {
    params->isInitialized = true;
    return true;
  }
  params->isInitialized = false;
  std::cout << " costs: (" << cost1 << ", " << cost2 << ")\n";
  return false;
}

bool GetSolution_alt(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d xf,
                     Eigen::Vector2d vf, const double a_max,
                     SolutionParameters_alt* params, logger::Logger* the_log) {
  deltaPose = (xf - x0) / a_max;
  deltaVel = (vf - v0) / a_max;
  currentVel = v0 / a_max;

  // GetTimeBound();
  t_max = GetMinimumUpperTimeBound(x0, v0, xf, vf, a_max, INFINITY);

  double cost1 = 0;
  if (!params->isInitialized) {
    the_log->LogPrint("Initializing Parameters");
    GetGuess();
    cost1 = RunStage1();
  } else {
    the_log->LogPrint("Using Old Parameters as Initial Guess");
    theta_0 = params->theta_0;
    theta_T = params->theta_T;
    r = params->r;
  }

  double cost2 = RunStage2();
  the_log->LogPrint("TSOCS Final Cost: %f", cost2);

  *params = SolutionParameters_alt(theta_0, theta_T, r, T);

  if (cost2 < kThreshold) {
    params->isInitialized = true;
    return true;
  }

  if (params->isInitialized) {
    the_log->LogPrint("Retyring Without Initial Guess");
    GetGuess();
    RunStage1();
    double cost3 = RunStage2();
    the_log->LogPrint("TSOCS Second Try Cost: %f", cost3);
    if (cost3 < cost2) {
      *params = SolutionParameters_alt(theta_0, theta_T, r, T);
    }
    if (cost3 < kThreshold) {
      params->isInitialized = true;
      return true;
    }
  }

  params->isInitialized = false;
  std::cout << " costs: (" << cost1 << ", " << cost2 << ")\n";
  return false;
}

void GetState_alt(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d* xt,
                  Eigen::Vector2d* vt, const double a_max, const double t,
                  SolutionParameters_alt params) {
  theta_0 = params.theta_0;
  theta_T = params.theta_T;
  double r = params.r;

  (*xt)[0] = X(theta_0, theta_T, r, params.T, t) * a_max + v0.x() * t + x0.x();
  (*xt)[1] = X(theta_0 - M_PI / 2, theta_T - M_PI / 2, r, params.T, t) * a_max +
             v0.y() * t + x0.y();

  (*vt)[0] = V(theta_0, theta_T, r, params.T, t) * a_max + v0.x();
  (*vt)[1] = V(theta_0 - M_PI / 2, theta_T - M_PI / 2, r, params.T, t) * a_max +
             v0.y();
}

void GetState_alt(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d* xt,
                  Eigen::Vector2d* vt, const double a_max, const double t,
                  SolutionParameters_alt params, logger::Logger* the_log) {
  theta_0 = params.theta_0;
  theta_T = params.theta_T;
  double r = params.r;

  (*xt)[0] = X(theta_0, theta_T, r, params.T, t) * a_max + v0.x() * t + x0.x();
  (*xt)[1] = X(theta_0 - M_PI / 2, theta_T - M_PI / 2, r, params.T, t) * a_max +
             v0.y() * t + x0.y();
  the_log->LogPrint("Expected Pos: (%f, %f)", (*xt)[0], (*xt)[1]);

  (*vt)[0] = V(theta_0, theta_T, r, params.T, t) * a_max + v0.x();
  (*vt)[1] = V(theta_0 - M_PI / 2, theta_T - M_PI / 2, r, params.T, t) * a_max +
             v0.y();
  the_log->LogPrint("Expected Vel: (%f, %f)", (*vt)[0], (*vt)[1]);
  the_log->LogPrint(
      "Desired Delta Velocity: (%f, %f)",
      V(theta_0, theta_T, r, params.T, t) * a_max,
      V(theta_0 - M_PI / 2, theta_T - M_PI / 2, r, params.T, t) * a_max);
}

std::vector<Eigen::Vector2f> GetPath_alt(Eigen::Vector2d x0, Eigen::Vector2d v0,
                                         const double a_max,
                                         const unsigned int npts,
                                         SolutionParameters_alt params) {
  Eigen::Vector2f x_prev;
  Eigen::Vector2f v_prev;

  const double delta_t = params.T / npts;

  std::vector<Eigen::Vector2f> path_points;

  for (unsigned int i = 1; i <= npts; i++) {
    Eigen::Vector2d x_t;
    Eigen::Vector2d v_t;

    GetState_alt(x0, v0, &x_t, &v_t, a_max, i * delta_t, params);
    path_points.push_back(x_t.cast<float>());
  }
  return path_points;
}

}  // namespace temp


