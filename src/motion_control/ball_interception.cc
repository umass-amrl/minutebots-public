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

#include <vector>

#include "eigen3/Eigen/Dense"
#include "motion_control/ball_interception.h"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/tsocs_old.h"

using Eigen::Vector2d;
using ceres::abs;

static const double kEpsilonSq = kEpsilon * kEpsilon;

Eigen::Vector2d robot_pos0;
Eigen::Vector2d robot_vel0;
Eigen::Vector2d ball_pos0;
Eigen::Vector2d ball_dir;

double ball_vel0;
double ball_accel;

template <typename T>
T F1(const T a, const T b) {
  return (sqrt(a * a + b * b));
}

template <typename T>
T F2(const T a, const T b, const T c, const T d) {
  return a * c + b * d;
}

template <typename T>
T F3(const T a, const T b, const T c, const T d) {
  return b * c - a * d;
}

template <typename T>
T F5(const T f1, const T f4, const T f2) {
  T f5 = f1 * f4 + f2;
  if (f5 == T(0)) {
    return T(kEpsilonSq);
  } else if (f5 < T(0)) {
    return std::abs<T>(f5);
  }
  return f5;
}

template <typename T>
T G1(const T f1, const T f2, const T f4, const T t) {
  T g1 = sqrt(t * t * f1 * f1 + T(2) * t * f2 + f4 * f4);
  return g1;
}

template <typename T>
T G2(const T g1, const T f1, const T f2, const T t) {
  T g2 = f1 * g1 + t * f1 * f1 + f2;
  if (g2 == T(0)) {
    return T(kEpsilonSq);
  } else if (g2 < T(0)) {
    return std::abs<T>(g2);
  }
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

template <typename T>
T X(const T a, const T b, const T c, const T d, const T t) {
  const T f1 = F1(a, b);
  const T f2 = F2(a, b, c, d);
  const T f3 = F3(a, b, c, d);
  const T f4 = F1(c, d);
  const T f5 = F5(f1, f4, f2);
  const T g1 = G1(f1, f2, f4, t);
  const T g2 = G2(g1, f1, f2, t);

  const T apart = a * (g1 * (f1 * f2 + t * PowN<T, 3>(f1)) + f3 * f3 * log(g2) -
                       f1 * f2 * f4 - f3 * f3 * log(f5)) /
                  (T(2) * PowN<T, 5>(f1));
  const T bpart = b * f3 * (log(g2) * (f1 * f1 * t + f2) - f1 * g1 + f1 * f4 -
                            f2 * log(f5)) /
                  PowN<T, 5>(f1);
  const T tpart = t * (a * f4 / (f1 * f1) + b * f3 * log(f5) / PowN<T, 3>(f1));

  const T sum = apart + bpart - tpart;
  return sum;
}

template <typename T>
T V(const T a, const T b, const T c, const T d, const T t) {
  const T f1 = F1(a, b);
  const T f2 = F2(a, b, c, d);
  const T f3 = F3(a, b, c, d);
  const T f4 = F1(c, d);
  const T f5 = F5(f1, f4, f2);
  const T g1 = G1(f1, f2, f4, t);
  const T g2 = G2(g1, f1, f2, t);
  const T result = (a * g1 / (f1 * f1) + b * f3 * log(g2) / (f1 * f1 * f1) -
                    a * f4 / (f1 * f1) - b * f3 * log(f5) / (f1 * f1 * f1));
  //   if (ceres::IsNaN(result)) {
  //     LOG(WARNING) << "f1 = " << f1 << ", f2 = " << f2 << ", f3 = " << f3
  //                  << ", f4 = " << f4 << ", f5 = " << f5 << ", g1 = " << g1
  //                  << ", g2 = " << g2 << ", t = " << t << "\n";
  //   }
  return result;
}

struct InterceptDist {
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const c,
                  const T* const d, const T* const t, T* residual) const {
    if (*t <= T(0)) {
      return false;
    }

    T x1 = X(*a, *b, *c, *d, *t) + T(robot_vel0.x()) * t[0];
    T x2 = X(*b, *a, *d, *c, *t) + T(robot_vel0.y()) * t[0];

    if (ceres::IsNaN(x1)) {
      std::cout << "x1 is nan a = " << *a << ", b = " << *b << ", c = " << *c
                << ", d = " << *d << ", T = " << *t << std::endl;
      const T f1 = F1(*a, *b), f2 = F2(*a, *b, *c, *d), f4 = F1(*c, *d);
      std::cout << "f5 = " << F5(f1, f4, f2) << std::endl;
      std::cout << "g2 = " << G2(G1(f1, f2, f4, *t), f1, f2, *t) << std::endl;
    }
    if (ceres::IsNaN(x2)) {
      std::printf("x2 is nan \n");
    }

    T t_ball_rest = T(ball_vel0 / ball_accel);
    T ball_pos_x, ball_pos_y;
    if (t[0] > t_ball_rest) {
      ball_pos_x =
          T(ball_pos0.x()) + T(ball_vel0 * ball_dir.x()) * t_ball_rest -
          T(.5) * T(ball_accel) * t_ball_rest * t_ball_rest * T(ball_dir.x());
      ball_pos_y =
          T(ball_pos0.y()) + T(ball_vel0 * ball_dir.y()) * t_ball_rest -
          T(.5) * T(ball_accel) * t_ball_rest * t_ball_rest * T(ball_dir.y());
    } else {
      ball_pos_x = T(ball_pos0.x()) + T(ball_vel0 * ball_dir.x()) * t[0] -
                   T(.5) * T(ball_accel) * t[0] * t[0] * T(ball_dir.x());
      ball_pos_y = T(ball_pos0.y()) + T(ball_vel0 * ball_dir.y()) * t[0] -
                   T(.5) * T(ball_accel) * t[0] * t[0] * T(ball_dir.y());
    }

    residual[0] = T(robot_pos0.x()) + x1 - ball_pos_x;
    residual[1] = T(robot_pos0.y()) + x2 - ball_pos_y;

    //               << residual[0] << ", "
    //               << residual[1] << "\n";

    return true;
  }
};

struct InterceptVel {
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const c,
                  const T* const d, const T* const t, T* residual) const {
    if (*t <= T(0)) {
      return false;
    }

    T v1 = V(*a, *b, *c, *d, *t);
    T v2 = V(*b, *a, *d, *c, *t);

    if (ceres::IsNaN(v1)) {
      std::printf("v1 is nan \n");
    }
    if (ceres::IsNaN(v2)) {
      std::printf("v2 is nan \n");
    }

    T t_ball_rest = T(ball_vel0 / ball_accel);
    T ball_vel_x, ball_vel_y;
    if (t[0] > t_ball_rest) {
      ball_vel_x = T(0);
      ball_vel_y = T(0);
    } else {
      ball_vel_x =
          (T(ball_vel0 * ball_dir.x()) - T(ball_dir.x() * ball_accel) * t[0]);
      ball_vel_y =
          (T(ball_vel0 * ball_dir.y()) - T(ball_dir.y() * ball_accel) * t[0]);
    }

    residual[0] = T(robot_vel0.x()) + v1 - ball_vel_x;
    residual[1] = T(robot_vel0.y()) + v2 - ball_vel_y;

    return true;
  }
};

double RunSolver(double* a, double* b, double* c, double* d, double* t_tot) {
  static const int kMaxIterations = 100;

  ceres::Solver::Options options;

  options.max_num_iterations = kMaxIterations;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;

  ceres::Problem problem;
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<InterceptDist, 2, 1, 1, 1, 1, 1>(
          new InterceptDist),
      NULL, a, b, c, d, t_tot);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<InterceptVel, 2, 1, 1, 1, 1, 1>(
          new InterceptVel),
      NULL, a, b, c, d, t_tot);

  problem.SetParameterLowerBound(t_tot, 0, 0);

  ceres::Solver::Summary summary2;

  Solve(options, &problem, &summary2);

  //           << summary2.total_time_in_seconds << std::endl;

  return summary2.final_cost;
}

struct InterceptDist intercept_dist;
struct InterceptVel intercept_vel;

double BallCostFunction(double a, double b, double c, double d, double T) {
  double residual1[2], residual2[2];
  // g++ complains if residual arrays aren't initialized
  residual1[0] = 0;
  residual1[1] = 0;
  residual2[0] = 0;
  residual2[1] = 0;
  intercept_dist(&a, &b, &c, &d, &T, residual1);
  intercept_vel(&a, &b, &c, &d, &T, residual2);
  static const double kUseVelocityCost = true;
  double result = residual1[0] * residual1[0] + residual1[1] * residual1[1];
  if (kUseVelocityCost) {
    result += residual2[0] * residual2[0] + residual2[1] * residual2[1];
  }
  return result;
}

bool GetInterceptSolution(Eigen::Vector2d robot_pos, Eigen::Vector2d robot_vel,
                          Eigen::Vector2d ball_pos, Eigen::Vector2d ball_vel,
                          const double ball_acc, const double a_max,
                          SolutionParameters* params) {
  robot_pos0 = robot_pos / a_max;
  robot_vel0 = robot_vel / a_max;
  ball_pos0 = ball_pos / a_max;
  ball_vel0 = ball_vel.norm() / a_max;
  if (ball_vel0 > 0) {
    ball_dir = ball_vel.normalized();
  } else {
    ball_dir = Vector2d(0, 0);
  }
  ball_accel = ball_acc / a_max;

  double a, b, c, d, t_tot;

  // gets path from tsocs in which robot ends where ball begins
  // uses result as initial guess
  if (!params->isInitialized) {
    bool success =
        tsocs::GetSolution(robot_pos, robot_vel, ball_pos,
                           ball_vel, a_max, params);
    params->isInitialized = success;
  }

  // gets path from tsocs in which robot ends where ball comes to t_ball_rest
  // uses result as initial guess
  // const double rest_time = ball_accel / ball_vel0;
  // Eigen::Vector2d restPos = ball_pos0 + ball_vel0 * ball_dir * rest_time
  //      - .5 * ball_acc * ball_dir * rest_time * rest_time;
  // Eigen::Vector2d restVel = ball_vel0 * ball_dir -
  //     ball_acc * ball_dir * rest_time;

  //      ", " << restPos.y() << std::endl;
  //      ", " << restVel.y() << std::endl;
  // const bool foundGuess = GetSolution(robot_pos,
  //     robot_vel, restPos, restVel, a_max, params);

  a = params->a;
  b = params->b;
  c = params->c;
  d = params->d;
  t_tot = params->T;
  double cost = RunSolver(&a, &b, &c, &d, &t_tot);
  params->a = a;
  params->b = b;
  params->c = c;
  params->d = d;
  params->T = t_tot;
  params->cost = cost;
  params->isInitialized = cost < tsocs::kTSOCSThreshold;
  if (std::isnan(cost) || cost == -1) {
    return false;
  }
  return cost < tsocs::kTSOCSThreshold;
}

void GetState(Eigen::Vector2d robot_pos_init, Eigen::Vector2d robot_vel_init,
              Eigen::Vector2d ball_pos_init, Eigen::Vector2d ball_vel_init,
              Eigen::Vector2d* robot_pos_t, Eigen::Vector2d* robot_vel_t,
              Eigen::Vector2d* ball_pos_t, Eigen::Vector2d* ball_vel_t,
              const double a_max, const double t, const double ball_accel,
              SolutionParameters params) {
  double a, b, c, d;

  a = params.a;
  b = params.b;
  c = params.c;
  d = params.d;

  (*robot_pos_t)[0] =
      X(a, b, c, d, t) * a_max + robot_vel_init.x() * t + robot_pos_init.x();
  (*robot_pos_t)[1] =
      X(b, a, d, c, t) * a_max + robot_vel_init.y() * t + robot_pos_init.y();

  //   float delta_x1 = a_max * X(a, b, c, d, t) +
  //       robot_vel_init.x() * t;
  //   float delta_x2 = a_max * X(b, a, d, c, t) +
  //       robot_vel_init.y() * t;
  //
  //             << delta_x2 << "\n";

  (*robot_vel_t)[0] = V(a, b, c, d, t) * a_max + robot_vel_init.x();
  (*robot_vel_t)[1] = V(b, a, d, c, t) * a_max + robot_vel_init.y();

  if (t < ball_vel_init.norm() / ball_accel) {
    (*ball_pos_t) =
        ball_pos_init + ball_vel_init * t -
        .5 * ball_accel * t * t * ball_vel_init / ball_vel_init.norm();
    (*ball_vel_t) =
        ball_vel_init - ball_accel * t * ball_vel_init / ball_vel_init.norm();
  } else {
    const double t_ball_rest = ball_vel_init.norm() / ball_accel;
    (*ball_pos_t) = ball_pos_init + ball_vel_init * t_ball_rest -
                    .5 * ball_accel * t_ball_rest * t_ball_rest *
                        ball_vel_init / ball_vel_init.norm();
    (*ball_vel_t) = 0.0 * ball_vel_init;
  }
}

std::vector<Eigen::Vector2f> GetPath(Eigen::Vector2d x0, Eigen::Vector2d v0,
                                     const double a_max,
                                     const unsigned int npts,
                                     const double ball_accel,
                                     SolutionParameters params) {
  Eigen::Vector2f x_prev;
  Eigen::Vector2f v_prev;

  const double delta_t = params.T / npts;

  std::vector<Eigen::Vector2f> path_points;

  for (unsigned int i = 1; i <= npts; i++) {
    Vector2d ball_origin(0, 0);
    Vector2d ball_rest(0, 0);
    Vector2d x_t;
    Vector2d v_t;
    Vector2d b_t;
    Vector2d bvel_t;

    GetState(x0, v0, ball_origin, ball_rest, &x_t, &v_t, &b_t, &bvel_t, a_max,
             i * delta_t, ball_accel, params);
    path_points.push_back(x_t.cast<float>());
  }
  return path_points;
}
