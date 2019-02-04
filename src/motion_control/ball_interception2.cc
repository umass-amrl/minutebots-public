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
#include "motion_control/ball_interception2.h"
#include "motion_control/optimal_control_1d.h"

using Eigen::Vector2d;
using ceres::abs;

namespace tsocs {

static const double kMinVCostCoef = 0.01;

BallInterception::BallInterception() : BallInterception(Vector2d(0, 0),
                                                        Vector2d(0, 0),
                                                        Vector2d(0, 0),
                                                        Vector2d(0, 0),
                                                        1,
                                                        1) {}


BallInterception::BallInterception(Vector2d x0,
                                   Vector2d v0,
                                   Vector2d ball_x0,
                                   Vector2d ball_v0,
                                   double a_max,
                                   double ball_accel) :
      Tsocs(x0, v0, Vector2d(0, 0), Vector2d(0, 0), a_max),
      ball_x0(ball_x0 / a_max),
      ball_v0(ball_v0.norm() / a_max),
      ball_accel(ball_accel / a_max) {
  if (this->ball_v0 > 0) {
    ball_dir = ball_v0.normalized();
  } else {
    ball_dir = Vector2d(0, 0);
  }
}

struct BallInterception::InterceptDist {
  explicit InterceptDist(BallInterception* intercept) : intercept(intercept) {}

  BallInterception* intercept;

  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const c,
                  const T* const d, const T* const t, T* residual) const {
    if (*t <= T(0)) {
      return false;
    }

    T x1 = X(*a, *b, *c, *d, *t) + T(intercept->v0.x()) * t[0] +
           T(intercept->x0.x());
    T x2 = X(*b, *a, *d, *c, *t) + T(intercept->v0.y()) * t[0] +
           T(intercept->x0.y());

    if (ceres::IsNaN(x1)) {
      std::cout << "x1 is nan a = " << *a << ", b = " << *b << ", c = " << *c
                << ", d = " << *d << ", T = " << *t << std::endl;
    }
    if (ceres::IsNaN(x2)) {
      std::printf("x2 is nan \n");
    }

    T t_ball_rest = T(intercept->ball_v0 / intercept->ball_accel);
    T ball_pos_x, ball_pos_y;
    if (t[0] > t_ball_rest) {
      ball_pos_x =
          T(intercept->ball_x0.x()) +
          T(intercept->ball_v0 * intercept->ball_dir.x()) * t_ball_rest -
          T(.5) * T(intercept->ball_accel) * t_ball_rest * t_ball_rest *
              T(intercept->ball_dir.x());
      ball_pos_y =
          T(intercept->ball_x0.y()) +
          T(intercept->ball_v0 * intercept->ball_dir.y()) * t_ball_rest -
          T(.5) * T(intercept->ball_accel) * t_ball_rest * t_ball_rest *
              T(intercept->ball_dir.y());
    } else {
      ball_pos_x = T(intercept->ball_x0.x()) +
                   T(intercept->ball_v0 * intercept->ball_dir.x()) * t[0] -
                   T(.5) * T(intercept->ball_accel) * t[0] * t[0] *
                       T(intercept->ball_dir.x());
      ball_pos_y = T(intercept->ball_x0.y()) +
                   T(intercept->ball_v0 * intercept->ball_dir.y()) * t[0] -
                   T(.5) * T(intercept->ball_accel) * t[0] * t[0] *
                       T(intercept->ball_dir.y());
    }
    residual[0] = T(sqrt(intercept->x_cost_coef)) * (x1 - ball_pos_x);
    residual[1] = T(sqrt(intercept->x_cost_coef)) * (x2 - ball_pos_y);
    return true;
  }
};

struct BallInterception::InterceptVel {
  explicit InterceptVel(BallInterception* intercept) : intercept(intercept) {}

  BallInterception* intercept;

  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const c,
                  const T* const d, const T* const t, T* residual) const {
    if (*t <= T(0)) {
      return false;
    }

    T v1 = V(*a, *b, *c, *d, *t) + T(intercept->v0.x());
    T v2 = V(*b, *a, *d, *c, *t) + T(intercept->v0.y());

    if (ceres::IsNaN(v1)) {
      std::printf("v1 is nan \n");
    }
    if (ceres::IsNaN(v2)) {
      std::printf("v2 is nan \n");
    }

    T t_ball_rest = T(intercept->ball_v0 / intercept->ball_accel);
    T ball_vel_x, ball_vel_y;
    if (t[0] > t_ball_rest) {
      ball_vel_x = T(0);
      ball_vel_y = T(0);
    } else {
      ball_vel_x = (T(intercept->ball_v0 * intercept->ball_dir.x()) -
                    T(intercept->ball_dir.x() * intercept->ball_accel) * t[0]);
      ball_vel_y = (T(intercept->ball_v0 * intercept->ball_dir.y()) -
                    T(intercept->ball_dir.y() * intercept->ball_accel) * t[0]);
    }
    residual[0] = T(intercept->v_cost_coef) * (v1 - ball_vel_x);
    residual[1] = T(intercept->v_cost_coef) * (v2 - ball_vel_y);
    return true;
  }
};

struct BallInterception::Reg_T {
  explicit Reg_T(BallInterception* intercept) : intercept(intercept) {}

  BallInterception* intercept;

  template <typename T>
  bool operator()(const T* const t, T* residual) const {
    residual[0] =
        sqrt(kTSOCSThreshold) / 2 *
        ceres::exp(T(10) * (*t / T(intercept->t_expected + kEpsilon) - 1.4));
    residual[0] = min(residual[0], T(100));
    return true;
  }
};

double BallInterception::RunSolver(double* a, double* b, double* c, double* d,
                                   double* t_tot) {
  static const int kMaxIterations = 100;

  ceres::Solver::Options options;

  options.max_num_iterations = kMaxIterations;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;

  ceres::Problem problem;
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<InterceptDist, 2, 1, 1, 1, 1, 1>(
          new InterceptDist(this)),
      NULL, a, b, c, d, t_tot);
  if (match_velocity_) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<InterceptVel, 2, 1, 1, 1, 1, 1>(
            new InterceptVel(this)),
        NULL, a, b, c, d, t_tot);
  }
  if (use_t_regularization) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Reg_T, 1, 1>(new Reg_T(this)), NULL,
        t_tot);
  }

  problem.SetParameterLowerBound(t_tot, 0, 0);

  ceres::Solver::Summary summary2;
  Solve(options, &problem, &summary2);
  return summary2.final_cost;
}

void BallInterception::NonMatchVelocityGuess(SolutionParameters* params) {
  double* a = &(params->a);
  double* b = &(params->b);
  double* c = &(params->c);
  double* d = &(params->d);
  double* t = &(params->T);
  static const int kMaxIterations = 100;
  ceres::Problem problem;
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<InterceptDist, 2, 1, 1, 1, 1, 1>(
          new InterceptDist(this)),
      NULL, a, b, c, d, t);
  problem.SetParameterBlockConstant(t);
  ceres::Solver::Options options;
  options.max_num_iterations = kMaxIterations;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary1;
  Solve(options, &problem, &summary1);
}

// when the robot should match the ball's velocity
bool BallInterception::GetInterceptSolution(SolutionParameters* params,
                                            logger::Logger* logger) {
  if (kDebug) {
    if (logger != NULL) {
      logger->LogPrintPush("GetInterceptSolution");
    }
  }
  if (params->isInitialized && params->T < 1.0 / kTransmitFrequency) {
    // we expect the current trajectory to be done, so don't use old params
    // as initial guess
    params->isInitialized = false;
    if (kDebug) {
      if (logger != NULL) {
        logger->LogPrint(
            "Expected time is %f < delta T, starting with new guess",
            params->T);
      }
    }
  }
  double a, b, c, d, t_tot;
  v_cost_coef = 1.0;
  if (params->isInitialized && params->cost < kTSOCSThreshold) {
    UpdateParameters(params);
    // TODO(afischer) think of a principled way to set this coefficient
    v_cost_coef = kMinVCostCoef;
    if (kDebug) {
      if (logger != NULL) {
        logger->LogPrint("Reusing old parameters");
        if (match_velocity_) {
          logger->LogPrint("Matching velocity. v cost coef = %f", v_cost_coef);
        } else {
          logger->LogPrint("Not matching velocity");
        }
      }
    }
  } else {
    // get a lower bound on the time to intercept
    double min_time = 0;
    // progressively find higher time lower bounds on ball interception until
    // the time lower bound is the stopping time for the ball, or we give up
    for (int i = 0; i < 5; i++) {
      min_time = min(min_time, ball_v0 / ball_accel);
      if (match_velocity_) {
        // make everything one dimensional by projecting onto displacement
        // vector
        // compute time lower bound via 1d optimal control of the 1d problem
        Eigen::Vector2d delta_pose =
            ball_x0 + ball_dir * ball_v0 * min_time -
            0.5 * ball_dir * ball_accel * min_time * min_time - x0;
        double delta_pose_norm = delta_pose.norm();
        Vector2d delta_pose_dir = delta_pose / delta_pose_norm;
        Vector2d ball_vel_end =
            ball_dir * ball_v0 - ball_dir * ball_accel * min_time;
        ControlSequence1D control;
        min_time = TimeOptimalControlAnyFinal1D(
            0, delta_pose_dir.dot(v0), delta_pose_norm,
            delta_pose_dir.dot(ball_vel_end), 0, 1, INFINITY, &control);
      } else {
        // make everything one dimensional by projecting onto displacement
        // vector
        // compute time lower bound with quadratic formula
        Eigen::Vector2d delta_pose =
            ball_x0 + ball_dir * ball_v0 * min_time -
            0.5 * ball_dir * ball_accel * min_time * min_time - x0;
        double delta_x_1d = delta_pose.norm();
        Vector2d delta_pose_dir = delta_pose / delta_x_1d;
        double v0_1d = delta_pose_dir.dot(v0);
        // delta_x = v_0 t + .5 a t ^2 - solve for t
        // smaller solution will be negative, so we want to add sqrt(b^2-4ac)
        min_time = -v0_1d + sqrt(v0_1d * v0_1d + 2 * delta_x_1d);
      }
    }
    if (match_velocity_) {
      // gets path from tsocs in which robot ends where ball is at the time
      // lower bound
      Eigen::Vector2d ball_x_end =
          ball_x0 + ball_dir * ball_v0 * min_time -
          0.5 * ball_dir * ball_accel * min_time * min_time;
      Eigen::Vector2d ball_v_end = ball_dir * ball_v0
        - ball_dir * ball_accel * min_time;
      xf = ball_x_end;
      vf = ball_v_end;
      delta_pose = xf - x0;
      delta_vel = vf - v0;
      use_t_regularization = false;
      this->GetSolution(params);
      if (kDebug) {
        if (logger != NULL) {
          logger->LogPrint("Guessing solution with TSOCS");
          logger->LogPrint("Time lower bound = %f", min_time);
          logger->LogPrint("TSOCS Guess Time = %F", params->T);
        }
      }
    } else {
      // for our initial guess: have the robot just accelerate towards where the
      // ball will be at the time lower bound
      Eigen::Vector2d ball_x_end =
          ball_x0 + ball_dir * ball_v0 * min_time -
          0.5 * ball_dir * ball_accel * min_time * min_time;
      Eigen::Vector2d ball_end_dir = (ball_x_end - x0).normalized();
      *params = SolutionParameters(ball_end_dir.x(), ball_end_dir.y(),
                                   ball_end_dir.x(), ball_end_dir.y(),
                                   min_time,
                                   0);
      NonMatchVelocityGuess(params);
      if (kDebug) {
        if (logger != NULL) {
          logger->LogPrint("Guessing solution for non match vel");
          logger->LogPrint("Time lower bound = %f", min_time);
        }
      }
    }
    params->isInitialized = false;
  }
  use_t_regularization = params->isInitialized;
  if (use_t_regularization) {
    t_expected = params->T;
    if (kDebug) {
      if (logger != NULL) {
        logger->LogPrint("Time Regularization Enabled");
        logger->LogPrint("Expected Time = %f", t_expected);
      }
    }
  }
  a = params->a;
  b = params->b;
  c = params->c;
  d = params->d;
  t_tot = params->T;
  double cost = RunSolver(&a, &b, &c, &d, &t_tot);
  use_t_regularization = false;
  if (kDebug) {
    if (logger != NULL) {
      logger->LogPrint("Actual Time = %f", t_tot);
      if (cost == -1) {
        logger->LogPrint("Cost is -1, something went wrong");
      } else {
        logger->LogPrint("log10(cost) = %f", log10(cost));
      }
    }
  }
  bool status;
  if (cost == -1) {  // indicates residual/jacobian evaluation failure
    params->isInitialized = false;
    status = false;
  } else {
    if (cost < kTSOCSThreshold || !params->isInitialized) {
      params->a = a;
      params->b = b;
      params->c = c;
      params->d = d;
      params->T = t_tot;
    }
    params->cost = cost;
    params->isInitialized = cost < kTSOCSThreshold || params->isInitialized;
    status = cost < kTSOCSThreshold;
  }
  if (kDebug) {
    if (logger != NULL) {
      logger->Pop();
    }
  }
  return status;
}

void BallInterception::GetState(
    Eigen::Vector2d* robot_pos_t, Eigen::Vector2d* robot_vel_t,
    Eigen::Vector2d* ball_pos_t, Eigen::Vector2d* ball_vel_t,
    const double t, SolutionParameters params) {
  double a, b, c, d;
  a = params.a;
  b = params.b;
  c = params.c;
  d = params.d;

  (*robot_pos_t)[0] =
      a_max * (X(a, b, c, d, t) + v0.x() * t + x0.x());
  (*robot_pos_t)[1] =
      a_max * (X(b, a, d, c, t) + v0.y() * t + x0.y());

  (*robot_vel_t)[0] = a_max * (V(a, b, c, d, t) + v0.x());
  (*robot_vel_t)[1] = a_max * (V(b, a, d, c, t) + v0.y());

  if (t < ball_v0 / ball_accel) {
    (*ball_pos_t) =
        a_max * (ball_x0 + ball_dir * ball_v0 * t -
        .5 * ball_accel * t * t * ball_dir);
    (*ball_vel_t) =
        a_max * (ball_dir * ball_v0 - ball_accel * t * ball_dir);
  } else {
    const double t_ball_rest = ball_v0 / ball_accel;
    (*ball_pos_t) = a_max * (ball_x0 + ball_dir * ball_v0 * t_ball_rest -
                    .5 * ball_accel * t_ball_rest * t_ball_rest * ball_dir);
    (*ball_vel_t) = Vector2d(0, 0);
  }
}

std::vector<Eigen::Vector2f> BallInterception::GetPath(
  const unsigned int npts,
  SolutionParameters params) {
  std::vector<Eigen::Vector2f> path_points;
  Vector2d x_t;
  Vector2d v_t;
  Vector2d ball_x_t;
  Vector2d ball_v_t;
  // have to use the path_points.size() check because if params.T is 0 it will
  // just hang indefinitely
  for (double time = 0.0; path_points.size() < npts;
       time += params.T / (npts - 1)) {
    GetState(&x_t, &v_t, &ball_x_t, &ball_v_t,
             time, params);
    path_points.push_back(x_t.cast<float>());
  }
  return path_points;
}

}  // namespace tsocs
