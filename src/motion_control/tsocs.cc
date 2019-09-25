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
#include "motion_control/tsocs.h"
#include "motion_control/tsocs_old.h"

using std::isnan;
using ntoc::ControlSequence1D;
using math_util::AngleMod;
using Eigen::Vector2d;

ScopedFile tsocs_data_fid("TSOCS-data.txt", "a");

namespace tsocs {

const double kTSOCSThreshold = 1e-5;
const double kMaxRegularization = 100.0;
static const bool kRescaleGuess = true;
static const bool kOpenLoop = false;
static const double kDeltaT = 1.0 / kTransmitFrequency;
// maximum ceres iterations
static const int kMaxIterations = 100;

Tsocs::Tsocs() : Tsocs(Vector2d(0, 0), Vector2d(0, 0), Vector2d(0, 0),
                   Vector2d(0, 0), kDefaultRobotAcceleration) {}

Tsocs::Tsocs(Vector2d x0, Vector2d v0, Vector2d xf, Vector2d vf, double a_max)
    : x0(x0 / a_max),
      v0(v0 / a_max),
      xf(xf / a_max),
      vf(vf / a_max),
      a_max(a_max),
      t_reg_ratio(configuration_reader::CONFIG_t_reg_ratio),
      k1(configuration_reader::CONFIG_k1),
      k2(configuration_reader::CONFIG_k2) {
  if (isnan(x0.x()) || isnan(x0.y()) || isnan(v0.x()) || isnan(v0.y()) ||
      isnan(xf.x()) || isnan(xf.y()) || isnan(vf.x()) || isnan(vf.y())) {
    printf("Tsocs constructor was passed nan values\n");
    cout << "x0 = " << x0 << endl;
    cout << "v0 = " << v0 << endl;
    cout << "xf = " << xf << endl;
    cout << "vf = " << vf << endl;
  }
  delta_pose = (xf - x0) / a_max;
  delta_vel = (vf - v0) / a_max;
  use_t_regularization = false;
}

// scales all parmeters so that the mean of their absolute valuese is 1
SolutionParameters ScaleParameters(SolutionParameters params) {
  double mean = (std::abs(params.a) + std::abs(params.b) + std::abs(params.c) +
                 std::abs(params.d)) /
                4;
  if (mean != 0) {
    params.a /= mean;
    params.b /= mean;
    params.c /= mean;
    params.d /= mean;
  }
  return params;
}

static double GetFeasibleTime(double tp, double tm, int ai, double dX,
                              double dV, double vi) {
  double tpt = 2 * tp - ai * dV;
  double xp =
      vi * tpt + ai * tp * tpt - ai * (tp * tp + (tpt - tp) * (tpt - tp)) / 2;

  if (fabs(xp - dX) < kEpsilon) {
    return tp;
  }

  return tm;
}

static double GetAxisSolution(double x, double v0, double vf, int* sign_u0) {
  // switching funciton at time 0
  double x_switch_init;
  double tp, tm;

  if (v0 > vf) {
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

SolutionParameters Tsocs::GetNonProjectionGuess() {
  double a, b, c, d;
  const double theta_init =
      delta_pose.norm() == 0 ? 0 : atan2(delta_pose.y(), delta_pose.x());
  const double cos_theta0 = cos(theta_init);
  const double sin_theta0 = sin(theta_init);
  int sign_uo_x, sign_uo_y;

  const double axis1_time =
      GetAxisSolution(delta_pose.x() / cos_theta0, v0.x() / cos_theta0,
                      vf.x() / cos_theta0, &sign_uo_x);
  const double axis2_time =
      GetAxisSolution(delta_pose.y() / sin_theta0, v0.y() / sin_theta0,
                      vf.y() / sin_theta0, &sign_uo_y);
  c = sign_uo_x * cos(theta_init);
  d = sign_uo_y * sin(theta_init);
  a = -c / (axis1_time + kEpsilon);
  b = -d / (axis2_time + kEpsilon);
  return SolutionParameters(a, b, c, d, 0, 0);
}

SolutionParameters Tsocs::GetProjectionGuess() {
  double a, b, c, d, t;
  // project starting and ending velocities onto displacement vector
  Eigen::Vector2d delta_pose_unit = delta_pose.normalized();

  ControlSequence1D control;
  TimeOptimalControlAnyFinal1D(0, delta_pose_unit.dot(v0), delta_pose.norm(),
                               delta_pose_unit.dot(vf), 0, 1, INFINITY,
                               &control);
  if (control.num_phases == 1) {
    t = control.phases[0].duration;
    c = delta_pose.x() * control.phases[0].acceleration;
    d = delta_pose.y() * control.phases[0].acceleration;
    a = delta_pose.x() * control.phases[0].acceleration;
    b = delta_pose.y() * control.phases[0].acceleration;
  } else if (control.num_phases == 2) {
    t = control.phases[0].duration + control.phases[1].duration;
    c = delta_pose_unit.x() * control.phases[0].duration *
        control.phases[0].acceleration;
    d = delta_pose_unit.y() * control.phases[0].duration *
        control.phases[0].acceleration;
    a = -delta_pose_unit.x() * control.phases[0].acceleration;
    b = -delta_pose_unit.y() * control.phases[0].acceleration;
  } else {
    t = 0;
    a = 1;
    b = 2;
    c = 3;
    d = 4;
  }
  return SolutionParameters(a, b, c, d, t, 0);
}

SolutionParameters Tsocs::GetGuess() { return GetNonProjectionGuess(); }

double Tsocs::RunStage1(SolutionParameters* params) {
  double* a = &(params->a);
  double* b = &(params->b);
  double* c = &(params->c);
  double* d = &(params->d);
  double* t = &(params->T);
  if (kRescaleGuess) {
    *params = ScaleParameters(*params);
  }
  ceres::Problem stage1;
  stage1.AddResidualBlock(
      new ceres::AutoDiffCostFunction<Xdist, 2, 1, 1, 1, 1, 1>(new Xdist(this)),
      NULL, a, b, c, d, t);
  stage1.AddResidualBlock(
      new ceres::AutoDiffCostFunction<Vdist, 2, 1, 1, 1, 1, 1>(new Vdist(this)),
      NULL, a, b, c, d, t);
  stage1.SetParameterBlockConstant(t);

  ceres::Solver::Options options;
  options.max_num_iterations = kMaxIterations;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary1;
  Solve(options, &stage1, &summary1);
  return summary1.final_cost;
}

class TsocsCallback : public ceres::IterationCallback {
 public:
  TsocsCallback() {}

  virtual ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary) {
    //     printf("%05d, %f, %f, %f, %f, %f, %f, %f\n", summary.iteration, a, b,
    //     c, d,
    //            t_tot, summary.cost, summary.gradient_norm);
    return ceres::SOLVER_CONTINUE;
  }
};

double Tsocs::RunStage2(SolutionParameters* params) {
  double* a = &(params->a);
  double* b = &(params->b);
  double* c = &(params->c);
  double* d = &(params->d);
  double* t = &(params->T);
  if (kRescaleGuess) {
    *params = ScaleParameters(*params);
  }
  ceres::Problem stage2;
  stage2.AddResidualBlock(
      new ceres::AutoDiffCostFunction<Xdist, 2, 1, 1, 1, 1, 1>(new Xdist(this)),
      NULL, a, b, c, d, t);
  stage2.AddResidualBlock(
      new ceres::AutoDiffCostFunction<Vdist, 2, 1, 1, 1, 1, 1>(new Vdist(this)),
      NULL, a, b, c, d, t);
  if (use_t_regularization) {
    stage2.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Reg_T, 1, 1>(new Reg_T(this)), NULL, t);
  }

  stage2.SetParameterLowerBound(t, 0, 0);

  // options.log_to_stderr = true;

  ceres::Solver::Options options;
  options.max_num_iterations = kMaxIterations;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;
  options.update_state_every_iteration = true;
  TsocsCallback callback;
  if (false) {
    options.callbacks.push_back(&callback);
  }
  ceres::Solver::Summary summary2;
  Solve(options, &stage2, &summary2);

  if (summary2.message == "Residual and Jacobian evaluation failed.") {
    /*
    std::cout << summary2.FullReport() << std::endl;
    printf("cost is %f\n", summary2.final_cost);
    */
    return kTSOCSThreshold * 2;
  }
  return summary2.final_cost;
}

double Tsocs::GetTimeBound() {
  // time to accelerate to rest
  const double vInitMag = v0.norm();
  // position after accelerating to rest, first rest point
  const Eigen::Vector2d x_rest = v0 * vInitMag / 2;

  // time of direct acceleration to end velocity:
  const double T_l2 = (vf).norm();

  // vector from second rest point to final point:
  const Eigen::Vector2d x_prime = delta_vel * vf.norm() / 2;
  // dist between two rest points
  const double x_tilda = (delta_pose - x_rest - x_prime).norm();
  double T1 = 2 * sqrt(x_tilda);  // travel time between rest points
  // total time for 1D problems and acceleration to rest
  return T_l2 + T1 + vInitMag;
}

bool Tsocs::GetSolutionSetFromGuess(std::vector<SolutionParameters>* params,
                                    SolutionParameters guess) {
  use_t_regularization = false;
  if (isnan(x0.x()) || isnan(x0.y()) || isnan(v0.x()) || isnan(v0.y()) ||
      isnan(xf.x()) || isnan(xf.y()) || isnan(vf.x()) || isnan(vf.y())) {
    printf("GetSolutionSetFromGuess was passed nan values\n");
    cout << "x0 = " << x0 << endl;
    cout << "v0 = " << v0 << endl;
    cout << "xf = " << xf << endl;
    cout << "vf = " << vf << endl;
  }
  SolutionParameters cur_params(guess);
  cur_params.T = GetTimeBound();
  params->push_back(cur_params);

  double cost1 = RunStage1(&cur_params);
  cur_params.cost = cost1;
  params->push_back(cur_params);

  double cost2 = RunStage2(&cur_params);
  cur_params.cost = cost2;
  cur_params.isInitialized = cost2 < kTSOCSThreshold;
  params->push_back(cur_params);
  return cost2 < kTSOCSThreshold;
}

bool Tsocs::GetSolutionSet(std::vector<SolutionParameters>* params,
                           int* perturbations) {
  use_t_regularization = false;
  if (isnan(x0.x()) || isnan(x0.y()) || isnan(v0.x()) || isnan(v0.y()) ||
      isnan(xf.x()) || isnan(xf.y()) || isnan(vf.x()) || isnan(vf.y())) {
    printf("GetSolutionSet was passed nan values\n");
    cout << "x0 = " << x0 << endl;
    cout << "v0 = " << v0 << endl;
    cout << "xf = " << xf << endl;
    cout << "vf = " << vf << endl;
  }
  SolutionParameters cur_params = GetGuess();
  cur_params.T = GetMinimumUpperTimeBound(x0, v0, xf, vf, 1, INFINITY);
  params->push_back(cur_params);
  double cost1 = RunStage1(&cur_params);
  cur_params.cost = cost1;
  params->push_back(cur_params);
  SolutionParameters params_stage1(cur_params);
  double cost2;
  cost2 = RunStage2(&cur_params);
  cur_params.cost = cost2;
  cur_params.isInitialized = cost2 < kTSOCSThreshold;
  params->push_back(cur_params);
  return cost2 < kTSOCSThreshold;
}

// Gets a solution using Ceres without 2 stages
bool Tsocs::GetSolutionNoStages(SolutionParameters* params) {
  if (isnan(x0.x()) || isnan(x0.y()) || isnan(v0.x()) || isnan(v0.y()) ||
      isnan(xf.x()) || isnan(xf.y()) || isnan(vf.x()) || isnan(vf.y())) {
    printf("GetSolutionNoStages was passed nan values\n");
    cout << "x0 = " << x0 << endl;
    cout << "v0 = " << v0 << endl;
    cout << "xf = " << xf << endl;
    cout << "vf = " << vf << endl;
  }
  double cost = RunStage2(params);
  params->cost = cost;
  params->isInitialized = cost < kTSOCSThreshold;
  return cost < kTSOCSThreshold;
}

// Moves the start of the adjoint line forward 1 time step
void Tsocs::UpdateParameters(SolutionParameters* params) {
  params->c += params->a * kDeltaT;
  params->d += params->b * kDeltaT;
  params->T -= kDeltaT;
}

bool Tsocs::GetSolution(SolutionParameters* params, logger::Logger* logger) {
  if (isnan(x0.x()) || isnan(x0.y()) || isnan(v0.x()) || isnan(v0.y()) ||
      isnan(xf.x()) || isnan(xf.y()) || isnan(vf.x()) || isnan(vf.y())) {
    printf("GetSolution was passed nan values\n");
    cout << "x0 = " << x0 << endl;
    cout << "v0 = " << v0 << endl;
    cout << "xf = " << xf << endl;
    cout << "vf = " << vf << endl;
  }
  SolutionParameters params_old = *params;
  if (params->isInitialized) {
    use_t_regularization = true;
    t_expected = params->T - 1.0 / kTransmitFrequency;
  }

  if (!params->isInitialized || params->cost >= kTSOCSThreshold) {
    if (logger != NULL) {
      logger->LogPrint("Guessing Parameters");
    }
    *params = GetGuess();
    params->T = GetMinimumUpperTimeBound(x0, v0, xf, vf, 1, INFINITY);
    double cost1 = RunStage1(params);
    if (logger != NULL) {
      logger->LogPrint("Time Upper Bound: %f", params->T);
      logger->LogPrint("Stage 1 log10(cost): %f", log10(cost1));
    }
  } else {
    if (logger != NULL) {
      logger->LogPrint("Using Old Parameters as Initial Guess");
    }
    UpdateParameters(params);
    if (kOpenLoop) {
      return true;
    }
  }
  if (use_t_regularization) {
    double prop_accel = static_cast<double>(delta_vel.norm()) / t_expected;
    prop_accel = max(1.0, prop_accel);

    v_cost_coef = max(pow(1 - prop_accel, 1.0), min_v_cost_coef);

    if (logger != NULL) {
      logger->LogPrint("V prop: %f", prop_accel);
      logger->LogPrint("V cost coef: %f", v_cost_coef);
      logger->LogPrint("X cost coef: %f", x_cost_coef);
    }
  } else {
    v_cost_coef = 1.0;
  }
  double cost2 = RunStage2(params);
  if (cost2 >= kTSOCSThreshold && params->isInitialized &&
      params->cost < kTSOCSThreshold) {
    if (logger != NULL) {
      logger->LogPrint("TSOCS failed, retrying");
      *params = GetGuess();
      RunStage1(params);
      cost2 = RunStage2(params);
    }
  }
  if (logger != NULL) {
    if (use_t_regularization) {
      logger->LogPrint("Expected t: %f", t_expected);
    }
    logger->LogPrint("Actual t:   %f", params->T);
    logger->LogPrint("log10(cost): %f", log10(cost2));
  }

  if (kCollectTSOCSData) {
    fprintf(tsocs_data_fid, "\tCOST COEFS: %f %f\n", x_cost_coef,
            v_cost_coef);
    fprintf(tsocs_data_fid, "\tT REGULARIZATION: %s\n",
            use_t_regularization ? "TRUE" : "FALSE");
    if (use_t_regularization) {
      fprintf(tsocs_data_fid, "\tEXPECTED T: %f\n", t_expected);
    }
  }
  if (cost2 >= kTSOCSThreshold) {
    *params = params_old;
    UpdateParameters(params);
  }
  params->cost = cost2;
  use_t_regularization = false;
  x_cost_coef = 1.0;
  v_cost_coef = 1.0;
  params->isInitialized = true;
  return cost2 < kTSOCSThreshold;
}

void Tsocs::GetState(Eigen::Vector2d* xt, Eigen::Vector2d* vt, const double t,
                     SolutionParameters params) {
  double a, b, c, d;
  a = params.a;
  b = params.b;
  c = params.c;
  d = params.d;
  (*xt)[0] = a_max * (X(a, b, c, d, t) + v0.x() * t + x0.x());
  (*xt)[1] = a_max * (X(b, a, d, c, t) + v0.y() * t + x0.y());

  (*vt)[0] = a_max * (V(a, b, c, d, t) + v0.x());
  (*vt)[1] = a_max * (V(b, a, d, c, t) + v0.y());
}

Vector2d Tsocs::GetAccelVector(const double t,
                               SolutionParameters params) {
  const double psi1 = params.a * t + params.c;
  const double psi2 = params.b * t + params.d;
  const double psi_mag = sqrt(psi1 * psi1 + psi2 * psi2);
  const double f = a_max / psi_mag;
  return Vector2d(f * psi1, f * psi2);
}


std::vector<Eigen::Vector2f> Tsocs::GetPath(const unsigned int npts,
                                            SolutionParameters params) {
  const double delta_t = params.T / (npts - 1);
  std::vector<Eigen::Vector2f> path_points;
  Eigen::Vector2d x_t;
  Eigen::Vector2d v_t;
  for (double time = 0; time <= params.T; time += delta_t) {
    GetState(&x_t, &v_t, time, params);
    path_points.push_back(x_t.cast<float>());
  }
  return path_points;
}

bool Tsocs::Finished(SolutionParameters params) {
  if ((x0 - xf).norm() < 5.0 / a_max && (v0 - vf).norm() < 50.0 / a_max) {
    return true;
  }
  return params.T <= 0;
}

double Tsocs::CostFunction(double a, double b, double c, double d, double T) {
  double residual1[2] = {0, 0}, residual2[2] = {0, 0};
  struct Xdist x_dist(this);
  struct Vdist v_dist(this);
  x_dist(&a, &b, &c, &d, &T, residual1);
  v_dist(&a, &b, &c, &d, &T, residual2);
  return (residual1[0] * residual1[0] + residual1[1] * residual1[1] +
          residual2[0] * residual2[0] + residual2[1] * residual2[1]) /
         2;
}

}  // namespace tsocs
