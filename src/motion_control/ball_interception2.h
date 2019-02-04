// Copyright 2017-2018 dbalaban@cs.umass.edu
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

#ifndef SRC_MOTION_CONTROL_BALL_INTERCEPTION2_H_
#define SRC_MOTION_CONTROL_BALL_INTERCEPTION2_H_

#include <ceres/ceres.h>
#include <ceres/types.h>
#include <math.h>
#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "motion_control/motion_control_structures.h"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/tsocs.h"
#include "motion_control/tsocs_templates.h"
#include "shared/common_includes.h"

using Eigen::Vector2d;
using ntoc::TimeOptimalControlAnyFinal1D;

namespace tsocs {

class BallInterception : public Tsocs {
 public:
  BallInterception();
  BallInterception(Vector2d x0, Vector2d v0, Vector2d ball_x, Vector2d ball_v,
                   double a_max, double ball_accel);

  // finds parameters which give time-optimal solution given:
  // acceleration magnitude of ball,
  // starting position and velocity of the robot and ball;
  // the robot should match the ball's vevlocity;
  // assumes: bounded acceleration, unbounded velocity of robot;
  // returns true if correct solution was found
  bool GetInterceptSolution(SolutionParameters* params,
                            logger::Logger* logger = NULL);

  // finds the expected position and velocity of robot and ball after time t
  // given starting position and velocity and an optimal solution
  // bounded acceleration, unbounded velocity
  void GetState(Eigen::Vector2d* robot_pos_t, Eigen::Vector2d* robot_vel_t,
                Eigen::Vector2d* ball_pos_t, Eigen::Vector2d* ball_vel_t,
                const double t, SolutionParameters params);

  std::vector<Eigen::Vector2f> GetPath(const unsigned int npts,
                                       SolutionParameters params);

  double BallCostFunction(double a, double b, double c, double d, double t_tot);
  bool match_velocity_ = true;

 private:
  void NonMatchVelocityGuess(SolutionParameters* params);
  struct InterceptDist;
  struct InterceptVel;
  struct InterceptOrientation;
  struct Reg_T;

  double RunSolver(double* a, double* b, double* c, double* d, double* t_tot);

  Eigen::Vector2d ball_x0;
  Eigen::Vector2d ball_dir;
  double ball_v0;
  double ball_accel;
  bool orientation_cost = false;
  static const bool kDebug = false;
};

}  // namespace tsocs

#endif  // SRC_MOTION_CONTROL_BALL_INTERCEPTION2_H_
