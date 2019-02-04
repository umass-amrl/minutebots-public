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

#ifndef SRC_MOTION_CONTROL_BALL_INTERCEPTION_H_
#define SRC_MOTION_CONTROL_BALL_INTERCEPTION_H_

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
#include "motion_control/tsocs_old.h"
#include "shared/common_includes.h"
// finds parameters which give time-optimal solution given:
// acceleration magnitude of ball,
// starting position and velocity of the robot and ball;
// assumes: bounded acceleration, unbounded velocity of robot;
// returns true if correct solution was found
bool GetInterceptSolution(Eigen::Vector2d robot_pos, Eigen::Vector2d robot_vel,
                          Eigen::Vector2d ball_pos, Eigen::Vector2d ball_vel,
                          const double ball_acc, const double a_max,
                          SolutionParameters* params);

// finds the expected position and velocity of robot and ball after time t
// given starting position and velocity and an optimal solution
// bounded acceleration, unbounded velocity
void GetState(Eigen::Vector2d robot_pos_init, Eigen::Vector2d robot_vel_init,
              Eigen::Vector2d ball_pos_init, Eigen::Vector2d ball_vel_init,
              Eigen::Vector2d* robot_pos_t, Eigen::Vector2d* robot_vel_t,
              Eigen::Vector2d* ball_pos_t, Eigen::Vector2d* ball_vel_t,
              const double a_max, const double t, const double ball_accel,
              SolutionParameters params);

std::vector<Eigen::Vector2f> GetPath(Eigen::Vector2d robot_pos_init,
                                     Eigen::Vector2d robot_vel_init,
                                     const double a_max,
                                     const unsigned int npts,
                                     const double ball_accel,
                                     SolutionParameters params);

double BallCostFunction(double a, double b, double c, double d, double t_tot);

#endif  // SRC_MOTION_CONTROL_BALL_INTERCEPTION_H_
