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

#include <eigen3/Eigen/Geometry>
#include <math.h>

#include <cmath>
#include <algorithm>
#include <vector>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "shared/common_includes.h"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/ntoc_2d.h"

#ifndef SRC_MOTION_CONTROL_SUB_OPTIMAL_CONTROLS_H_
#define SRC_MOTION_CONTROL_SUB_OPTIMAL_CONTROLS_H_

// finds the minimum time of all possible sub-optimal controllers
double GetMinimumUpperTimeBound(Eigen::Vector2d x0, Eigen::Vector2d v0,
                 Eigen::Vector2d xf, Eigen::Vector2d vf,
                 const double a_max, const double v_max);

double ThreeStageLinear(Eigen::Vector2d x0, Eigen::Vector2d v0,
                 Eigen::Vector2d xf, Eigen::Vector2d vf,
                 const double a_max, const double v_max);

bool RotateTowardsGoal(Eigen::Vector2d x0, Eigen::Vector2d v0,
                 Eigen::Vector2d xf, Eigen::Vector2d vf,
                 const double a_max, const double v_max,
                 std::vector<double>* times);

bool RotateTowardsRest(Eigen::Vector2d x0, Eigen::Vector2d v0,
                 Eigen::Vector2d xf, Eigen::Vector2d vf,
                 const double a_max, const double v_max,
                 std::vector<double>* times);

#endif  // SRC_MOTION_CONTROL_SUB_OPTIMAL_CONTROLS_H_
