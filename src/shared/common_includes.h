// Copyright 2017 - 2018 joydeepb@cs.umass.edu
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
//========================================================================

#ifndef SRC_SHARED_COMMON_INCLUDES_H_
#define SRC_SHARED_COMMON_INCLUDES_H_

#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <string>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "glog/logging.h"

#include "constants/constants.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "util/helpers.h"
#include "util/pthread_utils.h"
#include "util/timer.h"

using std::max;
using std::min;
using std::string;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Vector2f;
using Eigen::Vector3f;
using geometry::Heading;
using geometry::Perp;
using math_util::AngleDiff;
using math_util::AngleDist;
using math_util::AngleMod;
using math_util::DegToRad;
using math_util::RadToDeg;
using math_util::Ramp;
using math_util::Sq;
using math_util::Cube;
using field_dimensions::kFieldLength;
using field_dimensions::kFieldWidth;
using field_dimensions::kHalfFieldLength;
using field_dimensions::kHalfFieldWidth;
using field_dimensions::kGoalWidth;
using field_dimensions::kGoalDepth;
using field_dimensions::kBoundaryWidth;
using field_dimensions::kFieldLineWidth;
using field_dimensions::kDefenseStretch;
using field_dimensions::kCenterCircleRadius;
using field_dimensions::kFieldBoundary;

#endif  // SRC_SHARED_COMMON_INCLUDES_H_
