// Copyright 2018 kvedder@umass.edu
//
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Typedefs for UMass RoboCup SSL robots.
//
//========================================================================
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
#ifndef SRC_CONSTANTS_TYPEDEFS_H_
#define SRC_CONSTANTS_TYPEDEFS_H_

#include <eigen3/Eigen/Dense>

// Handles the differentiation between our and other team robot indices.
using OurRobotIndex = unsigned int;
using TheirRobotIndex = unsigned int;

// Used only upon input from SSL Vision.
using SSLVisionId = unsigned int;

using ObstacleIndex = unsigned int;

using Vector6f = Eigen::Matrix<float, 6, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3x6d = Eigen::Matrix<double, 3, 6>;
using Matrix6x3d = Eigen::Matrix<double, 6, 3>;


#endif  // SRC_CONSTANTS_TYPEDEFS_H_
