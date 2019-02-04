// Copyright 2018 kvedder@umass.edu
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

#ifndef SRC_MOTION_CONTROL_MOTION_MODEL_H_
#define SRC_MOTION_CONTROL_MOTION_MODEL_H_

namespace motion {

// A 1D motion model maximum acceleration and speed.
struct MotionModel {
  float a_max;
  float v_max;

  MotionModel() = delete;
  MotionModel(const float a_max, const float v_max)
      : a_max(a_max), v_max(v_max) {}

  MotionModel Scaled(const float s) const { return {s * a_max, s * v_max}; }
};

}  // namespace motion

#endif  // SRC_MOTION_CONTROL_MOTION_MODEL_H_
