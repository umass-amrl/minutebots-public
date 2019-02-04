// Copyright 2017 - 2018 joydeepb@cs.umass.edu, dbalaban@cs.umass.edu,
// kvedder@umass.edu, slane@cs.umass.edu
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
#include "logging/logger.h"
#include "motion_control/optimal_control_1d.h"

#ifndef SRC_MOTION_CONTROL_NTOC_2D_H_
#define SRC_MOTION_CONTROL_NTOC_2D_H_

extern bool collect_ntoc_data;

namespace ntoc {

// Combined 2D command schedule for NTOC.
struct ControlPhase2D {
  float duration;
  Eigen::Vector2f acceleration;
  bool enabled;

  ControlPhase2D() : duration(0.0f), acceleration(0.0f, 0.0f), enabled(false) {}
  ControlPhase2D(const float& duration, const Eigen::Vector2f& acceleration)
      : duration(duration), acceleration(acceleration), enabled(true) {}
};

struct ControlSequence2D {
  std::array<ControlPhase2D, 6> phases;
  unsigned int num_phases;

  ControlSequence2D() : num_phases(0) {}
  ControlSequence2D(const ControlSequence2D& other) = default;
  ControlSequence2D(ControlSequence2D&& other) = default;
  ~ControlSequence2D() = default;

  // Helper functions for manageing control phases
  void AddPhase(const float duration, const Eigen::Vector2f& acceleration);
  void Reset();
  void LogSequence(logger::Logger* log);
};

// 2D motion model with independent maximum acceleration and speed for each
// axis.
//
// Merges two 1D command schedules into one combined 2D command schedule.
void MergeCommandSchedules(const ControlSequence1D& x,
                           const ControlSequence1D& y,
                           ControlSequence2D* combined, const float time_x,
                           const float time_y);

// Discretizes the control such that if the first control is shorter than
// delta_t, returns the average acceleration of all controls which
// ought to be applied in the time step delta_t
Eigen::Vector2f GetAverageAccel(const ControlSequence2D& controls,
                                const float delta_t);

// Discretizes the control such that if the first control is shorter than
// delta_t, returns the an acceleration such that the displacement is preserved
Eigen::Vector2f GetAccelToPreservePosition(const ControlSequence2D& controls,
                                           const float delta_t,
                                           const Vector2f& v0);

void TransformCoordinatesForNTOC(const pose_2d::Pose2Df& current_pose,
                                 const Eigen::Vector2f& current_velocity,
                                 const Eigen::Vector2f& desired_position,
                                 Eigen::Vector2f* transformed_position,
                                 Eigen::Vector2f* transformed_velocity);

// Solve the 2D near-time optimal control (NTOC) problem to bring the robot to
// the origin at rest, starting from an initial location of x0 and velocity of
// v0. Use the maximum acceleration a_max and the maximum velocity v_max.
// Return the 2D command schedule in control, and the total time as the return
// value.
float NTOC2D(const Eigen::Vector2f& x0,
             const Eigen::Vector2f& v0,
             const motion::MotionModel& motion_model,
             ControlSequence2D* control);

float NTOC2D(const Eigen::Vector2f& x0,
             const Eigen::Vector2f& v0,
             const motion::MotionModel& motion_model,
             ControlSequence2D* control,
             float* max_x_velocity,
             float* max_y_velocity);

// Return a list of points which partitions the path from start to goal.
//
// num_path_points is not the exact number of points; total points will be at
// most num_path_points + controls.size()
std::vector<Eigen::Vector2f> GetPath(const Eigen::Vector2f& x0,
                                     const Eigen::Vector2f& v0,
                                     const ControlSequence2D& controls,
                                     const size_t num_path_points,
                                     const float total_time);

bool NTOCFinished(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d xf);

}  // namespace ntoc

#endif  // SRC_MOTION_CONTROL_NTOC_2D_H_
