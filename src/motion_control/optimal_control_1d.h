// Copyright 2017 - 2018 joydeepb@cs.umass.edu, dbalaban@cs.umass.edu,
// kvedder@umass.edu
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

#ifndef SRC_MOTION_CONTROL_OPTIMAL_CONTROL_1D_H_
#define SRC_MOTION_CONTROL_OPTIMAL_CONTROL_1D_H_

#include <array>

#include "eigen3/Eigen/Dense"
#include "logging/logger.h"
#include "motion_control/motion_model.h"

namespace ntoc {

struct ControlPhase1D {
  float duration;
  float acceleration;
  bool enabled;

  ControlPhase1D() : duration(0.0f), acceleration(0.0f), enabled(false) {}
  ControlPhase1D(const float& duration, const float& acceleration)
      : duration(duration), acceleration(acceleration), enabled(true) {}
  ControlPhase1D(const ControlPhase1D& other) = default;
  ControlPhase1D(ControlPhase1D&& other) = default;
  ~ControlPhase1D() = default;
};

struct ControlSequence1D {
  std::array<ControlPhase1D, 4> phases;
  unsigned int num_phases;

  ControlSequence1D() : num_phases(0) {}
  ControlSequence1D(const ControlSequence1D& other) = default;
  ControlSequence1D(ControlSequence1D&& other) = default;
  ~ControlSequence1D() = default;

  // Helper functions for manageing control phases
  void AddPhase(const float duration, const float acceleration);
  void Reset();
  void LogSequence(logger::Logger* log, bool is_linear);
};

// Not your usual signum() function: it returns -1 iff x is negative, else +1.
int Sign(const float& x);

// Discretizes the control such that if the first control is shorter than
// delta_t, returns the average acceleration of all controls which
// ought to be applied in the time step delta_t
float GetAverageAccel(const ControlSequence1D& controls, const float delta_t);

// Discretizes the control such that if the first control is shorter than
// delta_t, returns the an acceleration such that the displacement is preserved
float GetAccelToPreservePosition(const ControlSequence1D& controls,
                                 const float delta_t, const float& v0);

// Solve the 1D time optimal control problem to bring the robot to the origin at
// rest, starting from an initial location of x0 and velocity of v0. Use the
// maximum acceleration a_max and the maximum velocity v_max. Return the command
// schedule in control, and the total time as the return value.
float TimeOptimalControlZeroFinal1D(float x0, float v0, float t0, float a_max,
                                    float v_max, ControlSequence1D* control);

float TimeOptimalControlZeroFinal1D(float x0, float v0, float t0,
                                    const motion::MotionModel& motion_model,
                                    ControlSequence1D* control);

// Allows for variable final velocity.
float TimeOptimalControlAnyFinal1D(float x0, float v0, float xf, float vf,
                                   float t0, float a_max, float v_max,
                                   ControlSequence1D* control);

float TimeOptimalControlAnyFinal1D(float x0, float v0, float xf, float vf,
                                   float t0,
                                   const motion::MotionModel& motion_model,
                                   ControlSequence1D* control);

// Takes a 1D problem with 2D variables and converts to 1D
// displacement and velocity. These vectors must be parallel.
float TimeOptimalControlAnyFinal1D(Eigen::Vector2d x0, Eigen::Vector2d v0,
                                   Eigen::Vector2d xf, Eigen::Vector2d vf,
                                   float t0, float a_max, float v_max,
                                   ControlSequence1D* control);

float GetMaxVelocity(float v0, float vmax, const ControlSequence1D& control);

}  // namespace ntoc

#endif  // SRC_MOTION_CONTROL_OPTIMAL_CONTROL_1D_H_
