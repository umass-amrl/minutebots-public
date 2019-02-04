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

#include "safety/dss_helpers.h"

#include <iomanip>

#include "math/geometry.h"

using math_util::Sq;
using math_util::Cube;
using math_util::Pow;

namespace safety {

// Used as kEpsilon for all of the following math.
static constexpr float kDSSMathEpsilon = 0.01f;

static constexpr bool kDebug = false;

float ClosestDistanceTime(const Position& p0, const Velocity& v0,
                          const Acceleration& a) {
  const float a_param = a.squaredNorm();
  const float b_param = 3 * a.dot(v0);
  const float c_param = 2 * (v0.squaredNorm() + a.dot(p0));
  const float d_param = 2 * p0.dot(v0);

  float x0 = 0;
  float x1 = 0;
  float x2 = 0;

  if (fabs(a_param) < kEpsilon) {
    // Solve quadratic.
    // Returns the number of roots found.
    if (math_util::SolveQuadratic(b_param, c_param, d_param, &x0, &x1) == 2) {
      if (x0 > 0) {
        return x0;
      }
      return x1;
    }
    return x0;
  }
  // Solve cubic.
  // Returns the number of roots found.
  if (math_util::SolveCubic(a_param, b_param, c_param, d_param, &x0, &x1,
                            &x2) == 3) {
    if (x0 > 0) {
      return x0;
    } else if (x1 > 0) {
      return x1;
    }
    return x2;
  }
  return x0;
}

float ClampedClosestDistanceTime(const Position& p0, const Velocity& v0,
                                 const Acceleration& a, const float min_time,
                                 const float max_time) {
  return math_util::Clamp(ClosestDistanceTime(p0, v0, a), min_time, max_time);
}

float DistanceSquared(const Position& p0, const Velocity& v0,
                      const Acceleration& a, const float t) {
  const float t0_t1 = p0.dot(p0 + (2 * v0 * t));
  const float t2 = (v0.squaredNorm() + a.dot(p0)) * Sq(t);
  const float t3_t4 = a.dot(v0 * Cube(t) + 0.25f * a * Pow<float, 4>(t));
  return t0_t1 + t2 + t3_t4;
}

const Acceleration ComputeAccelerationWithCommand(
    const Velocity& current_velocity, const Velocity& commanded_velocity,
    const motion::MotionModel& motion_model) {
  if (kDebug) {
    NP_CHECK(current_velocity.squaredNorm() <=
             Sq(motion_model.v_max) + kDSSMathEpsilon);
  }
  const Acceleration accel = CapAcceleration(
      (commanded_velocity - current_velocity) / kControlTickTime, motion_model);
  NP_CHECK_MSG(accel.squaredNorm() <= Sq(motion_model.a_max) + kDSSMathEpsilon,
               std::setprecision(20)
                   << "Accel norm: " << accel.norm() << " Max accel: "
                   << motion_model.a_max << "Sq norm: " << accel.squaredNorm()
                   << " Sq a_max: " << Sq(motion_model.a_max));
  return accel;
}

const Velocity ComputeCommandWithAcceleration(
    const Velocity& current_velocity, const Acceleration& acceleration,
    const motion::MotionModel& motion_model) {
  if (kDebug) {
    NP_CHECK_MSG(
        acceleration.squaredNorm() <= Sq(motion_model.a_max) + kDSSMathEpsilon,
        std::setprecision(20) << "Acc norm: " << acceleration.norm()
                              << " a max: " << motion_model.a_max);
  }
  return current_velocity + acceleration * kControlTickTime;
}

const Velocity CapVelocity(const Velocity& velocity,
                           const motion::MotionModel& motion_model) {
  if (velocity.squaredNorm() > Sq(motion_model.v_max)) {
    return geometry::GetNormalizedOrZero(velocity) *
           (motion_model.v_max - kDSSMathEpsilon);
  }
  return velocity;
}

const Acceleration CapAcceleration(const Acceleration& acceleration,
                                   const motion::MotionModel& motion_model) {
  if (acceleration.squaredNorm() > Sq(motion_model.a_max)) {
    return geometry::GetNormalizedOrZero(acceleration) *
           (motion_model.a_max - kDSSMathEpsilon);
  }
  return acceleration;
}

const Acceleration CapAccelerationAtMaxVelocity(
    const Acceleration& acceleration, const Velocity& current_velocity,
    const Time control_period, const motion::MotionModel& motion_model) {
  // Vf = Vi + a * t
  const Velocity final_velocity =
      current_velocity + acceleration * control_period;
  if (final_velocity.squaredNorm() <= Sq(motion_model.v_max)) {
    return acceleration;
  }

  // (Vf - Vi) / t = a
  return ((geometry::GetNormalizedOrZero(final_velocity) * motion_model.v_max -
           current_velocity) /
          control_period);
}

const Position CalculateDeltaPosition(const Velocity& current_velocity,
                                      const Acceleration& acceleration,
                                      const float control_period) {
  return current_velocity * control_period +
         0.5 * acceleration * Sq(control_period);
}

const Position CalculateEndPositionWithCommand(
    const Position& position, const Velocity& current_velocity,
    const Velocity& command_velocity, const float control_period,
    const motion::MotionModel& motion_model) {
  // Acceleration is over the given tick period scaled to a full second.
  const Acceleration capped_acceleration =
      CapAcceleration(ComputeAccelerationWithCommand(
                          command_velocity, current_velocity, motion_model),
                      motion_model);
  return position + CalculateDeltaPosition(current_velocity,
                                           capped_acceleration, control_period);
}

const Position CalculateEndPositionWithAccel(
    const Position& position, const Velocity& current_velocity,
    const Acceleration& current_acceleration, const float control_period,
    const motion::MotionModel& motion_model) {
  if (kDebug &&
      current_velocity.squaredNorm() >
          Sq(motion_model.v_max) + kDSSMathEpsilon) {
    LOG(ERROR) << std::setprecision(20)
               << "Vel norm: " << current_velocity.norm()
               << " Max vel: " << motion_model.v_max
               << "Sq norm: " << current_velocity.squaredNorm()
               << " Sq v_max: " << Sq(motion_model.v_max);
  }
  const Acceleration capped_acceleration =
      CapAcceleration(current_acceleration, motion_model);
  return position + CalculateDeltaPosition(current_velocity,
                                           capped_acceleration, control_period);
}

const Velocity CalculateDeltaVelocity(const Acceleration& acceleration,
                                      const float control_period) {
  return acceleration * control_period;
}

const Velocity CalculateEndVelocityWithCommand(
    const Velocity& current_velocity, const Velocity& command_velocity,
    const float control_period, const motion::MotionModel& motion_model) {
  // Acceleration is over the given control period.
  const Acceleration capped_acceleration =
      CapAcceleration(ComputeAccelerationWithCommand(
                          command_velocity, current_velocity, motion_model),
                      motion_model);
  return current_velocity +
         CalculateDeltaVelocity(capped_acceleration, control_period);
}

const Velocity CalculateEndVelocityWithAccel(
    const Velocity& current_velocity, const Acceleration& current_acceleration,
    const float control_period, const motion::MotionModel& motion_model) {
  if (kDebug) {
    NP_CHECK(current_velocity.squaredNorm() <=
             Sq(motion_model.v_max) + kDSSMathEpsilon);
  }
  const Acceleration capped_acceleration =
      CapAcceleration(current_acceleration, motion_model);
  return current_velocity +
         CalculateDeltaVelocity(capped_acceleration, control_period);
}

float CalculateTimeToRest(const Velocity& velocity,
                          const motion::MotionModel& motion_model) {
  if (kDebug && !kProduction &&
      velocity.squaredNorm() > Sq(motion_model.v_max) + kDSSMathEpsilon) {
    LOG(ERROR) << "Time to rest velocity over v_max";
  }
  return geometry::GetNormOrZero(velocity) / motion_model.a_max;
}

const bool IsInCollision(const float squared_distance, const float r1,
                         const float r2, const float safety_margin) {
  NP_CHECK(r1 > 0);
  NP_CHECK(r2 > 0);
  return squared_distance < Sq(r1 + r2 + safety_margin);
}

}  // namespace safety
