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

#ifndef SRC_SAFETY_DSS_HELPERS_H_
#define SRC_SAFETY_DSS_HELPERS_H_

#include "constants/constants.h"
#include "motion_control/motion_model.h"

namespace safety {

using Position = Eigen::Vector2f;
using Velocity = Eigen::Vector2f;
using Acceleration = Eigen::Vector2f;
using Time = float;

// The Time of a single control tick. GCC Believes this to be unused, though it
// is not.
static const Time kControlTickTime __attribute__((unused)) =
    1.0f / kTransmitFrequency;

// The Time of the command execution. GCC Believes this to be unused, though it
// is not.
static const Time kDSSCommandExecutionPeriod __attribute__((unused)) =
    9.0f / kTransmitFrequency;

// Computes the time at which relative position is smallest distance, given
// relative velocity and acceleration.
//
// Math is on upper right side of the whiteboard:
// https://photos.google.com/share/AF1QipMA9N-3dfxxEMLFeWcGcTDHucRfSAT1DvK_qfjyHfodb6VDm5iKAIkWiX_pWrRIIw/photo/AF1QipOhS7PVJysLsA86iVpytJAPKBzoLN_EH0HSi3EB?key=ZFdiNGI5LXBrMmQyYVRmOXlJVExRaXRsUDQxdS1n
float ClosestDistanceTime(const Position& p0, const Velocity& v0,
                          const Acceleration& a);

// Computes the time at which relative position is smallest distance, given
// relative velocity and acceleration, and then clamps the time between a given
// min and max.
float ClampedClosestDistanceTime(const Position& p0, const Velocity& v0,
                                 const Acceleration& a, const float min_time,
                                 const float max_time);

// Squared relative distance at a given time.
//
// Math is on upper left side of the whiteboard:
// https://photos.google.com/share/AF1QipMA9N-3dfxxEMLFeWcGcTDHucRfSAT1DvK_qfjyHfodb6VDm5iKAIkWiX_pWrRIIw/photo/AF1QipOhS7PVJysLsA86iVpytJAPKBzoLN_EH0HSi3EB?key=ZFdiNGI5LXBrMmQyYVRmOXlJVExRaXRsUDQxdS1n
float DistanceSquared(const Position& p0, const Velocity& v0,
                      const Acceleration& a, const float t);

// Converts a single tick difference into a true acceleration in mm/s^2.
//
// Assumes that the delta in command is exactly 1 timestep.
const Acceleration ComputeAccelerationWithCommand(
    const Velocity& current_velocity, const Velocity& commanded_velocity,
    const motion::MotionModel& motion_model);

// Converts an acceleration in mm/s^2 to a single tick velocity delta.
//
// Assumes that the delta in command is exactly 1 timestep.
const Velocity ComputeCommandWithAcceleration(
    const Velocity& current_velocity, const Acceleration& acceleration,
    const motion::MotionModel& motion_model);

// Ensures that velocity will not exceed cap.
// Returns units in mm/s.
const Velocity CapVelocity(const Velocity& velocity,
                           const motion::MotionModel& motion_model);

// Ensures that acceleration will not exceed accel cap for the given timeslice.
// Returns units in mm/s^2.
const Acceleration CapAcceleration(const Acceleration& acceleration,
                                   const motion::MotionModel& motion_model);

// Ensures that acceleration will not exceed velocity cap for the given
// timeslice.
// Returns units in mm/s^2.
const Acceleration CapAccelerationAtMaxVelocity(
    const Acceleration& acceleration, const Velocity& current_velocity,
    const Time control_period, const motion::MotionModel& motion_model);

// Gets the change in position of the robot at the end of the control period,
// given its motion parameters.
// \delta x = v_i \delta t + 0.5 a t^2
const Position CalculateDeltaPosition(const Velocity& current_velocity,
                                      const Acceleration& acceleration,
                                      const float control_period);

// Gets the position of the robot at the end of the control period, given the
// start parameters.
//
// Assumes that the delta in command is exactly 1 timestep.
const Position CalculateEndPositionWithCommand(
    const Position& position, const Velocity& current_velocity,
    const Velocity& command_velocity, const float control_period,
    const motion::MotionModel& motion_model);

// Gets the position of the robot at the end of the control period, given the
// start parameters.
const Position CalculateEndPositionWithAccel(
    const Position& position, const Velocity& current_velocity,
    const Acceleration& current_acceleration, const float control_period,
    const motion::MotionModel& motion_model);

const Velocity CalculateDeltaVelocity(const Acceleration& acceleration,
                                      const float control_period);

// Gets the velocity of the robot at the end of the control period, given the
// start parameters.
//
// Assumes that the delta in command is exactly 1 timestep.
const Velocity CalculateEndVelocityWithCommand(
    const Velocity& current_velocity, const Velocity& command_velocity,
    const float control_period, const motion::MotionModel& motion_model);

// Gets the velocity of the robot at the end of the control period, given the
// start parameters.
const Velocity CalculateEndVelocityWithAccel(
    const Velocity& current_velocity, const Acceleration& current_acceleration,
    const float control_period, const motion::MotionModel& motion_model);

// Computes the time for a robot with the given velocity and model to come to
// rest.
float CalculateTimeToRest(const Velocity& velocity,
                          const motion::MotionModel& motion_model);

const bool IsInCollision(const float squared_distance, const float r1,
                         const float r2, const float safety_margin);

}  // namespace safety

#endif  // SRC_SAFETY_DSS_HELPERS_H_
