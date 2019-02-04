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

#include <float.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include "motion_control/optimal_control_1d.h"
#include "shared/common_includes.h"

using motion::MotionModel;
using std::max;
using std::min;

namespace ntoc {

void ControlSequence1D::AddPhase(const float duration,
                                 const float acceleration) {
  if (num_phases < 4) {
    if (duration < kEpsilon) {
      return;
    }
    if (num_phases > 0 &&
       (fabs(phases[num_phases - 1].acceleration - acceleration) <= kEpsilon)) {
      phases[num_phases-1].duration += duration;
    } else {
      phases[num_phases].duration = duration;
      phases[num_phases].acceleration = acceleration;
      phases[num_phases].enabled = true;
      num_phases++;
    }
  } else {
    LOG(ERROR) << "Tried to add too many phases to a 1D Optimal Control set";
  }
}

void ControlSequence1D::Reset() {
  for (auto& phase : phases) {
    phase.duration = 0.0f;
    phase.acceleration = 0.0f;
    phase.enabled = false;
  }

  num_phases = 0;
}

void ControlSequence1D::LogSequence(logger::Logger* log, bool is_linear) {
  if (is_linear)
    log->LogPrint("Planned Linear Control Sequence");
  else
    log->LogPrint("Planned Angular Control Sequence");
  log->Push();
  for (const auto& phase : phases) {
    if (phase.enabled) {
      log->LogPrint("Accel = %.3f, Duration = %.3f \n", phase.acceleration,
                    phase.duration);
    } else {
      break;
    }
  }
  log->Pop();
}

// Not your usual signum() function: it returns -1 iff x is negative, else +1.
int Sign(const float& x) {
  if (x <= -FLT_MIN) {
    return -1;
  } else {
    return 1;
  }
}

inline void VerifyControlSequence(const ControlSequence1D& controls) {
  if (!kProduction) {
    bool found_disabled = false;
    for (const auto& phase : controls.phases) {
      if (!phase.enabled) {
        found_disabled = true;
        continue;
      }
      if (found_disabled) {
        NP_CHECK(!phase.enabled);
      }
    }
  }
}

float GetAverageAccel(const ControlSequence1D& controls, const float delta_t) {
  float time_remain = delta_t;
  float weighted_total = 0;

  VerifyControlSequence(controls);

  for (const auto& control : controls.phases) {
    if (!control.enabled) {
      break;
    } else if (control.duration < time_remain) {
      weighted_total += control.acceleration * control.duration;
      time_remain -= control.duration;
    } else {
      weighted_total += control.acceleration * time_remain;
      break;
    }
  }

  return (weighted_total / delta_t);
}

float GetAccelToPreservePosition(const ControlSequence1D& controls,
                                 const float delta_t, const float& v0) {
  float time_remain = delta_t;
  float delta_x = 0;
  float v = v0;

  VerifyControlSequence(controls);

  for (const auto& control : controls.phases) {
    if (!control.enabled) {
      break;
    }

    const float& accel = control.acceleration;
    const float& t = control.duration;
    if (t < time_remain) {
      delta_x += v * t + 0.5 * accel * Sq(t);
      v += accel * t;
      time_remain -= t;
    } else {
      delta_x += v * time_remain + 0.5 * accel * Sq(time_remain);
      v += accel * time_remain;
      time_remain = 0.0;
      break;
    }
  }

  const float desired_accel = 2 * (delta_x - v0 * delta_t) / Sq(delta_t);
  return desired_accel;
}

float TimeOptimalControlZeroFinal1D(float x0, float v0, float t0,
                                    const MotionModel& motion_model,
                                    ControlSequence1D* control) {
  return TimeOptimalControlZeroFinal1D(x0, v0, t0, motion_model.a_max,
                                       motion_model.v_max, control);
}

float TimeOptimalControlZeroFinal1D(float x0, float v0, float t0, float a_max,
                                    float v_max, ControlSequence1D* controls) {
  float x = x0;
  float v = v0;
  float t = t0;

  // Check if the robot's velocity is larger than the maximum
  if (v_max < fabs(v0)) {
    // Lower velocity to maximum
    const float a = -Sign(v0) * a_max;
    const float t1 = (fabs(v0) - v_max) / a_max;

    // Update the initial conditions for the rest of the function
    x = x + v * t1 + 0.5 * a * Sq(t1);
    v = Sign(v0) * v_max;
    t += t1;

    // Add slow down phase to begining of controls
    controls->AddPhase(t1, a);
  }

  // Check if the robot is moving in the wrong direction.
  const bool wrong_direction = ((x * v) > kEpsilon);

  // The distance the robot will take to come to a halt.
  const float stopping_distance = Sq(v) / (2.0 * a_max);

  // Check if the robot will overshoot the target.
  const bool will_overshoot = (stopping_distance > fabs(x0));

  if (wrong_direction || will_overshoot) {
    // First, come to a stop, next solve the TOC problem from there.
    const float a = -Sign(v) * a_max;
    const float t1 = -v / a;
    x = x + v * t1 + 0.5 * a * Sq(t1);
    v = 0.0f;
    t += t1;
    controls->AddPhase(t1, a);
  }

  const float a = (fabs(v) < kEpsilon) ? (-Sign(x) * a_max) : (Sign(v) * a_max);
  // The time the velocity of the robot will peak if it only accelerates and
  // then decelerates to the target.
  const float peak_vel_time =
      (-fabs(v) + sqrt(0.5 * Sq(v) + a_max * fabs(x))) / a_max;
  const float peak_vel = v + a * peak_vel_time;
  if (fabs(peak_vel) > v_max) {
    // Three phases:
    // 1. Accelerate to v_max.
    // 2. Cruise at v_max.
    // 3. Decelerate to stop.
    const float t1 = (v_max - fabs(v)) / a_max;
    const float s1 = fabs(v) * t1 + 0.5 * a_max * Sq(t1);
    const float t3 = v_max / a_max;
    // const float s3 = 0.5 * a_max * Sq(t3);
    const float s3 = 0.5 * v_max * t3;
    const float s2 = fabs(x) - s1 - s3;
    const float t2 = s2 / v_max;

    controls->AddPhase(t1, a);

    if (t2 > kEpsilon) controls->AddPhase(t2, 0.0f);
    if (t3 > kEpsilon) controls->AddPhase(t3, -a);
    return (t + t1 + t2 + t3);
  } else {
    // Two phases:
    // 1. Accelerate to peak vel.
    // 2. Decelerate to stop.
    const float t1 = peak_vel_time;
    const float t2 = fabs(peak_vel / a);

    controls->AddPhase(t1, a);
    controls->AddPhase(t2, -a);

    return (t + t1 + t2);
  }
}

float TimeOptimalControlAnyFinal1D(float x0, float v0, float xf, float vf,
                                   float t0, float a_max, float v_max,
                                   ControlSequence1D* controls) {
  xf -= x0;
  x0 = 0;

  // Check if the robot's final velocity is larger than the maximum
  if (v_max < fabs(vf)) {
    LOG(ERROR) << "Attempting to reach a velocity larger than v_max. ("
               << fabs(vf) << " vs v_max " << v_max << ")";
               exit(-1);
    vf = Sign(vf) * v_max;
  }

  // Check if the robot's current velocity is larger than the maximum
  if (v_max < fabs(v0)) {
    // Lower velocity to maximum
    const float a = -Sign(v0) * a_max;
    const float t = (fabs(v0) - v_max) / a_max;
    const float x1 = v0 * t + 0.5 * a * Sq(t);
    controls->AddPhase(t, a);

    return TimeOptimalControlAnyFinal1D(x1, Sign(v0) * v_max, xf, vf, t0 + t,
                                        a_max, v_max, controls);
  }

  int accel_sign;
  float x_switch_init;
  // find the value of the switching function at initial velocity
  if (v0 > vf) {
    x_switch_init = xf + .5 * (Sq(vf) - Sq(v0)) / a_max;
    accel_sign = -1;
  } else {
    x_switch_init = xf + .5 * (Sq(v0) - Sq(vf)) / a_max;
    accel_sign = 1;
  }

  float v_peak, t1;
  // If initial conditions are on the switching function, accelerate to goal.
  // If above switching function, decelerate until intercept switching function.
  // If below, accelerate until intercept.
  if (fabs(x_switch_init - x0) < kEpsilon) {
    t1 = (vf - v0) / (accel_sign * a_max);
    controls->AddPhase(t1, accel_sign * a_max);
    return t0 + t1;
  } else if (x0 > x_switch_init) {
    accel_sign = -1;
    v_peak = -sqrt(.5 * (Sq(v0) + Sq(vf)) - a_max * xf);
    t1 = v0 / a_max + sqrt(.5 * (Sq(v0) + Sq(vf)) - a_max * xf) / a_max;
  } else {
    accel_sign = 1;
    v_peak = sqrt(.5 * (Sq(v0) + Sq(vf)) + a_max * xf);
    t1 = -v0 / a_max + sqrt(.5 * (Sq(v0) + Sq(vf)) + a_max * xf) / a_max;
  }

  // If acceleration goes beyond velocity bounds, reach v_max then coast until
  // switching function.
  if (fabs(v_peak) > v_max) {
    t1 = (v_max - accel_sign * v0) / a_max;
    const float x_switch_point =
        xf + .5 * accel_sign * (Sq(vf) - Sq(v_max)) / a_max;
    const float x_max_vel = v0 * t1 + .5 * accel_sign * a_max * Sq(t1);

    if (t1 > 0) {
      controls->AddPhase(t1, accel_sign * a_max);
    }
    const float t2 = (x_switch_point - x_max_vel) / (accel_sign * v_max);

    if (t2 > 0) {
      controls->AddPhase(t2, 0);
    }
    const float t3 = (v_max - accel_sign * vf) / a_max;
    if (t3 > 0) {
      controls->AddPhase(t3, -accel_sign * a_max);
    }

    return (t0 + t1 + t2 + t3);
  } else {
    if (t1 > 0) {
      controls->AddPhase(t1, accel_sign * a_max);
    }
    const float t2 = (v_peak - vf) / (accel_sign * a_max);
    if (t2 > 0) {
      controls->AddPhase(t2, -accel_sign * a_max);
    }
    return (t0 + t1 + t2);
  }
}

float TimeOptimalControlAnyFinal1D(float x0, float v0, float xf, float vf,
                                   float t0, const MotionModel& motion_model,
                                   ControlSequence1D* control) {
  return TimeOptimalControlAnyFinal1D(x0, v0, xf, vf, t0, motion_model.a_max,
                                      motion_model.v_max, control);
}

float TimeOptimalControlAnyFinal1D(Eigen::Vector2d x0, Eigen::Vector2d v0,
                                   Eigen::Vector2d xf, Eigen::Vector2d vf,
                                   float t0, float a_max, float v_max,
                                   ControlSequence1D* controls) {
  const Eigen::Vector2d displ = xf - x0;
  constexpr double kTolerance = 1e-10;
  const float test1 = fabs(displ.normalized().dot(v0.normalized()));
  const float test2 = fabs(displ.normalized().dot(vf.normalized()));
  const float test3 = fabs(v0.normalized().dot(vf.normalized()));
  bool is_feasible = true;
  if (fabs(test1 - 1.0f) > kTolerance) {
    LOG(ERROR) << "1D SOLVER CALLED WHILE "
               << "DISPLACEMENT VECTOR NOT PARALLEL TO INITIAL VELOCITY";
    is_feasible = false;
  } else if (fabs(test2 - 1.0f) > kTolerance) {
    LOG(ERROR) << "1D SOLVER CALLED WHILE "
               << "DISPLACEMENT VECTOR NOT PARALLEL TO FINAL VELOCITY";
    is_feasible = false;
  } else if (fabs(test3 - 1.0f) > kTolerance) {
    LOG(ERROR) << "1D SOLVER CALLED WHILE "
               << "INITIAL VELOCITY NOT PARALLEL TO FINAL VELOCITY";
    is_feasible = false;
  }

  if (!is_feasible) {
    LOG(ERROR) << "1D SOLUTION INFEASIBLE, RETURNING DO NOTHING CONTROL";
    controls->AddPhase(0, 0);
    return 0;
  }
  return TimeOptimalControlAnyFinal1D(0, v0.norm() * Sign(test1), displ.norm(),
                                      vf.norm() * Sign(test2), t0, a_max, v_max,
                                      controls);
}

float GetMaxVelocity(float v0, float vmax, const ControlSequence1D& control) {
  // The maximum velocity is going to be the velocity after the first control
  // phase
  float init_speed = fabs(v0);
  float final_speed =
      fabs(v0 + control.phases[0].acceleration * control.phases[0].duration);
  return max(min(init_speed, vmax), final_speed);
}


}  // namespace ntoc
