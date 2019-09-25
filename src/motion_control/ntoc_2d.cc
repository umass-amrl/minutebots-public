// Copyright 2017 - 2019 joydeepb@cs.umass.edu, dbalaban@cs.umass.edu,
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

#include <algorithm>
#include <vector>

#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "motion_control/ntoc_2d.h"
#include "motion_control/optimal_control_1d.h"

using Eigen::Matrix2Xf;
using Eigen::VectorXf;
using Eigen::Matrix3f;
using motion::MotionModel;

bool collect_ntoc_data = false;

namespace ntoc {

void ControlSequence2D::AddPhase(const float duration,
                                 const Vector2f& acceleration) {
  if (num_phases < 6) {
    phases[num_phases].duration = duration;
    phases[num_phases].acceleration = acceleration;
    phases[num_phases].enabled = true;
    num_phases++;
  } else {
    LOG(ERROR) << "Tried to add too many phases to a 2D Optimal Control set";
  }
}

void ControlSequence2D::Reset() {
  for (auto& phase : phases) {
    phase.duration = 0.0f;
    phase.acceleration.x() = 0.0f;
    phase.acceleration.y() = 0.0f;
    phase.enabled = false;
  }

  num_phases = 0;
}

void ControlSequence2D::LogSequence(logger::Logger* log) {
  log->LogPrint("Planned Control Sequence \n");
  log->Push();
  for (const auto& phase : phases) {
    if (phase.enabled) {
      log->LogPrint("Accel = (%.3f, %.3f), Duration = %.3f \n",
                    phase.acceleration.x(), phase.acceleration.y(),
                    phase.duration);
    } else {
      break;
    }
  }
  log->Pop();
}

void MergeCommandSchedules(const ControlSequence1D& x,
                           const ControlSequence1D& y,
                           ControlSequence2D* combined, const float time_x,
                           const float time_y) {
  constexpr bool kDebug = false;

  size_t point_x = 0;
  size_t point_y = 0;

  // Used to hold how much extra time is left on the current control period for
  // control phases that were not exhausted.
  float time_remain_x = 0.0f;
  float time_remain_y = 0.0f;

  float duration = 0;

  // Nessicarily will terminate after at most 6
  // iterations, which will prevent infinite loops.
  for (size_t i = 0;
       (i < 6) && (point_x < x.num_phases && point_y < y.num_phases); ++i) {
    const ControlPhase1D& control_x = x.phases[point_x];
    const ControlPhase1D& control_y = y.phases[point_y];

    const float time_x =
        (time_remain_x > kEpsilon) ? time_remain_x : control_x.duration;
    const float time_y =
        (time_remain_y > kEpsilon) ? time_remain_y : control_y.duration;

    if (fabs(time_x - time_y) < kEpsilon) {
      duration = (time_x + time_y) / 2;
      time_remain_x = 0.0f;
      time_remain_y = 0.0f;
      ++point_x;
      ++point_y;
    } else if (time_x < time_y) {
      duration = time_x;
      time_remain_x = 0.0f;
      time_remain_y = time_y - time_x;
      ++point_x;
    } else {
      duration = time_y;
      time_remain_x = time_x - time_y;
      time_remain_y = 0.0f;
      ++point_y;
    }

    if (kDebug) {
      std::printf("time_x, time_y: %f, %f \n", time_x, time_y);
      std::printf("x_info; pointer, time_remain: %zu, %f \n", point_x,
                  time_remain_x);
      std::printf("y_info; pointer, time_remain: %zu, %f \n", point_y,
                  time_remain_y);
    }

    combined->AddPhase(duration,
                       {control_x.acceleration, control_y.acceleration});
  }

  // Just advance Y and then break
  for (; point_y < y.num_phases; point_y++) {
    const ControlPhase1D& control_y = y.phases[point_y];
    duration = (time_remain_y > kEpsilon) ? time_remain_y : control_y.duration;
    time_remain_y = 0.0f;
    combined->AddPhase(duration, {0.0f, control_y.acceleration});
  }

  // Just advance X and then break
  for (; point_x < x.num_phases; point_x++) {
    const ControlPhase1D& control_x = x.phases[point_x];
    duration = (time_remain_x > kEpsilon) ? time_remain_x : control_x.duration;
    time_remain_x = 0.0f;
    combined->AddPhase(duration, {control_x.acceleration, 0.0f});
  }
}

void TransformCoordinatesForNTOC(const pose_2d::Pose2Df& current_pose,
                                 const Vector2f& current_velocity,
                                 const Vector2f& desired_position,
                                 Vector2f* transformed_position,
                                 Vector2f* transformed_velocity) {
  *transformed_position = current_pose.translation - desired_position;

  const Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);
  *transformed_velocity = robot_to_world_rotation * current_velocity;
}

Vector2f GetAccelToPreservePosition(const ControlSequence2D& controls,
                                    const float delta_t, const Vector2f& v0) {
  double time_remain = delta_t;
  Vector2f delta_x(0, 0);
  Vector2f v = v0;

  for (const auto& control : controls.phases) {
    if (control.enabled) {
      const Vector2f& accel = control.acceleration;
      const float& t = control.duration;
      if (t < time_remain) {
        delta_x = delta_x + v * t + 0.5 * accel * Sq(t);
        v = v + accel * t;
        time_remain -= t;
      } else {
        delta_x = delta_x + v * time_remain + 0.5 * accel * Sq(time_remain);
        v = v + accel * time_remain;
        time_remain = 0.0;
        break;
      }
    } else {
      break;
    }
  }

  return (2 * (delta_x - v0 * delta_t) / Sq(delta_t));
}

Vector2f GetAverageAccel(const ControlSequence2D& controls,
                         const float delta_t) {
  double time_remain = delta_t;
  Vector2f weighted_total(0, 0);

  for (const auto& control : controls.phases) {
    if (control.enabled) {
      const Vector2f& accel = control.acceleration;
      const float& duration = control.duration;
      if (duration < time_remain) {
        weighted_total += accel * duration;
        time_remain -= duration;
      } else {
        weighted_total += accel * time_remain;
        time_remain = 0.0;
        break;
      }
    } else {
      break;
    }
  }

  return (weighted_total / delta_t);
}

float NTOC2D(const Vector2f& x0,
             const Vector2f& v0,
             const MotionModel& motion_model,
             ControlSequence2D* control) {
  float x_velocity;
  float y_velocity;
  return NTOC2D(x0, v0, motion_model, control, &x_velocity, &y_velocity);
}


float NTOC2D(const Eigen::Vector2f& x0,
             const Eigen::Vector2f& v0,
             const MotionModel& motion_model,
             ControlSequence2D* control,
             float* max_x_velocity,
             float* max_y_velocity) {
  constexpr bool kDebug = false;
  constexpr size_t kMaxIterations = 20;
  static const float kEpsilon = 1e-4;

  control->Reset();

  float angle_max = M_PI / 2;
  float angle_min = 0;
  float angle = M_PI / 4;

  float time_x = -1.0;
  float time_y = -1.0;

  ControlSequence1D control_x;
  ControlSequence1D control_y;

  if (kDebug) {
    printf("Initial Position: (%f, %f)\n", x0.x(), x0.y());
    printf("Initial Velocity: (%f, %f)\n", v0.x(), v0.y());
    printf("Angle %f\n", angle);
  }

  for (size_t i = 0; i < kMaxIterations && (fabs(time_x - time_y) > kEpsilon ||
                                            time_x < 0 || time_y < 0);
       ++i) {
    control_x.Reset();
    control_y.Reset();
    time_x = TimeOptimalControlZeroFinal1D(
        x0.x(), v0.x(), 0.0, motion_model.Scaled(cos(angle)), &control_x);
    time_y = TimeOptimalControlZeroFinal1D(
        x0.y(), v0.y(), 0.0, motion_model.Scaled(sin(angle)), &control_y);

    // Within threshold, quit binary search.
    if (fabs(time_x - time_y) < kEpsilon) {
      break;
    }

    if (time_x < time_y) {
      angle_min = angle;
      angle = (angle_min + angle_max) / 2;
    } else if (time_x > time_y) {
      angle_max = angle;
      angle = (angle_min + angle_max) / 2;
    }
  }
  *max_x_velocity = GetMaxVelocity(v0.x(),
                                   motion_model.Scaled(cos(angle)).v_max,
                                   control_x);
  *max_y_velocity = GetMaxVelocity(v0.y(),
                                   motion_model.Scaled(sin(angle)).v_max,
                                   control_y);
  MergeCommandSchedules(control_x, control_y, control, time_x, time_y);

  return std::max(time_x, time_y);
}

std::vector<Vector2f> GetPath(const Vector2f& x0, const Vector2f& v0,
                              const ControlSequence2D& controls,
                              const size_t num_path_points,
                              const float total_time) {
  Vector2f x = x0;
  Vector2f v = v0;

  Vector2f x_prev = x;
  Vector2f v_prev = v;

  std::vector<Vector2f> path_points;

  for (const auto& phase : controls.phases) {
    if (phase.enabled) {
      const Vector2f& a = phase.acceleration;
      const float phase_time = phase.duration;

      const int phase_steps =
          1 + floor(num_path_points * phase_time / total_time);
      const float step_duration = phase_time / phase_steps;

      for (int i = 0; i < phase_steps; i++) {
        x = x_prev + v_prev * step_duration * i +
            0.5 * a * step_duration * step_duration * i * i;
        v = v_prev + a * step_duration * i;

        path_points.push_back(x);
      }

      x = x_prev + v_prev * phase_time + .5 * a * phase_time * phase_time;
      v = v_prev + a * phase_time;

      x_prev = x;
      v_prev = v;
    } else {
      break;
    }
  }

  return path_points;
}

bool NTOCFinished(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d xf) {
  return (x0 - xf).norm() < 5.0 && v0.norm() < 50.0;
}

}  // namespace ntoc
