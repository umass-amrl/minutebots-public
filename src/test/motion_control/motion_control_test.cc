// Copyright 2018 dbalaban@cs.umass.edu
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

#include <cmath>
#include <iostream>
#include <vector>

#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

#include "motion_control/ntoc_2d.h"
#include "motion_control/optimal_control_1d.h"
#include "shared/common_includes.h"
#include "src/motion_control/ball_interception.h"
#include "src/motion_control/ntoc_2d.h"
#include "src/motion_control/optimal_control_1d.h"
#include "src/motion_control/tsocs_old.h"

using std::vector;
using ntoc::ControlPhase1D;
using ntoc::ControlSequence1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using math_util::Sign;

namespace tsocs {

void CalcFinalPose(float x0, float v0, float a_max, float v_max,
                   const ControlSequence1D& control, float* x_ptr,
                   float* v_ptr) {
  static const bool kDebug = false;
  CHECK_NOTNULL(x_ptr);
  CHECK_NOTNULL(v_ptr);
  float& x = *x_ptr;
  float& v = *v_ptr;

  x = x0;
  v = v0;
  for (const auto& phase : control.phases) {
    const float t = phase.duration;
    const float a = phase.acceleration;
    if (kDebug) {
      printf("phase: t = %f, a = %f, x0 = %f, v0 = %f \n", t, a, x, v);
    }
    x = x + v * t + 0.5 * a * Sq(t);
    v = v + a * t;
  }
}

void CalcFinalPose2D(Eigen::Vector2f x0, Eigen::Vector2f v0,
                     MotionModel motion_model, ControlSequence2D controls,
                     Eigen::Vector2f* x_ptr, Eigen::Vector2f* v_ptr) {
  static const bool kDebug = false;
  Eigen::Vector2f& x = *x_ptr;
  Eigen::Vector2f& v = *v_ptr;

  x = x0;
  v = v0;
  for (const auto& phase : controls.phases) {
    const float t = phase.duration;
    Eigen::Vector2f a = phase.acceleration;
    if (kDebug) {
      std::printf("x, y, v_x, v_y: %f, %f, %f, %f \n", x.x(), x.y(), v.x(),
                  v.y());
      std::printf("acceleration: %f, %f; duration: %f \n", a.x(), a.y(), t);
    }
    x = x + v * t + 0.5 * a * Sq(t);
    v = v + a * t;
  }
  if (kDebug) {
    std::printf("x, y, v_x, v_y: %f, %f, %f, %f \n", x.x(), x.y(), v.x(),
                v.y());
  }
}

TEST(MotionControlTest, Optimal1D) {
  static const float kEpsilon = 1e-6;
  {
    float x0 = 3, v0 = 0.0, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    float time =
        TimeOptimalControlZeroFinal1D(x0, v0, t0, a_max, v_max, &control);
    ASSERT_FLOAT_EQ(2.1908902300206643, time);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = -3, v0 = 0.0, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    float time =
        TimeOptimalControlZeroFinal1D(x0, v0, t0, a_max, v_max, &control);
    ASSERT_FLOAT_EQ(2.1908902300206643, time);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    // Accelerate from -1m/s to 3m/s, cruise for 1s, decelerate for 1.2s.
    float x0 = -6.4, v0 = -1, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    float time =
        TimeOptimalControlZeroFinal1D(x0, v0, t0, a_max, v_max, &control);
    ASSERT_FLOAT_EQ(3.8, time);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = -6.4, v0 = -1, t0 = 0, a_max = 2.333859, v_max = 2.800632;
    ControlSequence1D control;
    TimeOptimalControlZeroFinal1D(x0, v0, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_LE(3, (int)control.num_phases);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    // Accelerate from 1m/s to -3m/s, cruise for 1s, decelerate for 1.2s.
    float x0 = 6.4, v0 = 1, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    float time =
        TimeOptimalControlZeroFinal1D(x0, v0, t0, a_max, v_max, &control);
    ASSERT_FLOAT_EQ(3.8, time);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = 0.5, v0 = -1.0, t0 = 0, a_max = 1.0, v_max = 1.0;
    float x = 0, v = 0;
    ControlSequence1D control;
    float time =
        TimeOptimalControlZeroFinal1D(x0, v0, t0, a_max, v_max, &control);
    ASSERT_FLOAT_EQ(1, time);
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    // Simple case: -x, moving towards origin, decelerate to a halt.
    float x0 = -0.5, v0 = 1.0, t0 = 0, a_max = 1.0, v_max = 1.0;
    float x = 0, v = 0;
    ControlSequence1D control;
    float time =
        TimeOptimalControlZeroFinal1D(x0, v0, t0, a_max, v_max, &control);
    ASSERT_FLOAT_EQ(1, time);
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = -0.5, v0 = 1.0, t0 = 0, a_max = 2.0, v_max = 1.0;
    float x = 0, v = 0;
    ControlSequence1D control;
    // float time = TimeOptimalControl1D(x0, v0, t0, a_max, v_max, &control);
    TimeOptimalControlZeroFinal1D(x0, v0, t0, a_max, v_max, &control);
    // ASSERT_FLOAT_EQ(1, time);
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = -0.5, v0 = 1.0, t0 = 0, a_max = 0.5, v_max = 1.0;
    float x = 0, v = 0;
    ControlSequence1D control;
    // float time = TimeOptimalControl1D(x0, v0, t0, a_max, v_max, &control);
    // ASSERT_FLOAT_EQ(1, time);
    TimeOptimalControlZeroFinal1D(x0, v0, t0, a_max, v_max, &control);
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = 0.0, v0 = 2.0, t0 = 0, a_max = 1.0, v_max = 1.0;
    ControlSequence1D control;
    TimeOptimalControlZeroFinal1D(x0, v0, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = -3, v0 = 1.0, t0 = 0, a_max = 0.758670, v_max = 0.910404;
    ControlSequence1D control;
    TimeOptimalControlZeroFinal1D(x0, v0, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = -6.4, v0 = -1.0, t0 = 0, a_max = 2.384996, v_max = 2.861995;
    ControlSequence1D control;
    TimeOptimalControlZeroFinal1D(x0, v0, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }
}

TEST(MotionControlTest, OptimalAny1D) {
  static const float kEpsilon = 1e-6;
  {
    float x0 = 3, v0 = 0.0, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    float time =
        TimeOptimalControlAnyFinal1D(x0, v0, 0, 0, t0, a_max, v_max, &control);
    ASSERT_FLOAT_EQ(2.1908902300206643, time);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = -3, v0 = 0.0, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    float time =
        TimeOptimalControlAnyFinal1D(x0, v0, 0, 0, t0, a_max, v_max, &control);
    ASSERT_FLOAT_EQ(2.1908902300206643, time);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    // Accelerate from -1m/s to 3m/s, cruise for 1s, decelerate for 1.2s.
    float x0 = -6.4, v0 = -1, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    float time =
        TimeOptimalControlAnyFinal1D(x0, v0, 0, 0, t0, a_max, v_max, &control);
    ASSERT_FLOAT_EQ(3.8, time);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = -6.4, v0 = -1, t0 = 0, a_max = 2.333859, v_max = 2.800632;
    ControlSequence1D control;
    TimeOptimalControlAnyFinal1D(x0, v0, 0, 0, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_LE(3, (int)control.num_phases);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    // Accelerate from 1m/s to -3m/s, cruise for 1s, decelerate for 1.2s.
    float x0 = 6.4, v0 = 1, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    float time =
        TimeOptimalControlAnyFinal1D(x0, v0, 0, 0, t0, a_max, v_max, &control);
    ASSERT_FLOAT_EQ(3.8, time);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = 0.5, v0 = -1.0, t0 = 0, a_max = 1.0, v_max = 1.0;
    float x = 0, v = 0;
    ControlSequence1D control;
    float time =
        TimeOptimalControlAnyFinal1D(x0, v0, 0, 0, t0, a_max, v_max, &control);
    ASSERT_FLOAT_EQ(1, time);
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    // Simple case: -x, moving towards origin, decelerate to a halt.
    float x0 = -0.5, v0 = 1.0, t0 = 0, a_max = 1.0, v_max = 1.0;
    float x = 0, v = 0;
    ControlSequence1D control;
    float time =
        TimeOptimalControlAnyFinal1D(x0, v0, 0, 0, t0, a_max, v_max, &control);
    ASSERT_FLOAT_EQ(1, time);
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = -0.5, v0 = 1.0, t0 = 0, a_max = 2.0, v_max = 1.0;
    float x = 0, v = 0;
    ControlSequence1D control;
    // float time = TimeOptimalControl1D(x0, v0, t0, a_max, v_max, &control);
    TimeOptimalControlAnyFinal1D(x0, v0, 0, 0, t0, a_max, v_max, &control);
    // ASSERT_FLOAT_EQ(1, time);
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = -0.5, v0 = 1.0, t0 = 0, a_max = 0.5, v_max = 1.0;
    float x = 0, v = 0;
    ControlSequence1D control;
    // float time = TimeOptimalControl1D(x0, v0, t0, a_max, v_max, &control);
    // ASSERT_FLOAT_EQ(1, time);
    TimeOptimalControlAnyFinal1D(x0, v0, 0, 0, t0, a_max, v_max, &control);
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = 0.0, v0 = 2.0, t0 = 0, a_max = 1.0, v_max = 1.0;
    ControlSequence1D control;
    TimeOptimalControlAnyFinal1D(x0, v0, 0, 0, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = -3, v0 = 1.0, t0 = 0, a_max = 0.758670, v_max = 0.910404;
    ControlSequence1D control;
    TimeOptimalControlAnyFinal1D(x0, v0, 0, 0, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = -6.4, v0 = -1.0, t0 = 0, a_max = 2.384996, v_max = 2.861995;
    ControlSequence1D control;
    TimeOptimalControlAnyFinal1D(x0, v0, 0, 0, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x));
    ASSERT_GE(kEpsilon, fabs(v));
  }

  {
    float x0 = 3, v0 = 0.0, xf = 5, vf = 2.5, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    TimeOptimalControlAnyFinal1D(x0, v0, xf, vf, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x - xf));
    ASSERT_GE(kEpsilon, fabs(v - vf));
  }

  {
    float x0 = 3, v0 = 0.0, xf = 5, vf = 2.5, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    TimeOptimalControlAnyFinal1D(x0, v0, xf, vf, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x - xf));
    ASSERT_GE(kEpsilon, fabs(v - vf));
  }

  {
    float x0 = 3, v0 = 1.0, xf = 7, vf = 2, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    TimeOptimalControlAnyFinal1D(x0, v0, xf, vf, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x - xf));
    ASSERT_GE(kEpsilon, fabs(v - vf));
  }

  {
    float x0 = 3, v0 = 1.0, xf = 7, vf = -2, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    TimeOptimalControlAnyFinal1D(x0, v0, xf, vf, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x - xf));
    ASSERT_GE(kEpsilon, fabs(v - vf));
  }

  {
    float x0 = 3, v0 = 0.0, xf = -5, vf = -2.5, t0 = 0, a_max = 2.5,
          v_max = 3.0;
    ControlSequence1D control;
    TimeOptimalControlAnyFinal1D(x0, v0, xf, vf, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x - xf));
    ASSERT_GE(kEpsilon, fabs(v - vf));
  }

  {
    float x0 = 3, v0 = 2.0, xf = 5, vf = -2.5, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    TimeOptimalControlAnyFinal1D(x0, v0, xf, vf, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x - xf));
    ASSERT_GE(kEpsilon, fabs(v - vf));
  }

  {
    float x0 = 3, v0 = 3.0, xf = 0, vf = -3, t0 = 0, a_max = 2.5, v_max = 3.0;
    ControlSequence1D control;
    TimeOptimalControlAnyFinal1D(x0, v0, xf, vf, t0, a_max, v_max, &control);
    float x = 0, v = 0;
    CalcFinalPose(x0, v0, a_max, v_max, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x - xf));
    ASSERT_GE(kEpsilon, fabs(v - vf));
  }
}

TEST(MotionControlTest, NearOptimal2D) {
  static const float kEpsilon = 2e-3;
  {
    const float a_max = 2.5 * 2 / sqrt(2), v_max = 3.0 * 2 / sqrt(2);
    const Eigen::Vector2f x0(3, 3);
    const Eigen::Vector2f v0(0, 0);
    const MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    float time = NTOC2D(x0, v0, motion_model, &control);
    ASSERT_FLOAT_EQ(2.1908902300206643, time);

    Eigen::Vector2f x;
    Eigen::Vector2f v;
    CalcFinalPose2D(x0, v0, motion_model, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x.x()));
    ASSERT_GE(kEpsilon, fabs(x.y()));
    ASSERT_GE(kEpsilon, fabs(v.x()));
    ASSERT_GE(kEpsilon, fabs(v.y()));
  }

  {
    const float a_max = 2.5 * 2 / sqrt(2), v_max = 3.0 * 2 / sqrt(2);
    const Eigen::Vector2f x0(3, -3);
    const Eigen::Vector2f v0(0, 0);
    const MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    float time = NTOC2D(x0, v0, motion_model, &control);
    ASSERT_FLOAT_EQ(2.1908902300206643, time);

    Eigen::Vector2f x;
    Eigen::Vector2f v;
    CalcFinalPose2D(x0, v0, motion_model, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x.x()));
    ASSERT_GE(kEpsilon, fabs(x.y()));
    ASSERT_GE(kEpsilon, fabs(v.x()));
    ASSERT_GE(kEpsilon, fabs(v.y()));
  }

  {
    const float a_max = 2.5, v_max = 3.0;
    const Eigen::Vector2f x0(3, -3);
    const Eigen::Vector2f v0(0, 0);
    const MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    float time = NTOC2D(x0, v0, motion_model, &control);
    ASSERT_LE(2.1908902300206643, time);

    Eigen::Vector2f x;
    Eigen::Vector2f v;
    CalcFinalPose2D(x0, v0, motion_model, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x.x()));
    ASSERT_GE(kEpsilon, fabs(x.y()));
    ASSERT_GE(kEpsilon, fabs(v.x()));
    ASSERT_GE(kEpsilon, fabs(v.y()));
  }

  {
    const float a_max = 2.5, v_max = 3.0;
    const Eigen::Vector2f x0(3, -6.4);
    const Eigen::Vector2f v0(0, -1);
    const MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    NTOC2D(x0, v0, motion_model, &control);

    Eigen::Vector2f x;
    Eigen::Vector2f v;
    CalcFinalPose2D(x0, v0, motion_model, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x.x()));
    ASSERT_GE(kEpsilon, fabs(x.y()));
    ASSERT_GE(kEpsilon, fabs(v.x()));
    ASSERT_GE(kEpsilon, fabs(v.y()));
  }

  {
    const float a_max = 2.5, v_max = 3.0;
    const Eigen::Vector2f x0(-3, 6.4);
    const Eigen::Vector2f v0(0, 1);
    const MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    NTOC2D(x0, v0, motion_model, &control);

    Eigen::Vector2f x;
    Eigen::Vector2f v;
    CalcFinalPose2D(x0, v0, motion_model, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x.x()));
    ASSERT_GE(kEpsilon, fabs(x.y()));
    ASSERT_GE(kEpsilon, fabs(v.x()));
    ASSERT_GE(kEpsilon, fabs(v.y()));
  }

  {
    const float a_max = 2.5, v_max = 3.0;
    const Eigen::Vector2f x0(-3, -6.4);
    const Eigen::Vector2f v0(0, -1);
    const MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    NTOC2D(x0, v0, motion_model, &control);

    Eigen::Vector2f x;
    Eigen::Vector2f v;
    CalcFinalPose2D(x0, v0, motion_model, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x.x()));
    ASSERT_GE(kEpsilon, fabs(x.y()));
    ASSERT_GE(kEpsilon, fabs(v.x()));
    ASSERT_GE(kEpsilon, fabs(v.y()));
  }

  {
    const float a_max = 2.5, v_max = 3.0;
    const Eigen::Vector2f x0(-3, -6.4);
    const Eigen::Vector2f v0(1, -1);
    const MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    NTOC2D(x0, v0, motion_model, &control);

    Eigen::Vector2f x;
    Eigen::Vector2f v;
    CalcFinalPose2D(x0, v0, motion_model, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x.x()));
    ASSERT_GE(kEpsilon, fabs(x.y()));
    ASSERT_GE(kEpsilon, fabs(v.x()));
    ASSERT_GE(kEpsilon, fabs(v.y()));
  }

  {
    const float a_max = 2.5;
    const float v_max = 2.0;
    const Eigen::Vector2f x0(.5, -6.4);
    const Eigen::Vector2f v0(-1, -1);
    const MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    NTOC2D(x0, v0, motion_model, &control);

    Eigen::Vector2f x;
    Eigen::Vector2f v;
    CalcFinalPose2D(x0, v0, motion_model, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x.x()));
    ASSERT_GE(kEpsilon, fabs(x.y()));
    ASSERT_GE(kEpsilon, fabs(v.x()));
    ASSERT_GE(kEpsilon, fabs(v.y()));
  }

  {
    const float a_max = 1, v_max = 3.0;
    const Eigen::Vector2f x0(.5, 0);
    const Eigen::Vector2f v0(-1, 2);
    const MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    NTOC2D(x0, v0, motion_model, &control);

    Eigen::Vector2f x;
    Eigen::Vector2f v;
    CalcFinalPose2D(x0, v0, motion_model, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x.x()));
    ASSERT_GE(kEpsilon, fabs(x.y()));
    ASSERT_GE(kEpsilon, fabs(v.x()));
    ASSERT_GE(kEpsilon, fabs(v.y()));
  }

  {
    const float a_max = 1, v_max = 3.0;
    const Eigen::Vector2f x0(1.5, 0);
    const Eigen::Vector2f v0(0, 0);
    const MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    NTOC2D(x0, v0, motion_model, &control);

    Eigen::Vector2f x;
    Eigen::Vector2f v;
    CalcFinalPose2D(x0, v0, motion_model, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x.x()));
    ASSERT_GE(kEpsilon, fabs(x.y()));
    ASSERT_GE(kEpsilon, fabs(v.x()));
    ASSERT_GE(kEpsilon, fabs(v.y()));
  }

  {
    const float a_max = 1, v_max = 3.0;
    const Eigen::Vector2f x0(0, 1.5);
    const Eigen::Vector2f v0(0, 0);
    const MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    NTOC2D(x0, v0, motion_model, &control);

    Eigen::Vector2f x;
    Eigen::Vector2f v;
    CalcFinalPose2D(x0, v0, motion_model, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x.x()));
    ASSERT_GE(kEpsilon, fabs(x.y()));
    ASSERT_GE(kEpsilon, fabs(v.x()));
    ASSERT_GE(kEpsilon, fabs(v.y()));
  }

  {
    const float a_max = kMaxRobotAcceleration / 1000;
    const float v_max = kMaxRobotVelocity / 1000;
    const Eigen::Vector2f x0(-.5, .25);
    const Eigen::Vector2f v0(2.5, 5);
    const MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    NTOC2D(x0, v0, motion_model, &control);

    Eigen::Vector2f x;
    Eigen::Vector2f v;
    CalcFinalPose2D(x0, v0, motion_model, control, &x, &v);
    ASSERT_GE(kEpsilon, fabs(x.x()));
    ASSERT_GE(kEpsilon, fabs(x.y()));
    ASSERT_GE(kEpsilon, fabs(v.x()));
    ASSERT_GE(kEpsilon, fabs(v.y()));
  }
}

TEST(MotionControlTest, Ball_Interception) {
  {
    Eigen::Vector2d robot_x0(0, 0);
    Eigen::Vector2d robot_v0(0, 0);
    Eigen::Vector2d ball_x0(0, 20);
    Eigen::Vector2d ball_v0(0, -1);

    const double ball_accel = .01;
    const double max_accel = 1;

    SolutionParameters params;
    const bool didIntercept = GetInterceptSolution(
        robot_x0, robot_v0, ball_x0, ball_v0, ball_accel, max_accel, &params);
    LOG(INFO) << "stopping time: " << 1 / .01 << std::endl;
    LOG(INFO) << "a, b, c, d, T: " << params.a << ", " << params.b << ", "
              << params.c << ", " << params.d << ", " << params.T << std::endl;
    ASSERT_TRUE(didIntercept);
  }

  {
    Eigen::Vector2d robot_x0(0, 0);
    Eigen::Vector2d robot_v0(0, 0);
    Eigen::Vector2d ball_x0(0, 10);
    Eigen::Vector2d ball_v0(1, 0);

    const double ball_accel = .01;
    const double max_accel = 1;

    SolutionParameters params;
    const bool didIntercept = GetInterceptSolution(
        robot_x0, robot_v0, ball_x0, ball_v0, ball_accel, max_accel, &params);
    ASSERT_TRUE(didIntercept);
  }

  {
    Eigen::Vector2d robot_x0(0, 0);
    Eigen::Vector2d robot_v0(-1, 0);
    Eigen::Vector2d ball_x0(0, 10);
    Eigen::Vector2d ball_v0(1, 0);

    const double ball_accel = .01;
    const double max_accel = 1;

    SolutionParameters params;
    const bool didIntercept = GetInterceptSolution(
        robot_x0, robot_v0, ball_x0, ball_v0, ball_accel, max_accel, &params);
    ASSERT_TRUE(didIntercept);
  }

  {
    Eigen::Vector2d robot_x0(0, 0);
    Eigen::Vector2d robot_v0(0, -1);
    Eigen::Vector2d ball_x0(0, 10);
    Eigen::Vector2d ball_v0(1, 0);

    const double ball_accel = .01;
    const double max_accel = 1;

    SolutionParameters params;
    const bool didIntercept = GetInterceptSolution(
        robot_x0, robot_v0, ball_x0, ball_v0, ball_accel, max_accel, &params);
    ASSERT_TRUE(didIntercept);
  }

  {
    Eigen::Vector2d robot_x0(0, 0);
    Eigen::Vector2d robot_v0(-1, -1);
    Eigen::Vector2d ball_x0(0, 10);
    Eigen::Vector2d ball_v0(1, 0);

    const double ball_accel = .01;
    const double max_accel = 1;

    SolutionParameters params;
    const bool didIntercept = GetInterceptSolution(
        robot_x0, robot_v0, ball_x0, ball_v0, ball_accel, max_accel, &params);
    ASSERT_TRUE(didIntercept);
  }

  {
    Eigen::Vector2d robot_x0(0, 0);
    Eigen::Vector2d robot_v0(0, 0);
    Eigen::Vector2d ball_x0(0, 10);
    Eigen::Vector2d ball_v0(0, -1);

    const double ball_accel = .1;
    const double max_accel = 1;

    SolutionParameters params;
    const bool didIntercept = GetInterceptSolution(
        robot_x0, robot_v0, ball_x0, ball_v0, ball_accel, max_accel, &params);
    ASSERT_TRUE(didIntercept);
  }

  {
    Eigen::Vector2d robot_x0(0, 0);
    Eigen::Vector2d robot_v0(0, 0);
    Eigen::Vector2d ball_x0(0, 10);
    Eigen::Vector2d ball_v0(1, 0);

    const double ball_accel = .1;
    const double max_accel = 1;

    SolutionParameters params;
    const bool didIntercept = GetInterceptSolution(
        robot_x0, robot_v0, ball_x0, ball_v0, ball_accel, max_accel, &params);
    ASSERT_TRUE(didIntercept);
  }

  {
    Eigen::Vector2d robot_x0(0, 0);
    Eigen::Vector2d robot_v0(-1, 0);
    Eigen::Vector2d ball_x0(0, 10);
    Eigen::Vector2d ball_v0(1, 0);

    const double ball_accel = .1;
    const double max_accel = 1;

    SolutionParameters params;
    const bool didIntercept = GetInterceptSolution(
        robot_x0, robot_v0, ball_x0, ball_v0, ball_accel, max_accel, &params);
    ASSERT_TRUE(didIntercept);
  }

  {
    Eigen::Vector2d robot_x0(0, 0);
    Eigen::Vector2d robot_v0(0, -1);
    Eigen::Vector2d ball_x0(0, 10);
    Eigen::Vector2d ball_v0(1, 0);

    const double ball_accel = .1;
    const double max_accel = 1;

    SolutionParameters params;
    const bool didIntercept = GetInterceptSolution(
        robot_x0, robot_v0, ball_x0, ball_v0, ball_accel, max_accel, &params);
    ASSERT_TRUE(didIntercept);
  }

  {
    Eigen::Vector2d robot_x0(0, 0);
    Eigen::Vector2d robot_v0(-1, -1);
    Eigen::Vector2d ball_x0(0, 10);
    Eigen::Vector2d ball_v0(1, 0);

    const double ball_accel = .1;
    const double max_accel = 1;

    SolutionParameters params;
    const bool didIntercept = GetInterceptSolution(
        robot_x0, robot_v0, ball_x0, ball_v0, ball_accel, max_accel, &params);
    ASSERT_TRUE(didIntercept);
  }
}

TEST(MotionControlTest, TSOCS) {
  unsigned int rand_seed_ = 1;

  {
    Eigen::Vector2d x0(0.157502, -0.325761);
    Eigen::Vector2d v0(0.110324, -0.330132);
    Eigen::Vector2d xf(0, 0);
    Eigen::Vector2d vf(0, 0);

    SolutionParameters params;
    LOG(INFO) << "x0: " << x0.x() << ", " << x0.y() << "\n";
    LOG(INFO) << "v0: " << v0.x() << ", " << v0.y() << "\n";
    const bool foundSolution = GetSolution(x0, v0, xf, vf, 1, &params);
    ASSERT_TRUE(foundSolution);
  }

  {
    Eigen::Vector2d x0;
    Eigen::Vector2d v0;
    Eigen::Vector2d xf;
    Eigen::Vector2d vf;

    for (int i = 0; i < 12; i++) {
      float x1 = -kHalfFieldLength +
                 2 * kHalfFieldLength *
                     static_cast<float>(rand_r(&rand_seed_)) /
                     static_cast<float>(RAND_MAX);
      float x2 = -kHalfFieldWidth +
                 2 * kHalfFieldWidth * static_cast<float>(rand_r(&rand_seed_)) /
                     static_cast<float>(RAND_MAX);

      float v1 = -kMaxRobotVelocity +
                 2 * kMaxRobotVelocity *
                     static_cast<float>(rand_r(&rand_seed_)) /
                     static_cast<float>(RAND_MAX);
      float v2 = -kMaxRobotVelocity +
                 2 * kMaxRobotVelocity *
                     static_cast<float>(rand_r(&rand_seed_)) /
                     static_cast<float>(RAND_MAX);

      v1 = v1 * sqrt(2) / 2;
      v2 = v2 * sqrt(2) / 2;

      x0 << x1, x2;
      v0 << v1, v2;
      xf << 0, 0;
      vf << 0, 0;
    }

    SolutionParameters params;
    LOG(INFO) << "x0: " << x0.x() << ", " << x0.y() << "\n";
    LOG(INFO) << "v0: " << v0.x() << ", " << v0.y() << "\n";
    const bool foundSolution =
        GetSolution(x0, v0, xf, vf, kMaxRobotAcceleration, &params);
    ASSERT_TRUE(foundSolution);
  }
}

}  // namespace tsocs
