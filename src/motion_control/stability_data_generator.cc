// Copyright 2017 - 2018 dbalaban@cs.umass.edu
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

#include <stdio.h>
#include <fstream>

#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "motion_control/ntoc_2d.h"
#include "motion_control/tsocs_old.h"
#include "util/random.h"

using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using math_util::Sign;

// actually number of divisions - 1
const float time_step_duration = 1.0 / 60.0;

int main(int argc, char** argv) {
  logger::NetLogger logger(
      logger::NetLogger(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT));
  //  must take at least one parameter to write to given file
  const char* write_to = argv[1];
  util_random::Random random(1241235141);

  Eigen::Vector2f x0, v0;
  x0.x() = random.UniformRandom(-kHalfFieldLength, kHalfFieldLength);
  x0.y() = random.UniformRandom(-kHalfFieldWidth, kHalfFieldWidth);
  //   x0.x() = 0.0;
  //   x0.y() = 0.0;
  v0.x() = random.UniformRandom(-kMaxRobotVelocity, kMaxRobotVelocity);
  v0.y() = random.UniformRandom(-kMaxRobotVelocity, kMaxRobotVelocity);
  v0 = v0 / sqrt(2.0);

  const Eigen::Vector2f xf(0.0, 0.0);
  const Eigen::Vector2f vf(0.0, 0.0);

  std::ofstream out;

  out.open(write_to);
  out << "t, TSOCS-T, NTOC-T \n";

  float time_elapsed = 0.0;
  float tsocs_T = INFINITY, ntoc_T = INFINITY;
  Eigen::Vector2f tsocs_x = x0;
  Eigen::Vector2f ntoc_x = x0;
  Eigen::Vector2f tsocs_v = v0;
  Eigen::Vector2f ntoc_v = v0;

  bool tsocs_is_finished = false;
  bool ntoc_is_finished = false;

  while (!tsocs_is_finished || !ntoc_is_finished) {
    SolutionParameters params;
    bool success = tsocs::GetSolution(tsocs_x.cast<double>(),
                                      tsocs_v.cast<double>(),
                                      xf.cast<double>(),
                                      vf.cast<double>(),
                                      kMaxRobotAcceleration,
                                      &params);
    tsocs_T = params.T;

    MotionModel motion_model(kMaxRobotAcceleration, INFINITY);
    ControlSequence2D ntoc_controls;
    ntoc_T = NTOC2D(ntoc_x, ntoc_v, motion_model, &ntoc_controls);

    out << time_elapsed << ", ";
    if (tsocs_is_finished || !success) {
      out << "NA, ";
    } else {
      out << tsocs_T << ", ";
    }

    if (ntoc_is_finished) {
      out << "NA";
    } else {
      out << ntoc_T;
    }
    out << " \n";

    tsocs_is_finished = tsocs_is_finished || tsocs_T < time_step_duration;
    ntoc_is_finished = ntoc_is_finished || ntoc_T < time_step_duration;

    Eigen::Vector2d tsocs_vt;
    Eigen::Vector2d tsocs_xt;
    tsocs::GetState(tsocs_x.cast<double>(), tsocs_v.cast<double>(), &tsocs_xt,
             &tsocs_vt, kMaxRobotAcceleration, time_step_duration, params);
    tsocs_x = tsocs_xt.cast<float>();
    tsocs_v = tsocs_vt.cast<float>();

    Eigen::Vector2f ntoc_accel =
        GetAverageAccel(ntoc_controls, time_step_duration);
    ntoc_x +=
        ntoc_v * time_step_duration + 0.5 * ntoc_accel * Sq(time_step_duration);
    ntoc_v += ntoc_accel * time_step_duration;

    time_elapsed += time_step_duration;
  }

  out.close();

  logger.SendData();
}
