// Copyright 2011-2018 dbalaban@cs.umass.edu
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
#include "motion_control/tsocs_old.h"
#include "motion_control/ball_interception.h"
#include "util/random.h"

// actually number of divisions - 1
const unsigned int num_divisions = 1000;
const float max_ball_speed = 8000;

int main(int argc, char **argv) {
  logger::NetLogger logger(logger::NetLogger(DATA_STREAM_DEBUG_IP,
                                             DATA_STREAM_DEBUG_PORT));
  //  must take at least one parameter to write to given file
  const char* write_to = argv[1];
  util_random::Random random(1241235141);

  Eigen::Vector2d x0, v0;
  float ball_speed = random.UniformRandom(1e-5, max_ball_speed);
  x0.x() = random.UniformRandom(-kHalfFieldLength, kHalfFieldLength);
  x0.y() = random.UniformRandom(-kHalfFieldWidth, kHalfFieldWidth);
//   x0.x() = 0.0;
//   x0.y() = 0.0;
  v0.x() = random.UniformRandom(-kMaxRobotVelocity, kMaxRobotVelocity);
  v0.y() = random.UniformRandom(-kMaxRobotVelocity, kMaxRobotVelocity);
  v0 = v0 / sqrt(2.0);

  LOG(INFO) << "Initial Robot Pos: " << x0.x() << ", " << x0.y() << "\n";
  LOG(INFO) << "Initial Robot Vel: " << v0.x() << ", " << v0.y() << "\n";

  std::ofstream out;

  out.open(write_to);
  out << "t, T, ball_pos, ball_speed \n";

  const float rest_time = ball_speed / kBallAcceleration;
  for (unsigned int i = 0; i <= num_divisions; i++) {
    const float t_i = i * rest_time / num_divisions;
    const float ball_x = ball_speed * t_i - 0.5 * kBallAcceleration * Sq(t_i);
    const float ball_v = ball_speed - kBallAcceleration * t_i;

    const Eigen::Vector2d ball_pos(ball_x, 0.0);
    const Eigen::Vector2d ball_vel(ball_v, 0.0);

    SolutionParameters params;
    bool success = tsocs::GetSolution(x0, v0, ball_pos,
                               ball_vel, kMaxRobotAcceleration,
                               &params);
    if (success) {
      out << t_i << ", " << params.T
          << ", " << ball_x << ", " << ball_v << "\n";
    } else {
      out << t_i <<  ", NA, " << ball_x << ", " << ball_v << "\n";
    }
  }

  const Eigen::Vector2d ball_pos(0.0, 0.0);
  const Eigen::Vector2d ball_vel(ball_speed, 0.0);

  SolutionParameters solution;
  solution.isInitialized = false;
  bool is_intercept_success =
      GetInterceptSolution(x0, v0, ball_pos, ball_vel,
                          kBallAcceleration,
                          kMaxRobotAcceleration,
                          &solution);
  if (is_intercept_success) {
    Eigen::Vector2d expected_robot_pos;
    Eigen::Vector2d expected_robot_vel;
    Eigen::Vector2d expected_ball_pos;
    Eigen::Vector2d expected_ball_vel;

    GetState(x0, v0, ball_pos, ball_vel, &expected_robot_pos,
&expected_robot_vel,
            &expected_ball_pos, &expected_ball_vel, kMaxRobotAcceleration,
            solution.T, kBallAcceleration, solution);

    std::vector<Vector2f> intercept_path_points = tsocs::GetPath(
        x0, v0, kMaxRobotAcceleration, 20, solution);

    LOG(INFO) << "Expected Intercept Robot Pos: "
              << expected_robot_pos.transpose() << "\n";
    LOG(INFO) << "Expected Intercept Robot Vel: "
              << expected_robot_vel.transpose() << "\n";
    LOG(INFO) << "Expected Intercept Ball Pos: "
              << expected_ball_pos.transpose() << "\n";
    LOG(INFO) << "Expected Intercept Ball Vel: "
              << expected_ball_vel.transpose() << "\n";
    LOG(INFO) << "Interception Solution Parameters: "
              << solution.a << ", " << solution.b << ", "
              << solution.c << ", " << solution.d << ", "
              << solution.T<< std::endl;

    out << solution.T << ", " << solution.T << ", "
        << expected_ball_pos.norm() << ", "
        << expected_ball_vel.norm() << "\n";

    SolutionParameters params;
    bool success = tsocs::GetSolution(x0, v0, expected_ball_pos,
                                expected_ball_vel, kMaxRobotAcceleration,
                                &params);
    tsocs::GetState(x0, v0, &expected_robot_pos, &expected_robot_vel,
            kMaxRobotAcceleration, params.T, params);
    std::vector<Vector2f> tsocs_path_points = tsocs::GetPath(
        x0, v0, kMaxRobotAcceleration, 30, params);

    LOG(INFO) << "TSOCS Solution Robot Pos: "
              << expected_robot_pos.transpose() << "\n";
    LOG(INFO) << "TSOCS Solution Robot Vel: "
              << expected_robot_vel.transpose() << "\n";
    LOG(INFO) << "Interception Solution Parameters: "
              << params.a << ", " << params.b << ", "
              << params.c << ", " << params.d << ", "
              << params.T<< std::endl;
    if (success) {
      out << params.T << ", " << params.T << ", "
          << expected_robot_pos.norm() << ", "
          << expected_robot_vel.norm() << "\n";
    } else {
      out << params.T <<  ", NA, " << expected_ball_vel.norm() << "\n";
    }

    logger.Clear();
    logger.AddPoints(intercept_path_points, 1, 1, 0, 1);
    logger.AddPoints(tsocs_path_points, 0, 0, 1, 1);
    logger.SetMessageTime(GetWallTime());
  }
  out.close();

  logger.SendData();
}
