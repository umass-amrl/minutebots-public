// Copyright 2017 kvedder@umass.edu
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

#include "eigen3/Eigen/Core"

#include "constants/constants.h"
#include "obstacles/ball_obstacle.h"

#ifndef SRC_STATE_WORLD_BALL_H_
#define SRC_STATE_WORLD_BALL_H_

namespace state {

class WorldBall {
 public:
  WorldBall();
  WorldBall(const Eigen::Vector2f& ball_position,
            const Eigen::Vector2f& ball_velocity);
  ~WorldBall();

  void operator=(const WorldBall& other_ball);

  void UpdatePosition(const Eigen::Vector2f& new_position);
  void UpdateVelocity(const Eigen::Vector2f& new_velocity);

  const Eigen::Vector2f& GetPosition() const;
  const Eigen::Vector2f& GetBallVelocity() const;
  const obstacle::BallObstacle& GetBallObstacle() const;

  void SetObservation(const Eigen::Vector2f& observation);
  const Eigen::Vector2f& GetObservation() const;

 private:
  Eigen::Vector2f ball_position_;
  Eigen::Vector2f ball_velocity_;
  obstacle::BallObstacle ball_obstacle_;

  Eigen::Vector2f observation_;
};
}  // namespace state

#endif  // SRC_STATE_WORLD_BALL_H_
