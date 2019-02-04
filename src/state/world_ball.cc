// Copyright 2017 - 2018 kvedder@umass.edu
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
#include "state/world_ball.h"

STANDARD_USINGS;

namespace state {

WorldBall::WorldBall()
    : ball_position_(Vector2f(0, 0)),
      ball_obstacle_(Vector2f(0, 0), obstacle::BallObstacle::BallType::SMALL) {}

WorldBall::WorldBall(const Vector2f& ball_position,
                     const Vector2f& ball_velocity)
    : ball_position_(ball_position),
      ball_velocity_(ball_velocity),
      ball_obstacle_(ball_position, obstacle::BallObstacle::BallType::SMALL) {}

WorldBall::~WorldBall() {}

void WorldBall::operator=(const WorldBall& other_ball) {
  ball_position_ = other_ball.ball_position_;
  ball_velocity_ = other_ball.ball_velocity_;
  ball_obstacle_ = other_ball.ball_obstacle_;
}

void WorldBall::UpdatePosition(const Vector2f& new_position) {
  ball_position_ = new_position;
  ball_obstacle_.UpdatePose(new_position);
}

void WorldBall::UpdateVelocity(const Vector2f& new_velocity) {
  ball_velocity_ = new_velocity;
}

const Vector2f& WorldBall::GetPosition() const { return ball_position_; }

const Vector2f& WorldBall::GetBallVelocity() const { return ball_velocity_; }

const obstacle::BallObstacle& WorldBall::GetBallObstacle() const {
  return ball_obstacle_;
}

void WorldBall::SetObservation(const Vector2f& observation) {
  observation_ = observation;
}

const Vector2f& WorldBall::GetObservation() const { return observation_; }

}  // namespace state
