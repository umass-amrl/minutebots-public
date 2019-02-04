// Copyright 2018 kvedder@umass.edu, slane@cs.umass.edu
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
#include "state/world_state.h"

#include <memory>
#include <vector>

#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "logging/logger.h"
#include "math/math_util.h"
#include "obstacles/ball_obstacle.h"
#include "obstacles/circle_obstacle.h"
#include "obstacles/obstacle.h"
#include "obstacles/rectangle_obstacle.h"
#include "state/position_velocity_state.h"
#include "state/soccer_state.h"
#include "state_estimation/KF_const_accel.h"
#include "state_estimation/default_motion_model.h"
#include "util/timer.h"

using datastructures::DenseArray;
using datastructures::OptionalValue;
using datastructures::OptionalValueMutable;
using Eigen::Vector2f;
using estimation::KFConstAccel;
using logger::Logger;
using math_util::AngleMod;
using motion_model::DefaultMotionModel;
using obstacle::BallObstacle;
using obstacle::CircleObstacle;
using obstacle::Obstacle;
using obstacle::RectangleObstacle;
using obstacle::RobotObstacle;
using pose_2d::Pose2Dd;
using pose_2d::Pose2Df;
using state::PositionVelocityState;
using state::WorldBall;
using std::endl;
using std::fprintf;
using std::map;
using std::unique_ptr;
using std::vector;
using team::Team;

namespace state {

#define PREP_MASTER_OBSTACLE_BUFFER_LOADING \
  size_t obstacle_buffer_current_index = 0;

#define ADD_TO_MASTER_OBSTACLE_BUFFER(ptr, enabled_status)                  \
  NP_CHECK(obstacle_buffer_current_index < master_obstacle_buffer->size()); \
  (*master_obstacle_buffer)[obstacle_buffer_current_index] =                \
      std::unique_ptr<obstacle::Obstacle>(ptr);                             \
  (*master_obstacle_buffer)[obstacle_buffer_current_index]->SetEnabled(     \
      enabled_status);                                                      \
  ++obstacle_buffer_current_index;

#define FINISH_MASTER_OBSTACLE_BUFFER_LOADING \
  NP_CHECK(kNumObstacles == obstacle_buffer_current_index);

// One obstacle memory allocation to rule them all!
std::array<std::unique_ptr<obstacle::Obstacle>, kNumObstacles>
    WorldState::master_obstacle_buffer;

static void InitMasterObstacleBuffer(
    std::array<std::unique_ptr<obstacle::Obstacle>, kNumObstacles>*
        master_obstacle_buffer) {
  static constexpr unsigned int kNumRobots = kMaxTeamRobots * kNumTeams;

  PREP_MASTER_OBSTACLE_BUFFER_LOADING;
  // Creates a robot obstacle for all possible robots.
  // Note that, by default, obstacles are disabled.
  for (size_t i = 0; i < kNumRobots; ++i) {
    ADD_TO_MASTER_OBSTACLE_BUFFER(new RobotObstacle(Pose2Df(0, Vector2f(0, 0))),
                                  false);
  }

  // Creates the various ball obstacles.
  ADD_TO_MASTER_OBSTACLE_BUFFER(new BallObstacle(Pose2Df(0, Vector2f(0, 0)),
                                                 BallObstacle::BallType::SMALL),
                                false);

  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new BallObstacle(Pose2Df(0, Vector2f(0, 0)),
                       BallObstacle::BallType::MEDIUM),
      false);

  ADD_TO_MASTER_OBSTACLE_BUFFER(new BallObstacle(Pose2Df(0, Vector2f(0, 0)),
                                                 BallObstacle::BallType::LARGE),
                                false);

  // ==========================================================
  //                    Rules Obstacles
  // ==========================================================
  // Our defense area
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(
          Pose2Df(0.0f, -kFieldXMax - field_dimensions::kDefenseStretch / 2.0f,
                  0.0f),
          obstacle::RULE, field_dimensions::kDefenseStretch * 3.0f,
          field_dimensions::kDefenseStretch * 2.0f),
      true);

  // Their defense area
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(
          Pose2Df(0.0f, kFieldXMax + field_dimensions::kDefenseStretch / 2.0f,
                  0.0f),
          obstacle::RULE, field_dimensions::kDefenseStretch * 3.0f,
          field_dimensions::kDefenseStretch * 2.0f),
      true);

  // Ball Padding when ball is not in play
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new CircleObstacle(Pose2Df(0, 0, 0), obstacle::RULE, 1000), true);

  // Center Circle for kickoff
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new CircleObstacle(Pose2Df(0, 0, 0), obstacle::RULE, 500), true);

  // Their Side of field for kickoff
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(
          Pose2Df(0,
                  field_dimensions::kFieldLength / 4.0f + kFieldBoundary / 2.0,
                  0),
          obstacle::RULE,
          field_dimensions::kFieldLength / 2.0f + kFieldBoundary,
          field_dimensions::kFieldWidth + 2.0 * kFieldBoundary),
      true);

  // Their Defense Area Padded
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(
          Pose2Df(0.0f, kFieldXMax + field_dimensions::kDefenseStretch / 2.0f,
                  0.0f),
          obstacle::RULE, field_dimensions::kDefenseStretch * 3.0f + 400.0,
          field_dimensions::kDefenseStretch * 2.0f + 400.0),
      true);

  // Our Penalty Kick Obstacle
  float penalty_obstacle_length = kDefenseStretch + 400;
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(
          Pose2Df(0, kFieldXMax - penalty_obstacle_length / 2.0, 0),
          obstacle::RULE, penalty_obstacle_length,
          field_dimensions::kFieldWidth),
      true);

  // Their Penalty Kick Obstacle
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(
          Pose2Df(0, -kFieldXMax + penalty_obstacle_length / 2.0, 0),
          obstacle::RULE, penalty_obstacle_length,
          field_dimensions::kFieldWidth),
      true);

  // ==========================================================
  //                    Static Obstacles
  // ==========================================================

  // Top Touch Line
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(Pose2Df(0, 0,
                                    kFieldYMax + 2 * kRobotRadius +
                                        kWallObstacleThickness / 2.0f),
                            obstacle::STATIC,
                            field_dimensions::kFieldLength + 4 * kRobotRadius +
                                2 * kWallObstacleThickness,
                            kWallObstacleThickness - 25),
      true);

  // Bottom Touch Line
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(Pose2Df(0, 0,
                                    -kFieldYMax - 2 * kRobotRadius -
                                        kWallObstacleThickness / 2.0f),
                            obstacle::STATIC,
                            field_dimensions::kFieldLength + 4 * kRobotRadius +
                                2 * kWallObstacleThickness,
                            kWallObstacleThickness - 25),
      true);

  // Our Goal Line
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(
          Pose2Df(
              0, -kFieldXMax - 2 * kRobotRadius - kWallObstacleThickness / 2.0f,
              0),
          obstacle::STATIC, kWallObstacleThickness - 25,
          field_dimensions::kFieldWidth + 4 * kRobotRadius +
              2 * kWallObstacleThickness),
      true);

  // Their Goal Line
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(
          Pose2Df(0,
                  kFieldXMax + 2 * kRobotRadius + kWallObstacleThickness / 2.0f,
                  0),
          obstacle::STATIC, kWallObstacleThickness - 25,
          field_dimensions::kFieldWidth + 4 * kRobotRadius +
              2 * kWallObstacleThickness),
      true);

  // Our Goal Left
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(
          Pose2Df(
              0, -kFieldXMax - field_dimensions::kGoalDepth / 2.0f,
              -(field_dimensions::kGoalWidth + kGoalObstacleThickness) / 2.0f),
          obstacle::STATIC, field_dimensions::kGoalDepth,
          kGoalObstacleThickness),
      true);

  // Our Goal Right
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(
          Pose2Df(
              0, -kFieldXMax - field_dimensions::kGoalDepth / 2.0f,
              (field_dimensions::kGoalWidth + kGoalObstacleThickness) / 2.0f),
          obstacle::STATIC, field_dimensions::kGoalDepth,
          kGoalObstacleThickness),
      true);

  // Our Goal Back
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(
          Pose2Df(0,
                  -kFieldXMax - field_dimensions::kGoalDepth -
                      kGoalObstacleThickness / 2.0f,
                  0),
          obstacle::STATIC, kGoalObstacleThickness,
          field_dimensions::kGoalWidth + 2 * kGoalObstacleThickness),
      true);

  // Their Goal
  ADD_TO_MASTER_OBSTACLE_BUFFER(
      new RectangleObstacle(
          Pose2Df(0,
                  kFieldXMax + field_dimensions::kGoalDepth / 2.0f +
                      kGoalObstacleThickness / 2.0f,
                  0),
          obstacle::STATIC,
          field_dimensions::kGoalDepth + kGoalObstacleThickness,
          field_dimensions::kGoalWidth + 2 * kGoalObstacleThickness),
      true);

  FINISH_MASTER_OBSTACLE_BUFFER_LOADING;
}

WorldState::WorldState(state::PositionVelocityState* position_velocity_state,
                       const Team team, const bool simulating)
    : world_time_(GetWallTime() + kLatency),
      last_world_time_(GetWallTime()),
      position_velocity_state_(position_velocity_state),
      last_pv_state_(*position_velocity_state),
      team_(team),
      simulating_(simulating),
      avg_processing_time_(kExpectedAverage_),
      time_queue_sum_(kExpectedAverage_ * kNumTimes_),
      processing_time_queue_(kNumTimes_) {
  if (simulating_) {
    world_time_ = 0 + kSimulatorLatency;
  }

  for (unsigned int i = 0; i < kMaxTeamRobots; i++) {
    our_motion_models_.push_back(DefaultMotionModel());
    their_motion_models_.push_back(DefaultMotionModel());
  }

  for (unsigned int i = 0; i < kNumTimes_; i++) {
    processing_time_queue_.Add(kExpectedAverage_);
  }

  current_shared_state_.Init();

  InitMasterObstacleBuffer(&master_obstacle_buffer);

  // Updates the enabled status as well as position of already known robots.
  //   UpdateState();
}

WorldState::~WorldState() {}

void WorldState::UpdateState(Logger* logger) {
  static bool kDebug = false;

  last_world_time_ = world_time_;
  // Update world time.
  if (simulating_) {
    // TODO(jaholtz): Update this to a more reasonable way to get the time from
    // the simulator.
    world_time_ = position_velocity_state_->GetTime() + kSimulatorLatency;
  } else {
    world_time_ = GetWallTime() + kLatency + avg_processing_time_;
  }

  double observation_time = position_velocity_state_->GetTime();

  logger->LogPrintPush("Forward Prediction");
  logger->LogPrint("Observation Time: %f", observation_time);
  logger->LogPrint("World Time: %f", world_time_);
  logger->LogPrint("Current Average Processing Time: %f", avg_processing_time_);
  // Enable and update existing robots.
  for (size_t i = 0;
       i < position_velocity_state_->GetOurTeamRobots().GetElementCount();
       ++i) {
    PositionVelocityState::RobotPositionVelocity* our_robot_pos_vel =
        position_velocity_state_->GetMutableOurTeamRobots()->GetMutable(i);

    if (kDebug) {
      logger->LogPrintPush("Robot %d", our_robot_pos_vel->ssl_vision_id);
    }

    our_motion_models_[i].SetSSLVisionID(our_robot_pos_vel->ssl_vision_id,
                                         true);

    Pose2Df current_position(our_robot_pos_vel->filtered_position);
    Pose2Df current_velocity(our_robot_pos_vel->filtered_velocity);

    ForwardPredictRobot(
        our_robot_pos_vel->ssl_vision_id, true, observation_time, world_time_,
        current_position, current_velocity, our_motion_models_[i],
        &our_robot_pos_vel->position, &our_robot_pos_vel->velocity, logger);
    if (kDebug) {
      logger->LogPrint("Filtered Position: %f, %f, %f",
                       our_robot_pos_vel->filtered_position.translation.x(),
                       our_robot_pos_vel->filtered_position.translation.y(),
                       our_robot_pos_vel->filtered_position.angle);

      logger->LogPrint("Current Position: %f, %f, %f",
                       our_robot_pos_vel->position.translation.x(),
                       our_robot_pos_vel->position.translation.y(),
                       our_robot_pos_vel->position.angle);

      logger->Pop();
    } else {
      logger->LogPrintPush("Robot %d", our_robot_pos_vel->ssl_vision_id);
      logger->LogPrint("Position: %f, %f, %f",
                       our_robot_pos_vel->position.translation.x(),
                       our_robot_pos_vel->position.translation.y(),
                       RadToDeg(our_robot_pos_vel->position.angle));
      logger->LogPrint("Velocity: %f, %f, %f",
                       our_robot_pos_vel->velocity.translation.x(),
                       our_robot_pos_vel->velocity.translation.y(),
                       RadToDeg(our_robot_pos_vel->velocity.angle));
      logger->Pop();
    }

    if (kLogPredictions_) {
      if (i == 0) {
        static ScopedFile predictions_fid("ws_predictions.csv", "w");
        static ScopedFile observations_fid("ws_observations.csv", "w");

        fprintf(observations_fid, "%f, %f, %f, %f\n",
                our_robot_pos_vel->observed_time,
                our_robot_pos_vel->observed_pose.translation.x(),
                our_robot_pos_vel->observed_pose.translation.y(),
                our_robot_pos_vel->observed_pose.angle);

        fprintf(predictions_fid, "%f, %f, %f, %f, %f, %f, %f\n", world_time_,
                our_robot_pos_vel->position.translation.x(),
                our_robot_pos_vel->position.translation.y(),
                our_robot_pos_vel->position.angle,
                our_robot_pos_vel->velocity.translation.x(),
                our_robot_pos_vel->velocity.translation.y(),
                our_robot_pos_vel->velocity.angle);
      }
    }

    master_obstacle_buffer[i]->UpdatePose(our_robot_pos_vel->position);
    master_obstacle_buffer[i]->SetEnabled(our_robot_pos_vel->confidence > 0);
  }

  // Disable non-existant robots.
  for (size_t i =
           position_velocity_state_->GetOurTeamRobots().GetElementCount();
       i < kMaxTeamRobots; ++i) {
    master_obstacle_buffer[i]->SetEnabled(false);
  }

  const auto& their_robot_index_offset = kMaxTeamRobots;

  // Enable and update existing robots.
  for (size_t i = 0;
       i < position_velocity_state_->GetTheirTeamRobots().GetElementCount();
       ++i) {
    PositionVelocityState::RobotPositionVelocity* their_robot_pos_vel =
        position_velocity_state_->GetMutableTheirTeamRobots()->GetMutable(i);

    if (kDebug) {
      logger->LogPrintPush("Robot %d", their_robot_pos_vel->ssl_vision_id);
    }

    their_motion_models_[i].SetSSLVisionID(their_robot_pos_vel->ssl_vision_id,
                                           false);

    Pose2Df current_position(their_robot_pos_vel->filtered_position);
    Pose2Df current_velocity(their_robot_pos_vel->filtered_velocity);

    ForwardPredictRobot(their_robot_pos_vel->ssl_vision_id, false,
                        observation_time, world_time_, current_position,
                        current_velocity, their_motion_models_[i],
                        &their_robot_pos_vel->position,
                        &their_robot_pos_vel->velocity, logger);
    current_velocity = their_robot_pos_vel->velocity;

    if (kDebug) {
      logger->LogPrint("Observed Position: %f, %f, %f",
                       their_robot_pos_vel->observed_pose.translation.x(),
                       their_robot_pos_vel->observed_pose.translation.y(),
                       their_robot_pos_vel->observed_pose.angle);
      logger->LogPrint("Filtered Position: %f, %f, %f",
                       their_robot_pos_vel->filtered_position.translation.x(),
                       their_robot_pos_vel->filtered_position.translation.y(),
                       their_robot_pos_vel->filtered_position.angle);
      logger->LogPrint("Filtered Velocity: %f, %f, %f",
                       their_robot_pos_vel->filtered_velocity.translation.x(),
                       their_robot_pos_vel->filtered_velocity.translation.y(),
                       their_robot_pos_vel->filtered_velocity.angle);
      logger->LogPrint("Current Position: %f, %f, %f",
                       their_robot_pos_vel->position.translation.x(),
                       their_robot_pos_vel->position.translation.y(),
                       their_robot_pos_vel->position.angle);
      logger->Pop();
    }

    master_obstacle_buffer[i + their_robot_index_offset]->UpdatePose(
        their_robot_pos_vel->position);
    master_obstacle_buffer[i + their_robot_index_offset]->SetEnabled(
        their_robot_pos_vel->confidence > 0);
  }

  // Disable non-existant robots.
  for (size_t i =
           position_velocity_state_->GetTheirTeamRobots().GetElementCount() +
           their_robot_index_offset;
       i < kMaxTeamRobots + their_robot_index_offset; ++i) {
    master_obstacle_buffer[i]->SetEnabled(false);
  }

  ClearPastCommands(position_velocity_state_->GetTime());

  PositionVelocityState::BallPositionVelocity* ball_position_velocity =
      position_velocity_state_->GetMutableBallPositionVelocity();

  Vector2f ball_position = ball_position_velocity->filtered_position;
  Vector2f ball_velocity = ball_position_velocity->filtered_velocity;

  double delta_t;

  if (simulating_) {
    delta_t = world_time_ - observation_time;
  } else {
    delta_t = world_time_ - kLatency + kBallLatency - observation_time;
  }

  logger->LogPrint("Ball Delta T: %f", delta_t);

  KFConstAccel::ForwardPredict(ball_position_velocity->filtered_position,
                               ball_position_velocity->filtered_velocity,
                               delta_t, &ball_position, &ball_velocity, logger);

  if (kDebug) {
    logger->LogPrint("Current Ball Position: %f, %f",
                     ball_position_velocity->filtered_position.x(),
                     ball_position_velocity->filtered_position.y());
    logger->LogPrint("Current Ball Velocity: %f, %f",
                     ball_position_velocity->filtered_velocity.x(),
                     ball_position_velocity->filtered_velocity.y());
  }
  logger->LogPrint("Predicted Ball Position: %f, %f", ball_position.x(),
                   ball_position.y());
  logger->LogPrint("Predicted Ball Velocity: %f, %f", ball_velocity.x(),
                   ball_velocity.y());

  ball_position_velocity->position = ball_position;
  ball_position_velocity->velocity = ball_velocity;

  static constexpr size_t kNumRobots = kMaxTeamRobots * kNumTeams;
  for (size_t i = kNumRobots; i < kNumRobots + kNumBallObstacles; ++i) {
    master_obstacle_buffer[i]->UpdatePose(
        Pose2Df(0, ball_position_velocity->position));
    master_obstacle_buffer[i]->SetEnabled(true);
  }

  // Update ball circle rules obstacle
  master_obstacle_buffer[kNumRobotObstacles + kNumBallObstacles + 2]
      ->UpdatePose(Pose2Df(0, ball_position_velocity->position));

  logger->Pop();
}

void WorldState::UpdateLastState(const PositionVelocityState& state) {
  last_pv_state_ = state;
}

void WorldState::AddSharedState(SharedState shared_state) {
  past_shared_states_.push_back(shared_state);
}

double WorldState::GetWorldTime() const { return world_time_; }

double WorldState::GetLastWorldTime() const { return last_world_time_; }

const std::size_t WorldState::GetNumOurRobots() const {
  return position_velocity_state_->GetOurTeamRobots().GetElementCount();
}

const std::size_t WorldState::GetNumTheirRobots() const {
  return position_velocity_state_->GetTheirTeamRobots().GetElementCount();
}

// Gets OurRobotIndex, or -1 if robot is not found.
const int WorldState::GetOurRobotIndex(SSLVisionId our_vision_id) const {
  for (OurRobotIndex i = 0;
       i < position_velocity_state_->GetOurTeamRobots().GetElementCount();
       ++i) {
    const auto& world_robot =
        position_velocity_state_->GetOurTeamRobots().Get(i);
    if (world_robot.ssl_vision_id == our_vision_id) {
      return i;
    }
  }
  LOG(ERROR) << "Unknown Our SSL Vision ID: " << our_vision_id << "\n";
  return -1;
}

// Gets TheirRobotIndex, or -1 if robot is not found.
const int WorldState::GetTheirRobotIndex(SSLVisionId their_vision_id) const {
  for (OurRobotIndex i = 0;
       i < position_velocity_state_->GetTheirTeamRobots().GetElementCount();
       ++i) {
    const auto& world_robot =
        position_velocity_state_->GetTheirTeamRobots().Get(i);
    if (world_robot.ssl_vision_id == their_vision_id) {
      return i;
    }
  }
  LOG(WARNING) << "Unknown Their SSL Vision ID: " << their_vision_id << "\n";
  return -1;
}

const PositionVelocityState::RobotPositionVelocity&
WorldState::GetOurRobotPosition(OurRobotIndex our_robot_index) const {
  return position_velocity_state_->GetOurTeamRobots().Get(our_robot_index);
}

const PositionVelocityState::RobotPositionVelocity&
WorldState::GetTheirRobotPosition(TheirRobotIndex their_robot_index) const {
  NP_CHECK(position_velocity_state_->GetTheirTeamRobots().GetElementCount() >
           their_robot_index);
  return position_velocity_state_->GetTheirTeamRobots().Get(their_robot_index);
}

const DenseArray<PositionVelocityState::RobotPositionVelocity, kMaxTeamRobots>&
WorldState::GetOurRobots() const {
  return position_velocity_state_->GetOurTeamRobots();
}

const DenseArray<PositionVelocityState::RobotPositionVelocity, kMaxTeamRobots>&
WorldState::GetTheirRobots() const {
  return position_velocity_state_->GetTheirTeamRobots();
}

const PositionVelocityState::BallPositionVelocity& WorldState::GetBallPosition()
    const {
  return position_velocity_state_->GetBallPositionVelocity();
}

const PositionVelocityState::BallPositionVelocity&
WorldState::GetLastBallPosition() const {
  return last_pv_state_.GetBallPositionVelocity();
}

const std::array<unique_ptr<Obstacle>, kNumObstacles>&
WorldState::GetAllObstacles() const {
  return master_obstacle_buffer;
}

const Team& WorldState::GetOurTeam() const { return team_; }

const PositionVelocityState& WorldState::GetPositionVelocityState() const {
  return *position_velocity_state_;
}

bool WorldState::CanChip(const SSLVisionId robot_id) const {
  // TODO(slane): Update IDs
  //   if (robot_id == 1 ||
  //       robot_id == 2 ||
  //       robot_id == 3 ||
  //       // robot_id == 4 ||
  //       robot_id == 5 ||
  //       // robot_id == 6 ||
  //       robot_id == 7) {
  //     return true;
  //   }

  return false;
}

void WorldState::ForwardPredictRobot(
    SSLVisionId ssl_vision_id, bool is_our_team, double start_timestamp,
    double end_timestamp, const Pose2Df& current_position,
    const Pose2Df& current_velocity, const DefaultMotionModel& model,
    Pose2Df* next_position, Pose2Df* next_velocity, Logger* logger) const {
  //   double current_time = start_timestamp;
  Vector6d current_state;
  current_state[0] = static_cast<double>(current_position.translation.x());
  current_state[1] = static_cast<double>(current_position.translation.y());
  current_state[2] = static_cast<double>(current_position.angle);
  current_state[3] = static_cast<double>(current_velocity.translation.x());
  current_state[4] = static_cast<double>(current_velocity.translation.y());
  current_state[5] = static_cast<double>(current_velocity.angle);

  Matrix6d jacobian;
  jacobian.setZero();

  Vector6d next_state;
  next_state.setZero();

  if (is_our_team) {
    Pose2Dd current_command =
        current_shared_state_.GetCommandByID(ssl_vision_id);
    double current_time = start_timestamp;

    for (const SharedState& shared_state : past_shared_states_) {
      double next_time = shared_state.GetCommandTime();

      if (next_time > end_timestamp) {
        break;
      }

      model.Predict(next_time - current_time, current_state, current_command,
                    &next_state, &jacobian, logger);

      current_command = shared_state.GetCommandByID(ssl_vision_id);
      current_time = next_time;

      current_state = next_state;
    }

    model.Predict(end_timestamp - current_time, current_state, current_command,
                  &next_state, &jacobian, logger);

    current_state = next_state;
  } else {
    model.Predict(end_timestamp - start_timestamp, current_state, &next_state,
                  &jacobian, logger);
    current_state = next_state;
  }

  next_position->translation.x() = static_cast<float>(current_state[0]);
  next_position->translation.y() = static_cast<float>(current_state[1]);
  next_position->angle = static_cast<float>(current_state[2]);
  next_velocity->translation.x() = static_cast<float>(current_state[3]);
  next_velocity->translation.y() = static_cast<float>(current_state[4]);
  next_velocity->angle = static_cast<float>(current_state[5]);
}

void WorldState::ClearPastCommands(double timestamp) {
  while (!past_shared_states_.empty()) {
    if (past_shared_states_[0].GetCommandTime() < timestamp) {
      current_shared_state_ = past_shared_states_[0];
      past_shared_states_.erase(past_shared_states_.begin());
    } else {
      break;
    }
  }
}

const DefaultMotionModel& WorldState::GetOurMotionModel(
    OurRobotIndex index) const {
  return our_motion_models_[index];
}

const DefaultMotionModel& WorldState::GetTheirMotionModel(
    TheirRobotIndex index) const {
  return their_motion_models_[index];
}

const void WorldState::UpdateProcessingTime(double new_time) {
  time_queue_sum_ -= processing_time_queue_.First();
  time_queue_sum_ += new_time;
  avg_processing_time_ = time_queue_sum_ / static_cast<double>(kNumTimes_);
  processing_time_queue_.Add(new_time);
}

}  // namespace state
