// Copyright 2017 - 2019 kvedder@umass.edu, slane@cs.umass.edu
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
#include "soccer/kalmanupdate.h"
#include <stdio.h>
#include <bitset>
#include <fstream>
#include <iomanip>
#include <queue>
#include <string>

#include "glog/logging.h"

#include "constants/constants.h"
#include "constants/typedefs.h"
#include "datastructures/bounded_queue.h"
#include "logging/logger.h"
#include "math/math_util.h"
#include "net/netraw.h"
#include "state/position_velocity_state.h"
#include "state/team.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "util/timer.h"

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"

STANDARD_USINGS;
using direction::Direction;
using estimation::ExtendedKalmanFilter;
using experimental_simulator::ExperimentalSim;
using experimental_simulator::Simulator;
using logger::Logger;
using math_util::AngleMod;
using net::UDPMulticastServer;
using pose_2d::Pose2Df;
using SSLVisionProto::SSL_DetectionBall;
using SSLVisionProto::SSL_DetectionFrame;
using SSLVisionProto::SSL_DetectionRobot;
using SSLVisionProto::SSL_WrapperPacket;
using state::PositionVelocityState;
using state::SharedRobotState;
using state::SharedState;
using state::WorldRobot;
using state::WorldState;
using std::atomic_bool;
using std::bitset;
using std::condition_variable;
using std::endl;
using std::make_pair;
using std::mutex;
using std::pair;
using std::queue;
using std::thread;
using std::unique_lock;
using std::vector;
using team::Team;
using threadsafe::ThreadSafeActor;
using threadsafe::ThreadSafePriorityQueue;

namespace app {
KalmanUpdate::KalmanUpdate(
    const bitset<kNumCameras>& camera_mask,
    const string& ssl_vision_udp_address,
    const int ssl_vision_udp_port,
    threadsafe::ThreadSafeActor<state::PositionVelocityState>*
        thread_safe_position_velocity_state,
    threadsafe::ThreadSafeActor<Logger>* thread_safe_logger,
    threadsafe::ThreadSafeQueue<state::SharedState>*
        thread_safe_shared_state_queue,
    const Direction& direction,
    const Team& team,
    bool use_message_time)
    : thread_safe_position_velocity_state_(thread_safe_position_velocity_state),
      thread_safe_logger_(thread_safe_logger),
      thread_safe_shared_state_queue_(thread_safe_shared_state_queue),
      direction_(direction),
      local_position_velocity_state_(),
      ball_position_queue_(kAverageBallNumSteps),
      ball_state_estimator(estimation::BallStateEstimator(direction)),
      last_ssl_update_timestamp_(GetWallTime()),
      team_(team),
      is_running_(true),
      ssl_vision_udp_address_(ssl_vision_udp_address),
      ssl_vision_udp_port_(ssl_vision_udp_port),
      camera_mask_(camera_mask),
      simulating_(false),
      use_message_time_(use_message_time) {
  state_estimation_test_type = 0;
  camera_param_tuning_mode = 0;
  for (uint i = 0; i < kNumCameras; ++i) {
    message_counts_.push_back(0);
    dropped_message_counts_.push_back(0);
    last_message_frame_number_.push_back(-1);
  }
}

KalmanUpdate::~KalmanUpdate() {
  // Signal the update thread that we are shutting down
  is_running_ = false;
  if (simulating_) {
    //     thread_safe_simulator_->Shutdown();
  }
  thread_safe_position_velocity_state_->Shutdown();
  thread_safe_logger_->Shutdown();
  if (update_thread_.joinable()) {
    update_thread_.join();
  }
}

void KalmanUpdate::Start() {
  update_thread_ = thread(&KalmanUpdate::HandleUpdate, this);
}

void KalmanUpdate::SimStart(ThreadSafeActor<Simulator*>* thread_safe_sim,
                            Simulator* simulator) {
  thread_safe_simulator_ = thread_safe_sim;
  local_simulator_ = simulator;
  PositionVelocityState start_state =
      local_simulator_->GetWorldState(Team::BLUE);
  for (auto robot : start_state.GetOurTeamRobots()) {
    AddNewRobotKalmanSim(0.0,
                      0,
                      robot.ssl_vision_id,
                      robot.position,
                      robot.velocity,
                      true,
                      local_position_velocity_state_.GetMutableOurTeamRobots(),
                      &our_team_kalman_);
  }
  state::PositionVelocityState::BallPositionVelocity ball =
      start_state.GetBallPositionVelocity();
  ball_state_estimator.SetBallState(ball);
  update_thread_ = thread(&KalmanUpdate::HandleSimUpdate, this);
}

void KalmanUpdate::Stop() { is_running_ = false; }

Pose2Df KalmanUpdate::ExtractPose(
    const SSL_DetectionRobot& observed_ssl_robot) {
  const float observed_x = (direction_ == Direction::POSITIVE)
                               ? observed_ssl_robot.x()
                               : observed_ssl_robot.x() * -1;
  const float observed_y = (direction_ == Direction::POSITIVE)
                               ? observed_ssl_robot.y()
                               : observed_ssl_robot.y() * -1;
  const float observed_angle =
      (observed_ssl_robot.has_orientation())
          ? ((direction_ == Direction::POSITIVE)
                 ? observed_ssl_robot.orientation()
                 : observed_ssl_robot.orientation() + M_PI)
          : 0.0f;

  return Pose2Df(observed_angle, Vector2f(observed_x, observed_y));
}

bool KalmanUpdate::IsObservationInField(const SSL_DetectionRobot& robot) {
  return (fabs(robot.x()) <
          field_dimensions::kHalfFieldLength + 2 * kRobotRadius) &&
         (fabs(robot.y()) <
          field_dimensions::kHalfFieldWidth + 2 * kRobotRadius);
}

void KalmanUpdate::RemoveTimedOutRobots(const double observ_time,
                                        const double timeout,
                                        const bool our_team,
                                        PositionVelocityArray* robots) {
  for (size_t i = 0; i < robots->GetElementCount();) {
    const double& prev_observ_time = robots->Get(i).observed_time;
    const double observ_delta = (observ_time - prev_observ_time);
    if (observ_delta > timeout) {
      robots->DeleteAndShiftLeft(i);
      // Reset the data to default constructed.
      robots->Set(robots->GetElementCount(), {});
      if (our_team) {
        our_team_kalman_.DeleteAndShiftLeft(i);
        // Reset the kalman filter that was left over from the delete.
        our_team_kalman_.GetMutable(our_team_kalman_.GetElementCount())
            ->second.Reset();
      } else {
        their_team_kalman_.DeleteAndShiftLeft(i);
        their_team_kalman_.GetMutable(their_team_kalman_.GetElementCount())
            ->second.Reset();
      }
    } else {
      ++i;
    }
  }
}

bool KalmanUpdate::UpdateIfExistingRobotKalman(
    const double detection_timestamp,
    const unsigned int camera_id,
    const unsigned int lead_camera_id,
    const ::SSLVisionProto::SSL_DetectionRobot& detection,
    PositionVelocityArray* robots,
    KalmanArray* kalman_array) {
  const Pose2Df detection_pose = ExtractPose(detection);
  for (size_t i = 0; i < robots->GetElementCount(); ++i) {
    const auto& existing_pos_vel = robots->Get(i);
    if (detection.robot_id() != existing_pos_vel.ssl_vision_id) {
      continue;
    }

    if (kDebug) {
      local_logger_.LogPrintPush("Update for SSL Vision ID: %d",
                                 detection.robot_id());
    }
    ExtendedKalmanFilter& existing_ekf = kalman_array->GetMutable(i)->second;
    existing_ekf.Update(detection_pose,
                        detection_timestamp,
                        camera_id,
                        lead_camera_id,
                        &local_logger_);
    Pose2Df updated_pose(0, 0, 0);
    Pose2Df updated_velocity(0, 0, 0);
    existing_ekf.GetCurrentState(&updated_pose, &updated_velocity);
    Pose2Df observed_velocity = existing_ekf.GetObservedVelocity();

    robots->Set(i,
                {detection.robot_id(),
                 updated_pose,
                 updated_velocity,
                 detection_pose,
                 observed_velocity,
                 detection_timestamp,
                 detection.confidence()});
    if (kDebug) {
      local_logger_.Pop();
    }
    return true;
  }
  return false;
}

bool KalmanUpdate::UpdateIfExistingRobotVision(
    const double detection_timestamp,
    const unsigned int camera_id,
    const unsigned int lead_camera_id,
    const SSL_DetectionRobot& detection,
    PositionVelocityArray* robots) {
  const Pose2Df detection_pose = ExtractPose(detection);
  for (size_t i = 0; i < robots->GetElementCount(); ++i) {
    const auto& existing_pos_vel = robots->Get(i);
    if (detection.robot_id() != existing_pos_vel.ssl_vision_id) {
      continue;
    }

    // TODO(kvedder): Come up with a bounded queue or something to get average
    // velocity.
    robots->Set(i,
                {detection.robot_id(),
                 detection_pose,
                 {0, 0, 0},
                 detection_pose,
                 {0, 0, 0},
                 detection_timestamp,
                 detection.confidence()});
    return true;
  }
  return false;
}

void KalmanUpdate::AddNewRobotKalmanSim(
  const double detection_timestamp,
  const unsigned int camera_id,
  const unsigned int robot_id,
  const Pose2Df pose,
  const Pose2Df velocity,
  bool is_ours,
  PositionVelocityArray* robots,
  KalmanArray* kalman_array) {
  kalman_array->InsertBack(
    {robot_id, {pose, velocity, detection_timestamp, camera_id}});
  robots->InsertBack({robot_id,
    pose,
    velocity,
    pose,
    velocity,
    detection_timestamp,
    1.0});
}

void KalmanUpdate::AddNewRobotKalman(
    const double detection_timestamp,
    const unsigned int camera_id,
    const unsigned int lead_camera_id,
    const ::SSLVisionProto::SSL_DetectionRobot& detection,
    bool is_ours,
    PositionVelocityArray* robots,
    KalmanArray* kalman_array) {
  const Pose2Df detection_pose = ExtractPose(detection);
  if (robots->GetElementCount() >= kMaxTeamRobots ||
      !IsObservationInField(detection) ||
      detection.confidence() <= kMinConfidence) {
    return;
  }
  kalman_array->InsertBack(
      {detection.robot_id(), {detection_pose, detection_timestamp, camera_id}});
  auto* kalman_pair =
      kalman_array->GetMutable(kalman_array->GetElementCount() - 1);
  Pose2Df updated_pose(0, 0, 0);
  Pose2Df updated_velocity(0, 0, 0);
  kalman_pair->second.GetCurrentState(&updated_pose, &updated_velocity);
  kalman_pair->second.SetSSLVisionID(detection.robot_id(), is_ours);

  robots->InsertBack({detection.robot_id(),
                      updated_pose,
                      updated_velocity,
                      detection_pose,
                      {0, 0, 0},
                      detection_timestamp,
                      detection.confidence()});
}

void KalmanUpdate::AddNewRobotVision(const double detection_timestamp,
                                     const unsigned int camera_id,
                                     const unsigned int lead_camera_id,
                                     const SSL_DetectionRobot& detection,
                                     PositionVelocityArray* robots) {
  const Pose2Df detection_pose = ExtractPose(detection);
  if (robots->GetElementCount() >= kMaxTeamRobots ||
      !IsObservationInField(detection) ||
      detection.confidence() <= kMinConfidence) {
    return;
  }

  // TODO(kvedder): Come up with a bounded queue or something to get average
  // velocity.
  robots->InsertBack({detection.robot_id(),
                      detection_pose,
                      {0, 0, 0},
                      detection_pose,
                      {0, 0, 0},
                      detection_timestamp,
                      detection.confidence()});
}

bool IsKalmanUpdated(
    const ::google::protobuf::RepeatedPtrField<
        ::SSLVisionProto::SSL_DetectionRobot>& detections,
    const std::pair<SSLVisionId, estimation::ExtendedKalmanFilter>&
        kalman_pair) {
  for (const auto& detection : detections) {
    if (detection.robot_id() == kalman_pair.first) {
      return true;
    }
  }
  return false;
}

bool IsStatePairUpdated(
    const ::google::protobuf::RepeatedPtrField<
        ::SSLVisionProto::SSL_DetectionRobot>& detections,
    const std::pair<SSLVisionId,
                    state::PositionVelocityState::RobotPositionVelocity>&
        state_pair) {
  for (const auto& detection : detections) {
    if (detection.robot_id() == state_pair.first) {
      return true;
    }
  }
  return false;
}

void KalmanUpdate::UpdateUnseenKalmanWithForwardPrediction(
    const double observation_time,
    PositionVelocityState::RobotPositionVelocity* robot_position_velocity,
    estimation::ExtendedKalmanFilter* kalman) {
  Pose2Df predicted_pose(0, 0, 0);
  Pose2Df predicted_velocity(0, 0, 0);
  kalman->ForwardPredict(
      observation_time, &predicted_pose, &predicted_velocity, &local_logger_);
  (*robot_position_velocity) = {robot_position_velocity->ssl_vision_id,
                                predicted_pose,
                                predicted_velocity,
                                robot_position_velocity->observed_pose,
                                robot_position_velocity->observed_velocity,
                                robot_position_velocity->observed_time,
                                robot_position_velocity->confidence};
}

void KalmanUpdate::UpdateUnseenVisionWithForwardPrediction(
    const double observation_time,
    PositionVelocityState::RobotPositionVelocity* robot_position_velocity) {
  const double time_delta =
      observation_time - robot_position_velocity->observed_time;
  const Pose2Df delta(
      robot_position_velocity->velocity.angle * time_delta,
      robot_position_velocity->velocity.translation * time_delta);
  const Pose2Df predicted_pose(
      robot_position_velocity->position.angle + delta.angle,
      robot_position_velocity->position.translation + delta.translation);
  (*robot_position_velocity) = {robot_position_velocity->ssl_vision_id,
                                predicted_pose,
                                robot_position_velocity->velocity,
                                robot_position_velocity->observed_pose,
                                robot_position_velocity->observed_velocity,
                                robot_position_velocity->observed_time,
                                robot_position_velocity->confidence};
}

void ReorderBySSLID(
    PositionVelocityArray* robots,
    datastructures::DenseArray<
        std::pair<SSLVisionId, estimation::ExtendedKalmanFilter>,
        kMaxTeamRobots>* kalman) {
  auto* robots_underlying_array = robots->GetMutableUnderlyingArray();

  // Sort by the ssl_vision_id as defined in operator<.
  std::sort(robots_underlying_array->begin(),
            robots_underlying_array->begin() + robots->GetElementCount());

  auto* kalman_underlying_array = kalman->GetMutableUnderlyingArray();

  // Sort by the ssl_vision_id as defined in the below lambda.
  std::sort(kalman_underlying_array->begin(),
            kalman_underlying_array->begin() + kalman->GetElementCount(),
            [](const pair<SSLVisionId, ExtendedKalmanFilter>& a,
               const pair<SSLVisionId, ExtendedKalmanFilter>& b) -> bool {
              return a.first < b.first;
            });
}

void ReorderBySSLID(PositionVelocityArray* robots) {
  auto* robots_underlying_array = robots->GetMutableUnderlyingArray();

  // Sort by the ssl_vision_id as defined in operator<.
  std::sort(robots_underlying_array->begin(),
            robots_underlying_array->begin() + robots->GetElementCount());
}

void KalmanUpdate::UpdatePositionsAndOrdering(
    const SSLVisionProto::SSL_DetectionFrame& detection_frame,
    unsigned int lead_camera_id) {
  const auto& our_robot_detection = (team_ == Team::YELLOW)
                                        ? detection_frame.robots_yellow()
                                        : detection_frame.robots_blue();
  const auto& their_robot_detection = (team_ == Team::YELLOW)
                                          ? detection_frame.robots_blue()
                                          : detection_frame.robots_yellow();
  const double observation_timestamp = detection_frame.t_capture();
  const unsigned int camera_id = detection_frame.camera_id();
  local_logger_.LogPrint("Last Observation Time: %f",
                         last_ssl_update_timestamp_);
  local_logger_.LogPrint("Observation Time: %f", observation_timestamp);
  // Update our team ordering.
  PositionVelocityArray* our_team_robots =
      local_position_velocity_state_.GetMutableOurTeamRobots();
  RemoveTimedOutRobots(observation_timestamp,
                       team_management::kOurTeamBotTimeout,
                       /*our_team =*/true,
                       our_team_robots);

  // While there is room, add the robots and be done.
  // TODO(kvedder): Add the robots in order of confidence.
  for (const auto& observed_ssl_robot : our_robot_detection) {
    const bool updated_existing =
        UpdateIfExistingRobotKalman(observation_timestamp,
                                    camera_id,
                                    lead_camera_id,
                                    observed_ssl_robot,
                                    our_team_robots,
                                    &our_team_kalman_);
    if (!updated_existing) {
      AddNewRobotKalman(observation_timestamp,
                        camera_id,
                        lead_camera_id,
                        observed_ssl_robot,
                        true,
                        our_team_robots,
                        &our_team_kalman_);
    }
  }

  // Update unseen kalman filters.
  for (size_t i = 0; i < our_team_kalman_.GetElementCount(); ++i) {
    auto* kalman_pair = our_team_kalman_.GetMutable(i);
    auto* current_position_velocity = our_team_robots->GetMutable(i);

    if (!IsKalmanUpdated(our_robot_detection, *kalman_pair)) {
      UpdateUnseenKalmanWithForwardPrediction(observation_timestamp,
                                              current_position_velocity,
                                              &(kalman_pair->second));
    }
  }

  ReorderBySSLID(our_team_robots, &our_team_kalman_);

  auto* their_team_robots =
      local_position_velocity_state_.GetMutableTheirTeamRobots();

  RemoveTimedOutRobots(observation_timestamp,
                       team_management::kTheirTeamBotTimeout,
                       /*our_team =*/false,
                       their_team_robots);

  // While there is room, add the robots and be done.
  // TODO(kvedder): Add the robots in order of confidence.
  for (const auto& observed_ssl_robot : their_robot_detection) {
    const bool updated_existing =
        UpdateIfExistingRobotKalman(observation_timestamp,
                                    camera_id,
                                    lead_camera_id,
                                    observed_ssl_robot,
                                    their_team_robots,
                                    &their_team_kalman_);
    if (!updated_existing) {
      AddNewRobotKalman(observation_timestamp,
                        camera_id,
                        lead_camera_id,
                        observed_ssl_robot,
                        false,
                        their_team_robots,
                        &their_team_kalman_);
    }
  }

  // Update unseen vision.
  for (size_t i = 0; i < their_team_kalman_.GetElementCount(); ++i) {
    auto* kalman_pair = their_team_kalman_.GetMutable(i);
    auto* current_position_velocity = their_team_robots->GetMutable(i);

    if (!IsKalmanUpdated(their_robot_detection, *kalman_pair)) {
      UpdateUnseenVisionWithForwardPrediction(observation_timestamp,
                                              current_position_velocity);
    }
  }

  ReorderBySSLID(their_team_robots);
  local_position_velocity_state_.SetTime(observation_timestamp);
}

void KalmanUpdate::UpdateRobotCommands() {
  vector<SharedState> local_shared_state_queue;
  thread_safe_shared_state_queue_->ReadAllAndEmpty(&local_shared_state_queue);
  if (!local_shared_state_queue.empty()) {
    local_shared_state_ = local_shared_state_queue.back();
  }
  for (const SharedState& state : local_shared_state_queue) {
    const vector<SharedRobotState>& robot_vector = state.GetSharedStatesRef();
    for (const SharedRobotState& robot_state : robot_vector) {
      if (robot_state.enabled) {
        for (size_t i = 0; i < local_position_velocity_state_.GetOurTeamRobots()
                                   .GetElementCount();
             ++i) {
          if (local_position_velocity_state_.GetOurTeamRobots()
                  .Get(i)
                  .ssl_vision_id == robot_state.ssl_vision_id) {
            our_team_kalman_.GetMutable(i)->second.UpdateCmd(robot_state);
          }
        }
      }
    }
  }
}

void KalmanUpdate::UpdateBall(const SSL_WrapperPacket& wrapper_packet) {
  const SSL_DetectionFrame& detection_frame = wrapper_packet.detection();
  // Store the latest ball detection frame
  if (detection_frame.balls().empty()) {
    return;
  }

  local_logger_.LogPrintPush("Ball Update");

  ball_state_estimator.Update(detection_frame,
                              local_position_velocity_state_,
                              &local_shared_state_,
                              &local_logger_);
  Vector2f ball_2d_velocity;
  Vector2f ball_2d_pos;
  ball_state_estimator.Get2DState(&ball_2d_pos, &ball_2d_velocity);

  *(local_position_velocity_state_.GetMutableBallPositionVelocity()) =
      PositionVelocityState::BallPositionVelocity(
          ball_2d_pos,
          ball_2d_velocity,
          ball_state_estimator.GetBallObservation(),
          ball_state_estimator.GetBallObservationTime(),
          ball_state_estimator.GetBallObservationCamera());

  // Chip kick related information
  local_position_velocity_state_.GetMutableBallPositionVelocity()
      ->is_chip_kicked = ball_state_estimator.IsChipKickDetected();

  local_position_velocity_state_.GetMutableBallPositionVelocity()
      ->chip_impact_point = ball_state_estimator.GetChipImpactPoint();

  local_position_velocity_state_.GetMutableBallPositionVelocity()
      ->chip_impact_time = ball_state_estimator.GetChipImpactTime();

  local_position_velocity_state_.GetMutableBallPositionVelocity()
      ->chip_initial_vel = ball_state_estimator.GetChipInitialVel();

  local_position_velocity_state_.GetMutableBallPositionVelocity()
      ->is_chip_kick_tracking_triggered =
      ball_state_estimator.IsChipKickOnsetDetected();

  local_position_velocity_state_.GetMutableBallPositionVelocity()
      ->chip_shot_time = ball_state_estimator.GetChipOnsetTime();

  local_logger_.LogPrint(
      "Ball Position: %f, %f", ball_2d_pos.x(), ball_2d_pos.y());
  local_logger_.LogPrint(
      "Ball 2D velocity: %f, %f", ball_2d_velocity.x(), ball_2d_velocity.y());
  local_logger_.LogPrint("Ball Speed: %f", ball_2d_velocity.norm());

  if (ball_state_estimator.IsChipKickOnsetDetected()) {
    local_logger_.LogPrint("Chip kick onset detected.");
  }

  if (ball_state_estimator.IsChipKickDetected()) {
    Vector3f chip_init_vel = ball_state_estimator.GetChipInitialVel();
    Vector2f ball_impact_point = ball_state_estimator.GetChipImpactPoint();
    local_logger_.LogPrintPush("Chip kick being tracked.");
    local_logger_.LogPrint("Ball initial velocity: %f, %f, %f",
                           chip_init_vel.x(),
                           chip_init_vel.y(),
                           chip_init_vel.z());
    local_logger_.LogPrint("Ball impact point: %f, %f",
                           ball_impact_point.x(),
                           ball_impact_point.y());
    local_logger_.Pop();
  }
  local_logger_.Pop();
}

void KalmanUpdate::UpdateVisionStats(
    const SSL_DetectionFrame& detection_frame) {
  last_ssl_update_timestamp_ = detection_frame.t_capture();
  // Update packets received/dropped for the camera; (log it as well)
  const unsigned int camera_id = detection_frame.camera_id();
  message_counts_[camera_id]++;
  if (last_message_frame_number_[camera_id] != -1) {
    if (last_message_frame_number_[camera_id] !=
        static_cast<int>(detection_frame.frame_number()) - 1) {
      dropped_message_counts_[camera_id]++;
    }
  }
  last_message_frame_number_[camera_id] = detection_frame.frame_number();
  local_logger_.LogPrint("SSL Camera: %d, Received %d, Dropped: %d",
                         camera_id,
                         message_counts_[camera_id],
                         dropped_message_counts_[camera_id]);
}

void KalmanUpdate::PrintRobotStates(
    const string& robot_names,
    const datastructures::DenseArray<
        PositionVelocityState::RobotPositionVelocity,
        kMaxTeamRobots>& robots) {
  local_logger_.LogPrintPush(robot_names);
  for (const auto& robot : robots) {
    local_logger_.LogPrint("%X : (%f, %f, %f, %f, %f, %f)",
                           robot.ssl_vision_id,
                           robot.position.translation.x(),
                           robot.position.translation.y(),
                           robot.position.angle,
                           robot.velocity.translation.x(),
                           robot.velocity.translation.y(),
                           robot.velocity.angle);
  }
  local_logger_.Pop();
}

void KalmanUpdate::UpdateToLog() {
  local_logger_.LogPrintPush("States");
  PrintRobotStates("Our Robots",
                   local_position_velocity_state_.GetOurTeamRobots());
  PrintRobotStates("Their Robots",
                   local_position_velocity_state_.GetTheirTeamRobots());
  local_logger_.Pop();

  for (size_t i = 0; i < our_team_kalman_.GetElementCount(); i++) {
    our_team_kalman_.GetMutable(i)->second.LogTrajectories(&local_logger_);
  }
  for (size_t i = 0; i < their_team_kalman_.GetElementCount(); i++) {
    their_team_kalman_.GetMutable(i)->second.LogTrajectories(&local_logger_);
  }

  // ball_state_estimator.LogTrajectory(&local_logger_);
}

void KalmanUpdate::SortRobots(
    google::protobuf::RepeatedPtrField<SSL_DetectionRobot>* robots) {
  std::sort(robots->begin(),
            robots->end(),
            [](const SSL_DetectionRobot& a, const SSL_DetectionRobot& b) {
              if (a.robot_id() == b.robot_id()) {
                return a.confidence() > b.confidence();
              } else {
                return a.robot_id() < b.robot_id();
              }
            });
}

bool KalmanUpdate::TryReceiveVisionPacket(SSL_WrapperPacket* packet) {
  if (!vision_client_.TryReceiveProtobuf(packet)) return false;
  const double processing_time =
      packet->detection().t_sent() - packet->detection().t_capture();
  if (!use_message_time_) {
    packet->mutable_detection()->set_t_capture(GetWallTime() - processing_time);
  }

  if (packet->has_geometry()) {
    ball_state_estimator.UpdateCameraParams(packet->geometry());
  }

  if (packet->has_detection()) {
    if (camera_mask_.none() ||
        camera_mask_.test(packet->detection().camera_id())) {
      // Either the mask is not set, or it matches the mask. Accept packet.
      return true;
    } else {
      // The mask is set, and it does not match the mask. Reject packet.
      if (kDebug) {
        local_logger_.LogPrint("Dropping camera %d",
                               packet->detection().camera_id());
      }
      return false;
    }
  }
  // No detection packet.
  return false;
}

void KalmanUpdate::StartVisionReceiver() {
  CHECK(
      vision_client_.Open(ssl_vision_udp_address_, ssl_vision_udp_port_, true));
  // Set receive timeout to 50 milliseconds.
  CHECK(vision_client_.SetReceiveTimeout(50000));
  // Ensure that we are ready to receive SSL vision data.
  CHECK(vision_client_.IsOpen());
}

void KalmanUpdate::HandleSimUpdate() {
  std::ofstream ofs;
  if (!kProduction) {
    ofs.open("kalman_timing.txt", std::ofstream::out);
  }
  last_ssl_update_timestamp_ = 0;
  simulating_ = true;
  while (is_running_) {
    Simulator* temp = nullptr;
    const bool read_success = thread_safe_simulator_->ReadOrDefault(&temp);
    if (!read_success) {
      continue;
    }
    NP_NOT_NULL(temp);
    local_simulator_ = temp;
    UpdateRobotCommands();
    // Ask the simulator for the world state as ssl packets
    vector<SSL_WrapperPacket> packets =
        local_simulator_->GetSSLWrapperPackets();
    // Handle all of these packets one at a time
    for (SSL_WrapperPacket ssl_packet : packets) {
      local_logger_.LogPrintPush("Kalman Observation and Update");

      UpdateBall(ssl_packet);

      SSL_DetectionFrame& detection_frame = *(ssl_packet.mutable_detection());

      // Enforce the property of SSL Vision IDs being in ascending order.
      SortRobots(detection_frame.mutable_robots_blue());
      SortRobots(detection_frame.mutable_robots_yellow());

      const unsigned int lead_camera_id =
          ball_state_estimator.GetBallObservationCamera();
      UpdatePositionsAndOrdering(detection_frame, lead_camera_id);

      UpdateVisionStats(detection_frame);

      UpdateToLog();
      local_logger_.Pop();
      thread_safe_position_velocity_state_->Write(
          local_position_velocity_state_);

      thread_safe_logger_->Write(local_logger_);

      if (!kProduction) {
        ofs << std::setprecision(15) << GetMonotonicTime() << "\n";
      }
    }
    local_logger_.Clear();
  }
}

void KalmanUpdate::HandleUpdate() {
  StartVisionReceiver();
  SSL_WrapperPacket ssl_packet;
  while (is_running_) {
    if (!TryReceiveVisionPacket(&ssl_packet)) {
      continue;
    }

    UpdateRobotCommands();
    local_logger_.LogPrintPush("Kalman Observation and Update");

    SSL_DetectionFrame& detection_frame = *(ssl_packet.mutable_detection());
    UpdateVisionStats(detection_frame);

    // Enforce the property of SSL Vision IDs being in ascending order.
    SortRobots(detection_frame.mutable_robots_blue());
    SortRobots(detection_frame.mutable_robots_yellow());

    const unsigned int lead_camera_id =
        ball_state_estimator.GetBallObservationCamera();
    UpdatePositionsAndOrdering(detection_frame, lead_camera_id);

    UpdateBall(ssl_packet);

    UpdateToLog();
    local_logger_.Pop();
    thread_safe_position_velocity_state_->Write(local_position_velocity_state_);

    thread_safe_logger_->Write(local_logger_);

    local_logger_.Clear();
  }
}
}  // namespace app
