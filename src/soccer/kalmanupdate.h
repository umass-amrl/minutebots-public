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
// ================================================================]========

#ifndef SRC_SOCCER_KALMANUPDATE_H_
#define SRC_SOCCER_KALMANUPDATE_H_

#include <algorithm>
#include <atomic>
#include <bitset>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "constants/constants.h"
#include "experimental_sim/experimental_sim.h"
#include "gtest/gtest_prod.h"
#include "logging/logger.h"
#include "math/poses_2d.h"
#include "net/netraw.h"
#include "state/direction.h"
#include "state/position_velocity_state.h"
#include "state/shared_state.h"
#include "state/world_state.h"
#include "state_estimation/ball_state_estimator.h"
#include "state_estimation/camera_parameters_tuner.h"
#include "state_estimation/chip_kick_tester.h"
#include "state_estimation/extended_kalman_filter.h"
#include "thread_safe/thread_safe_actor.h"
#include "thread_safe/thread_safe_priority_queue.h"
#include "thread_safe/thread_safe_queue.h"

#include "messages_robocup_ssl_wrapper.pb.h"

namespace app {
using PositionVelocityArray = datastructures::DenseArray<
    state::PositionVelocityState::RobotPositionVelocity, kMaxTeamRobots>;
using KalmanArray = datastructures::DenseArray<
    std::pair<SSLVisionId, estimation::ExtendedKalmanFilter>, kMaxTeamRobots>;

class KalmanUpdate {
 public:
  KalmanUpdate() = delete;
  KalmanUpdate(const std::bitset<kNumCameras>& camera_mask,
               const std::string& ssl_vision_udp_address,
               const int ssl_vision_udp_port,
               threadsafe::ThreadSafeActor<state::PositionVelocityState>*
                   thread_safe_position_velocity_state,
               threadsafe::ThreadSafeActor<logger::Logger>* thread_safe_logger,
               threadsafe::ThreadSafeQueue<state::SharedState>*
                   thread_safe_shared_state_queue,
               const direction::Direction& direction,
               const team::Team& team,
               bool use_message_time);

  KalmanUpdate(const KalmanUpdate& other) = delete;
  KalmanUpdate(KalmanUpdate&& other) = delete;

  ~KalmanUpdate();

  void Start();

  void SimStart(threadsafe::ThreadSafeActor<
                    experimental_simulator::Simulator*>* thread_safe_sim,
                experimental_simulator::Simulator* simulator);

  void Stop();

  // Used to switch to test mode for state estimation
  uint state_estimation_test_type;
  uint camera_param_tuning_mode;

 private:
  static const bool kDebug = true;

  void RemoveTimedOutRobots(const double observ_time, const double timeout,
                            const bool our_team, PositionVelocityArray* robots);

  bool UpdateIfExistingRobotKalman(
      const double detection_timestamp,
      const unsigned int camera_id,
      const unsigned int lead_camera_id,
      const ::SSLVisionProto::SSL_DetectionRobot& detection,
      PositionVelocityArray* robots,
      KalmanArray* kalman_array);

  bool UpdateIfExistingRobotVision(
      const double detection_timestamp,
      const unsigned int camera_id,
      const unsigned int lead_camera_id,
      const ::SSLVisionProto::SSL_DetectionRobot& detection,
      PositionVelocityArray* robots);

  void AddNewRobotKalman(const double detection_timestamp,
                         const unsigned int camera_id,
                         const unsigned int lead_camera_id,
                         const ::SSLVisionProto::SSL_DetectionRobot& detection,
                         bool is_ours,
                         PositionVelocityArray* robots,
                         KalmanArray* kalman_array);

  void AddNewRobotVision(const double detection_timestamp,
                         const unsigned int camera_id,
                         const unsigned int lead_camera_id,
                         const ::SSLVisionProto::SSL_DetectionRobot& detection,
                         PositionVelocityArray* robots);

  void UpdateUnseenKalmanWithForwardPrediction(
      const double observation_time,
      state::PositionVelocityState::RobotPositionVelocity*
          robot_position_velocity,
      estimation::ExtendedKalmanFilter* kalman);

  void UpdateUnseenVisionWithForwardPrediction(
      const double observation_time,
      state::PositionVelocityState::RobotPositionVelocity*
          robot_position_velocity);

  FRIEND_TEST(StateEstimationTest, TestOrdering);

  // Updates the ordering of the Kalman filters and position velocity state in
  // order to keep their indices in sync.
  //
  // Performs the actual update of the state of each Kalman filter and forward
  // predicts that to update each Position Velocity state.
  void UpdatePositionsAndOrdering(
      const SSLVisionProto::SSL_DetectionFrame& detection_frame,
      const unsigned int lead_camera_id);

  void HandleUpdate();

  void HandleSimUpdate();

  bool IsObservationInField(const SSL_DetectionRobot& robot);

  pose_2d::Pose2Df ExtractPose(const SSL_DetectionRobot& observed_ssl_robot);

  void UpdateRobotCommands();

  void UpdateBall(const SSLVisionProto::SSL_WrapperPacket& wrapper_packet);

  void PrintRobotStates(const std::string& robot_names,
                        const datastructures::DenseArray<
                            state::PositionVelocityState::RobotPositionVelocity,
                            kMaxTeamRobots>& robots);

  // Add debug logging text and drawings.
  void UpdateToLog();

  // Enforce the property of SSL Vision IDs being in ascending order.
  void SortRobots(google::protobuf::RepeatedPtrField<
                  SSLVisionProto::SSL_DetectionRobot>* robots);

  void UpdateVisionStats(
      const SSLVisionProto::SSL_DetectionFrame& detection_frame);

  bool TryReceiveVisionPacket(SSLVisionProto::SSL_WrapperPacket* packet);

  void StartVisionReceiver();

  // Input parameters.
  threadsafe::ThreadSafeActor<state::PositionVelocityState>*
      thread_safe_position_velocity_state_;
  threadsafe::ThreadSafeActor<logger::Logger>* thread_safe_logger_;
  threadsafe::ThreadSafeQueue<state::SharedState>*
      thread_safe_shared_state_queue_;
  const direction::Direction& direction_;

  // Local objects.
  state::PositionVelocityState local_position_velocity_state_;
  logger::Logger local_logger_;
  state::SharedState local_shared_state_;
  datastructures::BoundedQueue<std::pair<Eigen::Vector2f, double>>
      ball_position_queue_;
  estimation::BallStateEstimator ball_state_estimator;
  double last_ssl_update_timestamp_;
  team::Team team_;

  // Default constructing members.
  static const unsigned int kAverageBallNumSteps = 5;
  std::vector<int> message_counts_;
  std::vector<int> dropped_message_counts_;
  std::vector<int> last_message_frame_number_;
  std::thread update_thread_;
  datastructures::DenseArray<
      std::pair<SSLVisionId, estimation::ExtendedKalmanFilter>, kMaxTeamRobots>
      our_team_kalman_;
  datastructures::DenseArray<
      std::pair<SSLVisionId,
                estimation::ExtendedKalmanFilter>, kMaxTeamRobots>
      their_team_kalman_;

  std::atomic_bool is_running_;

  const string ssl_vision_udp_address_;
  const int ssl_vision_udp_port_;
  const std::bitset<kNumCameras> camera_mask_;
  net::UDPMulticastServer vision_client_;
  threadsafe::ThreadSafeActor<experimental_simulator::Simulator*>*
      thread_safe_simulator_;
  experimental_simulator::Simulator* local_simulator_;
  bool simulating_;
  bool use_message_time_;
};
}  // namespace app

#endif  // SRC_SOCCER_KALMANUPDATE_H_
