// Copyright 2017-2019 kvedder@umass.edu
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

#include "constants/constants.h"
#include "soccer/kalmanupdate.h"
#include "state/position_velocity_state.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
#include "thread_safe/thread_safe_actor.h"
#include "thread_safe/thread_safe_queue.h"

STANDARD_USINGS;
using direction::Direction;
using pose_2d::Pose2Df;
using state::PositionVelocityState;
using state::SharedState;
using state::WorldState;
using team::Team;
using logger::Logger;
using threadsafe::ThreadSafePriorityQueue;
using threadsafe::ThreadSafeQueue;
using threadsafe::ThreadSafeActor;
using std::vector;

namespace app {

TEST(StateEstimationTest, HelloWorld) { ASSERT_TRUE(true); }

TEST(StateEstimationTest, TestOrdering) {
  Direction direction = Direction::POSITIVE;
  Team team = Team::BLUE;
  vector<SSLVisionId> ssl_vision_ids;
  PositionVelocityState global_position_velocity_state;
  Logger global_logger;

  ThreadSafeQueue<SharedState> thread_safe_shared_state_queue;
  ThreadSafeActor<PositionVelocityState> thread_safe_position_velocity_state(
      global_position_velocity_state);
  ThreadSafeActor<Logger> thread_safe_kalman_logger(global_logger);

  KalmanUpdate kalman_update(std::bitset<kNumCameras>(),
                             DATA_STREAM_VISION_IP,
                             DATA_STREAM_VISION_PORT,
                             &thread_safe_position_velocity_state,
                             &thread_safe_kalman_logger,
                             &thread_safe_shared_state_queue,
                             direction,
                             team,
                             false);

  SSL_DetectionFrame detection_frame;
  detection_frame.set_camera_id(0);
  detection_frame.set_t_capture(0);
  detection_frame.set_t_sent(0);
  google::protobuf::RepeatedPtrField< ::SSLVisionProto::SSL_DetectionRobot>&
      our_robot_detection = *(detection_frame.mutable_robots_blue());

  for (int i = 0; i < static_cast<int>(kMaxTeamRobots); ++i) {
    SSL_DetectionRobot* our_ssl_robot = our_robot_detection.Add();
    our_ssl_robot->set_robot_id(i);
    our_ssl_robot->set_x(1000 * i + 1000);
    our_ssl_robot->set_y(0);
    our_ssl_robot->set_confidence(1.0);
    our_ssl_robot->set_orientation(0);
  }

  google::protobuf::RepeatedPtrField< ::SSLVisionProto::SSL_DetectionRobot>&
      their_robot_detection = *(detection_frame.mutable_robots_yellow());
  for (int i = 0; i < static_cast<int>(kMaxTeamRobots); ++i) {
    SSL_DetectionRobot* their_ssl_robot = their_robot_detection.Add();
    their_ssl_robot->set_robot_id(i);
    their_ssl_robot->set_x(-1000 * i - 1000);
    their_ssl_robot->set_y(0);
    their_ssl_robot->set_confidence(1.0);
    their_ssl_robot->set_orientation(0);
  }

  LOG(INFO) << "COMMAND: ";

  for (const auto& our : our_robot_detection) {
    LOG(INFO) << our.x() << " " << our.y();
  }

  for (const auto& their : their_robot_detection) {
    LOG(INFO) << their.x() << " " << their.y();
  }

  LOG(INFO) << "BEFORE: >>>>>>>";

  for (const auto& our_robot :
       kalman_update.local_position_velocity_state_.GetOurTeamRobots()) {
    LOG(INFO) << our_robot.ssl_vision_id << " (" << our_robot.position.angle
              << ", (" << our_robot.position.translation.x() << ", "
              << our_robot.position.translation.y() << "))"
              << our_robot.observed_time;
  }

  for (const auto& their_robot :
       kalman_update.local_position_velocity_state_.GetTheirTeamRobots()) {
    LOG(INFO) << their_robot.ssl_vision_id << " (" << their_robot.position.angle
              << ", (" << their_robot.position.translation.x() << ", "
              << their_robot.position.translation.y() << "))"
              << their_robot.observed_time;
  }

  LOG(INFO) << "<<<<<<<<";

  kalman_update.UpdatePositionsAndOrdering(detection_frame, 0);

  LOG(INFO) << "AFTER: >>>>>>>";

  for (const auto& our_robot :
       kalman_update.local_position_velocity_state_.GetOurTeamRobots()) {
    LOG(INFO) << our_robot.ssl_vision_id << " (" << our_robot.position.angle
              << ", (" << our_robot.position.translation.x() << ", "
              << our_robot.position.translation.y() << "))"
              << our_robot.observed_time;
  }

  for (const auto& their_robot :
       kalman_update.local_position_velocity_state_.GetTheirTeamRobots()) {
    LOG(INFO) << their_robot.ssl_vision_id << " (" << their_robot.position.angle
              << ", (" << their_robot.position.translation.x() << ", "
              << their_robot.position.translation.y() << "))"
              << their_robot.observed_time;
  }

  LOG(INFO) << "<<<<<<<<";

  for (auto& our_ssl_robot : our_robot_detection) {
    our_ssl_robot.set_x(our_ssl_robot.x() + 20);
  }

  for (auto& their_ssl_robot : their_robot_detection) {
    their_ssl_robot.set_x(their_ssl_robot.x() + 20);
  }

  kalman_update.UpdatePositionsAndOrdering(detection_frame, 0);

  LOG(INFO) << "AFTER V2: >>>>>>>";

  for (const auto& our_robot :
       kalman_update.local_position_velocity_state_.GetOurTeamRobots()) {
    LOG(INFO) << our_robot.ssl_vision_id << " (" << our_robot.position.angle
              << ", (" << our_robot.position.translation.x() << ", "
              << our_robot.position.translation.y() << "))"
              << our_robot.observed_time;
  }

  for (const auto& their_robot :
       kalman_update.local_position_velocity_state_.GetTheirTeamRobots()) {
    LOG(INFO) << their_robot.ssl_vision_id << " (" << their_robot.position.angle
              << ", (" << their_robot.position.translation.x() << ", "
              << their_robot.position.translation.y() << "))"
              << their_robot.observed_time;
  }

  LOG(INFO) << "<<<<<<<<";

  thread_safe_position_velocity_state.Shutdown();
  thread_safe_kalman_logger.Shutdown();
  thread_safe_shared_state_queue.Shutdown();
}

}  // namespace app
