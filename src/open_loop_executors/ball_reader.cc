// Copyright 2018 - 2019 slane@cs.umass.edu
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

#include "open_loop_executors/ball_reader.h"

#include <thread>

#include "constants/constants.h"
#include "constants/includes.h"
#include "constants/typedefs.h"
#include "math/poses_2d.h"
#include "net/netraw.h"
#include "state/team.h"
#include "thread_safe/thread_safe_actor.h"

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using team::Team;
using threadsafe::ThreadSafeActor;
using SSLVisionProto::SSL_DetectionBall;
using SSLVisionProto::SSL_DetectionFrame;
using SSLVisionProto::SSL_DetectionRobot;
using SSLVisionProto::SSL_WrapperPacket;
using std::thread;

namespace open_loop {

BallReader::BallReader(
    const unsigned int camera_id,
    ThreadSafeActor<BallObservation>* thread_safe_observation)
    : camera_id_(camera_id),
      thread_safe_observation_(thread_safe_observation),
      is_running_(true) {}

void BallReader::Run() {
  SSL_WrapperPacket packet;
  while (is_running_) {
    if (!vision_client_.TryReceiveProtobuf(&packet)) continue;
    if (packet.has_detection()) {
      if (packet.detection().camera_id() != camera_id_) continue;
      const double processing_time =
          packet.detection().t_sent() - packet.detection().t_capture();
      const double capture_time = GetWallTime() - processing_time;
      const auto& detection_frame = packet.detection();
      BallObservation ball_observation;
      ball_observation.time = capture_time;
      for (const SSL_DetectionBall& ball : detection_frame.balls()) {
        ball_observation.observations.push_back(
            {{ball.x(), ball.y()}, ball.confidence()});
      }

      if (!ball_observation.observations.empty()) {
        thread_safe_observation_->Write(ball_observation);
      }
    }
  }
}

void BallReader::Start() {
  vision_client_.Open(DATA_STREAM_VISION_IP, DATA_STREAM_VISION_PORT, true);
  vision_client_.SetReceiveTimeout(50000);

  update_thread_ = thread(&BallReader::Run, this);
}

void BallReader::Stop() {
  is_running_ = false;
  update_thread_.join();
}

}  // namespace open_loop
