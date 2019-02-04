// Copyright 2018 slane@cs.umass.edu
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

#include "open_loop_executors/camera_reader.h"

#include <thread>

#include "constants/constants.h"
#include "constants/typedefs.h"
#include "constants/includes.h"
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

Observation::Observation()
  : obs(0, 0, 0),
    time(0) {}


CameraReader::CameraReader(Team team,
                           SSLVisionId robot_id,
                           unsigned int camera_id,
                           ThreadSafeActor<Observation>*
                                thread_safe_observation)
  : team_(team),
    robot_id_(robot_id),
    camera_id_(camera_id),
    thread_safe_observation_(thread_safe_observation),
    is_running_(true) {
}
void CameraReader::Run() {
  SSL_WrapperPacket packet;
  Observation observation;
  while (is_running_) {
    if (!vision_client_.TryReceiveProtobuf(&packet))
      continue;
    if (packet.has_detection()) {
      if (packet.detection().camera_id() != camera_id_)
        continue;
      const double processing_time =
          packet.detection().t_sent() - packet.detection().t_capture();
      double capture_time = GetWallTime() - processing_time;

      bool found_robot = false;
      Pose2Df pose(0, 0, 0);

      const auto& detection_frame = packet.detection();
      const auto& our_robot_detection = (team_ == Team::YELLOW)
                                        ? detection_frame.robots_yellow()
                                        : detection_frame.robots_blue();
      for (const auto& robot : our_robot_detection) {
        if (robot.robot_id() != robot_id_) {
          continue;
        } else {
          found_robot = true;
          pose.translation.x() = robot.x();
          pose.translation.y() = robot.y();
          pose.angle = robot.orientation();
          break;
        }
      }

      if (found_robot) {
        observation.obs = pose;
        observation.time = capture_time;
        thread_safe_observation_->Write(observation);
      }
    }
  }
}

void CameraReader::Start() {
  vision_client_.Open(DATA_STREAM_VISION_IP, DATA_STREAM_VISION_PORT, true);
  vision_client_.SetReceiveTimeout(50000);

  update_thread_ = thread(&CameraReader::Run, this);
}

void CameraReader::Stop() {
  is_running_ = false;
  update_thread_.join();
}


}  // namespace open_loop
