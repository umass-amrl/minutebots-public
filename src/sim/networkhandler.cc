// Copyright 2016 - 2019 kvedder@umass.edu
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

#include "sim/networkhandler.h"

#include <glog/logging.h>
#include <condition_variable>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>

#include "constants/constants.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "util/colorize.h"
#include "util/timer.h"

STANDARD_USINGS;
using colorize::ColorGreen;
using pose_2d::Pose2Df;
using net::UDPMulticastServer;
using SSLVisionProto::SSL_DetectionBall;
using SSLVisionProto::SSL_DetectionFrame;
using SSLVisionProto::SSL_DetectionRobot;
using SSLVisionProto::SSL_FieldLineSegment;
using SSLVisionProto::SSL_FieldCicularArc;
using SSLVisionProto::SSL_GeometryData;
using SSLVisionProto::SSL_GeometryFieldSize;
using SSLVisionProto::SSL_WrapperPacket;
using std::string;
using std::cout;
using std::lock_guard;
using std::endl;
using std::cerr;
using std::thread;
using std::vector;
using std::atomic_bool;
using std::string;
using std::vector;
using std::mutex;
using std::condition_variable;
using std::queue;

namespace simulator {

NetworkHandler::NetworkHandler(WorldState* world_state,
                               const std::string& input_udp_address,
                               const int input_udp_port,
                               const std::string& output_udp_address,
                               const int output_udp_port,
                               const double output_loop_rate) :
      world_state(world_state),
      input_udp_address(input_udp_address),
      input_udp_port(input_udp_port),
      output_udp_address(output_udp_address),
      output_udp_port(output_udp_port),
      output_loop_rate(output_loop_rate) {
  is_running = true;
}

NetworkHandler::~NetworkHandler() {
  // Signal loops of the worker threads to terminate.
  is_running = false;

  threadsafe_input_queue.Shutdown();

  // Join all threads.
  output_thread.join();
  input_thread.join();
}

void NetworkHandler::Start() {
  input_thread = thread(&NetworkHandler::HandleInput, this, input_udp_address,
                        input_udp_port);
  output_thread = thread(&NetworkHandler::HandleOutput, this,
                         output_udp_address, output_udp_port);
}

void NetworkHandler::HandleInput(const string udpAddress, const int udpPort) {
  UDPMulticastServer udp_server;
  CHECK(udp_server.Open(udpAddress, udpPort, true));
  // Set receive timeout to 20 milliseconds.
  CHECK(udp_server.SetReceiveTimeout(20000));
  CHECK(udp_server.IsOpen());

  RadioProtocolWrapper wrapper;

  while (is_running.load()) {
    if (!udp_server.TryReceiveProtobuf(&wrapper)) continue;
    if (ValidateVelocityCommandIntegrity(wrapper)) {
      world_state->Update(wrapper);
    } else {
      LOG(ERROR) << "Recieved invalid RadioProtocolWrapper as input!";
    }
  }

  udp_server.Close();
}

bool NetworkHandler::ValidateVelocityCommandIntegrity(
    const RadioProtocolWrapper& velocity_command) {
  if (velocity_command.command_size() == 0) {
    LOG(ERROR) << "No velocity commands given to simulator!";
    return false;
  }
  return true;
}

bool IsInCameraField(const Vector2f& position, const int camera_id) {
  bool in_frame = false;
  switch (camera_id) {
    case 0:
      in_frame = ((position.x() >= 0) && (position.y() >= 0));
      break;
    case 1:
      in_frame = ((position.x() <= 0) && (position.y() >= 0));
      break;
    case 2:
      in_frame = ((position.x() <= 0) && (position.y() <= 0));
      break;
    case 3:
      in_frame = ((position.x() >= 0) && (position.y() <= 0));
      break;
    default:
      LOG(FATAL) << "Camera ID out of range (" << camera_id << ")";
  }
  return in_frame;
}

// Sets up the metadata for each given SSL Packet.
void NetworkHandler::MakeSSLWrapperPackets(
    int frame_index,
    util_random::Random* random,
    vector<SSL_WrapperPacket>* packets) {
  // Setup the messages for the four cameras with default constructed
  // SSL_WrapperPackets.
  vector<SSL_WrapperPacket>& wrapper_packets = *packets;
  wrapper_packets.resize(kNumCameras);
  for (size_t camera_id = 0; camera_id < kNumCameras; ++camera_id) {
    SSL_WrapperPacket& wrapper_packet = wrapper_packets[camera_id];
    wrapper_packet.Clear();
    // TODO(kvedder): Setup packet from actual world state.
    SSL_DetectionFrame* detection = wrapper_packet.mutable_detection();
    // If we need to define field lines again, look at commit:
    // 5e130f22d8d0224df26ff84d8c1da86f3aa400b8

    detection->set_frame_number(frame_index);
    // TODO(kvedder): Add lag between time captured and time sent.
    auto time_capture = GetWallTime();
    detection->set_t_capture(time_capture);
    detection->set_t_sent(time_capture);
    detection->set_camera_id(camera_id);  // ID associated with the proto.

    // Check to see if ball is within this camera's frame.
    // Frames are numbered as per this chart:
    // https://raw.githubusercontent.com/RoboCup-SSL/ssl-vision/wiki/images/4cam_setup.png
    const Pose2Df& ball_position = world_state->GetBall().GetPose2Df();

    if (IsInCameraField(ball_position.translation, camera_id)) {
      SSL_DetectionBall* ball = detection->add_balls();
      ball->set_confidence(1);  // Full confidence.
      ball->set_x(ball_position.translation.x());
      ball->set_y(ball_position.translation.y());
      // TODO(kvedder): Figure out what set_pixel does.
      ball->set_pixel_x(0);
      ball->set_pixel_y(0);
    }

    for (const Robot& robot : world_state->GetRobots()) {
      if (random->UniformRandom() > kSimulatorPacketLossPercent) {
        const Pose2Df& pose = robot.GetPose();
        const float velocity_norm = robot.GetVelocity().norm();

        SSL_DetectionRobot* ssl_robot = nullptr;
        if (IsInCameraField(pose.translation, camera_id)) {
          // TODO(kvedder): Figure out how to delineate teams.
          if (robot.GetTeam() == team::Team::BLUE) {
            ssl_robot = detection->add_robots_blue();
          } else {
            ssl_robot = detection->add_robots_yellow();
          }

          ssl_robot->set_robot_id(robot.GetId());
          ssl_robot->set_confidence(1);
          if (kSimulatorRobotNoiseStdDev > 0) {
            ssl_robot->set_x(
                random->Gaussian(pose.translation.x(),
                                 kSimulatorRobotNoiseStdDev * velocity_norm));
            ssl_robot->set_y(
                random->Gaussian(pose.translation.y(),
                                 kSimulatorRobotNoiseStdDev * velocity_norm));
            ssl_robot->set_orientation(
                random->Gaussian(pose.angle, kSimulatorRobotAngleNoiseStdDev));
          } else {
            ssl_robot->set_x(pose.translation.x());
            ssl_robot->set_y(pose.translation.y());
            ssl_robot->set_orientation(pose.angle);
          }
          // TODO(kvedder): Figure out what set_pixel does.
          ssl_robot->set_pixel_x(0);
          ssl_robot->set_pixel_y(0);
        }
      }
    }
  }
}

void NetworkHandler::HandleOutput(const string udpAddress, const int udpPort) {
  UDPMulticastServer udp_server;
  if (!udp_server.Open(udpAddress, udpPort, false)) {
    LOG(ERROR) << "Error opening UDP port of HandleOutput thread, exiting.";
  } else {
  }
  CHECK(udp_server.IsOpen());

  util_random::Random random;
  RateLoop loop(output_loop_rate);
  int frame_index = 0;
  vector<SSL_WrapperPacket> packets;
  while (is_running) {
    MakeSSLWrapperPackets(frame_index, &random, &packets);
    for (const auto& output : packets) {
      if (!udp_server.SendProtobuf(output)) {
        LOG(ERROR) << "Send output error";
      }
    }
    ++frame_index;
    loop.Sleep();
  }

  udp_server.Close();
}

}  // namespace simulator
