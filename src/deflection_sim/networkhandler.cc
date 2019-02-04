// Copyright 2018 kvedder@umass.edu, jaholtz@cs.umass.edu
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

#include "deflection_sim/networkhandler.h"

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
using math_util::AngleDiff;

namespace simulator {

NetworkHandler::NetworkHandler(WorldState* world_state,
                               const string& input_udp_address,
                               const int input_udp_port,
                               const string& output_udp_address,
                               const int output_udp_port,
                               const double worker_loop_rate,
                               const double output_loop_rate,
                               const double input_loop_rate)
    : world_state(world_state),
      input_udp_address(input_udp_address),
      input_udp_port(input_udp_port),
      output_udp_address(output_udp_address),
      output_udp_port(output_udp_port),
      worker_loop_rate(worker_loop_rate),
      output_loop_rate(output_loop_rate),
      input_loop_rate(input_loop_rate),
      success_(false),
      failure_(false) {
  is_running = true;
}

NetworkHandler::~NetworkHandler() {
  // Signal loops of the worker threads to terminate.
  is_running = false;

  threadsafe_input_queue.Shutdown();
  threadsafe_output_queue.Shutdown();

  // Join all threads.
  work_thread.join();
  output_thread.join();
  input_thread.join();
}

void NetworkHandler::Start() {
  input_thread = thread(&NetworkHandler::HandleInput, this, input_udp_address,
                        input_udp_port);
  output_thread = thread(&NetworkHandler::HandleOutput, this,
                         output_udp_address, output_udp_port);
  work_thread = thread(&NetworkHandler::Work, this);
}

bool NetworkHandler::Success() {
  return success_;
}

bool NetworkHandler::DeflectionSuccess() {
//   Vector2f ball = world_state->GetBall().GetPose2Df().translation;
  const float robot_angle =
    world_state->GetRobots()[0].GetPose().angle;
  Vector2f ball_velocity = world_state->GetBall().GetVelocity();
  const Vector2f x_dir = Heading(robot_angle);
  const float relative_ball_vel = x_dir.dot(ball_velocity);
  const float relative_percent =
      (relative_ball_vel / ball_velocity.norm()) * 100;
  const bool kicked = relative_percent > 85;
  if (success_ || kicked) {
    return true;
  }
  return false;
}

bool NetworkHandler::DockingSuccess() {
  Vector2f dock_location(0, 0);
//   Vector2f ball = world_state->GetBall().GetPose2Df().translation;
  const float robot_heading =
    world_state->GetRobots()[0].GetPose().angle;
  const Vector2f position =
    world_state->GetRobots()[0].GetPose().translation;
//   Vector2f ball_velocity = world_state->GetBall().GetVelocity();
  const Vector2f robot_to_dock_displace =
      dock_location - position;
  const Eigen::Vector2f robot_direction = Heading(robot_heading);
  const Vector2f y_dir(-sin(robot_heading), cos(robot_heading));
  float x_dist = robot_direction.dot(robot_to_dock_displace);
  const float rotation_difference = RadToDeg(AngleDiff(robot_heading,
                                                       0.0f));

  float y_dist = y_dir.dot(robot_to_dock_displace);
  const bool docked = fabs(y_dist) < 40
                      && fabs(x_dist) < 40
                      && fabs(rotation_difference) < 10.0;
  if (docked) {
    return true;
  }
  return false;
}

bool NetworkHandler::Failure() {
  return failure_;
}

void NetworkHandler::HandleInput(const string udpAddress, const int udpPort) {
  UDPMulticastServer udp_server;
  if (!udp_server.Open(udpAddress, udpPort, true)) {
    LOG(ERROR) << "Error opening UDP port of HandleInput thread, exiting.";
  } else {
  }
  CHECK(udp_server.IsOpen());


  RadioProtocolWrapper wrapper;
  RateLoop loop(input_loop_rate);

  while (is_running.load()) {
    if (udp_server.TryReceiveProtobuf(&wrapper)) {
      threadsafe_input_queue.Add(wrapper);
    }
    loop.Sleep();
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
      LOG(FATAL) << "Camera ID out of range\n";
  }
  return in_frame;
}

void NetworkHandler::SetupSSLWrapperPacket(
    vector<SSL_WrapperPacket*>* wrapper_packets, int frame_index,
    util_random::Random* random) {
  for (int camera_id = 0; camera_id < static_cast<int>(wrapper_packets->size());
       ++camera_id) {
    SSL_WrapperPacket* wrapper_packet = (*wrapper_packets)[camera_id];
    // TODO(kvedder): Setup packet from actual world state.
    SSL_DetectionFrame* detection = wrapper_packet->mutable_detection();
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
        SSL_DetectionRobot* ssl_robot;
        const Pose2Df& pose = robot.GetPose();
        const float velocity_norm = robot.GetVelocity().norm();

        if (IsInCameraField(pose.translation, camera_id)) {
          // TODO(kvedder): Figure out how to delineate teams.
          if (robot.GetId() < 6) {
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
            ssl_robot->set_orientation(random->Gaussian(pose.angle,
                                       kSimulatorRobotAngleNoiseStdDev));
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

bool CheckSuccess(Vector2f ball) {
  Vector2f goal = {4500, 0};
  float square_dist = pow(ball[0] - goal[0], 2) + pow(ball[1] - goal[1], 2);
  if (square_dist < pow(500, 2)) {
    std::cout << "SUCCESS!" << std::endl;
    return true;
  }
  return false;
}

void NetworkHandler::Work() {
  vector<RadioProtocolWrapper> local_queue;
  RateLoop loop(worker_loop_rate);

  util_random::Random random;

  int frame_index = 0;
  while (is_running) {
    threadsafe_input_queue.ReadAllAndEmpty(&local_queue);

    if (!local_queue.empty()) {
      // Only read the most recent packet.
      const RadioProtocolWrapper& velocity_command =
          local_queue[local_queue.size() - 1];
      if (ValidateVelocityCommandIntegrity(velocity_command)) {
        world_state->Update(velocity_command);
      } else {
        LOG(ERROR) << "Recieved invalid RadioProtocolWrapper as input!";
      }
    }
    local_queue.clear();

    // Setup the messages for the four cameras.
    vector<SSL_WrapperPacket*> wrapper_packets;
    SSL_WrapperPacket message_camera_0;
    wrapper_packets.push_back(&message_camera_0);
    SSL_WrapperPacket message_camera_1;
    wrapper_packets.push_back(&message_camera_1);
    SSL_WrapperPacket message_camera_2;
    wrapper_packets.push_back(&message_camera_2);
    SSL_WrapperPacket message_camera_3;
    wrapper_packets.push_back(&message_camera_3);

    SetupSSLWrapperPacket(&wrapper_packets, frame_index, &random);
    Vector2f ball_position = world_state->GetBall().GetPose2Df().translation;
    const int frame_cap = 2000;  // 1000
    if (CheckSuccess(ball_position)) {
      success_ = true;
    } else if (frame_index > frame_cap) {
      failure_ = true;
    }
    ++frame_index;
    for (const SSL_WrapperPacket* message : wrapper_packets) {
      threadsafe_output_queue.Add(*message);
    }

    loop.Sleep();
  }
}

void NetworkHandler::HandleOutput(const string udpAddress, const int udpPort) {
  UDPMulticastServer udp_server;
  if (!udp_server.Open(udpAddress, udpPort, false)) {
    LOG(ERROR) << "Error opening UDP port of HandleOutput thread, exiting.";
  } else {
  }
  CHECK(udp_server.IsOpen());


  vector<SSL_WrapperPacket> local_queue;

  RateLoop loop(output_loop_rate);

  while (is_running) {
    threadsafe_output_queue.ReadAllAndEmpty(&local_queue);
    auto time_sent = GetWallTime();
    for (SSL_WrapperPacket& output : local_queue) {
      output.mutable_detection()->set_t_sent(time_sent);
      if (!udp_server.SendProtobuf(output)) {
        LOG(ERROR) << "Send output error";
      } else {
      }
    }
    local_queue.clear();
    loop.Sleep();
  }

  udp_server.Close();
}

}  // namespace simulator
