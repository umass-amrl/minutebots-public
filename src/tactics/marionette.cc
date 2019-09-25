// Copyright 2019 jspitzer@cs.umass.edu, kvedder@umass.edu,
// joydeepb@cs.umass.edu
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
#include "js_bots.pb.h"
#include "tactics/marionette.h"

STANDARD_USINGS;
using JSBotsProto::Puppet;
using motion::MotionModel;
using obstacle::ObstacleFlag;
using pose_2d::Pose2Df;
using state::SharedState;
using state::WorldState;
using tactics::TacticIndex;

namespace tactics {
static constexpr char kReadAddress[] = "224.5.23.3";
static constexpr int kReadDuration = 10;
static constexpr int kReadPort = 41234;
static constexpr float kMaxAcc = 1000;
static constexpr float kMaxVel = 1000;

void ReceiveThread(const std::atomic_bool* needs_shutdown,
                   net::UDPMulticastServer* udp_server,
                   std::atomic<Marionette::PuppetData>* shared_puppet_data,
                   const std::atomic_uint* ssl_vision_id) {
  if (!udp_server->Open(kReadAddress, kReadPort, true)) {
    LOG(FATAL) << "Failed to open Marionette server.";
  }

  Puppet message;
  while (!(needs_shutdown->load())) {
    if (udp_server->TryReceiveProtobuf(&message) &&
        message.ssl_vision_id() == ssl_vision_id->load()) {
      if (!message.kick() && !message.halt()) {
        shared_puppet_data->store({message.x(), message.y(), message.theta(),
          false, false});
      } else {
        Marionette::PuppetData local_puppet_data = shared_puppet_data->load();

        if (message.kick()) {
          shared_puppet_data->store({local_puppet_data.x, local_puppet_data.y,
            local_puppet_data.theta, true, false});
        } else {
          shared_puppet_data->store({local_puppet_data.x, local_puppet_data.y,
            local_puppet_data.theta, false, true});
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kReadDuration));
  }
}

Marionette::Marionette(const WorldState& world_state,
                       TacticArray* tactic_list,
                       SharedState* shared_state,
                       OurRobotIndex our_robot_index,
                       state::SoccerState* soccer_state)
    : Tactic(world_state,
             tactic_list,
             shared_state,
             our_robot_index,
             soccer_state),
      udp_server_(),
      shared_puppet_data_(PuppetData(0, 0, 0, false, true)),
      needs_shutdown_(false),
      ssl_vision_id_(0),
      recieve_thread_(
          ReceiveThread,
          &needs_shutdown_,
          &udp_server_,
          &shared_puppet_data_,
          &ssl_vision_id_) {}

Marionette::~Marionette() {
  needs_shutdown_ = true;
  recieve_thread_.join();
  udp_server_.Close();
}

void Marionette::Run() {
  ssl_vision_id_.store(world_state_.GetOurRobotPosition(our_robot_index_)
    .ssl_vision_id);
  logger::Logger* logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  PuppetData local_puppet_data = shared_puppet_data_.load();

  NTOC_Controller* ntoc_controller = static_cast<NTOC_Controller*>(
      (*tactic_list_)[TacticIndex::NTOC].get());
  MotionModel motion_model(kMaxAcc, kMaxVel);
  ntoc_controller->SetMotionModel(motion_model);

  if (!local_puppet_data.kick && !local_puppet_data.halt) {
    logger->LogPrint("Marionette to: x: %f, y: %f, theta: %f",
                 local_puppet_data.x,
                 local_puppet_data.y,
                 local_puppet_data.theta);

    auto flags = ObstacleFlag::GetAllExceptTeam(world_state_, *soccer_state_,
      our_robot_index_, our_robot_index_) & ~ObstacleFlag::GetDefenseAreas()
      & ~ObstacleFlag::GetAllBalls();
    safety::DSS2::SetObstacleFlag(our_robot_index_, flags);

    EightGridNavigation* controller = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());
    controller->SetObstacles(flags);
    controller->SetGoal(
        {local_puppet_data.theta, local_puppet_data.x, local_puppet_data.y});
    controller->Run();

  } else if (local_puppet_data.kick) {
    if (ShouldKick()) {
      logger->LogPrint("Marionette kicking: theta: %f",
                   local_puppet_data.theta);

      ThreeKick* controller = static_cast<ThreeKick*>((*tactic_list_)
        [TacticIndex::THREE_KICK].get());
      controller->SetGoal({local_puppet_data.theta, 0, 0});
      controller->Run();

      state::SharedRobotState* state =
          shared_state_->GetSharedState(our_robot_index_);
      state->flat_kick = 1.0;

      if (controller->IsComplete()) {
        Reset();
      }
    }
  } else {
    logger->LogPrint("Halting Marionette");
    (*tactic_list_)[TacticIndex::HALT].get()->Run();
  }
}

bool Marionette::ShouldKick() {
  return true;  // TODO(sp1tz)
}

void Marionette::Reset() {
  PuppetData local_puppet_data = shared_puppet_data_.load();
  shared_puppet_data_.store({local_puppet_data.x, local_puppet_data.y,
            local_puppet_data.theta, false, false});
}

void Marionette::Init() {}

void Marionette::SetGoal(const Pose2Df& pose) {}
}  // namespace tactics
