// Copyright 2018 - 2019 kvedder@umass.edu ikhatri@umass.edu
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
#include "state/soccer_robot.h"

#include <string>
#include <utility>

#include "obstacles/safety_margin.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "tactics/ball_interception.h"
#include "tactics/ball_placement.h"
#include "tactics/better_ball_placement.h"
#include "tactics/catch.h"
#include "tactics/coercive_attacker.h"
#include "tactics/custom_route.h"
#include "tactics/deflection.h"
#include "tactics/direct_free_kicker.h"
#include "tactics/dive_controller.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/forward_backward.h"
#include "tactics/goalie.h"
#include "tactics/guard_point.h"
#include "tactics/halt.h"
#include "tactics/indirect_free_kicker.h"
#include "tactics/joystick_controller.h"
#include "tactics/kick.h"
#include "tactics/kickoff_kicker.h"
#include "tactics/navigate_to_catch.h"
#include "tactics/navigate_to_intercept.h"
#include "tactics/ntoc_controller.h"
#include "tactics/pass_fail_primary_attacker.h"
#include "tactics/pd_controller.h"
#include "tactics/penalty_kicker.h"
#include "tactics/penalty_recieve.h"
#include "tactics/primary_attacker.h"
#include "tactics/simple_attacker.h"
#include "tactics/primary_defender.h"
#include "tactics/random_points.h"
#include "tactics/receiver.h"
#include "tactics/safe_follow_ball.h"
#include "tactics/secondary_attacker.h"
#include "tactics/secondary_defender.h"
#include "tactics/setup_attacker.h"
#include "tactics/state_machine_example.h"
#include "tactics/state_machine_tactic.h"
#include "tactics/stox_pivot.h"
#include "tactics/tertiary_defender.h"
#include "tactics/test_attacker.h"
#include "tactics/test_catch.h"
#include "tactics/test_defender.h"
#include "tactics/test_dive_controller.h"
#include "tactics/test_ntoc.h"
#include "tactics/test_passback.h"
#include "tactics/test_passing.h"
#include "tactics/three_kick.h"
#include "tactics/triangle.h"
#include "tactics/triangle_id_dependent.h"
#include "util/array_util.h"

using obstacle::SafetyMargin;
using std::unique_ptr;
using tactics::Tactic;
using tactics::TacticIndex;

namespace state {

// // Move constructor.
// SoccerRobot::SoccerRobot(SoccerRobot&& other)
//     : world_state_(other.world_state_),
//       ssl_vision_id_(other.ssl_vision_id_),
//       current_tactic_(other.current_tactic_),
//       previous_tactic_(other.previous_tactic_),
//       our_robot_index(other.our_robot_index),
//       soccer_state_(other.soccer_state_),
//       enabled(other.enabled),
//       shared_state(other.shared_state),
//       bot_logger(logger::Logger()) {
//   for (auto& tactic : other.tactic_list_) {
//     tactic_list_.emplace_back(std::move(tactic));
//   }
// }

// SoccerRobot& SoccerRobot::operator=(SoccerRobot&& other) {
//   ssl_vision_id_ = std::move(other.ssl_vision_id_);
//   current_tactic_ = std::move(other.current_tactic_);
//   previous_tactic_ = std::move(other.previous_tactic_);
//   our_robot_index = std::move(other.our_robot_index);
//   soccer_state_ = std::move(other.soccer_state_);
//   enabled = std::move(other.enabled);
//   shared_state = std::move(other.shared_state);
//   bot_logger = std::move(other.bot_logger);
//   this->tactic_list_.clear();
//   for (auto& tactic : other.tactic_list_) {
//     tactic_list_.emplace_back(std::move(tactic));
//   }
//   return *this;
// }

SoccerRobot::SoccerRobot(const WorldState& world_state,
                         const TacticIndex default_tactic,
                         SoccerState* soccer_state,
                         SharedState* shared_state,
                         const OurRobotIndex our_robot_index)
    : world_state_(world_state),
      ssl_vision_id_(0),
      current_tactic_(default_tactic),
      previous_tactic_(default_tactic),
      our_robot_index_(our_robot_index),
      soccer_state_(soccer_state),
      previous_command_(our_robot_index, ssl_vision_id_),
      enabled_(false),
      tactic_list_(),
      shared_state(shared_state),
      bot_logger() {
  for (auto& e : tactic_list_) {
    e = std::unique_ptr<Tactic>();
  }
}

SoccerRobot::SoccerRobot(const SoccerRobot& other)
    : world_state_(other.world_state_),
      ssl_vision_id_(other.ssl_vision_id_),
      current_tactic_(other.current_tactic_),
      previous_tactic_(other.previous_tactic_),
      our_robot_index_(other.our_robot_index_),
      soccer_state_(other.soccer_state_),
      previous_command_(other.previous_command_),
      enabled_(other.enabled_),
      shared_state(other.shared_state),
      bot_logger(other.bot_logger) {}

SoccerRobot& SoccerRobot::operator=(const SoccerRobot& other) {
  ssl_vision_id_ = other.ssl_vision_id_;
  current_tactic_ = other.current_tactic_;
  previous_tactic_ = other.previous_tactic_;
  soccer_state_ = other.soccer_state_;
  previous_command_ = other.previous_command_;
  enabled_ = other.enabled_;
  shared_state = other.shared_state;
  bot_logger = other.bot_logger;
  // Needed to wipe the state of the tactic so that it doesn't get
  // confused.
  tactic_list_[previous_tactic_]->Reset();
  tactic_list_[current_tactic_]->Reset();
  return *this;
}

SoccerRobot::~SoccerRobot() {}

bool SoccerRobot::operator<(const SoccerRobot& other) {
  if (enabled_ && !other.enabled_) {
    return true;
  } else if (!enabled_ && other.enabled_) {
    return false;
  } else {
    return ssl_vision_id_ < other.ssl_vision_id_;
  }
}

void SoccerRobot::SetRobotData(SSLVisionId ssl_vision_id,
                               OurRobotIndex our_robot_index) {
  ssl_vision_id_ = ssl_vision_id;
  our_robot_index_ = our_robot_index;
  enabled_ = true;
  //   LOG(WARNING) << "Enabling ssl_vision_id " << ssl_vision_id
  //   << " at index
  //   "
  //                << our_robot_index;
}

void SoccerRobot::ClearRobotData() {
  ssl_vision_id_ = 0;
  our_robot_index_ = 0;
  enabled_ = false;
}

void SoccerRobot::SetGoal(pose_2d::Pose2Df goal_pose) const {
  tactic_list_[current_tactic_]->SetGoal(goal_pose);
}

const pose_2d::Pose2Df SoccerRobot::GetGoal() const {
  pose_2d::Pose2Df goal = tactic_list_[current_tactic_]->GetGoal();
  return goal;
}

const float SoccerRobot::GetCost() const {
  const float cost = tactic_list_[current_tactic_]->GetCost();
  return cost;
}

void SoccerRobot::RunTactic() {
  if (current_tactic_ != previous_tactic_) {
    tactic_list_[previous_tactic_]->Reset();
  }

  //   LOG(WARNING) << "Tactic list size: " << tactic_list_.size();
  //   LOG(WARNING) << "Current Tactic: " << current_tactic_;
  tactic_list_[current_tactic_]->Run();

  previous_tactic_ = current_tactic_;
  previous_command_ = *(
      soccer_state_->GetMutableSharedState()->GetSharedState(our_robot_index_));
}

tactics::TacticIndex SoccerRobot::GetTacticByName(
    const std::string& name) const {
  for (size_t t = 0; t < tactic_list_.size(); ++t) {
    if (name == tactic_list_[t]->Name()) {
      return static_cast<tactics::TacticIndex>(t);
    }
  }
  LOG(WARNING) << "Unable to find tactic named " << name << std::endl;
  return tactics::TacticIndex::HALT;
}

#define ADD_TACTIC(index_name, cstr) \
  tactic_list_[TacticIndex::index_name] = unique_ptr<Tactic>(cstr);

void SoccerRobot::InitTactics() {
  NP_CHECK(tactic_list_.size() == tactics::TacticIndex::TACTIC_COUNT);

  ADD_TACTIC(DEFLECTION,
             new tactics::Deflection("Deflection",
                                     world_state_,
                                     &tactic_list_,
                                     shared_state,
                                     our_robot_index_,
                                     soccer_state_));

  ADD_TACTIC(FORWARD_BACKWARD,
             new tactics::ForwardBackward(world_state_,
                                          &tactic_list_,
                                          shared_state,
                                          our_robot_index_,
                                          soccer_state_));

  ADD_TACTIC(RANDOM_POINTS,
             new tactics::RandomPoints(world_state_,
                                       &tactic_list_,
                                       shared_state,
                                       our_robot_index_,
                                       soccer_state_));

  ADD_TACTIC(TRIANGLE,
             new tactics::Triangle(world_state_,
                                   &tactic_list_,
                                   shared_state,
                                   our_robot_index_,
                                   soccer_state_));

  ADD_TACTIC(CUSTOM_ROUTE,
             new tactics::CustomRoute(world_state_,
                                      &tactic_list_,
                                      shared_state,
                                      our_robot_index_,
                                      soccer_state_));

  ADD_TACTIC(TRIANGLE_ID_DEPENDENT,
             new tactics::TriangleIdDependent(world_state_,
                                              &tactic_list_,
                                              shared_state,
                                              our_robot_index_,
                                              soccer_state_));

  ADD_TACTIC(JOYSTICK_CONTROLLER,
             new tactics::JoystickController(world_state_,
                                             &tactic_list_,
                                             shared_state,
                                             our_robot_index_,
                                             soccer_state_));

  ADD_TACTIC(GOALIE,
             new tactics::Goalie(world_state_,
                                 &tactic_list_,
                                 shared_state,
                                 our_robot_index_,
                                 soccer_state_));

  ADD_TACTIC(PRIMARY_DEFENDER,
             new tactics::PrimaryDefender(world_state_,
                                          &tactic_list_,
                                          shared_state,
                                          our_robot_index_,
                                          soccer_state_));

  ADD_TACTIC(SECONDARY_DEFENDER,
             new tactics::SecondaryDefender(world_state_,
                                            &tactic_list_,
                                            shared_state,
                                            our_robot_index_,
                                            soccer_state_));

  ADD_TACTIC(TERTIARY_DEFENDER,
             new tactics::TertiaryDefender(world_state_,
                                           &tactic_list_,
                                           shared_state,
                                           our_robot_index_,
                                           soccer_state_));

  ADD_TACTIC(GUARD_POINT,
             new tactics::GuardPoint(world_state_,
                                     &tactic_list_,
                                     shared_state,
                                     our_robot_index_,
                                     soccer_state_));

  ADD_TACTIC(STOX_PIVOT,
             new tactics::STOXPivot(world_state_,
                                    &tactic_list_,
                                    shared_state,
                                    our_robot_index_,
                                    soccer_state_));

  ADD_TACTIC(NTOC,
             new tactics::NTOC_Controller(world_state_,
                                          &tactic_list_,
                                          shared_state,
                                          our_robot_index_,
                                          soccer_state_));

  ADD_TACTIC(TEST_NTOC,
             new tactics::TestNTOC(world_state_,
                                   &tactic_list_,
                                   shared_state,
                                   our_robot_index_,
                                   soccer_state_));

  ADD_TACTIC(HALT,
             new tactics::Halt(world_state_,
                               &tactic_list_,
                               shared_state,
                               our_robot_index_,
                               soccer_state_));

  ADD_TACTIC(KICKOFF,
             new tactics::KickoffKicker("KickoffKicker",
                                        world_state_,
                                        &tactic_list_,
                                        shared_state,
                                        our_robot_index_,
                                        soccer_state_));

  ADD_TACTIC(PENALTY_KICK,
             new tactics::PenaltyKicker(world_state_,
                                        &tactic_list_,
                                        shared_state,
                                        our_robot_index_,
                                        soccer_state_));

  ADD_TACTIC(PENALTY_RECIEVE,
             new tactics::PenaltyRecieve(world_state_,
                                         &tactic_list_,
                                         shared_state,
                                         our_robot_index_,
                                         soccer_state_));

  ADD_TACTIC(DIRECT_FREE_KICKER,
             new tactics::DirectFreeKicker("DirectFreeKicker",
                                           world_state_,
                                           &tactic_list_,
                                           shared_state,
                                           our_robot_index_,
                                           soccer_state_));

  ADD_TACTIC(INDIRECT_FREE_KICKER,
             new tactics::IndirectFreeKicker("IndirectFreeKicker",
                                             world_state_,
                                             &tactic_list_,
                                             shared_state,
                                             our_robot_index_,
                                             soccer_state_));

  ADD_TACTIC(BALL_INTERCEPTION,
             new tactics::InterceptionController(world_state_,
                                                 &tactic_list_,
                                                 shared_state,
                                                 our_robot_index_,
                                                 soccer_state_));

  ADD_TACTIC(NAVIGATE_TO_INTERCEPTION,
             new tactics::NavigateToIntercept(world_state_,
                                              &tactic_list_,
                                              shared_state,
                                              our_robot_index_,
                                              soccer_state_));

  ADD_TACTIC(NAVIGATE_TO_CATCH,
             new tactics::NavigateToCatch(world_state_,
                                          &tactic_list_,
                                          shared_state,
                                          our_robot_index_,
                                          soccer_state_));

  ADD_TACTIC(KICK,
             new tactics::Kick("Kick",
                               world_state_,
                               &tactic_list_,
                               shared_state,
                               our_robot_index_,
                               soccer_state_));

  ADD_TACTIC(THREE_KICK,
             new tactics::ThreeKick("ThreeKick",
                                    world_state_,
                                    &tactic_list_,
                                    shared_state,
                                    our_robot_index_,
                                    soccer_state_));

  ADD_TACTIC(CATCH,
             new tactics::Catch(world_state_,
                                &tactic_list_,
                                shared_state,
                                our_robot_index_,
                                soccer_state_));

  ADD_TACTIC(TEST_CATCH,
             new tactics::TestCatch(world_state_,
                                    &tactic_list_,
                                    shared_state,
                                    our_robot_index_,
                                    soccer_state_));

  ADD_TACTIC(SAFE_BALL_FOLLOW,
             new tactics::SafeFollowBall(world_state_,
                                         &tactic_list_,
                                         shared_state,
                                         our_robot_index_,
                                         soccer_state_));

  ADD_TACTIC(SECONDARY_ATTACKER,
             new tactics::SecondaryAttacker(world_state_,
                                            &tactic_list_,
                                            shared_state,
                                            our_robot_index_,
                                            soccer_state_));

  ADD_TACTIC(DIVE_CONTROLLER,
             new tactics::DiveController(world_state_,
                                         &tactic_list_,
                                         shared_state,
                                         our_robot_index_,
                                         soccer_state_));

  ADD_TACTIC(TEST_DIVE_CONTROLLER,
             new tactics::TestDiveController(world_state_,
                                             &tactic_list_,
                                             shared_state,
                                             our_robot_index_,
                                             soccer_state_));

  ADD_TACTIC(PRIMARY_ATTACKER,
             new tactics::PrimaryAttacker("PrimaryAttacker",
                                          world_state_,
                                          &tactic_list_,
                                          shared_state,
                                          our_robot_index_,
                                          soccer_state_));

  ADD_TACTIC(SIMPLE_ATTACKER,
             new tactics::SimpleAttacker("SimpleAttacker",
                                          world_state_,
                                          &tactic_list_,
                                          shared_state,
                                          our_robot_index_,
                                          soccer_state_));

  ADD_TACTIC(PASS_FAIL_PRIMARY_ATTACKER,
             new tactics::PassFailPrimaryAttacker("PassFailPrimaryAttacker",
                                                  world_state_,
                                                  &tactic_list_,
                                                  shared_state,
                                                  our_robot_index_,
                                                  soccer_state_));

  ADD_TACTIC(STATE_MACHINE_EXAMPLE,
             new tactics::StateMachineExample("StateMachineExample",
                                              world_state_,
                                              &tactic_list_,
                                              shared_state,
                                              our_robot_index_,
                                              soccer_state_));

  ADD_TACTIC(BALL_PLACEMENT,
             new tactics::BallPlacement("BallPlacement",
                                        world_state_,
                                        &tactic_list_,
                                        shared_state,
                                        our_robot_index_,
                                        soccer_state_));

  ADD_TACTIC(TEST_PASSING,
             new tactics::TestPassing(world_state_,
                                      &tactic_list_,
                                      shared_state,
                                      our_robot_index_,
                                      soccer_state_));

  ADD_TACTIC(RECEIVER,
             new tactics::Receiver(world_state_,
                                   &tactic_list_,
                                   shared_state,
                                   our_robot_index_,
                                   soccer_state_));

  ADD_TACTIC(TEST_PASSBACK,
             new tactics::TestPassback(world_state_,
                                       &tactic_list_,
                                       shared_state,
                                       our_robot_index_,
                                       soccer_state_));

  ADD_TACTIC(SETUP_ATTACKER,
             new tactics::SetupAttacker("Setup Attacker",
                                        world_state_,
                                        &tactic_list_,
                                        shared_state,
                                        our_robot_index_,
                                        soccer_state_));

  ADD_TACTIC(PD_CONTROLLER,
             new tactics::PDController(world_state_,
                                       &tactic_list_,
                                       shared_state,
                                       our_robot_index_,
                                       soccer_state_));

  ADD_TACTIC(COERCIVE_ATTACKER,
             new tactics::CoerciveAttacker("Coercive Attacker",
                                           world_state_,
                                           &tactic_list_,
                                           shared_state,
                                           our_robot_index_,
                                           soccer_state_));

  ADD_TACTIC(EIGHT_GRID,
             new tactics::EightGridNavigation(world_state_,
                                              &tactic_list_,
                                              shared_state,
                                              our_robot_index_,
                                              soccer_state_,
                                              false));

  ADD_TACTIC(BETTER_BALL_PLACEMENT,
             new tactics::BetterBallPlacement(world_state_,
                                              &tactic_list_,
                                              shared_state,
                                              our_robot_index_,
                                              soccer_state_));

  ADD_TACTIC(TEST_DEFENDER,
             new tactics::TestDefender(world_state_,
                                       &tactic_list_,
                                       shared_state,
                                       our_robot_index_,
                                       soccer_state_));
  ADD_TACTIC(TEST_ATTACKER,
             new tactics::TestAttacker(world_state_,
                                       &tactic_list_,
                                       shared_state,
                                       our_robot_index_,
                                       soccer_state_));

  // Init all tactics.
  for (auto& tactic : tactic_list_) {
    // Tactic will be null if there is no initialization for it
    // above.
    if (tactic.get() != nullptr) {
      tactic->Init();
    }
  }
}

}  // namespace state
