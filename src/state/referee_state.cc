// Copyright 2017-2018 jaholtz@cs.umass.edu
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

#include "state/referee_state.h"

using MinuteBotsProto::SSL_Referee;
using team::Team;
using Eigen::Vector2f;

namespace state {

RefereeState::RefereeState(const Team& team)
    : team_(team),
      has_message_(false) {
}

RefereeState::~RefereeState() {
}

void RefereeState::SetRefereeMessage(SSL_Referee referee) {
  referee_message_ = referee;
  has_message_ = true;
}

bool RefereeState::HasMessage() {
  return has_message_;
}

SSL_Referee RefereeState::GetRefereeMessage() {
  return referee_message_;
}

const SSL_Referee::Stage RefereeState::GetGameStage() const {
  return referee_message_.stage();
}

const bool RefereeState::IsStage(SSL_Referee::Stage  stage) const {
  if (referee_message_.stage() == stage) {
    return true;
  }
  return false;
}

const bool RefereeState::IsNormalFirstHalfPre() const {
  return IsStage(referee_message_.NORMAL_FIRST_HALF_PRE);
}

const bool RefereeState::IsNormalFirstHalf() const {
  return IsStage(referee_message_.NORMAL_FIRST_HALF);
}

const bool RefereeState::IsNormalHalftime() const {
  return IsStage(referee_message_.NORMAL_HALF_TIME);
}

const bool RefereeState::IsNormalSecondHalfPre() const {
  return IsStage(referee_message_.NORMAL_SECOND_HALF_PRE);
}

const bool RefereeState::IsNormalSecondHalf() const {
  return IsStage(referee_message_.NORMAL_SECOND_HALF);
}

const bool RefereeState::IsExtraTimeBreak() const {
  return IsStage(referee_message_.EXTRA_TIME_BREAK);
}

const bool RefereeState::IsExtraFirstHalfPre() const {
  return IsStage(referee_message_.EXTRA_FIRST_HALF_PRE);
}

const bool RefereeState::IsExtraFirstHalf() const {
  return IsStage(referee_message_.EXTRA_FIRST_HALF);
}

const bool RefereeState::IsExtraHalfTime() const {
  return IsStage(referee_message_.EXTRA_HALF_TIME);
}

const bool RefereeState::IsExtraSecondHalfPre() const {
  return IsStage(referee_message_.EXTRA_SECOND_HALF_PRE);
}

const bool RefereeState::IsExtraSecondHalf() const {
  return IsStage(referee_message_.EXTRA_SECOND_HALF);
}

const bool RefereeState::IsPenaltyShootoutBreak() const {
  return IsStage(referee_message_.PENALTY_SHOOTOUT_BREAK);
}

const bool RefereeState::IsPenaltyShootout() const {
  return IsStage(referee_message_.PENALTY_SHOOTOUT);
}

const bool RefereeState::IsPostGame() const {
  return IsStage(referee_message_.POST_GAME);
}

const int RefereeState::GetGameStageTime() const {
  return referee_message_.stage_time_left();
}

const SSL_Referee::Command RefereeState::GetRefCommand() const {
  return referee_message_.command();
}

const bool RefereeState::IsCommand(const SSL_Referee::Command& command) const {
  if (referee_message_.command() == command) {
    return true;
  }
  return false;
}

const bool RefereeState::IsCommandConst(const SSL_Referee::Command& command)
    const {
  if (referee_message_.command() == command) {
    return true;
  }
  return false;
}

const bool RefereeState::IsHalt()  const {
  return IsCommand(referee_message_.HALT);
}

const bool RefereeState::IsStop()  const {
  return IsCommand(referee_message_.STOP);
}

const bool RefereeState::IsNormalStart() const {
  return IsCommandConst(referee_message_.NORMAL_START);
}

const bool RefereeState::IsForceStart()  const {
  return IsCommand(referee_message_.FORCE_START);
}

const bool RefereeState::IsPrepareKickoffUs()  const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.PREPARE_KICKOFF_YELLOW);
  } else {
    return IsCommand(referee_message_.PREPARE_KICKOFF_BLUE);
  }
  return false;
}

const bool RefereeState::IsPrepareKickoffThem()  const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.PREPARE_KICKOFF_BLUE);
  } else {
    return IsCommand(referee_message_.PREPARE_KICKOFF_YELLOW);
  }
  return false;
}

const bool RefereeState::IsPreparePenaltyUs()  const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.PREPARE_PENALTY_YELLOW);
  } else {
    return IsCommand(referee_message_.PREPARE_PENALTY_BLUE);
  }
  return false;
}

const bool RefereeState::IsPreparePenaltyThem() const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.PREPARE_PENALTY_BLUE);
  } else {
    return IsCommand(referee_message_.PREPARE_PENALTY_YELLOW);
  }
  return false;
}

const bool RefereeState::IsDirectFreeUs() const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.DIRECT_FREE_YELLOW);
  } else {
    return IsCommand(referee_message_.DIRECT_FREE_BLUE);
  }
  return false;
}

const bool RefereeState::IsDirectFreeThem() const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.DIRECT_FREE_BLUE);
  } else {
    return IsCommand(referee_message_.DIRECT_FREE_YELLOW);
  }
  return false;
}

const bool RefereeState::IsIndirectFreeUs() const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.INDIRECT_FREE_YELLOW);
  } else {
    return IsCommand(referee_message_.INDIRECT_FREE_BLUE);
  }
  return false;
}

const bool RefereeState::IsIndirectFreeThem() const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.INDIRECT_FREE_BLUE);
  } else {
    return IsCommand(referee_message_.INDIRECT_FREE_YELLOW);
  }
  return false;
}

const bool RefereeState::IsTimeoutUs() const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.TIMEOUT_YELLOW);
  } else {
    return IsCommand(referee_message_.TIMEOUT_BLUE);
  }
  return false;
}

const bool RefereeState::IsTimeoutThem() const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.TIMEOUT_BLUE);
  } else {
    return IsCommand(referee_message_.TIMEOUT_YELLOW);
  }
  return false;
}

const bool RefereeState::IsGoalUs() const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.GOAL_YELLOW);
  } else {
    return IsCommand(referee_message_.GOAL_BLUE);
  }
  return false;
}

const bool RefereeState::IsGoalThem() const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.GOAL_BLUE);
  } else {
    return IsCommand(referee_message_.GOAL_YELLOW);
  }
  return false;
}

const bool RefereeState::IsBallPlacementUs() const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.BALL_PLACEMENT_YELLOW);
  } else {
    return IsCommand(referee_message_.BALL_PLACEMENT_BLUE);
  }
  return false;
}

const bool RefereeState::IsBallPlacementThem() const {
  if (team_ == team::Team::YELLOW) {
    return IsCommand(referee_message_.BALL_PLACEMENT_BLUE);
  } else {
    return IsCommand(referee_message_.BALL_PLACEMENT_YELLOW);
  }
  return false;
}

const int RefereeState::GetNumCommandsIssued() const {
  return referee_message_.command_counter();
}

const int RefereeState::GetCommandTimeStamp() const {
  return referee_message_.command_timestamp();
}

const int RefereeState::GetOurScore() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.yellow().score();
  } else {
    return referee_message_.blue().score();
  }
  return 0;
}

const int RefereeState::GetTheirScore() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.blue().score();
  } else {
    return referee_message_.yellow().score();
  }
  return 0;
}

const int RefereeState::GetOurRedCards() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.yellow().red_cards();
  } else {
    return referee_message_.blue().red_cards();
  }
  return 0;
}

const int RefereeState::GetTheirRedCards() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.blue().red_cards();
  } else {
    return referee_message_.yellow().red_cards();
  }
  return 0;
}

const int RefereeState::GetOurYellowCards() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.yellow().yellow_cards();
  } else {
    return referee_message_.blue().yellow_cards();
  }
  return 0;
}

const int RefereeState::GetTheirYellowCards() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.blue().yellow_cards();
  } else {
    return referee_message_.yellow().yellow_cards();
  }
  return 0;
}

const int RefereeState::GetOurActiveYellowCards() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.yellow().yellow_card_times_size();
  } else {
    return referee_message_.blue().yellow_card_times_size();
  }
  return 0;
}

const int RefereeState::GetTheirActiveYellowCards() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.blue().yellow_card_times_size();
  } else {
    return referee_message_.yellow().yellow_card_times_size();
  }
  return 0;
}

const int RefereeState::GetOurYellowCardTime(int card) const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.yellow().yellow_card_times(card);
  } else {
    return referee_message_.blue().yellow_card_times(card);
  }
  return 0;
}

const int RefereeState::GetTheirYellowCardTime(int card) const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.blue().yellow_card_times(card);
  } else {
    return referee_message_.yellow().yellow_card_times(card);
  }
  return 0;
}

const int RefereeState::GetOurTimeouts() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.yellow().timeouts();
  } else {
    return referee_message_.blue().timeouts();
  }
  return 0;
}

const int RefereeState::GetTheirTimeouts() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.blue().timeouts();
  } else {
    return referee_message_.yellow().timeouts();
  }
  return 0;
}

const int RefereeState::GetOurTimeoutMicroSeconds() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.yellow().timeout_time();
  } else {
    return referee_message_.blue().timeout_time();
  }
  return 0;
}

const int RefereeState::GetTheirTimeoutMicroSeconds() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.blue().timeout_time();
  } else {
    return referee_message_.yellow().timeout_time();
  }
  return 0;
}

const int RefereeState::GetOurGoaliePattern() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.yellow().goalie();
  } else {
    return referee_message_.blue().goalie();
  }
  return 0;
}

const int RefereeState::GetTheirGoaliePattern() const {
  if (team_ == team::Team::YELLOW) {
    return referee_message_.blue().goalie();
  } else {
    return referee_message_.yellow().goalie();
  }
  return 0;
}

bool RefereeState::HasDesignatedPosition() const {
  return referee_message_.has_designated_position();
}

const Vector2f RefereeState::GetDesignatedPostion() const {
  if (referee_message_.has_designated_position()) {
    const Vector2f point = {referee_message_.designated_position().x(),
                            referee_message_.designated_position().y()};
    return point;
  }
  const Vector2f point = {0, 0};
  return point;
}



}  // namespace state
