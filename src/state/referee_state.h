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

#include "shared/common_includes.h"
#include "referee.pb.h"
#include "state/team.h"

#ifndef SRC_STATE_REFEREE_STATE_H_
#define SRC_STATE_REFEREE_STATE_H_

namespace state {

class RefereeState {
 public:
  explicit RefereeState(const team::Team& team);
  ~RefereeState();

  void SetRefereeMessage(MinuteBotsProto::SSL_Referee referee);
  bool HasMessage();
  MinuteBotsProto::SSL_Referee GetRefereeMessage();

  // return enum for game stage
  // GetGameStage()
  const MinuteBotsProto::SSL_Referee::Stage GetGameStage() const;

  // IsGameStage predicates to query specific game states
  const bool IsStage(MinuteBotsProto::SSL_Referee::Stage stage) const;
  const bool IsNormalFirstHalfPre() const;
  const bool IsNormalFirstHalf() const;
  const bool IsNormalHalftime() const;
  const bool IsNormalSecondHalfPre() const;
  const bool IsNormalSecondHalf() const;
  const bool IsExtraTimeBreak() const;
  const bool IsExtraFirstHalfPre() const;
  const bool IsExtraFirstHalf() const;
  const bool IsExtraHalfTime() const;
  const bool IsExtraSecondHalfPre() const;
  const bool IsExtraSecondHalf() const;
  const bool IsPenaltyShootoutBreak() const;
  const bool IsPenaltyShootout() const;
  const bool IsPostGame() const;

  // return int for time remaining in stage
  const int GetGameStageTime() const;

  // return enum for command (which are the fine states of the game)
  const MinuteBotsProto::SSL_Referee::Command GetRefCommand() const;

  // IsCommand predicates to query specific commands
  const bool IsCommand(const MinuteBotsProto::SSL_Referee::Command& command)
      const;
  const bool IsCommandConst(const MinuteBotsProto::SSL_Referee::Command&
      command) const;
  const bool IsHalt()  const;
  const bool IsStop() const;
  const bool IsNormalStart() const;
  const bool IsForceStart() const;
  const bool IsPrepareKickoffUs() const;
  const bool IsPrepareKickoffThem() const;
  const bool IsPreparePenaltyUs() const;
  const bool IsPreparePenaltyThem() const;
  const bool IsDirectFreeUs() const;
  const bool IsDirectFreeThem() const;
  const bool IsIndirectFreeUs() const;
  const bool IsIndirectFreeThem() const;
  const bool IsTimeoutUs() const;
  const bool IsTimeoutThem() const;
  const bool IsGoalUs() const;
  const bool IsGoalThem() const;
  const bool IsBallPlacementUs() const;
  const bool IsBallPlacementThem() const;

  // return int for commands issued.
  const int GetNumCommandsIssued() const;

  // return int for the timestamp the command was issued
  const int GetCommandTimeStamp() const;

  // return int for the score
  const int GetOurScore() const;
  const int GetTheirScore() const;

  // return int for number of red cards
  const int GetOurRedCards() const;
  const int GetTheirRedCards() const;

  // return int for number of yellow cards
  const int GetOurYellowCards() const;
  const int GetTheirYellowCards() const;
  const int GetOurActiveYellowCards() const;
  const int GetTheirActiveYellowCards() const;

  // return time remaining on a yellow_card
  const int GetOurYellowCardTime(int card) const;
  const int GetTheirYellowCardTime(int card) const;

  // return int for number of timeouts that can still be called
  const int GetOurTimeouts() const;
  const int GetTheirTimeouts() const;

  // return int for the number of microseconds of timeout left for the team.
  const int GetOurTimeoutMicroSeconds() const;
  const int GetTheirTimeoutMicroSeconds() const;

  const int GetOurGoaliePattern() const;
  const int GetTheirGoaliePattern() const;

  bool HasDesignatedPosition() const;
  // return vector2d for the designated point for the current action
  const Eigen::Vector2f GetDesignatedPostion() const;

 private:
  MinuteBotsProto::SSL_Referee referee_message_;
  team::Team team_;
  bool has_message_;
};
}  // namespace state
#endif  // SRC_STATE_REFEREE_STATE_H_
