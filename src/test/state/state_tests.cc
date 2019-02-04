// Copyright 2017-2018 kvedder@umass.edu
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
#include "state/position_velocity_state.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
#include "state/team.h"
#include "soccer_logging.pb.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using state::PositionVelocityState;

namespace state {

TEST(StateTest, TeamComparison) {
  {
    const team::Team t1 = team::Team::BLUE;
    const MinuteBotsProto::RobotState_Team t2 =
        MinuteBotsProto::RobotState_Team_TEAM_BLUE;
    ASSERT_TRUE(t1 == t2);
  }
  {
    const team::Team t1 = team::Team::YELLOW;
    const MinuteBotsProto::RobotState_Team t2 =
        MinuteBotsProto::RobotState_Team_TEAM_YELLOW;
    ASSERT_TRUE(t1 == t2);
  }
  {
    const team::Team t1 = team::Team::BLUE;
    const MinuteBotsProto::RobotState_Team t2 =
        MinuteBotsProto::RobotState_Team_TEAM_YELLOW;
    ASSERT_FALSE(t1 == t2);
  }
  {
    const team::Team t1 = team::Team::YELLOW;
    const MinuteBotsProto::RobotState_Team t2 =
        MinuteBotsProto::RobotState_Team_TEAM_BLUE;
    ASSERT_FALSE(t1 == t2);
  }
}

TEST(StateTest, HelloWorld) { ASSERT_TRUE(true); }

}  // namespace state
