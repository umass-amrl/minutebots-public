// Copyright 2016 - 2018 slane@cs.umass.edu
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

#ifndef SRC_OBSTACLES_OBSTACLE_FLAG_H_
#define SRC_OBSTACLES_OBSTACLE_FLAG_H_

#include <bitset>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "constants/constants.h"
#include "obstacles/obstacle.h"

// Forward declare WorldState.
namespace state {
class WorldState;
class SoccerState;
}  // namespace state

namespace obstacle {
// This class serves a dual purpose:
//
// - Instances of this indicate which obstacles to consider and provide an
// iterator to access each of the elements.
//
// - Static methods in this generate the proper formulation of the instances to
// be used as above.
class ObstacleFlag {
 private:
  class ObstacleFlagIter {
   public:
    ObstacleFlagIter(const ObstacleFlag& p_vec, const size_t pos);

    // The following three methods together form the basis of an iterator which
    // allows us to iterate over all of the selected obstacles.
    bool operator!=(const ObstacleFlagIter& other) const;
    const Obstacle* const operator*() const;
    const ObstacleFlagIter& operator++();

   private:
    size_t pos_;
    const ObstacleFlag& p_vec_;
  };

  class IndexedObstacleFlagIter {
   public:
    IndexedObstacleFlagIter(const ObstacleFlag& p_vec, const size_t pos);

    // The following three methods together form the basis of an iterator which
    // allows us to iterate over all of the selected obstacles.
    bool operator!=(const IndexedObstacleFlagIter& other) const;
    const std::pair<size_t, Obstacle*> operator*() const;
    const IndexedObstacleFlagIter& operator++();

   private:
    size_t pos_;
    const ObstacleFlag& p_vec_;
  };

  // Instance member's bitflags.
  std::bitset<kNumObstacles> flags_;

 public:
  ObstacleFlag() = default;
  ObstacleFlag(const ObstacleFlag& other) = default;
  ObstacleFlag(ObstacleFlag&& other) = default;
  explicit ObstacleFlag(const std::bitset<kNumObstacles>& flags);
  ~ObstacleFlag();

  std::string GetString() const;
  std::bitset<kNumObstacles> GetFlags() const;

  ObstacleFlagIter begin() const;
  ObstacleFlagIter end() const;

  IndexedObstacleFlagIter IndexedBegin() const;
  IndexedObstacleFlagIter IndexedEnd() const;

  // Convienience functions for combining multiple flags
  ObstacleFlag operator|(const ObstacleFlag& other) const;
  ObstacleFlag operator&(const ObstacleFlag& other) const;
  ObstacleFlag operator~() const;
  ObstacleFlag& operator=(const ObstacleFlag& other) = default;
  ObstacleFlag& operator=(ObstacleFlag&& other) = default;

  // Returns an obstacle flag that references all of the obstacles
  static ObstacleFlag GetAll(const state::WorldState& world_state,
                             const state::SoccerState& soccer_state,
                             OurRobotIndex calling_robot_index);

  // Returns an obstacle flag that references all obstacles except for one of
  // the robot on our team with the input ID.
  static ObstacleFlag GetAllExceptTeam(const state::WorldState& world_state,
                                       const state::SoccerState& soccer_state,
                                       OurRobotIndex calling_robot_index,
                                       OurRobotIndex excluded_robot_index);

  // Returns an obstacle flag that references all of the static obstacles
  static ObstacleFlag GetStaticObstacles();

  // Returns an obstacle flag that references a single robot from our team
  static ObstacleFlag GetOurRobot(const state::WorldState& world_state,
                                  OurRobotIndex our_robot_index);

  // Returns an obstacle flag that references a single robot from the other team
  static ObstacleFlag GetOpponentRobot(TheirRobotIndex their_robot_index);

  // Returns an obstacle flag that references all of the robots on both teams
  static ObstacleFlag GetAllRobots();

  // Returns an obstacle flag that references all of the robots on both teams
  // except for the opponent robot with the input ID.
  static ObstacleFlag GetAllRobotsExceptOpponent(
      const state::WorldState& world_state, TheirRobotIndex their_robot_index);

  // Returns an obstacle flag that references all of the robots on both teams
  // except for our robot with the input ID.
  static ObstacleFlag GetAllRobotsExceptTeam(OurRobotIndex our_robot_index);

  // Returns an obstacle flag that references all of the robots on our team
  static ObstacleFlag GetOurRobots();

  // Returns an obstacle flag that references all of the robots on our team
  // except for the robot with the input ID.
  static ObstacleFlag GetOurRobotsExcept(const state::WorldState& world_state,
                                         OurRobotIndex our_robot_index);

  // Returns an obstacle flag that references all of the robots on the other
  // team
  static ObstacleFlag GetOpponentRobots();

  // Returns an obstacle flag that references all of the robots on the other
  // team except for the robot with the input ID.
  static ObstacleFlag GetOpponentRobotsExcept(
      const state::WorldState& world_state, TheirRobotIndex their_robot_index);

  // Returns an obstacle flag that references the small ball.
  static ObstacleFlag GetAllBalls();

  // Returns an obstacle flag that references the small ball.
  static ObstacleFlag GetBall();

  // Returns an obstacle flag that references the medium ball.
  static ObstacleFlag GetMediumBall();

  // Returns an obstacle flag that references the medium ball.
  static ObstacleFlag GetLargeBall();

  // Returns an obstacle flag that references nothing.
  static ObstacleFlag GetEmpty();

  // Returns an obstacle flag that references everything.
  static ObstacleFlag GetFull();

  static ObstacleFlag GetRulesObstacles(const state::WorldState& world_state,
                                        const state::SoccerState& soccer_state,
                                        OurRobotIndex calling_robot_index);

  static ObstacleFlag GetDefenseAreas();

  static ObstacleFlag GetOurDefenseArea();

  static ObstacleFlag GetTheirDefenseArea();

  static ObstacleFlag GetKickoffOtherHalfObstacle();
};

}  // namespace obstacle

#endif  // SRC_OBSTACLES_OBSTACLE_FLAG_H_
