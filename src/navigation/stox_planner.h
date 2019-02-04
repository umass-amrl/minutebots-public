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

#ifndef SRC_NAVIGATION_STOX_PLANNER_H_
#define SRC_NAVIGATION_STOX_PLANNER_H_

#include <vector>
#include <utility>

#include "eigen3/Eigen/Core"
#include "logging/logger.h"
#include "math/poses_2d.h"
#include "navigation/navigation.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"

namespace navigation {
class StoxPlanner : public Navigation {
 public:
  void Init(const obstacle::ObstacleFlag& obstacles);

  void Init(const obstacle::ObstacleFlag& obstacles,
            const obstacle::SafetyMargin& safety_margin);

  void Update(const obstacle::ObstacleFlag& obstacles,
              const Eigen::Vector2f& current_position,
              const Eigen::Vector2f& goal_position,
              logger::Logger* local_logger);

  void Update(const obstacle::ObstacleFlag& obstacles,
              const obstacle::SafetyMargin& safety_margin,
              const Eigen::Vector2f& current_position,
              const Eigen::Vector2f& goal_position,
              logger::Logger* local_logger);

  // Returns false if a new path was not found or if it entered one-shot mode
  // Returns true otherwise
  std::pair<bool, std::vector<Eigen::Vector2f>> Plan(
      logger::Logger* local_logger);

  const std::vector<Eigen::Vector2f>& GetPlan() const;

 private:
  bool CollisionFreePath(const obstacle::ObstacleFlag& obstacles,
                         const Eigen::Vector2f& position1,
                         const Eigen::Vector2f& position2,
                         pose_2d::Pose2Df* closest_obstacle_center,
                         float* closest_obstacle_radius,
                         Eigen::Vector2f* closest_point) const;

  // Recursive function that computes a path from the last point in the partial
  // trajectory to the specified goal point, terminates at a given search depth
  // Returns the length of the partial trajectory
  float FastPathPlanning(const obstacle::ObstacleFlag& obstacles,
                         const Eigen::Vector2f& start,
                         const Eigen::Vector2f& goal, int search_depth,
                         std::vector<Eigen::Vector2f>* return_trajectory,
                         logger::Logger* local_logger);

  bool SearchPoint(const obstacle::ObstacleFlag& obstacles,
                   const Eigen::Vector2f& start, const Eigen::Vector2f& goal,
                   const Eigen::Vector2f& obstacle_center,
                   const float obstacle_radius, int angle_sign,
                   Eigen::Vector2f* return_point, logger::Logger* local_logger);

  void JoinSegments(const std::vector<Eigen::Vector2f>& start_trajectory,
                    const std::vector<Eigen::Vector2f>& end_trajectory,
                    std::vector<Eigen::Vector2f>* full_trajectory);

  float CalculatePathLength(const std::vector<Eigen::Vector2f>& path) const;

  // Check an input point against all obstacles
  // Returns true if there is a Collision
  // Returns false otherwise
  bool PointCollidesWithObstacle(const obstacle::ObstacleFlag& obstacles,
                                 const Eigen::Vector2f& point) const;

  Eigen::Vector2f current_position_;
  Eigen::Vector2f start_position_;
  Eigen::Vector2f goal_position_;
  std::vector<Eigen::Vector2f> current_path_;
  obstacle::SafetyMargin safety_margin_;
  // obstacle::ObstacleFlag obstacles_;

  bool started_in_collision_;
};
}  // namespace navigation

#endif  // SRC_NAVIGATION_STOX_PLANNER_H_
