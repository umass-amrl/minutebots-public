// Copyright 2018 kvedder@umass.edu
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

#include "navigation/production/collision_grid.h"

#include <limits>
#include <utility>
#include <vector>

namespace navigation {
namespace production {
namespace eight_grid {

CollisionGrid::CollisionGrid()
    : distance_between_verticies_(std::numeric_limits<float>::max()) {}

CollisionGrid::CollisionGrid(const float distance_between_verticies)
    : distance_between_verticies_(distance_between_verticies) {}

std::bitset<kNumObstacles> AllIndexButGivenUnset(const size_t index) {
  auto invalidated_index = std::bitset<kNumObstacles>().reset();
  invalidated_index.set(index);
  return invalidated_index;
}

std::pair<int, int> CollisionGrid::GetGridIndexBounds(
    const int bounding_box_min, const int bounding_box_max) {
  const int min_index = static_cast<int>(std::ceil(
      static_cast<float>(bounding_box_min) / distance_between_verticies_));
  const int max_index = static_cast<int>(std::floor(
      static_cast<float>(bounding_box_max) / distance_between_verticies_));
  return {min_index, max_index};
}

void CollisionGrid::InvalidateObstacle(
    const obstacle::Obstacle* obstacle, const obstacle::SafetyMargin& margin,
    const std::bitset<kNumObstacles>& invalidated_bitset) {
  constexpr bool kDebug = false;
  if (kDebug) {
    LOG(INFO) << "Obstacle position: " << obstacle->GetPose().translation.x()
              << ", " << obstacle->GetPose().translation.y();
  }
  const auto& bb = obstacle->GetExternalBoundingBox();
  if (kDebug) {
    LOG(INFO) << "x: " << bb.x_ << " y: " << bb.y_ << " width: " << bb.width_
              << " height: " << bb.height_;
  }
  const int margin_amount =
      static_cast<int>(std::ceil(margin.GetMargin(obstacle->GetType())));
  const auto min_max_x =
      GetGridIndexBounds(bb.x_ - bb.half_width_ - margin_amount,
                         bb.x_ + bb.half_width_ + margin_amount);
  const auto min_max_y =
      GetGridIndexBounds(bb.y_ - bb.half_height_ - margin_amount,
                         bb.y_ + bb.half_height_ + margin_amount);
  for (int x = min_max_x.first; x <= min_max_x.second; ++x) {
    for (int y = min_max_y.first; y <= min_max_y.second; ++y) {
      auto result = colliding_verticies_.insert({{x, y}, invalidated_bitset});
      // Was not successfully inserted because there was already something in
      // the map.
      if (!result.second) {
        result.first->second |= invalidated_bitset;
      }

      if (!kProduction) {
        const auto find_result = colliding_verticies_.find({x, y});
        NP_CHECK(find_result != colliding_verticies_.end());
        NP_CHECK((find_result->second & invalidated_bitset).any());
      }
    }
  }
}

void CollisionGrid::Rebuild(const obstacle::ObstacleFlag& obstacles,
                            const obstacle::SafetyMargin& margin) {
  colliding_verticies_.clear();
  // Load of roughly 8500 elements, with a desired load factor of 0.75.
  constexpr size_t kReserveSize = 10625;
  colliding_verticies_.reserve(kReserveSize);
  for (auto it = obstacles.IndexedBegin(); it != obstacles.IndexedEnd(); ++it) {
    const std::pair<size_t, obstacle::Obstacle*> index_obstacle_pair = *it;
    const size_t& index = index_obstacle_pair.first;
    const obstacle::Obstacle* obstacle = index_obstacle_pair.second;
    const auto invalidated_bitset = AllIndexButGivenUnset(index);
    InvalidateObstacle(obstacle, margin, invalidated_bitset);
  }
}

void CollisionGrid::RebuildDynamic(const obstacle::SafetyMargin& margin) {
  const obstacle::ObstacleFlag obstacles =
      ~obstacle::ObstacleFlag::GetStaticObstacles();
  Rebuild(obstacles, margin);
}

void CollisionGrid::RebuildStatic(const obstacle::SafetyMargin& margin) {
  const obstacle::ObstacleFlag obstacles =
      obstacle::ObstacleFlag::GetStaticObstacles();
  Rebuild(obstacles, margin);
}

bool CollisionGrid::IsColliding(const GridVertex& vertex,
                                const obstacle::ObstacleFlag& obstacles) const {
  const auto find_result = colliding_verticies_.find(vertex);
  if (find_result == colliding_verticies_.end()) {
    return false;
  }
  return (find_result->second & obstacles.GetFlags()).any();
}

// void CollisionGrid::DrawVertices(logging::Logger* logger) {
//   for (const auto& pair : soccer_state_->GetStaticCollisionGrid()) {
//     robot_logger->AddCircle(pair.first, 5, 1, 0, 0, 1);
//   }
// }

void CollisionGrid::DrawVertices(
    logger::Logger* logger, const opengl_helpers::Color4f& color,
    const obstacle::ObstacleFlag& obstacle_flag) const {
  for (const auto& pair : colliding_verticies_) {
    if ((pair.second & obstacle_flag.GetFlags()).none()) {
      continue;
    }
    const Vector2f pos(pair.first.x() * distance_between_verticies_,
                       pair.first.y() * distance_between_verticies_);
    logger->AddCircle(pos, 2, color.r, color.g, color.b, color.a);
  }
}

std::vector<std::vector<bool>> CollisionGrid::GenerateDenseOccupancyGrid(
    const obstacle::ObstacleFlag& obstacles) const {
  std::vector<std::vector<bool>> grid;
  static const int x_min =
      -std::ceil(field_dimensions::kHalfFieldLength / kEightGridSquareSize);
  static const int x_max =
      std::ceil(field_dimensions::kHalfFieldLength / kEightGridSquareSize);
  static const int y_min =
      -std::ceil(field_dimensions::kHalfFieldWidth / kEightGridSquareSize);
  static const int y_max =
      std::ceil(field_dimensions::kHalfFieldWidth / kEightGridSquareSize);

  for (int x = x_min; x <= x_max; ++x) {
    grid.push_back({});
    for (int y = y_min; y <= y_max; ++y) {
      const GridVertex v(x, y);
      grid[grid.size() - 1].push_back(IsColliding(v, obstacles));
    }
  }

  return grid;
}

}  // namespace eight_grid
}  // namespace production
}  // namespace navigation
