// Copyright 2017 kvedder@umass.edu
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

#include "navigation/eight_grid.h"

#include <unistd.h>
#include <eigen3/Eigen/Core>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <random>
#include <vector>

STANDARD_USINGS;
using Eigen::Vector2i;

namespace navigation {
namespace eight_grid {
static const Eigen::Vector2i kStartNodePredecessor(
    std::numeric_limits<int>::max() - 1, std::numeric_limits<int>::max() - 1);

EightGrid::EightGrid(const obstacle::ObstacleFlag& obstacles,
                     const obstacle::SafetyMargin& safety_margin,
                     const float distance_between_vertices)
    : Navigation(obstacles, safety_margin),
      distance_between_vertices_(distance_between_vertices),
      total_vertex_count_((static_cast<int>(field_dimensions::kFieldLength /
                                            distance_between_vertices_) +
                           1) *
                          (static_cast<int>(field_dimensions::kFieldWidth /
                                            distance_between_vertices_) +
                           1)),
      closed_vertices_(total_vertex_count_, false),
      path_map_(total_vertex_count_),
      static_invalid_vertices_(CalculateInvalidStaticVertices()) {
  if (!kProduction) {
    if (distance_between_vertices <= 0) {
      LOG(FATAL) << "Distance between verticies cannot be <= 0!. Actual value: "
                 << distance_between_vertices;
    }
  }
}

void EightGrid::Update(const obstacle::ObstacleFlag& obstacles,
                       const obstacle::SafetyMargin& safety_margin,
                       const Vector2f& current_pose, const Vector2f& goal_pose,
                       logger::Logger* logger) {
  navigation::Navigation::Update(obstacles, safety_margin, current_pose,
                                 goal_pose, logger);
  dynamic_invalid_vertices_ = CalculateInvalidVertices(logger);
}

float EightGrid::Heuristic(const Vector2i& position, const Vector2i& goal) {
  const int x_diff = std::abs(position.x() - goal.x());
  const int y_diff = std::abs(position.y() - goal.y());

  int smaller, larger;
  if (x_diff < y_diff) {
    smaller = x_diff;
    larger = y_diff;
  } else {
    smaller = y_diff;
    larger = x_diff;
  }

  return distance_between_vertices_ * (larger - smaller) +
         smaller * distance_between_vertices_ * kSqrtTwo;
}

std::array<std::pair<Vector2i, float>, 8> EightGrid::GetNeighbors(
    const Vector2i& vertex) const {
  return {{{Vector2i(vertex.x() + 1, vertex.y() - 1),
            distance_between_vertices_ * kSqrtTwo},
           {Vector2i(vertex.x() + 1, vertex.y() + 1),
            distance_between_vertices_ * kSqrtTwo},
           {Vector2i(vertex.x() + 1, vertex.y()), distance_between_vertices_},
           {Vector2i(vertex.x(), vertex.y() - 1), distance_between_vertices_},
           {Vector2i(vertex.x(), vertex.y() + 1), distance_between_vertices_},
           {Vector2i(vertex.x() - 1, vertex.y() - 1),
            distance_between_vertices_ * kSqrtTwo},
           {Vector2i(vertex.x() - 1, vertex.y() + 1),
            distance_between_vertices_ * kSqrtTwo},
           {Vector2i(vertex.x() - 1, vertex.y()), distance_between_vertices_}}};
}

void EightGrid::PrintQueue(
    std::priority_queue<EightGrid::PriorityQueueVertex,
                        vector<EightGrid::PriorityQueueVertex>>
        pq) const {
  while (!pq.empty()) {
    const PriorityQueueVertex& top = pq.top();
    LOG(INFO) << "d(" << top.total_distance << ") h(" << top.heuristic_value
              << ") curr: " << top.current_vertex.transpose()
              << " prev: " << top.previous_vertex.transpose();
    pq.pop();
  }
}

int EightGrid::VertexToIndex(const Vector2i& vec) {
  if (!kProduction) {
    if (std::abs(vec.x()) >
            (field_dimensions::kHalfFieldLength / distance_between_vertices_) ||
        std::abs(vec.y()) >
            (field_dimensions::kHalfFieldWidth) / distance_between_vertices_) {
      LOG(FATAL) << "Out of field bounds: " << vec.transpose();
    }
  }

  const int halx_x_shift = static_cast<int>(field_dimensions::kHalfFieldLength /
                                            distance_between_vertices_);
  const int halx_y_shift = static_cast<int>(field_dimensions::kHalfFieldWidth /
                                            distance_between_vertices_);

  const Vector2i shifted_vec = vec + Vector2i(halx_x_shift, halx_y_shift);

  const int index = shifted_vec.x() + halx_x_shift * 2 * shifted_vec.y();

  if (!kProduction) {
    if (index < 0 || index > total_vertex_count_) {
      LOG(FATAL) << "Index out of range: " << index
                 << " (max: " << total_vertex_count_
                 << ") Orig position: " << vec.transpose();
    }
  }

  return index;
}

std::unordered_map<Vector2i, std::array<bool, 8>, EightGrid::GridHasher>
EightGrid::CalculateInvalidStaticVertices() {
  const auto static_flags = obstacle::ObstacleFlag::GetStaticObstacles();
  std::unordered_map<Vector2i, std::array<bool, 8>, EightGrid::GridHasher>
      check_map;
  std::array<bool, 8> all_invalid_path_array;
  all_invalid_path_array.fill(false);
  std::array<bool, 8> valid_path_array;
  valid_path_array.fill(true);
  bool has_collisions = false;

  const int max_field_x_index = static_cast<int>(
      field_dimensions::kHalfFieldLength / distance_between_vertices_);
  const int max_field_y_index = static_cast<int>(
      field_dimensions::kHalfFieldWidth / distance_between_vertices_);

  for (const auto* obstacle : static_flags) {
    const float margin_amount = safety_margin_.GetMargin(obstacle->GetType());

    const Vector2i grid_position =
        FreeSpaceToGridVertex(obstacle->GetPose().translation);
    const auto external_bounding_box = obstacle->GetExternalBoundingBox();

    const int check_spread_x = static_cast<int>(
        std::ceil(external_bounding_box.half_width_ + margin_amount) /
            distance_between_vertices_ +
        2);

    const int check_spread_y = static_cast<int>(
        std::ceil(external_bounding_box.half_height_ + margin_amount) /
            distance_between_vertices_ +
        2);

    const int check_spread_x_positive =
        (check_spread_x + grid_position.x() > max_field_x_index)
            ? max_field_x_index - grid_position.x()
            : check_spread_x;
    const int check_spread_x_negative =
        (-check_spread_x + grid_position.x() < -max_field_x_index)
            ? -max_field_x_index - grid_position.x()
            : -check_spread_x;

    const int check_spread_y_positive =
        (check_spread_y + grid_position.y() > max_field_y_index)
            ? max_field_y_index - grid_position.y()
            : check_spread_y;
    const int check_spread_y_negative =
        (-check_spread_y + grid_position.y() < -max_field_y_index)
            ? -max_field_y_index - grid_position.y()
            : -check_spread_y;

    for (int x = check_spread_x_negative; x <= check_spread_x_positive; ++x) {
      for (int y = check_spread_y_negative; y <= check_spread_y_positive; ++y) {
        const Vector2i check_position = grid_position + Vector2i(x, y);

        if (obstacle->PointCollision(GridVertexToFreeSpace(check_position),
                                     margin_amount)) {
          check_map.insert({check_position, all_invalid_path_array});
          continue;
        }

        const auto& neighbors = GetNeighbors(check_position);
        for (size_t i = 0; i < neighbors.size(); ++i) {
          if (obstacle->LineCollision(GridVertexToFreeSpace(check_position),
                                      GridVertexToFreeSpace(neighbors[i].first),
                                      margin_amount)) {
            has_collisions = true;
            valid_path_array[i] = false;
          }
        }

        if (has_collisions) {
          check_map.insert({check_position, valid_path_array});
          has_collisions = false;
          valid_path_array.fill(true);
        }
      }
    }
  }

  return check_map;
}

std::unordered_map<Vector2i, std::array<bool, 8>, EightGrid::GridHasher>
EightGrid::CalculateInvalidVertices(logger::Logger* logger) {
  std::unordered_map<Vector2i, std::array<bool, 8>, EightGrid::GridHasher>
      check_map(total_vertex_count_);
  std::array<bool, 8> all_invalid_path_array;
  all_invalid_path_array.fill(false);
  std::array<bool, 8> valid_path_array;
  valid_path_array.fill(true);
  bool has_collisions = false;

  const int max_field_x_index = static_cast<int>(
      field_dimensions::kHalfFieldLength / distance_between_vertices_);
  const int max_field_y_index = static_cast<int>(
      field_dimensions::kHalfFieldWidth / distance_between_vertices_);

  for (const auto* obstacle : obstacles_) {
    if (obstacle->GetType() == obstacle::ObstacleType::STATIC) {
      continue;
    }

    if (obstacle->GetType() == obstacle::ObstacleType::ROBOT) {
      logger->AddCircle(obstacle->GetPose().translation, obstacle->GetRadius(),
                        0.5, 1, 0, 1);
    }

    const auto bounding_box = obstacle->GetExternalBoundingBox();
    const auto& center = obstacle->GetPose().translation;

    const float margin_amount = safety_margin_.GetMargin(obstacle->GetType());

    logger->AddLine(
        center + Vector2f(bounding_box.half_width_, bounding_box.half_height_),
        center + Vector2f(bounding_box.half_width_, -bounding_box.half_height_),
        0, 0, 0, 1);
    logger->AddLine(
        center - Vector2f(bounding_box.half_width_, bounding_box.half_height_),
        center - Vector2f(bounding_box.half_width_, -bounding_box.half_height_),
        0, 0, 0, 1);

    logger->AddLine(
        center + Vector2f(bounding_box.half_width_, bounding_box.half_height_),
        center + Vector2f(-bounding_box.half_width_, bounding_box.half_height_),
        0, 0, 0, 1);
    logger->AddLine(
        center - Vector2f(bounding_box.half_width_, bounding_box.half_height_),
        center - Vector2f(-bounding_box.half_width_, bounding_box.half_height_),
        0, 0, 0, 1);

    logger->AddLine(
        center + Vector2f(bounding_box.half_width_ + margin_amount,
                          bounding_box.half_height_ + margin_amount),
        center + Vector2f(bounding_box.half_width_ + margin_amount,
                          -bounding_box.half_height_ - margin_amount),
        0, 0, 0, 1);
    logger->AddLine(
        center - Vector2f(bounding_box.half_width_ + margin_amount,
                          bounding_box.half_height_ + margin_amount),
        center - Vector2f(bounding_box.half_width_ + margin_amount,
                          -bounding_box.half_height_ - margin_amount),
        0, 0, 0, 1);

    logger->AddLine(
        center + Vector2f(bounding_box.half_width_ + margin_amount,
                          bounding_box.half_height_ + margin_amount),
        center + Vector2f(-bounding_box.half_width_ - margin_amount,
                          bounding_box.half_height_ + margin_amount),
        0, 0, 0, 1);
    logger->AddLine(
        center - Vector2f(bounding_box.half_width_ + margin_amount,
                          bounding_box.half_height_ + margin_amount),
        center - Vector2f(-bounding_box.half_width_ - margin_amount,
                          bounding_box.half_height_ + margin_amount),
        0, 0, 0, 1);

    const Vector2i grid_position =
        FreeSpaceToGridVertex(obstacle->GetPose().translation);
    //     const auto internal_bounding_box =
    //     obstacle->GetInternalBoundingBox();
    const auto external_bounding_box = obstacle->GetExternalBoundingBox();

    //     const int index_offset_width = static_cast<int>(
    //         std::floor(static_cast<float>(internal_bounding_box.half_width_)
    //         /
    //                    distance_between_vertices_));
    //     const int index_offset_height = static_cast<int>(
    //         std::floor(static_cast<float>(internal_bounding_box.half_height_)
    //         /
    //                    distance_between_vertices_));

    const int check_spread_x = static_cast<int>(
        std::ceil(external_bounding_box.half_width_ + margin_amount) /
            distance_between_vertices_ +
        2);

    const int check_spread_y = static_cast<int>(
        std::ceil(external_bounding_box.half_height_ + margin_amount) /
            distance_between_vertices_ +
        2);

    const int check_spread_x_positive =
        (check_spread_x + grid_position.x() > max_field_x_index)
            ? max_field_x_index - grid_position.x()
            : check_spread_x;
    const int check_spread_x_negative =
        (-check_spread_x + grid_position.x() < -max_field_x_index)
            ? -max_field_x_index - grid_position.x()
            : -check_spread_x;

    const int check_spread_y_positive =
        (check_spread_y + grid_position.y() > max_field_y_index)
            ? max_field_y_index - grid_position.y()
            : check_spread_y;
    const int check_spread_y_negative =
        (-check_spread_y + grid_position.y() < -max_field_y_index)
            ? -max_field_y_index - grid_position.y()
            : -check_spread_y;

    for (int x = check_spread_x_negative; x <= check_spread_x_positive; ++x) {
      for (int y = check_spread_y_negative; y <= check_spread_y_positive; ++y) {
        const Vector2i check_position = grid_position + Vector2i(x, y);

        const auto free_space = GridVertexToFreeSpace(check_position);

        //         if (std::abs(x) < index_offset_width &&
        //             std::abs(y) < index_offset_height) {
        //           logger->AddPoint(free_space.x(), free_space.y(), 0,0,1,1);
        //           continue;
        //         }

        if (obstacle->PointCollision(GridVertexToFreeSpace(check_position),
                                     margin_amount)) {
          auto insert_pair =
              check_map.insert({check_position, all_invalid_path_array});

          if (!insert_pair.second) {
            insert_pair.first->second = all_invalid_path_array;
          }

          logger->AddPoint(free_space.x(), free_space.y(), 1, 0, 0, 1);
          continue;
        }

        const auto& neighbors = GetNeighbors(check_position);
        for (size_t i = 0; i < neighbors.size(); ++i) {
          if (obstacle->LineCollision(GridVertexToFreeSpace(check_position),
                                      GridVertexToFreeSpace(neighbors[i].first),
                                      margin_amount)) {
            has_collisions = true;
            valid_path_array[i] = false;
          }
        }

        //         logger->AddPoint(free_space.x(), free_space.y(), 1,1,1,1);

        if (has_collisions) {
          auto insert_pair =
              check_map.insert({check_position, valid_path_array});
          if (!insert_pair.second) {
            auto& existing_array = insert_pair.first->second;
            for (size_t i = 0; i < existing_array.size(); ++i) {
              existing_array[i] = existing_array[i] && valid_path_array[i];
            }
          }
          //           logger->AddPoint(free_space.x(), free_space.y(), 0, 0, 1,
          //           1);
          has_collisions = false;
          valid_path_array.fill(true);
        }
      }
    }
  }

  return check_map;
}

int64_t ConvertCostToIntegral(const float& cost) {
  return static_cast<int64_t>(cost * 100.0f);
}

bool EightGrid::AStarSearch(const Vector2f& start, const Vector2f& goal,
                            vector<Vector2i>* waypoint_list,
                            logger::Logger* logger) {
  if (!kProduction) {
    if (!waypoint_list->empty()) {
      LOG(FATAL) << "Given non-empty vector to add waypoints to!!!!";
    }
  }

  const Vector2i start_grid_vertex = FreeSpaceToOpenGridVertex(start);
  const Vector2i goal_grid_vertex = FreeSpaceToOpenGridVertex(goal);

  // Add the start node as the initial position in the priority queue.
  float heuristic_value = Heuristic(start_grid_vertex, goal_grid_vertex);
  priority_queue_.push(PriorityQueueVertex(
      0, heuristic_value, start_grid_vertex, kStartNodePredecessor));

  while (!priority_queue_.empty()) {
    const PriorityQueueVertex top_priorirty_queue_vertex =
        priority_queue_.top();
    priority_queue_.pop();
    const Vector2i& current_vertex = top_priorirty_queue_vertex.current_vertex;
    const Vector2i& previous_vertex =
        top_priorirty_queue_vertex.previous_vertex;

    // Check to see that the current vertex isn't in the closed vertex list.
    const int current_vertex_closed_index = VertexToIndex(current_vertex);
    if (!kProduction) {
      if (current_vertex_closed_index < 0 ||
          current_vertex_closed_index >=
              static_cast<int>(closed_vertices_.size())) {
        LOG(FATAL) << "Current Vertex closed index out of bounds: ("
                   << current_vertex_closed_index << " vs max "
                   << closed_vertices_.size() << ")";
      }
    }
    if (closed_vertices_[current_vertex_closed_index]) {
      continue;
    }

    closed_vertices_[current_vertex_closed_index] = true;
    path_map_.insert({current_vertex, previous_vertex});

    // Goal found, begin unwind process.
    if (Heuristic(current_vertex, goal_grid_vertex) <
        distance_between_vertices_) {
      Vector2i current_map_key = current_vertex;
      const Vector2i start_map_key = start_grid_vertex;
      while (current_map_key != start_map_key) {
        waypoint_list->push_back(current_map_key);
        current_map_key = path_map_.find(current_map_key)->second;
      }
      waypoint_list->push_back(start_map_key);
      std::reverse(waypoint_list->begin(), waypoint_list->end());
      while (!priority_queue_.empty()) {
        priority_queue_.pop();
      }
      path_map_.clear();
      std::fill(closed_vertices_.begin(), closed_vertices_.end(), false);
      return true;
    }

    const auto static_find_result =
        static_invalid_vertices_.find(current_vertex);
    const auto dynamic_find_result =
        dynamic_invalid_vertices_.find(current_vertex);
    bool potential_dynamic_collision =
        (dynamic_find_result != dynamic_invalid_vertices_.end());
    bool potential_static_collision =
        (static_find_result != static_invalid_vertices_.end());

    const auto neighbors = GetNeighbors(current_vertex);

    for (size_t i = 0; i < neighbors.size(); ++i) {
      const Vector2i& neighbor_position = neighbors[i].first;
      if (std::abs(neighbor_position.x()) >
              static_cast<int>(field_dimensions::kHalfFieldLength /
                               distance_between_vertices_) ||
          std::abs(neighbor_position.y()) >
              static_cast<int>(field_dimensions::kHalfFieldWidth /
                               distance_between_vertices_)) {
        continue;
      }

      if ((potential_dynamic_collision && !dynamic_find_result->second[i]) ||
          (potential_static_collision && !static_find_result->second[i])) {
        continue;
      }

      const float distance =
          top_priorirty_queue_vertex.total_distance + neighbors[i].second;
      const float heuristic = Heuristic(neighbors[i].first, goal_grid_vertex);
      priority_queue_.push(PriorityQueueVertex(
          distance, heuristic, neighbors[i].first, current_vertex));
    }
  }

  while (!priority_queue_.empty()) {
    priority_queue_.pop();
  }
  path_map_.clear();
  std::fill(closed_vertices_.begin(), closed_vertices_.end(), false);
  return false;
}

Vector2i EightGrid::FreeSpaceToGridVertex(const Vector2f& free_space_vector) {
  const float x_multiple = free_space_vector.x() / distance_between_vertices_;
  const float y_multiple = free_space_vector.y() / distance_between_vertices_;
  const float x_remaining =
      x_multiple - static_cast<float>(static_cast<int>(x_multiple));
  const float y_remaining =
      y_multiple - static_cast<float>(static_cast<int>(y_multiple));
  const int x_index =
      static_cast<int>(x_multiple) +
      ((fabs(x_remaining) > 0.5) ? math_util::Sign<float>(x_remaining) : 0);
  const int y_index =
      static_cast<int>(y_multiple) +
      ((fabs(y_remaining) > 0.5) ? math_util::Sign<float>(y_remaining) : 0);
  return {x_index, y_index};
}

Vector2i EightGrid::FreeSpaceToOpenGridVertex(
    const Vector2f& free_space_vector) {
  const Vector2i closest_vector = FreeSpaceToGridVertex(free_space_vector);
  Vector2i proposed_vector = closest_vector;

  int neighbors_index = 8;
  std::array<std::pair<Eigen::Vector2i, float>, 8> neighbors;

  // Guarenteed termination.
  for (int i = 0; i < total_vertex_count_; ++i) {
    bool proposed_free = true;
    for (const auto* obstacle : obstacles_) {
      if (obstacle->PointCollision(
              GridVertexToFreeSpace(proposed_vector),
              safety_margin_.GetMargin(obstacle->GetType()))) {
        proposed_free = false;
        break;
      }
    }

    if (proposed_free) {
      return proposed_vector;
    }

    if (neighbors_index >= 8) {
      neighbors = GetNeighbors(proposed_vector);
      neighbors_index = 0;
    }

    if (!kProduction) {
      if (neighbors_index < 0 || neighbors_index >= 8) {
        LOG(FATAL) << "Neighbors index is out of range.";
      }
    }

    proposed_vector = neighbors[neighbors_index].first;
    neighbors_index++;
  }

  return closest_vector;
}

Vector2f EightGrid::GridVertexToFreeSpace(const Vector2i& in) {
  return in.cast<float>() * distance_between_vertices_;
}

std::pair<bool, std::vector<Vector2i>> EightGrid::PlanGridIndex(
    logger::Logger* logger) {
  // TODO(kvedder): More intelligently pick vertices such that they are not
  // potentially in collision.
  bool is_in_collision = false;
  for (const auto* obstacle : obstacles_) {
    if (obstacle->PointCollision(
            start_position_, safety_margin_.GetMargin(obstacle->GetType())) ||
        obstacle->PointCollision(
            goal_position_, safety_margin_.GetMargin(obstacle->GetType()))) {
      is_in_collision = true;
      break;
    }
  }

  if (is_in_collision) {
    logger->LogPrint("Eight Grid starting in collision, cannot plan!");
    return {false, {}};
  }

  vector<Vector2i> path;
  bool found_path = AStarSearch(start_position_, goal_position_, &path, logger);
  return {found_path, path};
}

std::pair<bool, std::vector<Vector2f>> EightGrid::Plan(logger::Logger* logger) {
  const std::pair<bool, std::vector<Vector2i>> integer_plan =
      PlanGridIndex(logger);
  vector<Vector2f> path_floats(integer_plan.second.size());
  for (size_t i = 0; i < integer_plan.second.size(); ++i) {
    path_floats[i] = GridVertexToFreeSpace(integer_plan.second[i]);
  }
  return {integer_plan.first, path_floats};
}

const std::unordered_map<Vector2i, std::array<bool, 8>, EightGrid::GridHasher>&
EightGrid::GetDynamicInvalidVertices() {
  return dynamic_invalid_vertices_;
}

const std::unordered_map<Vector2i, std::array<bool, 8>, EightGrid::GridHasher>&
EightGrid::GetStaticInvalidVertices() {
  return static_invalid_vertices_;
}

}  // namespace eight_grid
}  // namespace navigation
