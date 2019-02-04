// Copyright 2017 slane@cs.umass.edu
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

#include "navigation/static_tangent_planner.h"

#include <limits>
#include <string>
#include <fstream>
#include <iostream>

#include "algorithms/branch_and_bound.h"
#include "math/geometry.h"
#include "navigation/navigation_util.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"

using algorithms::BranchAndBound;
using geometry::EuclideanDistance;
using geometry::GetTangentPoints;
using geometry::LineLineIntersection;
using geometry::Perp;
using geometry::IsParallel;
using logger::Logger;
using obstacle::ObstacleFlag;
using obstacle::ObstacleType;
using obstacle::SafetyMargin;
using std::endl;
using std::numeric_limits;
using std::ofstream;
using std::pair;
using std::vector;
using std::cout;

namespace navigation {
Vector2f TangentAssignment::start(0.0, 0.0);
Vector2f TangentAssignment::goal(0.0, 0.0);

TangentAssignment::TangentAssignment() {
  path.push_back(start);

  path_length = EuclideanDistance(start, goal);
}

StaticTangentPlanner::StaticTangentPlanner(obstacle::ObstacleFlag obstacle_flag,
                                           obstacle::SafetyMargin margin)
    : Navigation(obstacle_flag, margin),
      dump_to_file_(false) {}

pair<bool, vector<Vector2f>> StaticTangentPlanner::Plan(Logger* logger) {
  TangentAssignment::start = start_position_;
  TangentAssignment::goal = goal_position_;

  robot_centers.clear();

  // It's assumed that the obstacles here are *only* the robots
  for (const auto& robot : obstacles_) {
    robot_centers.push_back(robot->GetPose().translation);
  }

  if (dump_to_file_) {
    float radius = kRobotRadius + safety_margin_.GetMargin(ObstacleType::ROBOT);

    ofstream obs_file;
    string obs_filename(filename_);

    obs_filename.append(".obs");

    obs_file.open(obs_filename.c_str());
    for (const auto& center : robot_centers) {
      obs_file << center.x() << ", " << center.y() << ", " << radius << endl;
    }
    obs_file.close();

    // Clear the path file
    ofstream path_file;
    path_file.open(filename_.c_str());
    path_file << "";
    path_file.close();
  }

  bool found_goal;
  vector<Vector2f> path;
  float path_length;
  TangentAssignment optimal_assignment;
  found_goal = Search(&path_length, &optimal_assignment);

//   if (dump_to_file_ && found_goal) {
//     DumpAssignment(optimal_assignment);
//   }

  return {found_goal, optimal_assignment.path};
}

void StaticTangentPlanner::EnableDumpToFile(string filename) {
  dump_to_file_ = true;
  filename_ = filename;
}

float StaticTangentPlanner::CalculateHeuristic(
    const TangentAssignment& assignment) {
  return assignment.path_length;
}

float StaticTangentPlanner::CalculateLowerBound(
    const TangentAssignment& assignment) {
  return assignment.path_length;
}

float StaticTangentPlanner::CalculateUpperBound(
    const TangentAssignment& assignment) {
  return numeric_limits<float>::max();
}

bool StaticTangentPlanner::Optimize(const TangentAssignment& assignment,
                                    float* optimal_cost) {
  *optimal_cost = assignment.path_length;

  bool is_solution =  CollisionFreePath(obstacles_,
                                        safety_margin_,
                                        assignment.path.back(),
                                        goal_position_);

  if (is_solution && dump_to_file_) {
    DumpAssignment(assignment);
  }

  return is_solution;
}

void StaticTangentPlanner::Branch(const TangentAssignment& assignment,
                                  vector<TangentAssignment>* branches) {
  const float margin = safety_margin_.GetMargin(ObstacleType::ROBOT) +
                       1;

//   if (dump_to_file_) {
//     DumpAssignment(assignment);
//   }

  for (unsigned int i = 0; i < robot_centers.size(); i++) {
    if (!IsInAssignment(assignment, i)) {
      if (assignment.obstacle_indices.size() == 0) {
        // Add the tangent lines from the start to the goal
        Vector2f start_left_tangent;
        Vector2f start_right_tangent;
        GetTangentPoints(start_position_,
                         robot_centers[i],
                         kRobotRadius + margin,
                         &start_right_tangent,
                         &start_left_tangent);

        Vector2f goal_left_tangent;
        Vector2f goal_right_tangent;

        // The right and left designations are swapped here because I care
        // about the designation relative to the start, not the goal
        GetTangentPoints(goal_position_,
                         robot_centers[i],
                         kRobotRadius + margin,
                         &goal_left_tangent,
                         &goal_right_tangent);

        Vector2f left_intersect;
        if (IsParallel(start_position_,
                       start_left_tangent,
                       goal_left_tangent,
                       goal_position_)) {
          left_intersect = (start_position_ + goal_position_)/2.0f;
        } else {
          left_intersect = LineLineIntersection(start_position_,
                                                start_left_tangent,
                                                goal_left_tangent,
                                                goal_position_);
        }
        Vector2f right_intersect;
        if (IsParallel(start_position_,
                       start_right_tangent,
                       goal_right_tangent,
                       goal_position_)) {
          right_intersect = (start_position_ + goal_position_)/2.0f;
        } else {
          right_intersect = LineLineIntersection(start_position_,
                                                 start_right_tangent,
                                                 goal_right_tangent,
                                                 goal_position_);
        }

        if (CollisionFreePath(obstacles_,
                              safety_margin_,
                              start_position_,
                              left_intersect)) {
          TangentAssignment branch(assignment);
          branch.path_length = EuclideanDistance(start_position_,
                                                 left_intersect) +
                               EuclideanDistance(left_intersect,
                                                 goal_position_);
          branch.obstacle_indices.push_back(i);
          branch.path.push_back(left_intersect);
          branch.is_right.push_back(false);
          branches->push_back(branch);

          if (dump_to_file_) {
            DumpAssignment(assignment);
          }
        }

        if (CollisionFreePath(obstacles_,
                              safety_margin_,
                              start_position_,
                              right_intersect)) {
          TangentAssignment branch(assignment);
          branch.path_length = EuclideanDistance(start_position_,
                                                 right_intersect) +
                               EuclideanDistance(right_intersect,
                                                 goal_position_);
          branch.obstacle_indices.push_back(i);
          branch.path.push_back(right_intersect);
          branch.is_right.push_back(true);
          branches->push_back(branch);

          if (dump_to_file_) {
            DumpAssignment(assignment);
          }
        }
      } else {
        Vector2f left_bitangenta;
        Vector2f left_bitangentb;
        Vector2f right_bitangenta;
        Vector2f right_bitangentb;

        GetBitangentPoints(robot_centers[assignment.obstacle_indices.back()],
                      robot_centers[i],
                      kRobotRadius + margin,
                      assignment.is_right.back(),
                      &right_bitangenta,
                      &right_bitangentb,
                      &left_bitangenta,
                      &left_bitangentb);


        Vector2f goal_left_tangent;
        Vector2f goal_right_tangent;

        // The right and left designations are swapped here because I care
        // about the designation relative to the start, not the goal
        GetTangentPoints(goal_position_,
                         robot_centers[i],
                         kRobotRadius + margin,
                         &goal_left_tangent,
                         &goal_right_tangent);

        Vector2f previous_right_intersect;
        if (IsParallel(assignment.path.end()[-2],
                       assignment.path.back(),
                       right_bitangenta,
                       right_bitangentb)) {
          previous_right_intersect =
              (assignment.path.end()[-2] + right_bitangentb) / 2.0f;
        } else {
          previous_right_intersect =
              LineLineIntersection(assignment.path.end()[-2],
                                   assignment.path.back(),
                                   right_bitangenta,
                                   right_bitangentb);
        }

        Vector2f previous_left_intersect;
        if (IsParallel(assignment.path.end()[-2],
                       assignment.path.back(),
                       left_bitangenta,
                       left_bitangentb)) {
          previous_left_intersect =
            (assignment.path.end()[-2] + left_bitangentb) / 2.0f;
        } else {
          previous_left_intersect =
          LineLineIntersection(assignment.path.end()[-2],
                              assignment.path.back(),
                              left_bitangenta,
                              left_bitangentb);
        }

        Vector2f goal_right_intersect;
        if (IsParallel(right_bitangenta,
                       right_bitangentb,
                       goal_right_tangent,
                       goal_position_)) {
          goal_right_intersect =
              (goal_position_ + right_bitangentb) / 2.0f;
        } else {
          goal_right_intersect =
              LineLineIntersection(right_bitangenta,
                                   right_bitangentb,
                                   goal_right_tangent,
                                   goal_position_);
        }

        Vector2f goal_left_intersect;
        if (IsParallel(left_bitangenta,
                       left_bitangentb,
                       goal_left_tangent,
                       goal_position_)) {
          goal_left_intersect =
              (goal_position_ + left_bitangentb) / 2.0f;
        } else {
          goal_right_intersect =
            LineLineIntersection(left_bitangenta,
                                 left_bitangentb,
                                 goal_left_tangent,
                                 goal_position_);
        }

        if (CollisionFreePath(obstacles_,
                              safety_margin_,
                              assignment.path.end()[-2],
                              previous_left_intersect) &&
            CollisionFreePath(obstacles_,
                              safety_margin_,
                              previous_left_intersect,
                              goal_left_intersect)) {
          TangentAssignment branch(assignment);
          branch.obstacle_indices.push_back(i);
          branch.path.back() = previous_left_intersect;
          branch.path.push_back(goal_left_intersect);
          branch.path_length = GetPathLength(branch);
          branch.is_right.push_back(false);
          branches->push_back(branch);

          if (dump_to_file_) {
            DumpAssignment(assignment);
          }
        }

        if (CollisionFreePath(obstacles_,
                              safety_margin_,
                              assignment.path.end()[-2],
                              previous_right_intersect) &&
            CollisionFreePath(obstacles_,
                              safety_margin_,
                              previous_right_intersect,
                              goal_right_intersect)) {
          TangentAssignment branch(assignment);
          branch.obstacle_indices.push_back(i);
          branch.path.back() = previous_right_intersect;
          branch.path.push_back(goal_right_intersect);
          branch.path_length = GetPathLength(branch);
          branch.is_right.push_back(true);
          branches->push_back(branch);

          if (dump_to_file_) {
            DumpAssignment(assignment);
          }
        }
      }
    }
  }
}

bool StaticTangentPlanner::IsInAssignment(const TangentAssignment& assignment,
                                          unsigned int index) {
  for (const auto& obstacle_index : assignment.obstacle_indices) {
    if (obstacle_index == index)
      return true;
  }
  return false;
}

float StaticTangentPlanner::GetPathLength(const TangentAssignment& assignment) {
  float length = 0;
  Vector2f previous_point = start_position_;
  for (const auto& point : assignment.path) {
    length += EuclideanDistance(previous_point, point);
    previous_point = point;
  }
  length += EuclideanDistance(previous_point, goal_position_);

  return length;
}


void StaticTangentPlanner::GetBitangentPoints(const Vector2f& p1,
                                         const Vector2f& p2,
                                         const float& radius,
                                         bool from_right,
                                         Vector2f* right_tangenta,
                                         Vector2f* right_tangentb,
                                         Vector2f* left_tangenta,
                                         Vector2f* left_tangentb) {
  Vector2f dir = p2 - p1;
  dir.normalize();
  Vector2f perp = Perp(dir);

  const float epsilon = 2*kEpsilon;

  if (from_right) {
    *right_tangenta = p1 - (radius + epsilon) * perp;
    *right_tangentb = p2 - (radius + epsilon) * perp;

    Vector2f ref_tangent;
    Vector2f junk_tangent;

    GetTangentPoints(p1,
                     p2,
                     2*radius + epsilon,
                     &junk_tangent,
                     &ref_tangent);

    Vector2f tangent_dir = ref_tangent - p2;
    tangent_dir.normalize();

    *left_tangenta = p1 - (radius + epsilon)*tangent_dir;
    *left_tangentb = ref_tangent - (radius)*tangent_dir;

  } else {
    *left_tangenta = p1 + (radius + epsilon) * perp;
    *left_tangentb = p2 + (radius + epsilon) * perp;

    Vector2f ref_tangent;
    Vector2f junk_tangent;

    GetTangentPoints(p1,
                     p2,
                     2*radius + kEpsilon,
                     &ref_tangent,
                     &junk_tangent);

    Vector2f tangent_dir = ref_tangent - p2;
    tangent_dir.normalize();

    *right_tangenta = p1 - (radius + kEpsilon)*tangent_dir;
    *right_tangentb = ref_tangent - (radius)*tangent_dir;
  }
}

void StaticTangentPlanner::DumpAssignment(const TangentAssignment& assignment) {
  ofstream file;
  file.open(filename_.c_str(), std::ios::app);

  for (const auto& point : assignment.path) {
    file << point.x() << ", " << point.y() << ", ";
  }

  file << goal_position_.x() << ", " << goal_position_.y();
  file << endl;
  file.close();
}


}  // namespace navigation
