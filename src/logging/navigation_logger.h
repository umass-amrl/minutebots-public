// Copyright 2018 slane@cs.umass.edu
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

#ifndef SRC_LOGGING_NAVIGATION_LOGGER_H_
#define SRC_LOGGING_NAVIGATION_LOGGER_H_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "shared/common_includes.h"
#include "navigation_logging.pb.h"

namespace navigation_logger {
class NavigationLogger {
 public:
  // Add a new robot to the current log entry
  // Takes as input:
  //      ssl_vision_id: The SSL Vision ID of the relevant robot
  //      planning_time: How long did it take to generate the plan
  //      x_waypoints: The vector of X waypoints included in the plan. The
  // first entry is assumed to be the current position and the last entry is
  // assumed to be the goal.
  //      y_waypoints: The vector of Y waypoints included in the plan. The
  // first entry is assumed to be the current position and the last entry is
  // assumed to be the goal.
  //      theta_waypoints: The vector of theta waypoints included in the plan.
  // The first entry is assumed to be the current position and the last entry is
  // assumed to be the goal.
  static void AddRobot(int32_t ssl_vision_id,
                       double planning_time,
                       bool found_path,
                       float start_x,
                       float start_y,
                       float start_theta,
                       float goal_x,
                       float goal_y,
                       float goal_theta,
                       const std::vector<float>& x_waypoint,
                       const std::vector<float>& y_waypoints,
                       const std::vector<float>& theta_waypoints);

  static void AddRobot(int32_t ssl_vision_id,
                       double planning_time,
                       bool found_path,
                       float start_x,
                       float start_y,
                       float start_theta,
                       float goal_x,
                       float goal_y,
                       float goal_theta,
                       const std::vector<Eigen::Vector2f>& waypoints);

  // Print the current entry to the log and clear it. Sets the current time in
  // the protobuf at this point.
  static void LogAndResetEntry();

  // Initialization for the navigation logger class
  // Takes as Input:
  //       team_name: string that stores your team name
  //       algorithm_name: string that stores the name of your
  // navigation planning algorithm.
  //       plans_theta: bool that should be set to true if your navigation
  // algorithm plans for both translation and rotation.
  //       plans_kinodynamics: bool that indicates whether your navigation
  // planning algorithm returns kinodynamcally feasible paths.
  //       plans_jointly: bool that indicates whether your planning algorithm
  // plans for more than one robot at the same time.
  static void Init(std::string team_name,
                   std::string algorithm_name,
                   bool plans_theta,
                   bool plans_kinodynamics,
                   bool plans_jointly);

  static void Close();

 private:
  static const char* team_name_;
  static const char* algorithm_name_;

  static std::fstream file_;

  static bool plans_theta_;
  static bool plans_kinodynamics_;
  static bool plans_jointly_;

  static NavigationLoggingProto::NavigationLogEntry current_entry_;
};
}  // namespace navigation_logger

#endif  // SRC_LOGGING_NAVIGATION_LOGGER_H_
