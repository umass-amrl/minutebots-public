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

#include "logging/navigation_logger.h"

#include <sys/time.h>

#include <fstream>
#include <iostream>
#include <string>
#include <ctime>

#include "shared/common_includes.h"

using std::fstream;
using std::string;
using std::stringstream;
using std::vector;


namespace navigation_logger {

const char* NavigationLogger::team_name_;
const char* NavigationLogger::algorithm_name_;
fstream NavigationLogger::file_;

bool NavigationLogger::plans_theta_ = false;
bool NavigationLogger::plans_kinodynamics_ = false;
bool NavigationLogger::plans_jointly_ = false;
NavigationLoggingProto::NavigationLogEntry NavigationLogger::current_entry_;

void NavigationLogger::Init(string team_name,
                            string algorithm_name,
                            bool plans_theta,
                            bool plans_kinodynamics,
                            bool plans_jointly) {
  team_name_ = team_name.c_str();
  algorithm_name_ = algorithm_name.c_str();
  plans_theta_ = plans_theta;
  plans_kinodynamics_ = plans_kinodynamics;
  plans_jointly_ = plans_jointly;
  time_t t = time(0);  // get current time
  struct timeval tval;
  gettimeofday(&tval, NULL);
  struct tm* now = new tm();
  localtime_r(&t, now);
  std::stringstream ss;
  ss << "navigation_logs/";
  ss << "navlog_";
  ss << std::to_string(now->tm_mon).c_str() << "-";
  ss << std::to_string(now->tm_mday).c_str() << "-"
     << std::to_string(1900 + now->tm_year).c_str() << "_";
  ss << std::to_string((tval.tv_sec * 1000000) + tval.tv_usec).c_str()
     << ".log";

  delete now;

  file_.open(ss.str(), std::ios::out | std::ios::app | std::ios::binary);
}

void NavigationLogger::Close() {
  if (file_.is_open()) {
    file_.close();
  }
}

void NavigationLogger::AddRobot(int32_t ssl_vision_id,
                                double planning_time,
                                bool found_path,
                                float start_x,
                                float start_y,
                                float start_theta,
                                float goal_x,
                                float goal_y,
                                float goal_theta,
                                const vector<float>& x_waypoints,
                                const vector<float>& y_waypoints,
                                const vector<float>& theta_waypoints) {
  if (file_.is_open()) {
    auto* robot_entry = current_entry_.add_robot_plans();
    robot_entry->set_robot_id(ssl_vision_id);
    robot_entry->set_planning_time(planning_time);

    robot_entry->set_current_x(start_x);
    robot_entry->set_current_y(start_y);
    robot_entry->set_current_theta(start_theta);

    for (const auto& waypoint : x_waypoints) {
      robot_entry->add_waypoints_x(waypoint);
    }

    for (const auto& waypoint : y_waypoints) {
      robot_entry->add_waypoints_y(waypoint);
    }

    for (const auto& waypoint : theta_waypoints) {
      robot_entry->add_waypoints_theta(waypoint);
    }

    robot_entry->set_goal_x(goal_x);
    robot_entry->set_goal_y(goal_y);
    robot_entry->set_goal_theta(goal_theta);

    robot_entry->set_found_path(found_path);
  }
}

void NavigationLogger::AddRobot(int32_t ssl_vision_id,
                                double planning_time,
                                bool found_path,
                                float start_x,
                                float start_y,
                                float start_theta,
                                float goal_x,
                                float goal_y,
                                float goal_theta,
                                const vector<Vector2f>& waypoints) {
  if (file_.is_open()) {
    auto* robot_entry = current_entry_.add_robot_plans();
    robot_entry->set_robot_id(ssl_vision_id);
    robot_entry->set_planning_time(planning_time);

    robot_entry->set_current_x(start_x);
    robot_entry->set_current_y(start_y);
    robot_entry->set_current_theta(start_theta);

    for (const auto& waypoint : waypoints) {
      robot_entry->add_waypoints_x(waypoint.x());
      robot_entry->add_waypoints_y(waypoint.y());
    }

    robot_entry->set_goal_x(goal_x);
    robot_entry->set_goal_y(goal_y);
    robot_entry->set_goal_theta(goal_theta);

    robot_entry->set_found_path(found_path);
  }
}

void NavigationLogger::LogAndResetEntry() {
  if (file_.is_open()) {
    current_entry_.set_team_name(string(team_name_));
    current_entry_.set_planning_alg_name(string(algorithm_name_));
    current_entry_.set_plans_theta(plans_theta_);
    current_entry_.set_plans_kinodynamics(plans_kinodynamics_);
    current_entry_.set_plans_jointly(plans_jointly_);

    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    const double time =
      static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec)*(1.0E-9);

    current_entry_.set_current_time(time);

    int size = current_entry_.ByteSize();
    file_.write(reinterpret_cast<char*>(&size), sizeof(int));
    current_entry_.SerializeToOstream(&file_);
    file_.flush();

    current_entry_.Clear();
  }
}
}  // namespace navigation_logger
