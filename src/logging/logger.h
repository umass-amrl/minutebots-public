// Copyright 2011-2019 jaholtz@cs.umass.edu
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
#include <cxxabi.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <cstdarg>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gui/graph.h"
#include "gui/viewer.h"
#include "net/netraw.h"
#include "re2/re2.h"
#include "shared/common_includes.h"
#include "state/referee_state.h"
#include "state/team.h"
#include "state/world_state.h"

#ifndef SRC_LOGGING_LOGGER_H_
#define SRC_LOGGING_LOGGER_H_

namespace logger {

std::string demangle(const char* name);

void NewTextTree(const string& str, MinuteBotsProto::TextTree* tree);

// A datapoint for kick tuning.
struct KickData {
  // Difference between current angle and target angle
  float angle_error;

  // Radial distance between ball and robot
  float radial_dist;

  // Relative y velocity between the robot and the ball.
  float y_prime_velocity;

  // Relative velocity between the robot and the ball.
  float relative_velocity;

  // Error in alignment along the y-axis in robot frame.
  float align_error;

  // Robot rotational velocity.
  float rotational_velocity;

  bool kick;

  // Default constructor: do nothing.
  KickData() {}

  // Initialization constructor.
  KickData(float angle_error, float radial_dist, float y_prime_velocity,
           float relative_velocity, float align_error,
           float rotational_velocity)
      : angle_error(angle_error),
        radial_dist(radial_dist),
        y_prime_velocity(y_prime_velocity),
        relative_velocity(relative_velocity),
        align_error(align_error),
        rotational_velocity(rotational_velocity) {}
  // Constructor from message
  KickData(const MinuteBotsProto::SoccerDebugMessage& message,
           const int& robot_number);
};

// A class which takes log commands and stores them in an internal protobuff,
// and is used for both text statements and viewer drawings.
// This logger does no string parsing until the full text tree is requested.
class Logger {
 public:
  Logger();
  // Merges the contents of two loggers.
  void MergeLoggers(Logger input_logger);

  void SetWorldState(MinuteBotsProto::WorldState world_state);

  void SetWorldState(Logger input_logger);

  void SetWorldState(const state::WorldState& state);

  void SetRobotTargets(const pose_2d::Pose2Df& goal_pose,
                       const float& target_angle, const int& robot_id);

  // Indents the log tree by one step.
  void Push();

  // Prints given header string and then indents the log tree by one step.
  void Push(const string& header);

  // Unindents the log tree by one step.
  void Pop();
  // Clears the contents of the logger.
  void Clear();

  // Adds a line to the drawings of the stored Soccer Debug Message.
  void AddLine(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2,
               const float& r, const float& g, const float& b, const float& a);

  // Adds a line to the drawings of the stored Soccer Debug Message.
  void AddLine(const float& x1, const float& y1, const float& x2,
               const float& y2, const float& r, const float& g, const float& b,
               const float& a);

  // Adds an arc with the given parameters to the stored drawings.
  void AddCircle(const Eigen::Vector2f p, const float& radius, const float& r,
                 const float& g, const float& b, const float& a);

  // Adds an arc with the given parameters to the stored drawings.
  void AddArc(const Eigen::Vector2f p, const float& radius,
              const float& angle_start, const float& angle_end, const float& r,
              const float& g, const float& b, const float& a);

  // Adds an ellipse with the given parameters to the stored drawings.
  void AddEllipse(const Eigen::Vector2f p, const float& radius,
                  const float& radius_2, const float& angle, const float& r,
                  const float& g, const float& b, const float& a);

  // Adds an arc with the given parameters to the stored drawings.
  void AddArc(const float& x, const float& y, const float& radius,
              const float& angle_start, const float& angle_end, const float& r,
              const float& g, const float& b, const float& a);

  // Adds a point with the given parameters to the stored drawings.
  void AddPoint(const float& x, const float& y, const float& r, const float& g,
                const float& b, const float& a);

  // Adds a point with the given parameters to the stored drawings.
  void AddPoint(const Eigen::Vector2f& p, const float& r, const float& g,
                const float& b, const float& a);

  // Adds a set of points with the given parameters to the stored drawings.
  void AddPoints(const std::vector<Eigen::Vector2f>& points, const float& r,
                 const float& g, const float& b, const float& a);

  // Draw a path specified by a sequence of points.
  void AddPath(const std::vector<Eigen::Vector2f>& points, const float& r,
               const float& g, const float& b, const float& a);

  void AddStateMachineData(const MinuteBotsProto::StateMachineData& data);

  // Replaces the stored world state with the updated world state.
  void UpdateWorldState(const MinuteBotsProto::WorldState& world_state);

  // Adds a robot to the stored world state with the given parameters.
  void AddRobot(const int& id, const MinuteBotsProto::RobotState::Team& team,
                const float& theta, const float& x, const float& y,
                const float& confidence);
  // Creates a data entry for the specific robots id.
  gui::DataEntry RobotDataEntry(const SSLVisionId& id);
  // Writes the robot data entry for the given id to the text log.
  void LogRobotDataEntry(const SSLVisionId& id);
  // Logs the number of dropped/received messages to the text log.
  void LogMessageStats(int received, int dropped);

  // Adds a ball to the stored world state with the given parameters.
  void AddBall(const float& x, const float& y);

  // Creates a ball data entry.
  gui::DataEntry BallDataEntry();

  // Retrieves the logged SoccerDebugMessage from this Logger.
  MinuteBotsProto::SoccerDebugMessage ReturnMessage();

  // Sets the SoccerDebugMessage associated with this logger to input message.
  void SetMessage(MinuteBotsProto::SoccerDebugMessage message);

  void SetMessageTime(const double time);

  void SetTeam(const bool& team_yellow);

  // Logs the data from the game state
  void LogRefState(state::RefereeState referee_state);

  // WrapPrintLog is a templated helper function
  // that must be defined in the header.
  // Should not be called, LogPrint should be called for logging text
  // statements instead.
  void WrapPrintLog(const string& format, MinuteBotsProto::PrintLog* print_log,
                    const string& single) {
    string type_name = typeid(single).name();
    string demangled_name = demangle(type_name.c_str());
    print_log->add_type_list(demangled_name);
    print_log->add_data(single);
    print_log->set_format_str(format);
  }

  void WrapPrintLog(const string& format, MinuteBotsProto::PrintLog* print_log,
                    char const* single) {
    string type_name = typeid(single).name();
    string demangled_name = demangle(type_name.c_str());
    print_log->add_type_list(demangled_name);
    print_log->add_data(single);
    print_log->set_format_str(format);
  }

  template <typename T>
  void WrapPrintLog(const string& format, MinuteBotsProto::PrintLog* print_log,
                    const T& single) {
    const string type_name = typeid(single).name();
    string demangled_name = demangle(type_name.c_str());
    print_log->add_type_list(demangled_name);
    const char* temp = reinterpret_cast<const char*>(&single);

    const int size = sizeof(single);
    string in_string(temp, size);
    //     for (int i = 0; i < size; ++i) {
    //       in_string.push_back(temp[i]);
    //     }

    print_log->add_data(in_string);
    print_log->set_format_str(format);
  }

  template <typename T, typename... Args>
  void WrapPrintLog(const string& format, MinuteBotsProto::PrintLog* print_log,
                    const T& first, Args... args) {
    WrapPrintLog(format, print_log, first);
    WrapPrintLog(format, print_log, args...);
  }

  // templated function which handles saving of print statements to logs,
  // syntax is similar to fprintf
  // format: format string using format specificers of the form "%d"
  // Called as LogPrint("Print a float %f", float)
  // Number of arguements can be arbitrarily long.
  template <typename T, typename... Args>
  void LogPrint(const string& format, const T& first, Args... args) {
    if (print_tree_.size() > 0) {
      MinuteBotsProto::PrintLog* logs = print_tree_.back();
      if (logs->nested_size() >= 0) {
        logs->add_nested();
        MinuteBotsProto::PrintLog* print_log =
            logs->mutable_nested(logs->nested_size() - 1);
        WrapPrintLog(format, print_log, first, args...);

      } else {
        WrapPrintLog(format, logs, first, args...);
        logs->add_nested();
      }
    } else {
      debug_message_.add_print_logs();
      MinuteBotsProto::PrintLog* logs = debug_message_.mutable_print_logs(
          debug_message_.print_logs_size() - 1);
      WrapPrintLog(format, logs, first, args...);
    }
  }

  // Overload of LogPrint to handle pure text strings with
  // no format specifiers
  void LogPrint(const string& format) {
    if (print_tree_.size() > 0) {
      MinuteBotsProto::PrintLog* logs = print_tree_.back();
      if (logs->nested_size() >= 0) {
        logs->add_nested();
        MinuteBotsProto::PrintLog* print_log =
            logs->mutable_nested(logs->nested_size() - 1);
        print_log->set_format_str(format);
      }
    } else {
      debug_message_.add_print_logs();
      MinuteBotsProto::PrintLog* logs = debug_message_.mutable_print_logs(
          debug_message_.print_logs_size() - 1);
      logs->set_format_str(format);
    }
  }

  // Print text and indent the tree by one step.
  // Templated function which handles saving of print statements to logs,
  // syntax is similar to fprintf
  // format: format string using format specificers of the form "%d"
  // Called as LogPrint("Print a float %f", float)
  // Number of arguements can be arbitrarily long.
  template <typename T, typename... Args>
  void LogPrintPush(const string& format, const T& first, Args... args) {
    if (print_tree_.size() > 0) {
      MinuteBotsProto::PrintLog* logs = print_tree_.back();
      if (logs->nested_size() >= 0) {
        logs->add_nested();
        MinuteBotsProto::PrintLog* print_log =
            logs->mutable_nested(logs->nested_size() - 1);
        WrapPrintLog(format, print_log, first, args...);

      } else {
        WrapPrintLog(format, logs, first, args...);
        logs->add_nested();
      }
    } else {
      debug_message_.add_print_logs();
      MinuteBotsProto::PrintLog* logs = debug_message_.mutable_print_logs(
          debug_message_.print_logs_size() - 1);
      WrapPrintLog(format, logs, first, args...);
    }
    Push();
  }

  // Print text and indent the tree by one step.
  void LogPrintPush(const string& text) {
    LogPrint(text);
    Push();
  }

 protected:
  std::vector<MinuteBotsProto::PrintLog*> print_tree_;
  MinuteBotsProto::SoccerDebugMessage debug_message_;
  MinuteBotsProto::SoccerDebugMessage last_message_;
};

class ReadLogger : public Logger {
 public:
  explicit ReadLogger(const string& filename);
  // Gets the next message from the associated log file.
  MinuteBotsProto::LogWrapper GetNextMessage();
  // Gets the previous message from the associated log file.
  MinuteBotsProto::LogWrapper GetPreviousMessage();
  // Sets the pause status for the ReadWriteLogger
  void Pause();
  // Sets the live status and jumps to the end of the file.
  void Live();

  // Builds an index of the associated file.
  void BuildKickTuningIndex();
  void BuildTuningIndex();
  void BuildIndex();
  int GetMapSize();
  // writes kick tuning parameters to file.
  void WriteKickTuningData();
  void SetTransition(const int& index, const int& transition);
  void UnsetTransition(const int& index, const int& transition);
  void ContinueTransition(const int& index, const int& transition);
  void EndFile(const int& index);

  void AddIndexEntry(const int& file_index);
  void GoToIndex(const int& file_index);

  // Get all messages which are between time_1 and time_2 from the log file.
  std::vector<MinuteBotsProto::SoccerDebugMessage> GetMessagesInRange(
      const double time_1, const double time_2);

 protected:
  // File to read from.
  std::fstream read_file_;
  // Size of the last message read.
  int last_message_size_;
  // Current index into the file being read.
  int read_index_;
  // Pause status
  bool pause_;
  // Vector of kick data from a file
  std::vector<KickData> kick_data_;
  // index in the kick
  int kick_data_index;
  std::map<int, MinuteBotsProto::SoccerDebugMessage> tuning_index_;
  string machine_name_;
  // Map from timestamps to indexes in the file.
  std::map<const double, int> time_file_map_;
  // Maps from integers to indexes in the file.
  std::map<const int, int> count_file_map_;
};

class WriteLogger : public Logger {
 public:
  explicit WriteLogger(const string& filename);
  // Writes a protobuff in the LogWrapper type to a file
  void WriteProto(const MinuteBotsProto::LogWrapper& message);
  // Get the size of the associated file.
  int GetFileSize();
  // Write to the associated file for this logger
  void WriteData();

 protected:
  std::fstream file_;
};

class NetLogger : public Logger {
 public:
  NetLogger(const string& address, const int& port) {
    server_.Open(address, port, false);
    server_.SetSplitLarge(kNetworkSplitLarge);
  }
  // Send over the associated server for this logger
  void SendData();

 protected:
  net::UDPMulticastServer server_;
  int count_ = 0;
};

// Given a print log rebuilds the associated string.
void BuildString(const MinuteBotsProto::PrintLog& p_log, string* print_string);

void TestRe();

// Given a full set of print logs, rebuilds the associated textree into log.
void BuildTextLog(const MinuteBotsProto::PrintLogs& p_logs,
                  MinuteBotsProto::TextTree* log);

void BuildTextLog(const MinuteBotsProto::SoccerDebugMessage message,
                  MinuteBotsProto::TextTree* log);

}  // namespace logger

#endif  // SRC_LOGGING_LOGGER_H_
