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

#include "logging/logger.h"
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <iomanip>  // std::setprecision
#include <map>
#include "third_party/json.hpp"

using gui::Viewer;
using MinuteBotsProto::SoccerDebugMessage;
using MinuteBotsProto::DebugDrawings;
using MinuteBotsProto::PrintLog;
using MinuteBotsProto::PrintLogs;
using MinuteBotsProto::LogWrapper;
using MinuteBotsProto::TextTree;
using MinuteBotsProto::RobotState;
using std::ios;
using std::vector;
using state::WorldRobot;
using team::Team;
using pose_2d::Pose2Df;
using MinuteBotsProto::StateMachineData;
using MinuteBotsProto::PossibleTransition;
using MinuteBotsProto::Trace;

namespace logger {

// Necessary for rebuilding print strings with format specificers.
// Placed here to avoid compiling more than once.
RE2 re_compile("((?U).*%(?U).*(d|i|u|s|o|x|X|f|F|e|E|g|G|a|A|c|s|p|n|%))");

std::string demangle(const char* name) {
  int status = -4;  // some arbitrary value to eliminate the compiler warning
  // enable c++11 by passing the flag -std=c++11 to g++
  std::unique_ptr<char, void (*)(void*)> res{
      abi::__cxa_demangle(name, NULL, NULL, &status), std::free};

  return (status == 0) ? res.get() : name;
}

void NewTextTree(const string& str, TextTree* tree) { tree->set_text(str); }

Logger::Logger() {}

gui::DataEntry Logger::RobotDataEntry(const SSLVisionId& id) {
  if (last_message_.has_time() &&
      last_message_.world_state().robots_size() >= static_cast<int>(id) &&
      debug_message_.world_state().robots_size() >= static_cast<int>(id)) {
    double time_2 = debug_message_.world_state().robots(id).ssl_time();
    double time_1 = last_message_.world_state().robots(id).ssl_time();
    double time_diff = time_2 - time_1;

    if (fabs(time_diff) < kEpsilon) return gui::DataEntry();

    Eigen::Vector2f current_pos = {
        debug_message_.world_state().robots(id).pose_raw().loc().x(),
        debug_message_.world_state().robots(id).pose_raw().loc().y()};

    Eigen::Vector2f last_pos = {
        last_message_.world_state().robots(id).pose_raw().loc().x(),
        last_message_.world_state().robots(id).pose_raw().loc().y()};

    float current_angle =
        debug_message_.world_state().robots(id).pose_raw().angle();

    float previous_angle =
        last_message_.world_state().robots(id).pose_raw().angle();

    Vector2f velocity = current_pos - last_pos;

    float angular_vel = (current_angle - previous_angle) / time_diff;

    velocity = velocity / time_diff;
    float speed = velocity.norm();
    gui::DataEntry entry(time_2, current_pos, current_angle, speed, velocity,
                         angular_vel);
    return entry;
  }
  return gui::DataEntry();
}

WriteLogger::WriteLogger(const string& filename) {
  file_.open(filename,
             std::ios::in | std::ios::out | std::ios::app | std::ios::binary);
}

void WriteLogger::WriteProto(const MinuteBotsProto::LogWrapper& message) {
  int size = message.ByteSize();
  file_.write(reinterpret_cast<char*>(&size), sizeof(int));
  message.SerializeToOstream(&file_);
  file_.write(reinterpret_cast<char*>(&size), sizeof(int));
  file_.flush();
}

void WriteLogger::WriteData() {
  if (file_.is_open()) {
    LogWrapper wrapper;
    *wrapper.mutable_soccer_debug() = debug_message_;
    WriteProto(wrapper);
  }
}

gui::DataEntry Logger::BallDataEntry() {
  const bool kDebug = false;
  if (last_message_.world_state().balls_size() > 0 &&
      debug_message_.world_state().balls_size() > 0 &&
      last_message_.world_state().has_ball_time() &&
      debug_message_.world_state().has_ball_time()) {
    double time_2 = debug_message_.world_state().ball_time();
    double time_1 = last_message_.world_state().ball_time();
    double time_diff = time_2 - time_1;

    Eigen::Vector2f current_pos = {
        debug_message_.world_state().ball_observation(0).x(),
        debug_message_.world_state().ball_observation(0).y()};

    Eigen::Vector2f last_pos = {
        last_message_.world_state().ball_observation(0).x(),
        last_message_.world_state().ball_observation(0).y()};

    Vector2f velocity = current_pos - last_pos;

    velocity = velocity / time_diff;
    float speed = velocity.norm();
    if (kDebug) {
      std::cout << "Current pos: " << current_pos << std::endl;
      std::cout << "last_pos: " << last_pos << std::endl;
      std::cout << "time_diff: " << time_diff << std::endl;
      std::cout << "speed: " << speed << std::endl;
    }
    gui::DataEntry entry(debug_message_.time(), current_pos, 0, speed, velocity,
                         0);
    return entry;
  }
  return gui::DataEntry();
}

ReadLogger::ReadLogger(const string& filename) {
  read_file_.open(filename, std::ios::in | std::ios::binary);
  //   BuildIndex();
  read_file_.seekg(0, read_file_.beg);
  pause_ = false;
}

MinuteBotsProto::LogWrapper ReadLogger::GetNextMessage() {
  MinuteBotsProto::LogWrapper message;

  // get length of file (problematic when file being written to?)
  const int current = read_file_.tellg();
  read_file_.seekg(0, read_file_.end);
  const int length = read_file_.tellg();
  read_file_.seekg(current, read_file_.beg);

  if (length != current) {
    int message_size = 0;
    read_file_.read(reinterpret_cast<char*>(&message_size), sizeof(int));
    const int kMessageSize = message_size;
    char temp_message[kMessageSize];
    read_file_.read(temp_message, kMessageSize);
    bool success = message.ParseFromArray(temp_message, kMessageSize);
    read_file_.read(reinterpret_cast<char*>(&message_size), sizeof(int));
    if (!success) {
      message.Clear();
      *(message.mutable_soccer_debug()) = last_message_;
    } else {
      last_message_size_ = message_size;
      last_message_ = message.soccer_debug();
    }
  } else {
    message.Clear();
    *(message.mutable_soccer_debug()) = last_message_;
  }
  kick_data_index++;
  debug_message_ = message.soccer_debug();
  return message;
}

MinuteBotsProto::LogWrapper ReadLogger::GetPreviousMessage() {
  MinuteBotsProto::LogWrapper message;
  const int index = read_file_.tellg();
  const int size = last_message_size_ + (sizeof(int) * 3);
  if (index > size) {
    // Seek to above the size for the message we want to read
    read_file_.seekg(-sizeof(int), std::ios::cur);
    read_file_.seekg(-last_message_size_, std::ios::cur);
    read_file_.seekg(-sizeof(int) * 2, std::ios::cur);
    // Find the size of the message we want to read
    int message_size = 0;
    read_file_.read(reinterpret_cast<char*>(&message_size), sizeof(int));
    // Seek to above the message and read it in.
    read_file_.seekg(-sizeof(int), std::ios::cur);
    read_file_.seekg(-message_size, std::ios::cur);
    const int kMessageSize = message_size;
    char temp_message[kMessageSize];
    read_file_.read(temp_message, kMessageSize);
    bool success = message.ParseFromArray(temp_message, kMessageSize);
    // Set the last_message_size_ equal to the size of the message we just read.
    read_file_.read(reinterpret_cast<char*>(&message_size), sizeof(int));
    last_message_size_ = message_size;
    if (!success) {
      message.Clear();
      *(message.mutable_soccer_debug()) = last_message_;
    }
  } else {
    std::cout << "Index smaller than message size" << std::endl;
    std::cout << "Index: " << index << " Message size: " << size << std::endl;
    message = GetNextMessage();
  }
  kick_data_index--;
  return message;
}

void ReadLogger::Pause() { pause_ = true; }

void ReadLogger::Live() {
  pause_ = false;
//   int message_size = 0;
//   read_file_.seekg(0, std::ios::end);
//   read_file_.seekg(-sizeof(int), std::ios::cur);
//   read_file_.read(reinterpret_cast<char*>(&message_size), sizeof(int));
//   last_message_size_ = message_size;
//   read_file_.seekg(0, std::ios::end);
}

void Logger::LogRobotDataEntry(const SSLVisionId& id) {
  gui::DataEntry entry = RobotDataEntry(id);
  LogPrint("Observations for Robot %d", id);
  Push();
  LogPrint("Time %f", entry.t);
  LogPrint("Pose x: %f, y: %f, theta %f", entry.loc.x(), entry.loc.y(),
           entry.angle);
  LogPrint("Velocity x: %f, y: %f, theta %f", entry.vel.x(), entry.vel.y(),
           entry.omega);
  LogPrint("Speed: %f", entry.speed);
  Pop();
}

KickData::KickData(const SoccerDebugMessage& message, const int& robot_number) {
  float target_angle = 0;
  target_angle = message.world_state().robots(robot_number).target_angle();
  Pose2Df robot_goal_pose(0, 0, 0);
  robot_goal_pose.translation.x() =
      message.world_state().robots(robot_number).goal_pose().loc().x();
  robot_goal_pose.translation.y() =
      message.world_state().robots(robot_number).goal_pose().loc().y();
  robot_goal_pose.angle =
      message.world_state().robots(robot_number).goal_pose().angle();
  Vector2f current_ball_pose(0, 0);
  current_ball_pose.x() = message.world_state().ball_observation(0).x();
  current_ball_pose.y() = message.world_state().ball_observation(0).y();
  Vector2f current_ball_velocity(0, 0);
  current_ball_velocity.x() = message.world_state().ball_velocities(0).x();
  current_ball_velocity.y() = message.world_state().ball_velocities(0).y();
  Pose2Df current_robot_pose(0, 0, 0);
  current_robot_pose.translation.x() =
      message.world_state().robots(robot_number).pose_raw().loc().x();
  current_robot_pose.translation.y() =
      message.world_state().robots(robot_number).pose_raw().loc().y();
  current_robot_pose.angle =
      message.world_state().robots(robot_number).pose_raw().angle();
  Pose2Df current_robot_velocity(0, 0, 0);
  current_robot_velocity.translation.x() =
      message.world_state().robots(0).velocity().x();
  current_robot_velocity.translation.y() =
      message.world_state().robots(0).velocity().y();
  current_robot_velocity.angle =
      message.world_state().robots(0).rotational_velocity();

  const Eigen::Rotation2Df robot_to_world_rotation(current_robot_pose.angle);
  const Eigen::Rotation2Df world_to_target_rotation(-target_angle);
  const Eigen::Rotation2Df robot_to_target_rotation =
      world_to_target_rotation * robot_to_world_rotation;

  const Vector2f current_velocity_world =
      robot_to_world_rotation * current_robot_velocity.translation;

  const Vector2f robot_prime_vel =
      robot_to_target_rotation * current_robot_velocity.translation;

  const Vector2f robot_to_ball_displace =
      current_ball_pose - current_robot_pose.translation;

  Vector2f desired_norm(cos(robot_goal_pose.angle), sin(robot_goal_pose.angle));
  // the balls center should be at (kRobotRadius + kBallRadius) * desired_norm
  // relative to the robot's center at collision time
  Vector2f collision_point =
      current_robot_pose.translation + kInterceptionRadius * desired_norm;
  const float robot_heading = current_robot_pose.angle;
  const Vector2f y_dir(-sin(robot_heading), cos(robot_heading));

  float y_dist = y_dir.dot(robot_to_ball_displace);
  float robot_y_prime_vel = robot_prime_vel.y();
  float ball_y_prime_vel =
      (world_to_target_rotation * current_ball_velocity).y();
  float rel_y_prime_vel = robot_y_prime_vel - ball_y_prime_vel;

  const Vector2f relative_velocity_vector =
      current_velocity_world - current_ball_velocity;

  angle_error = fabs(AngleDiff(target_angle, current_robot_pose.angle));
  radial_dist = (collision_point - current_ball_pose).norm();
  y_prime_velocity = fabs(rel_y_prime_vel);
  relative_velocity = (relative_velocity_vector).norm();
  align_error = fabs(y_dist);
  rotational_velocity = fabs(current_robot_velocity.angle);

  float thresholds_angle_ = 10;
  float thresholds_distance_ = 120;
  float thresholds_align_ = 1;
  float thresholds_y_prime_vel_ = 300;
  float thresholds_angular_vel_ = 30;
  float thresholds_relative_vel_ = 300;

  const bool is_at_angle = angle_error < thresholds_angle_;
  const bool is_at_radial_dist = radial_dist < thresholds_distance_;
  const bool is_y_prime_at_relative_rest =
      y_prime_velocity < thresholds_y_prime_vel_;
  const bool is_at_relative_rest = relative_velocity < thresholds_relative_vel_;
  const bool is_in_alignment = align_error < thresholds_align_;
  const bool is_rotation_at_rest =
      rotational_velocity < thresholds_angular_vel_;
  const bool should_kick =
      (is_at_angle && is_at_radial_dist && is_at_relative_rest &&
       is_in_alignment && is_rotation_at_rest && is_y_prime_at_relative_rest);

  kick = should_kick;
}

void ReadLogger::SetTransition(const int& index, const int& transition) {
  if (transition != -1) {
    SoccerDebugMessage* tuning_message = &tuning_index_[index];
    int count = transition;
    for (int i = 0; i < tuning_message->tuning_data_size(); ++i) {
      StateMachineData* data = tuning_message->mutable_tuning_data(i);
      for (int j = 0; j < data->transitions_size(); ++j) {
        PossibleTransition* transition = data->mutable_transitions(j);
        if (count == 0) {
          std::cout << "Transition Set Successfully" << std::endl;
          transition->set_human_constraint(true);
          transition->set_should_transition(true);
        }
        count--;
      }
    }
  }
}

void ReadLogger::UnsetTransition(const int& index,
                                 const int& transition) {
  if (transition != -1) {
    SoccerDebugMessage* tuning_message = &tuning_index_[index];
    int count = transition;
    for (int i = 0; i < tuning_message->tuning_data_size(); ++i) {
      StateMachineData* data = tuning_message->mutable_tuning_data(i);
      std::cout << data->transitions_size() << std::endl;
      for (int j = 0; j < data->transitions_size(); ++j) {
        PossibleTransition* transition = data->mutable_transitions(j);
        if (count == 0) {
          transition->set_human_constraint(true);
          transition->set_should_transition(false);
        }
        count--;
      }
    }
  }
}

// TODO(jaholtz) figure out how to shoehorn the docker into this function.
// State machines could have their own 'WriteContinue' methods...?
void WriteContinueMode(const PossibleTransition& transition,
                       const SoccerDebugMessage& soccer_message) {
  // Load the attacker_config json file
  std::ifstream json_file("src/configs/attacker_config.json");
  nlohmann::json config_json;
  json_file >> config_json;
  json_file.close();
  // Add continue information to it
  config_json["initial_state"] = transition.start_state();
  config_json["continue_state"] = transition.potential_state();
  // Write it back to a file
  std::ofstream output_json("src/configs/attacker_config.json");
  output_json << std::setw(4) << config_json << std::endl;
  output_json.close();
  // Open a new json file
  nlohmann::json world_json;

  // Add the parameters of the current world state to it
//   for (auto block : transition.blocks()) {
//     for (auto clause : block.clauses()) {
//       world_json[clause.rhs()] = clause.lhs();
//     }
//   }

  MinuteBotsProto::WorldState world = soccer_message.world_state();
  // TODO(jaholtz) identify which robot to get this information for, or
  // get information for all robots
  world_json["robot_x"] = world.robots(0).pose().loc().x();
  world_json["robot_y"] = world.robots(0).pose().loc().y();
  world_json["robot_a"] = world.robots(0).pose().angle();
  world_json["robot_vx"] = world.robots(0).velocity().x();
  world_json["robot_vy"] = world.robots(0).velocity().y();
  world_json["robot_va"] = world.robots(0).rotational_velocity();

  // Get the ball information (assuming only one ball)
  world_json["ball_x"] = world.balls(0).x();
  world_json["ball_y"] = world.balls(0).y();
  world_json["ball_vx"] = world.ball_velocities(0).x();
  world_json["ball_vy"] = world.ball_velocities(0).y();

  // Write it to a file.
  std::ofstream world_output_json("src/configs/initial_state.json");
  world_output_json << std::setw(4) << world_json << std::endl;
  world_output_json.close();
}

void ReadLogger::ContinueTransition(const int& index,
                                    const int& transition) {
  if (transition != -1) {
    SoccerDebugMessage* tuning_message = &tuning_index_[index];
    int count = transition;
    for (int i = 0; i < tuning_message->tuning_data_size(); ++i) {
      StateMachineData* data = tuning_message->mutable_tuning_data(i);
      std::cout << data->transitions_size() << std::endl;
      for (int j = 0; j < data->transitions_size(); ++j) {
        PossibleTransition* transition = data->mutable_transitions(j);
        if (count == 0) {
          transition->set_human_constraint(true);
          transition->set_should_transition(false);
          WriteContinueMode(*transition, *tuning_message);
        }
        count--;
      }
    }
  }
}

void ReadLogger::EndFile(const int& index) {
  // Create a trace file.
  Trace trace;
  // Fill the trace
  int count = 0;
  for (auto debug_message : tuning_index_) {
    if (count <= index) {
      for (int i = 0; i < debug_message.second.tuning_data_size(); ++i) {
        *(trace.add_trace_elements()) =
            debug_message.second.tuning_data(i);
      }
    }
    count++;
  }
  // Write that trace to a file.
  std::ofstream trace_file;
  trace_file.open("end_file_trace.txt");
  google::protobuf::io::OstreamOutputStream output_stream(&trace_file);
  string trace_string;
  google::protobuf::TextFormat::PrintToString(trace, &trace_string);
  trace_file << trace_string << std::endl;
  trace_file.close();
  std::cout << "Trace File Written" << std::endl;
}

void Logger::SetTeam(const bool& team_yellow) {
  debug_message_.set_team_yellow(team_yellow);
}

void ReadLogger::BuildKickTuningIndex() {
  // get length of file
  int current = read_file_.tellg();
  read_file_.seekg(0, read_file_.end);
  const int length = read_file_.tellg();
  read_file_.seekg(current, read_file_.beg);
  while (current != length) {
    GetNextMessage();
    current = read_file_.tellg();
    if (debug_message_.world_state().robots_size() > 0) {
      const SoccerDebugMessage current_message = ReturnMessage();
      KickData current_data(current_message, 0);
      kick_data_.push_back(current_data);
    }
  }
  read_file_.seekg(0, read_file_.beg);
  read_index_ = 0;
  kick_data_index = 0;
  std::cout << "Kick data" << kick_data_.size() << std::endl;
}

void ReadLogger::BuildTuningIndex() {
  // Set the machine we are tuning
  // TODO(jaholtz) make this include the state transition we are tuning.
  // get length of file
  int current = read_file_.tellg();
  read_file_.seekg(0, read_file_.end);
  const int length = read_file_.tellg();
  read_file_.seekg(current, read_file_.beg);
  std::map<int, SoccerDebugMessage> data;
  int count = 0;
  while (current != length) {
    GetNextMessage();
    current = read_file_.tellg();
    // This currently assumes there is only one instance of the given
    // state machine in the data file. This is not necessarily accurate,
    // and should be adjusted.
//     std::cout << debug_message_.tuning_data_size() << std::endl;
    data[count] = debug_message_;
    count++;
  }
  tuning_index_ = data;
  read_file_.seekg(0, read_file_.beg);
  read_index_ = 0;
  kick_data_index = 0;
}

void ReadLogger::BuildIndex() {
  std::cout << "Building Index" << std::endl;
  // get length of file
  int current = read_file_.tellg();
  read_file_.seekg(0, read_file_.end);
  const int length = read_file_.tellg();
  read_file_.seekg(current, read_file_.beg);
  int count = 0;
  while (current != length) {
    count_file_map_[count] = current;
    GetNextMessage();
    current = read_file_.tellg();
    count++;
  }
  read_file_.seekg(0, read_file_.beg);
  read_index_ = 0;
  std::cout << "Built Index" << std::endl;
}

void ReadLogger::AddIndexEntry(const int& file_index) {
  count_file_map_[count_file_map_.size()] = file_index;
}

void ReadLogger::GoToIndex(const int& file_index) {
  if (uint(file_index) < count_file_map_.size()) {
    read_file_.seekg(count_file_map_[file_index], read_file_.beg);
  }
}

int ReadLogger::GetMapSize() {
  return count_file_map_.size();
}

int WriteLogger::GetFileSize() {
  // get length of file
  int current = file_.tellg();
  file_.seekg(0, file_.end);
  const int length = file_.tellg();
  file_.seekg(current, file_.beg);
  file_.seekg(0, file_.beg);
  return length;
}

void ReadLogger::WriteKickTuningData() {
  std::ofstream kick_data_file;
  kick_data_file.open("kick_tuning_data.txt");
  for (const auto& data : tuning_index_) {
    LogWrapper wrapper;
    *wrapper.mutable_soccer_debug() = data.second;
    int size = wrapper.ByteSize();
    kick_data_file.write(reinterpret_cast<char*>(&size), sizeof(int));
    wrapper.SerializeToOstream(&kick_data_file);
    kick_data_file.write(reinterpret_cast<char*>(&size), sizeof(int));
    kick_data_file.flush();
  }
  kick_data_file.close();
}

void Logger::LogMessageStats(int received, int dropped) {
  LogPrint("Messages Received: %d, Dropped: %d, Ratio: %f", received, dropped,
           static_cast<float>(dropped) / static_cast<float>(received));
}

// Merges the text logs and the drawings from the two loggers, adding the
// data from the input logger to this logger. Does not merge world state or
// ssl information.
void Logger::MergeLoggers(Logger input_logger) {
  SoccerDebugMessage input_message = input_logger.ReturnMessage();
  //   PrintLog input_logs = input_message.print_logs();
  if (print_tree_.size() > 0) {
    PrintLog* logs = print_tree_.back();
    for (int i = 0; i < input_message.print_logs().size(); ++i) {
      logs->add_nested();
      (*(logs->mutable_nested(logs->nested_size() - 1)))
          .CopyFrom(input_message.print_logs(i));
    }
  } else {
    for (int i = 0; i < input_message.print_logs_size(); ++i) {
      debug_message_.add_print_logs();
      (*(debug_message_.mutable_print_logs(debug_message_.print_logs_size() -
                                           1)))
          .CopyFrom(input_message.print_logs(i));
    }
  }

  if (input_message.has_drawings()) {
    DebugDrawings* drawings = debug_message_.mutable_drawings();
    const DebugDrawings& input_drawings = input_message.drawings();
    for (int i = 0; i < input_drawings.lines_size(); ++i) {
      *(drawings->add_lines()) = input_drawings.lines(i);
    }
    for (int i = 0; i < input_drawings.ellipses_size(); ++i) {
      *(drawings->add_ellipses()) = input_drawings.ellipses(i);
    }
    for (int i = 0; i < input_drawings.arcs_size(); ++i) {
      *(drawings->add_arcs()) = input_drawings.arcs(i);
    }
    for (int i = 0; i < input_drawings.points_size(); ++i) {
      *(drawings->add_points()) = input_drawings.points(i);
    }
  }

  for (int i = 0; i < input_message.tuning_data_size(); ++i) {
    AddStateMachineData(input_message.tuning_data(i));
  }
}

void Logger::SetWorldState(MinuteBotsProto::WorldState world_state) {
  *(debug_message_.mutable_world_state()) = world_state;
}

void Logger::SetWorldState(Logger input_logger) {
  SoccerDebugMessage input_message = input_logger.ReturnMessage();
  *(debug_message_.mutable_world_state()) = input_message.world_state();
}

void Logger::SetRobotTargets(const Pose2Df& goal_pose,
                             const float& target_angle, const int& robot_id) {
  MinuteBotsProto::RobotState* robot =
      debug_message_.mutable_world_state()->mutable_robots(robot_id);
  robot->mutable_goal_pose()->mutable_loc()->set_x(goal_pose.translation.x());
  robot->mutable_goal_pose()->mutable_loc()->set_y(goal_pose.translation.y());
  robot->mutable_goal_pose()->set_angle(goal_pose.angle);
  robot->set_target_angle(target_angle);
}

void Logger::SetWorldState(const state::WorldState& world_state) {
  MinuteBotsProto::WorldState world_state_proto;
  // Set the ball positions

  MinuteBotsProto::Vector2f ball, ball_observation, ball_velocity,
      ball_velocity_observed;
  ball.set_x(world_state.GetBallPosition().position.x());
  ball.set_y(world_state.GetBallPosition().position.y());
  ball_observation.set_x(world_state.GetBallPosition().observed_pose.x());
  ball_observation.set_y(world_state.GetBallPosition().observed_pose.y());
  ball_velocity.set_x(world_state.GetBallPosition().velocity.x());
  ball_velocity.set_y(world_state.GetBallPosition().velocity.y());
  const double time_diff =
      world_state.GetWorldTime() - world_state.GetLastWorldTime();
  Eigen::Vector2f observed_ball_vel =
      world_state.GetBallPosition().velocity -
      world_state.GetLastBallPosition().velocity;
  observed_ball_vel = observed_ball_vel / time_diff;
  ball_velocity_observed.set_x(observed_ball_vel.x());
  ball_velocity_observed.set_y(observed_ball_vel.y());

  *(world_state_proto.add_balls()) = ball;
  *(world_state_proto.add_ball_observation()) = ball_observation;
  *(world_state_proto.add_ball_velocities()) = ball_velocity;
  *(world_state_proto.add_ball_velocities_observed()) = ball_velocity;
  world_state_proto.set_ball_time(world_state.GetBallPosition().observed_time);
  const auto our_team_proto_enum =
      (world_state.GetOurTeam() == team::Team::YELLOW)
          ? MinuteBotsProto::RobotState_Team_TEAM_YELLOW
          : MinuteBotsProto::RobotState_Team_TEAM_BLUE;

  const auto their_team_proto_enum =
      (world_state.GetOurTeam() == team::Team::YELLOW)
          ? MinuteBotsProto::RobotState_Team_TEAM_BLUE
          : MinuteBotsProto::RobotState_Team_TEAM_YELLOW;

  for (const auto& our_robot :
       world_state.GetPositionVelocityState().GetOurTeamRobots()) {
    RobotState bot_state;
    bot_state.set_robot_id(our_robot.ssl_vision_id);
    bot_state.set_team(our_team_proto_enum);
    const Pose2Df& bot_pose = our_robot.position;
    bot_state.mutable_pose()->mutable_loc()->set_x(bot_pose.translation.x());
    bot_state.mutable_pose()->mutable_loc()->set_y(bot_pose.translation.y());
    bot_state.mutable_pose()->set_angle(bot_pose.angle);

    const Pose2Df& observed_pose = our_robot.observed_pose;
    bot_state.mutable_pose_raw()->mutable_loc()->set_x(
        observed_pose.translation.x());
    bot_state.mutable_pose_raw()->mutable_loc()->set_y(
        observed_pose.translation.y());
    bot_state.mutable_pose_raw()->set_angle(observed_pose.angle);

    bot_state.set_ssl_time(our_robot.observed_time);
    const Pose2Df& bot_velocity = our_robot.velocity;
    bot_state.mutable_velocity()->set_x(bot_velocity.translation.x());
    bot_state.mutable_velocity()->set_y(bot_velocity.translation.y());
    bot_state.set_rotational_velocity(bot_velocity.angle);

    const Pose2Df& obs_vel = our_robot.observed_velocity;
    bot_state.mutable_observed_velocity()->set_x(obs_vel.translation.x());
    bot_state.mutable_observed_velocity()->set_y(obs_vel.translation.y());
    bot_state.set_observed_rotational_velocity(obs_vel.angle);

    *(world_state_proto.add_robots()) = bot_state;
  }

  for (const auto& their_robot :
       world_state.GetPositionVelocityState().GetTheirTeamRobots()) {
    RobotState bot_state;
    bot_state.set_robot_id(their_robot.ssl_vision_id);
    bot_state.set_team(their_team_proto_enum);
    const Pose2Df& bot_pose = their_robot.position;
    bot_state.mutable_pose()->mutable_loc()->set_x(bot_pose.translation.x());
    bot_state.mutable_pose()->mutable_loc()->set_y(bot_pose.translation.y());
    bot_state.mutable_pose()->set_angle(bot_pose.angle);

    const Pose2Df& observed_pose = their_robot.observed_pose;
    bot_state.mutable_pose_raw()->mutable_loc()->set_x(
        observed_pose.translation.x());
    bot_state.mutable_pose_raw()->mutable_loc()->set_y(
        observed_pose.translation.y());
    bot_state.mutable_pose_raw()->set_angle(observed_pose.angle);

    bot_state.set_ssl_time(their_robot.observed_time);

    *(world_state_proto.add_robots()) = bot_state;
  }
  // Assign our world state the value of this newly created world_state_proto
  *(debug_message_.mutable_world_state()) = world_state_proto;
}

void Logger::AddStateMachineData(const StateMachineData& data) {
  *(debug_message_.add_tuning_data()) = data;
}


void Logger::Push() {
  if (print_tree_.size() > 0) {
    PrintLog* head = print_tree_.back();
    if (head->nested_size() > 0) {
      PrintLog* new_head = head->mutable_nested(head->nested_size() - 1);
      print_tree_.push_back(new_head);
    }
  } else {
    if (debug_message_.print_logs_size() > 0) {
      PrintLog* new_head = debug_message_.mutable_print_logs(
          debug_message_.print_logs_size() - 1);
      print_tree_.push_back(new_head);
    }
  }
}

void Logger::Push(const string& header) {
  LogPrint(header);
  Push();
}


void Logger::Pop() {
  if (print_tree_.size() >= 1) {
    print_tree_.pop_back();
  }
}

void Logger::Clear() {
  debug_message_.Clear();
  print_tree_.clear();
}

void Logger::AddLine(const Vector2f& p1, const Vector2f& p2, const float& r,
                     const float& g, const float& b, const float& a) {
  AddLine(p1.x(), p1.y(), p2.x(), p2.y(), r, g, b, a);
}

void Logger::AddLine(const float& x1, const float& y1, const float& x2,
                     const float& y2, const float& r, const float& g,
                     const float& b, const float& a) {
  MinuteBotsProto::DebugDrawings& drawings = *debug_message_.mutable_drawings();
  MinuteBotsProto::ColoredLine& line = *drawings.add_lines();
  line.mutable_value()->mutable_v0()->set_x(x1);
  line.mutable_value()->mutable_v0()->set_y(y1);
  line.mutable_value()->mutable_v1()->set_x(x2);
  line.mutable_value()->mutable_v1()->set_y(y2);
  line.mutable_color()->set_r(r);
  line.mutable_color()->set_g(g);
  line.mutable_color()->set_b(b);
  line.mutable_color()->set_a(a);
}

void Logger::AddCircle(const Eigen::Vector2f p, const float& radius,
                       const float& r, const float& g, const float& b,
                       const float& a) {
  AddArc(p, radius, 0, 2 * M_PI, r, g, b, a);
}

void Logger::AddArc(const Eigen::Vector2f p, const float& radius,
                    const float& angle_start, const float& angle_end,
                    const float& r, const float& g, const float& b,
                    const float& a) {
  AddArc(p.x(), p.y(), radius, angle_start, angle_end, r, g, b, a);
}

void Logger::AddEllipse(const Eigen::Vector2f p, const float& radius,
                        const float& radius_2, const float& angle,
                        const float& r, const float& g, const float& b,
                        const float& a) {
  MinuteBotsProto::DebugDrawings& drawings = *debug_message_.mutable_drawings();
  MinuteBotsProto::ColoredEllipse& ellipse = *drawings.add_ellipses();
  ellipse.mutable_center()->set_x(p.x());
  ellipse.mutable_center()->set_y(p.y());
  ellipse.set_radius_1(radius);
  ellipse.set_radius_2(radius_2);
  ellipse.set_angle(angle);
  ellipse.mutable_color()->set_r(r);
  ellipse.mutable_color()->set_g(g);
  ellipse.mutable_color()->set_b(b);
  ellipse.mutable_color()->set_a(a);
}

void Logger::AddArc(const float& x, const float& y, const float& radius,
                    const float& angle_start, const float& angle_end,
                    const float& r, const float& g, const float& b,
                    const float& a) {
  MinuteBotsProto::DebugDrawings& drawings = *debug_message_.mutable_drawings();
  MinuteBotsProto::ColoredArc& arc = *drawings.add_arcs();
  arc.mutable_center()->set_x(x);
  arc.mutable_center()->set_y(y);
  arc.set_radius(radius);
  arc.set_angle_start(angle_start);
  arc.set_angle_end(angle_end);
  arc.mutable_color()->set_r(r);
  arc.mutable_color()->set_g(g);
  arc.mutable_color()->set_b(b);
  arc.mutable_color()->set_a(a);
}

void Logger::AddPoint(const float& x, const float& y, const float& r,
                      const float& g, const float& b, const float& a) {
  MinuteBotsProto::DebugDrawings& drawings = *debug_message_.mutable_drawings();
  MinuteBotsProto::ColoredPoint& p = *drawings.add_points();
  p.mutable_value()->set_x(x);
  p.mutable_value()->set_y(y);
  p.mutable_color()->set_r(r);
  p.mutable_color()->set_g(g);
  p.mutable_color()->set_b(b);
  p.mutable_color()->set_a(a);
}

void Logger::AddPoint(const Vector2f& p,
                      const float& r,
                      const float& g,
                      const float& b,
                      const float& a) {
  AddPoint(p.x(), p.y(), r, g, b, a);
}

void Logger::AddPath(const vector<Vector2f>& points,
                     const float& r,
                     const float& g,
                     const float& b,
                     const float& a) {
  for (unsigned int i = 0; i + 1 < points.size(); i++) {
    const auto& p0 = points[i];
    const auto& p1 = points[i + 1];
    AddLine(p0.x(), p0.y(), p1.x(), p1.y(), r, g, b, a);
  }
}

void Logger::AddPoints(const std::vector<Eigen::Vector2f>& points,
                       const float& r,
                       const float& g,
                       const float& b,
                       const float& a) {
  for (const Vector2f& p : points) {
    AddPoint(p, r, g, b, a);
  }
}

void Logger::UpdateWorldState(const MinuteBotsProto::WorldState& world_state) {
  *(debug_message_.mutable_world_state()) = world_state;
}

void Logger::AddRobot(const int& id,
                      const MinuteBotsProto::RobotState::Team& team,
                      const float& theta, const float& x, const float& y,
                      const float& confidence) {
  MinuteBotsProto::RobotState& robot =
      *(debug_message_.mutable_world_state()->add_robots());
  robot.set_robot_id(id);
  robot.set_team(team);
  MinuteBotsProto::Pose2Df& pose = *robot.mutable_pose();
  pose.set_angle(theta);
  pose.mutable_loc()->set_x(x);
  pose.mutable_loc()->set_y(y);
  robot.set_confidence(confidence);
}

void Logger::AddBall(const float& x, const float& y) {
  MinuteBotsProto::Vector2f* ball =
      debug_message_.mutable_world_state()->add_balls();
  ball->set_x(x);
  ball->set_y(y);
}

void NetLogger::SendData() {
  if (server_.IsOpen()) {
    debug_message_.set_frame_number(count_);
    count_++;
    server_.SendProtobuf(debug_message_);
  }
}

void Logger::SetMessage(SoccerDebugMessage message) {
  last_message_ = debug_message_;
  debug_message_ = message;
}

void Logger::SetMessageTime(const double time) {
  debug_message_.set_time(time);
}

void Logger::LogRefState(state::RefereeState ref_state) {
  if (ref_state.HasMessage()) {
    MinuteBotsProto::SSL_Referee ref_message = ref_state.GetRefereeMessage();
    *(debug_message_.mutable_last_referee()) = ref_message;
  }
  static std::vector<std::string> StageStrings = {"normal_first_half_pre",
                                                  "normal_first_half",
                                                  "normal_half_time",
                                                  "normal_second_half_pre",
                                                  "normal_second_half",
                                                  "extra_time_break",
                                                  "extra_first_half_pre",
                                                  "extra_first_half",
                                                  "extra_half_time",
                                                  "extra_second_half_pre",
                                                  "extra_second_half",
                                                  "penalty_shootout_break",
                                                  "penalty_shootout",
                                                  "post_game"};

  static std::vector<std::string> CommandStrings = {"halt",
                                                    "stop",
                                                    "normal_start",
                                                    "force_start",
                                                    "prepare_kickoff_yellow",
                                                    "prepare_kickoff_blue",
                                                    "prepare_penalty_yellow",
                                                    "prepare_penalty_blue",
                                                    "direct_free_yellow",
                                                    "direct_free_blue",
                                                    "indirect_free_yellow",
                                                    "indirect_free_blue",
                                                    "timeout_yellow",
                                                    "timeout_blue",
                                                    "goal_yellow",
                                                    "goal_blue",
                                                    "ball_placement_yellow",
                                                    "ball_placement_blue"};
  Push();
  LogPrint("Referee State");
  Push();
  LogPrint("Stage: %s", StageStrings[ref_state.GetGameStage()]);
  LogPrint("Command: %s", CommandStrings[ref_state.GetRefCommand()]);
  Push();
  LogPrint("Command Timestamp: %d", ref_state.GetCommandTimeStamp());
  LogPrint("Commands Issued: %d", ref_state.GetNumCommandsIssued());
  Pop();
  LogPrint("MinuteBots");
  Push();
  LogPrint("Score: %d", ref_state.GetOurScore());
  LogPrint("Red Cards: %d", ref_state.GetOurRedCards());
  LogPrint("Yellow Cards: %d", ref_state.GetOurYellowCards());
  if (ref_state.GetOurActiveYellowCards() > 0) {
    LogPrint(
        "Yellow Card Time Remaining: %d",
        ref_state.GetOurYellowCardTime(ref_state.GetOurActiveYellowCards()));
  }
  LogPrint("Timeouts: %d", ref_state.GetOurTimeouts());
  LogPrint("Timeout time: %d microseconds",
           ref_state.GetOurTimeoutMicroSeconds());
  LogPrint("Goalie: %d", ref_state.GetOurGoaliePattern());
  Pop();
  LogPrint("Opponents");
  Push();
  LogPrint("Score: %d", ref_state.GetTheirScore());
  LogPrint("Red Cards: %d", ref_state.GetTheirRedCards());
  LogPrint("Yellow Cards: %d", ref_state.GetTheirYellowCards());
  if (ref_state.GetTheirActiveYellowCards() > 0) {
    LogPrint("Yellow Card Time Remaining: %d",
             ref_state.GetTheirYellowCardTime(
                 ref_state.GetTheirActiveYellowCards()));
  }
  LogPrint("Timeouts: %d", ref_state.GetTheirTimeouts());
  LogPrint("Timeout time: %d microseconds",
           ref_state.GetTheirTimeoutMicroSeconds());
  LogPrint("Goalie: %d", ref_state.GetTheirGoaliePattern());
  Pop();
  if (ref_state.HasDesignatedPosition()) {
    LogPrint("Designated Position x: %f, y: %f",
             ref_state.GetDesignatedPostion().x(),
             ref_state.GetDesignatedPostion().y());
  }
  Pop();
  Pop();
}

SoccerDebugMessage Logger::ReturnMessage() { return debug_message_; }

void TestRe() {
  const string format = "float: %f";
  string current = "empty";
  re2::StringPiece rest(format);
  // Regex for parsing out possible format specificers.
  const string re = "((?U).*%(?U).*(d|i|u|s|o|x|X|f|F|e|E|g|G|a|A|c|s|p|n|%))";
  while (re2::RE2::Consume(&rest, re, &current)) {
    std::cout << current << std::endl;
    std::cout << rest.as_string() << std::endl;
  }
}

void BuildString(const PrintLog& p_log, string* print_string) {
  const string format = p_log.format_str();
  *print_string = "";
  const string delimiter = "%";
  const char perc = '%';
  string current = "eleven";
  re2::StringPiece rest(format);
  // Get the type name of a string and demangle it.
  string type_name = typeid(string).name();
  string demangled_name = demangle(type_name.c_str());
  int i = 0;
  // Regex for parsing out possible format specificers.

  re_compile.ok();
  while (re2::RE2::Consume(&rest, re_compile, &current)) {
    const char last = current.back();
    // We don't want to parse any data if our format specificer is %%.
    if (i < p_log.data_size() && last != perc) {
      string temp = p_log.data(i);
      // integer values
      if (last == 'd') {
        int x = 0;
        memcpy(&x, &temp[0], sizeof(int));
        // This block is reused in each type with different values
        // Used to get the size of the string to print.
        size_t size = std::snprintf(nullptr, 0, current.c_str(), x) + 1;
        // Create a buffer to use for printing.
        std::unique_ptr<char[]> buf(new char[size]);
        // print into this buffer with the current format string.
        std::snprintf(buf.get(), size, current.c_str(), x);
        // Retrieve the string from the buffer.
        string temp_string = string(buf.get(), buf.get() + size - 1);
        // Add to the end of our print string.
        *print_string += temp_string;
      } else if (strcmp(p_log.type_list(i).c_str(), "double") == 0) {
        double x = 0;
        memcpy(&x, &temp[0], sizeof(x));
        size_t size = std::snprintf(nullptr, 0, current.c_str(), x) + 1;
        std::unique_ptr<char[]> buf(new char[size]);
        std::snprintf(buf.get(), size, current.c_str(), x);
        string temp_string = string(buf.get(), buf.get() + size - 1);
        *print_string += temp_string;
      } else if (strcmp(p_log.type_list(i).c_str(), "float") == 0) {
        float x = 0;
        memcpy(&x, &temp[0], sizeof(x));
        size_t size = std::snprintf(nullptr, 0, current.c_str(), x) + 1;
        std::unique_ptr<char[]> buf(new char[size]);
        std::snprintf(buf.get(), size, current.c_str(), x);
        string temp_string = string(buf.get(), buf.get() + size - 1);
        *print_string += temp_string;
      } else if (strcmp(p_log.type_list(i).c_str(), demangled_name.c_str()) ==
                 0) {
        std::string x = temp;
        size_t size = std::snprintf(nullptr, 0, current.c_str(), x.c_str()) + 1;
        std::unique_ptr<char[]> buf(new char[size]);
        std::snprintf(buf.get(), size, current.c_str(), x.c_str());
        string temp_string = string(buf.get(), buf.get() + size - 1);
        *print_string += temp_string;
      } else if (strcmp(p_log.type_list(i).c_str(), "char const*") == 0) {
        std::string x = temp;
        size_t size = std::snprintf(nullptr, 0, current.c_str(), x.c_str()) + 1;
        std::unique_ptr<char[]> buf(new char[size]);
        std::snprintf(buf.get(), size, current.c_str(), x.c_str());
        string temp_string = string(buf.get(), buf.get() + size - 1);
        *print_string += temp_string;
      }
      ++i;
    } else {
      string x = "";
      size_t size = std::snprintf(nullptr, 0, current.c_str(), x.c_str()) + 1;
      std::unique_ptr<char[]> buf(new char[size]);
      std::snprintf(buf.get(), size, current.c_str(), x.c_str());
      string temp_string = string(buf.get(), buf.get() + size - 1);
      *print_string += temp_string;
    }
  }
  if (rest.length() != 0) {
    string x = "";
    size_t size =
        std::snprintf(nullptr, 0, rest.ToString().c_str(), x.c_str()) + 1;
    std::unique_ptr<char[]> buf(new char[size]);
    std::snprintf(buf.get(), size, rest.ToString().c_str(), x.c_str());
    string temp_string = string(buf.get(), buf.get() + size - 1);
    *print_string += temp_string;
  }
}

void BuildTextLog(const PrintLog& p_logs, TextTree* log);

TextTree BuildTextTree(const PrintLog& p_log) {
  string log_string;
  BuildString(p_log, &log_string);
  //   log_string = "test";
  TextTree sub_log;
  sub_log.set_text(log_string);
  for (int i = 0; i < p_log.nested_size(); ++i) {
    PrintLog nested_logs = p_log.nested(i);
    *(sub_log.add_sub_tree()) = BuildTextTree(nested_logs);
  }
  return sub_log;
}

void BuildTextLog(const PrintLogs& p_logs, TextTree* log) {
  for (int i = 0; i < p_logs.logs_size(); ++i) {
    *log->add_sub_tree() = BuildTextTree(p_logs.logs(i));
  }
}

void BuildTextLog(const SoccerDebugMessage message, TextTree* log) {
  for (int i = 0; i < message.print_logs_size(); ++i) {
    *log->add_sub_tree() = BuildTextTree(message.print_logs(i));
  }
}

}  // namespace logger
