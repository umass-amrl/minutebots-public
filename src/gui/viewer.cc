// Copyright 2017-2019 joydeepb@cs.umass.edu
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

#include <vector>
#include <mutex>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include "QBoxLayout"
#include "QGridLayout"
#include "QKeyEvent"
#include "QList"
#include "QSplitter"
#include "QWidget"
#include "QString"


#include "gui/soccerview.h"
#include "gui/text_tree_viewer.h"
#include "gui/viewer.h"
#include "referee.pb.h"
#include "soccer_logging.pb.h"
#include "util/timer.h"

using MinuteBotsProto::SoccerDebugMessage;
using MinuteBotsProto::PrintLogs;
using MinuteBotsProto::TextTree;
using MinuteBotsProto::SSL_Referee;
using MinuteBotsProto::WorldState;
using net::UDPMulticastServer;
using std::stod;

const bool kGraphMode = true;
bool use_raw_vision_ = true;

namespace gui {

Viewer::Viewer(QWidget* parent,
               logger::ReadLogger* logger,
               const bool& tuning)
    : QWidget(parent),
      horizontal_splitter_(Qt::Horizontal),
      vertical_splitter_(Qt::Vertical),
      slider_(Qt::Horizontal),
      tuning_box_(),
      comment_box_(),
      tuning_mode_(tuning),
      file_logger_(logger) {
  comment_file_.open("viewer_logs/empty.txt", std::fstream::in |
      std::fstream::out |
      std::fstream::app);
  comment_box_.setFixedHeight(50);
  vertical_splitter_.addWidget(&comment_box_);
  vertical_splitter_.addWidget(&field_);
  // Setup Slider
  slider_.setFixedHeight(30);
  slider_.setFocusPolicy(Qt::StrongFocus);
  slider_.setTickPosition(QSlider::NoTicks);
  slider_.setTickInterval(10);
  slider_.setSingleStep(1);
  slider_.setMinimum(1);
  slider_.setMaximum(1);
  slider_.setValue(slider_.maximum());
  connect(&slider_, SIGNAL(sliderPressed()), this, SLOT(HandleSlider()));
  connect(&comment_box_,
          SIGNAL(returnPressed()),
          SLOT(HandleCommentBox()));
  vertical_splitter_.addWidget(&slider_);
  if (kGraphMode) {
    vertical_splitter_.addWidget(&graph_);
  }
  if (tuning_mode_) {
    vertical_splitter_.addWidget(&tuning_box_);
  }
  horizontal_splitter_.addWidget(&vertical_splitter_);
  horizontal_splitter_.addWidget(&text_log_);
  just_paused_ = false;
  QGridLayout* layout = new QGridLayout;
  layout->setMargin(0);
  layout->addWidget(&horizontal_splitter_);
  setLayout(layout);
  slider_.setFocus();
  // Reset();
}

void Viewer::resizeEvent(QResizeEvent* event) {
  // Reset();
}

void Viewer::SetCommentFile(string filename) {
  if (comment_file_.is_open()) {
    comment_file_.close();
  }
  comment_file_.open(filename,
                     std::fstream::in |
                     std::fstream::out |
                     std::fstream::app);
  string line;
  while ((getline(comment_file_, line))) {
    LOG(ERROR) << "GET LINE";
    std::stringstream ss(line);
    string time_string;
    double time;
    if (!getline(ss, time_string, ',')) break;
    time = stod(time_string);
    string comment;
    if (!getline(ss, comment, ',')) break;
    comment_map_[time] = comment;
  }
  LOG(ERROR) << "EXIT";
  comment_file_.close();
}

void Viewer::WriteCommentFile(string filename) {
  comment_file_.open(filename);
  for (auto entry : comment_map_) {
    comment_file_ << std::setprecision(20) << entry.first << ",";
    comment_file_ << entry.second << "\n";
  }
  comment_file_.close();
}

void Viewer::UpdateComment() {
  if (current_message_.has_time()) {
    const double time = current_message_.time();
    if (comment_map_.count(time) > 0) {
      QString qstr = QString::fromStdString(comment_map_[time]);
      comment_box_.setText(qstr);
    } else {
      QString qstr;
      comment_box_.setText(qstr);
    }
  }
}

void Viewer::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Escape) close();
  if (event->key() == Qt::Key_P) Pause();
  if (event->key() == Qt::Key_L) Live();
  if (event->key() == Qt::Key_H) RefHalt();
  if (event->key() == Qt::Key_K) SetTransition();
  if (event->key() == Qt::Key_N) UnsetTransition();
  if (event->key() == Qt::Key_C) ContinueTransition();
  if (event->key() == Qt::Key_E) EndFile();
  //   if (event->key() == Qt::Key_A) TestSelection();
  if (event->key() == Qt::Key_G) RefForceStart();
  if (event->key() == Qt::Key_Left && pause_status) Previous();
  if (event->key() == Qt::Key_Right && pause_status) Next();
  if (event->key() == Qt::Key_O) {
    use_raw_vision_ = !use_raw_vision_;
    graph_.Clear();
  }
  if (field_.underMouse()) {
    field_.keyPressEvent(event);
  } else if (graph_.underMouse() && kGraphMode) {
    graph_.keyPressEvent(event);
  } else if (text_log_.underMouse()) {
    text_log_.keyPressEvent(event);
  }
  QWidget::keyPressEvent(event);
  // Pass on event to possible callbacks.
  KeypressSignal(event);
}

void Viewer::HandleCommentBox() {
  if (current_message_.has_time()) {
    double time = current_message_.time();
    string text_contents = comment_box_.text().toStdString();
    comment_map_[time] = text_contents;
  }
}

void Viewer::HandleSlider() {
  UpdateSliderSize();
  if (IsLive()) {
  Pause();
  }
  if (!pause_status) {
    pause_status = true;
  }
}

void Viewer::UpdateSliderSize() {
  std::lock_guard<std::mutex> lock(slider_mutex);
  slider_.setMaximum(file_logger_->GetMapSize());
}

void Viewer::AdvanceSlider() {
  std::lock_guard<std::mutex> lock(slider_mutex);
  if (slider_.value() < slider_.maximum()) {
    slider_.setValue(slider_.value() + slider_.singleStep());
  }
}

void BallDataEntry(const WorldState& world,
                          DataEntry* entry_ptr) {
  DataEntry& entry = *entry_ptr;
  if (world.balls_size() < 1 || world.ball_velocities_size() < 1) {
    return;
  }
  const auto& loc = world.balls(0);
  const auto& vel = world.ball_velocities(0);
  entry.angle = 0;
  entry.omega = 0;
  entry.loc = Vector2f(loc.x(), loc.y());
  entry.vel = Vector2f(vel.x(), vel.y());
  entry.speed = entry.vel.norm();
}

void ObservedBallDataEntry(const MinuteBotsProto::SoccerDebugMessage& msg,
                           DataEntry* entry_ptr) {
  if (msg.world_state().balls_size() > 0 &&
    msg.world_state().ball_observation_size() > 0 &&
    msg.world_state().ball_velocities_observed_size() > 0) {
    Eigen::Vector2f current_pos = {
      msg.world_state().ball_observation(0).x(),
      msg.world_state().ball_observation(0).y()};

      Eigen::Vector2f velocity = {
        msg.world_state().ball_velocities_observed(0).x(),
        msg.world_state().ball_velocities_observed(0).y()};

        float speed = velocity.norm();

        gui::DataEntry entry(msg.time(),
                             current_pos,
                             0,
                             speed,
                             velocity,
                             0);
        *entry_ptr = entry;
    }
    return;
}

void RobotDataEntry(const WorldState& world,
                    team::Team team,
                    int robot_id,
                    DataEntry* entry_ptr) {
  DataEntry& entry = *entry_ptr;
  for (const MinuteBotsProto::RobotState& robot : world.robots()) {
    if (robot.robot_id() == robot_id &&
        robot.team() == team) {
      const auto& loc = robot.pose().loc();
      const auto& vel = robot.velocity();
      entry.loc = Vector2f(loc.x(), loc.y());
      if (use_raw_vision_) {
        entry.vel = Vector2f(robot.observed_velocity().x(),
                             robot.observed_velocity().y());
      } else {
        entry.vel = Vector2f(vel.x(), vel.y());
      }
      entry.speed = entry.vel.norm();
      entry.angle = robot.pose().angle();

      if (use_raw_vision_) {
        entry.omega = robot.observed_rotational_velocity();
      } else {
        entry.omega = robot.rotational_velocity();
      }
    }
  }
}

void Viewer::UpdateGraph(const SoccerDebugMessage& msg) {
  DataEntry entry;
  entry.t = msg.time();
  switch (field_.selection_) {
    case gui::SelectionType::kBallSelected : {
      BallDataEntry(msg.world_state(), &entry);
    } break;
    case gui::SelectionType::kRobotSelected : {
      RobotDataEntry(msg.world_state(),
                     field_.selected_robot_team_,
                     field_.selected_robot_id_,
                     &entry);
    } break;
    default: {
      // Just use the empty data entry.
    }
  }
  graph_.PushBack(entry);
  graph_.Update();
}

void Viewer::Update(const MinuteBotsProto::SoccerDebugMessage& msg) {
  current_message_ = msg;
  UpdateComment();
  if (kGraphMode) {
    UpdateGraph(msg);
  }
  if (!msg.has_time() || ((msg.time() - last_time_) >= .0333)) {
    last_time_ = msg.time();
    field_.Update(msg);
    if (!log_status_) {
      if (msg.has_text_log()) {
        text_log_.Update(msg);
      } else if (msg.print_logs_size() > 0) {
        TextTree tree;
        logger::BuildTextLog(msg, &tree);
        SoccerDebugMessage log_message;
        *(log_message.mutable_text_log()) = tree;
        text_log_.Update(log_message);
      }
    }
  }
}

void Viewer::AddIndexEntry(const int& index) {
  std::unique_lock<std::mutex> lock1(slider_mutex, std::defer_lock);
  lock1.lock();
  file_logger_->AddIndexEntry(index);
  lock1.unlock();
}

void Viewer::Playback() {
  if (!pause_status) {
    Viewer::NextLive();
    AdvanceSlider();
    slider_changed_ = true;
    just_paused_ = false;
  } else {
    if (just_paused_) {
      slider_.setValue(slider_.maximum());
      just_paused_ = false;
      slider_changed_ = true;
      file_logger_->GoToIndex(file_logger_->GetMapSize() - 1);
    }
    if (slider_.value() != last_slider_value_) {
      file_logger_->GoToIndex(slider_.value());
      Viewer::Next();
      slider_changed_ = true;
    } else {
      slider_changed_ = false;
    }
  }
  last_slider_value_ = slider_.value();
}

void Viewer::Pause() {
  UpdateSliderSize();
  if (!log_status_) {
    just_paused_  = true;
    log_status_ = true;
  }
  pause_status = !pause_status;
}

void Viewer::Live() {
  std::lock_guard<std::mutex> lock(slider_mutex);
  slider_.setValue(slider_.maximum());
  log_status_ = false;
  pause_status = false;
  file_logger_->Live();
}

void Viewer::RefForceStart() {
  const string udp_address = DATA_STREAM_REF_IP;
  const int udp_port = DATA_STREAM_REF_PORT;
  UDPMulticastServer udp_server;
  if (!udp_server.Open(udp_address, udp_port, false)) {
    std::cerr << "Error opening UDP port of Executor's "
              << "HandleExecution thread, exiting." << std::endl;
  }
  SSL_Referee message;
  SSL_Referee::TeamInfo* blue = message.mutable_blue();
  SSL_Referee::TeamInfo* yellow = message.mutable_yellow();
  blue->set_name("blue");
  blue->set_score(0);
  blue->set_red_cards(0);
  blue->set_yellow_cards(0);
  blue->set_timeouts(0);
  blue->set_timeout_time(0);
  blue->set_goalie(0);
  yellow->set_name("yellow");
  yellow->set_score(0);
  yellow->set_red_cards(0);
  yellow->set_yellow_cards(0);
  yellow->set_timeouts(0);
  yellow->set_timeout_time(0);
  yellow->set_goalie(0);
  message.set_packet_timestamp(0);
  message.set_command_timestamp(0);
  message.set_stage(MinuteBotsProto::SSL_Referee_Stage_NORMAL_FIRST_HALF);
  message.set_command(MinuteBotsProto::SSL_Referee_Command_FORCE_START);
  message.set_command_counter(1);
  if (!udp_server.SendProtobuf(message)) {
    std::cerr << "Send output error" << std::endl;
  }
}

void Viewer::RefHalt() {
  const string udp_address = DATA_STREAM_REF_IP;
  const int udp_port = DATA_STREAM_REF_PORT;
  UDPMulticastServer udp_server;
  if (!udp_server.Open(udp_address, udp_port, false)) {
    std::cerr << "Error opening UDP port of Executor's "
              << "HandleExecution thread, exiting." << std::endl;
  }
  SSL_Referee message;
  SSL_Referee::TeamInfo* blue = message.mutable_blue();
  SSL_Referee::TeamInfo* yellow = message.mutable_yellow();
  blue->set_name("blue");
  blue->set_score(0);
  blue->set_red_cards(0);
  blue->set_yellow_cards(0);
  blue->set_timeouts(0);
  blue->set_timeout_time(0);
  blue->set_goalie(0);
  yellow->set_name("yellow");
  yellow->set_score(0);
  yellow->set_red_cards(0);
  yellow->set_yellow_cards(0);
  yellow->set_timeouts(0);
  yellow->set_timeout_time(0);
  yellow->set_goalie(0);
  message.set_packet_timestamp(0);
  message.set_command_timestamp(0);
  message.set_stage(MinuteBotsProto::SSL_Referee_Stage_NORMAL_FIRST_HALF);
  message.set_command(MinuteBotsProto::SSL_Referee_Command_HALT);
  message.set_command_counter(1);
  if (!udp_server.SendProtobuf(message)) {
    std::cerr << "Send output error" << std::endl;
  }
}

void Viewer::SetLive(bool value) {
  log_status_ = !value;
}

void Viewer::ContinueTransition() {
  file_logger_->ContinueTransition(slider_.value(), tuning_box_.currentIndex());
}

void Viewer::EndFile() {
  std::cout << slider_.value() << std::endl;
  file_logger_->EndFile(slider_.value());
//   close();
}

void Viewer::SetTransition() {
  std::cout << "Transition Signaled" << std::endl;
  file_logger_->SetTransition(slider_.value(), tuning_box_.currentIndex());
}

void Viewer::UnsetTransition() {
  std::cout << "No Transition Signaled" << std::endl;
  file_logger_->UnsetTransition(slider_.value(), tuning_box_.currentIndex());
}

bool Viewer::IsLive() { return !log_status_; }

void Viewer::showEvent(QShowEvent* event) {
  Reset();
  QWidget::showEvent(event);
}

void Viewer::Replay(const MinuteBotsProto::SoccerDebugMessage& msg) {
  field_.Update(msg);
  graph_.Update();
  UpdateComment();
  last_time_ = msg.time();
  current_message_ = msg;
  if (tuning_mode_ && slider_changed_) {
    tuning_box_.clear();
    for (int i = 0; i < msg.tuning_data_size(); ++i) {
      const MinuteBotsProto::StateMachineData machine =
          msg.tuning_data(i);
      const string machine_name = machine.machine_name();
      for (int j = 0; j < machine.transitions_size(); ++j) {
        const MinuteBotsProto::PossibleTransition transition =
            machine.transitions(j);
        const string start_state = transition.start_state();
        string box_string =
            machine_name + "_" +
            start_state +
            "->" + transition.potential_state();
        if (transition.should_transition()) {
          box_string += "*";
        }
        tuning_box_.addItem(QString(box_string.c_str()));
      }
    }
  }
  if (msg.has_text_log()) {
    text_log_.Update(msg);
  } else if (msg.print_logs_size() > 0) {
    TextTree tree;
    logger::BuildTextLog(msg, &tree);
    SoccerDebugMessage log_message;
    *(log_message.mutable_text_log()) = tree;
    text_log_.Update(log_message);
  }
  if (kGraphMode) {
    UpdateGraph(msg);
  }
}

void Viewer::Previous() {
  MinuteBotsProto::LogWrapper wrapper = file_logger_->GetPreviousMessage();
  if (wrapper.has_soccer_debug()) {
    slider_changed_ = true;
    const MinuteBotsProto::SoccerDebugMessage& msg = wrapper.soccer_debug();
    Viewer::Replay(msg);
  }
}

void Viewer::Next() {
//   std::cout << "Next" << std::endl;
  MinuteBotsProto::LogWrapper wrapper = file_logger_->GetNextMessage();
  if (wrapper.has_soccer_debug()) {
    slider_changed_ = true;
    const MinuteBotsProto::SoccerDebugMessage& msg = wrapper.soccer_debug();
    Viewer::Replay(msg);
  }
}

void Viewer::NextLive() {
  MinuteBotsProto::LogWrapper wrapper = file_logger_->GetNextMessage();
  if (wrapper.has_soccer_debug()) {
    const MinuteBotsProto::SoccerDebugMessage& msg = wrapper.soccer_debug();
    const int time = msg.time();
    // Time diff currently assumed to be in microseconds
    if (last_time_ != 0) {
      double time_diff = time - last_time_;
      time_diff = time_diff * .000001;
      Sleep(time_diff);
    }
    Viewer::Replay(msg);
  }
}

void Viewer::Reset() {
  const float w = width();
  const float h = height();
  QList<int> horizontal_sizes;
  horizontal_sizes.push_back(0.75 * w);
  horizontal_sizes.push_back(0.25 * w);
  QList<int> vertical_sizes;
  vertical_sizes.push_back(0.1 * h);
  vertical_sizes.push_back(0.7 * h);
  vertical_sizes.push_back(0.01 * h);
  if (kGraphMode) {
    vertical_sizes.push_back(0.15 * h);
  }
  vertical_splitter_.setSizes(vertical_sizes);
  horizontal_splitter_.setSizes(horizontal_sizes);
  field_.ResetView();
  graph_.RedrawSignal();
}

}  // namespace gui
