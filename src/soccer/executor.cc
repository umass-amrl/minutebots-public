// Copyright 2017 - 2019 kvedder@umass.edu, jaholtz@cs.umass.edu
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
#include "soccer/executor.h"

#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <fstream>
#include <iomanip>  // std::setprecision
#include <string>

#include "constants/constants.h"

#include "eigen3/Eigen/Dense"
#include "evaluators/defense_evaluation.h"
#include "evaluators/offense_evaluation.h"
#include "evaluators/passing_evaluation.h"
#include "evaluators/penalty_recieve_evaluation.h"
#include "evaluators/stopped_evaluation.h"
#include "logging/navigation_logger.h"
#include "net/netraw.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/robot_obstacle.h"
#include "plays/skills_tactics_plays.h"
#include "safety/dss.h"
#include "safety/dss2.h"
#include "state/soccer_robot.h"
#include "state/soccer_state.h"
#include "state/team.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "util/timer.h"

#include "radio_protocol_wrapper.pb.h"
#include "referee.pb.h"

STANDARD_USINGS;
using defense::DefenseEvaluator;
using defense::PenaltyRecieveEvaluator;
using defense::StoppedEvaluator;
using Eigen::Matrix;
using logger::Logger;
using net::UDPMulticastServer;
using safety::DSS;
using safety::DSS2;
using state::SoccerRobot;
using state::SoccerState;
using state::SharedState;
using state::WorldState;
using state::PositionVelocityState;
using state::WorldRobot;
using state::RefereeState;
using obstacle::Obstacle;
using obstacle::ObstacleFlag;
using obstacle::RobotObstacle;
using std::atomic_bool;
using std::condition_variable;
using std::mutex;
using std::pair;
using std::thread;
using std::unique_lock;
using tactics::Tactic;
using tactics::TacticIndex;
using team::Team;
using pose_2d::Pose2Df;
using threadsafe::ThreadSafeActor;
using plays::SkillsTacticsPlays;
using MinuteBotsProto::PrintLog;
using MinuteBotsProto::PrintLogs;
using MinuteBotsProto::SoccerDebugMessage;
using MinuteBotsProto::SSL_Referee;
using state::PositionVelocityState;
using experimental_simulator::ExperimentalSim;
using experimental_simulator::Simulator;
using offense::TargetEvaluator;
using passing_evaluation::save_pass_ahead_heatmap;

namespace app {

// static bool kUseTestPlays = true;

Executor::Executor(
    const string& udp_address, const int udp_port, const int refbox_port,
    const bool& test_plays, const vector<float>& attacker_params,
    const string& trace_file_name,
    ThreadSafeActor<PositionVelocityState>* thread_safe_position_velocity_state,
    ThreadSafeActor<Logger>* thread_safe_kalman_logger,
    threadsafe::ThreadSafeQueue<state::SharedState>*
        thread_safe_shared_state_queue,
    const vector<pair<SSLVisionId, TacticIndex>>& default_tactics,
    const Team& team, const direction::Direction& direction,
    const bool& simulating,
    std::exception_ptr* exception)
    : local_position_velocity_state_(),
      simulating_(simulating),
      local_world_state_(&local_position_velocity_state_, team, simulating_),
      udp_address_(udp_address),
      udp_port_(udp_port),
      refbox_port_(refbox_port),
      test_plays_(test_plays),
      attacker_params_(attacker_params),
      trace_file_name_(trace_file_name),
      thread_safe_position_velocity_state_(thread_safe_position_velocity_state),
      thread_safe_kalman_logger_(thread_safe_kalman_logger),
      thread_safe_shared_state_queue_(thread_safe_shared_state_queue),
      soccer_state_(local_world_state_, team, direction),
      referee_state_(team),
      logger_(logger::NetLogger(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT)),
      skills_tactics_plays_(SkillsTacticsPlays(&soccer_state_)),
      use_stp_(default_tactics.size() == 0),
      default_tactics_(default_tactics),
      exception_(exception) {
  if (simulating_) {
    local_position_velocity_state_.SetTime(GetWallTime());
  } else {
    local_position_velocity_state_.SetTime(GetWallTime());
  }
  is_running_ = true;

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
  message.set_command_counter(0);
  message.set_stage(MinuteBotsProto::SSL_Referee_Stage_NORMAL_FIRST_HALF);
  message.set_command(MinuteBotsProto::SSL_Referee_Command_HALT);
  referee_state_.SetRefereeMessage(message);
  soccer_state_.SetRefereeState(referee_state_);
}

Executor::~Executor() {
  // Dump the trace to a file
  if (!trace_file_name_.empty()) {
    std::ofstream trace_file;
    trace_file.open(trace_file_name_);
    google::protobuf::io::OstreamOutputStream output_stream(&trace_file);
    google::protobuf::TextFormat::Print(tactic_trace, &output_stream);
  }
  // Signal the thread that we are shutting down
  is_running_ = false;

  thread_safe_kalman_logger_->Shutdown();
  thread_safe_position_velocity_state_->Shutdown();
  thread_safe_shared_state_queue_->Shutdown();

  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }
}

void Executor::Start() {
  simulating_ = false;
  execution_thread_ = thread(&Executor::HandleExecution, this);
}

void Executor::SimStart(ThreadSafeActor<Simulator*>* thread_safe_sim,
                        Simulator* simulator) {
  thread_safe_simulator_ = thread_safe_sim;
  const RadioProtocolWrapper wrapper =
  soccer_state_.GetMutableSharedState()->ConvertToRadioWrapper(
    local_world_state_, &logger_);
  local_simulator_ = simulator;
  simulating_ = true;
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
  message.set_command_counter(0);
  message.set_stage(MinuteBotsProto::SSL_Referee_Stage_NORMAL_FIRST_HALF);
  message.set_command(MinuteBotsProto::SSL_Referee_Command_NORMAL_START);
  referee_state_.SetRefereeMessage(message);
  soccer_state_.SetRefereeState(referee_state_);
  execution_thread_ = thread(&Executor::HandleExecution, this);
}

void Executor::Stop() { is_running_ = false; }

void Executor::MergeRobotMessages() {
  vector<SoccerRobot>* mutable_soccer_robots =
      soccer_state_.GetAllMutableSoccerRobots();
  for (SoccerRobot& robot : *mutable_soccer_robots) {
    if (robot.enabled_) {
      const auto& wr =
          local_world_state_.GetOurRobotPosition(robot.our_robot_index_);
      const string robot_str = StringPrintf("Robot %X", wr.ssl_vision_id);
      logger_.LogPrint(robot_str.c_str());
      logger_.Push();
      logger_.LogPrint("Tactic: %d", robot.current_tactic_);
      logger_.MergeLoggers(robot.bot_logger);
      logger_.Pop();

      // Adding to the trace of all the tactics
      if (!trace_file_name_.empty()) {
        auto message = robot.bot_logger.ReturnMessage();
        for (auto trace_element : message.tuning_data()) {
          *(tactic_trace.add_trace_elements()) = trace_element;
        }
      }
      robot.bot_logger.Clear();
    }
  }
}

template <typename Derived>
inline bool ContainsNan(const Eigen::MatrixBase<Derived>& x) {
  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      if (isnan(x(i, j))) {
        return true;
      }
    }
  }
  return false;
}

void Executor::HandleExecution() {
  try {
    static const bool kDebug = false;
    DSS dss(local_world_state_, &soccer_state_);
    DSS2 dss2(local_world_state_, &soccer_state_);
    DefenseEvaluator defense_evaluator;
    StoppedEvaluator stopped_evaluator;
    PenaltyRecieveEvaluator penalty_evaluator;

    std::ofstream ofs;
    if (!kProduction) {
      ofs.open("executor_timing.txt", std::ofstream::out);
    }

    // Init navigation logging if necessary
    if (kLogNavigation) {
      navigation_logger::NavigationLogger::Init("MinuteBots",
                                                "EightGrid",
                                                false,
                                                false,
                                                false);
    }

    // Used for broadcasting the velocity command.
    UDPMulticastServer udp_server;
    if (!udp_server.Open(udp_address_, udp_port_, false)) {
      LOG(FATAL) << "Error opening UDP port of Executor's "
                << "HandleExecution thread, exiting.";
    }
    CHECK(udp_server.IsOpen());

    // Used for reading in messages from the referee.
    UDPMulticastServer referee_server;
    if (!referee_server.Open((DATA_STREAM_REF_IP), refbox_port_, true)) {
      LOG(FATAL) << "Error opening UDP for referee "
                << "HandleExecution thread exiting.";
    }
    CHECK(referee_server.IsOpen());

    // Used to maintain a constant transmit rate to the robots.
    RateLoop loop(kTransmitFrequency);

    SSL_Referee referee_message;

    if (simulating_) {
      local_position_velocity_state_.SetTime(GetWallTime());
    } else {
      local_position_velocity_state_.SetTime(GetWallTime());
    }

    TargetEvaluator target_evaluator;
    double t_last_run = GetMonotonicTime();
    while (is_running_) {
      const double t_start = GetMonotonicTime();
      state::PositionVelocityState last_state = local_position_velocity_state_;
      if (thread_safe_position_velocity_state_->ReadOrDefault(
              &local_position_velocity_state_)) {
        local_world_state_.UpdateLastState(last_state);
      }
      local_world_state_.UpdateState(&logger_);
      soccer_state_.GetMutableSharedState()->ResetCommands(
          local_position_velocity_state_);
      soccer_state_.UpdateExistances();

      if (referee_server.TryReceiveProtobuf(&referee_message)) {
        referee_state_.SetRefereeMessage(referee_message);
        soccer_state_.SetRefereeState(referee_state_);
      }
      soccer_state_.UpdateGameState();
      if (!use_stp_) {
        // Set the default tactic for each soccer robot.
        for (auto& pair : default_tactics_) {
          soccer_state_.SetRobotTacticByRobotIndex(pair.first, pair.second);
        }
      }

      const double t_pre_stp = GetMonotonicTime();

      if (kDebug) {
        logger_.LogRobotDataEntry(kGraphRobot);
      }
      if (referee_state_.IsHalt() || referee_state_.IsTimeoutThem() ||
          referee_state_.IsTimeoutUs()) {
        vector<SoccerRobot>* mutable_soccer_robots =
            soccer_state_.GetAllMutableSoccerRobots();
        for (SoccerRobot& robot : *mutable_soccer_robots) {
          tactics::TacticIndex tactic_id = TacticIndex::HALT;
          soccer_state_.SetRobotTacticByRobotIndex(robot.our_robot_index_,
                                                  tactic_id);
        }
      } else if (use_stp_) {
        // Update goalie id if necessary
        DefenseEvaluator::SetGoalie(local_world_state_);
        // This will set the tactics for each robot in the soccer state
        skills_tactics_plays_.ExecutePlayEngine(referee_state_, test_plays_);
        logger_.MergeLoggers(skills_tactics_plays_.play_logger);
      }

      const double t_post_stp = GetMonotonicTime();

      const double t_pre_pass_table_build = GetMonotonicTime();
      target_evaluator.Update(local_world_state_, &soccer_state_,
                              offense::GetPoseCost);
      //         offense::WritePassFunctionData(local_world_state_,
      //                                        &soccer_state_,
      //                                        offense::GetPoseCost);
      const double t_post_pass_table_build = GetMonotonicTime();

      logger_.LogPrint("Executor");
      logger_.LogRefState(referee_state_);

      if (soccer_state_.IsNormalPlay()) {
        logger_.Push();
        logger_.LogPrint("Normal Play");
        logger_.Pop();
      } else {
        logger_.Push();
        logger_.LogPrint("Abnormal Play");
        logger_.Pop();
      }

      if (soccer_state_.IsBallMoved()) {
        logger_.Push();
        logger_.LogPrint("Ball Moved");
        logger_.Pop();
      } else {
        logger_.Push();
        logger_.LogPrint("Ball Not Moved");
        logger_.Pop();
      }

      if (soccer_state_.IsBallKicked()) {
        logger_.Push();
        logger_.LogPrint("Kicked");
        logger_.Pop();
      } else {
        logger_.Push();
        logger_.LogPrint("Not Kicked");
        logger_.Pop();
      }

      if (soccer_state_.IsKickoff()) {
        logger_.Push();
        logger_.LogPrint("Kickoff");
        logger_.Pop();
      } else {
        logger_.Push();
        logger_.LogPrint("Not Kickoff");
        logger_.Pop();
      }

      stopped_evaluator.SetStoppedTargets(local_world_state_, soccer_state_);

      if (soccer_state_.IsOurPenaltyKick() ||
          soccer_state_.IsTheirPenaltyKick()) {
        penalty_evaluator.SetRecieveTargets(local_world_state_, soccer_state_);
      }

      const double t_pre_defense = GetMonotonicTime();
      DefenseEvaluator::AssignDefenders(local_world_state_, soccer_state_);
      const double t_post_defense = GetMonotonicTime();

      const double t_pre_tactics = GetMonotonicTime();
      soccer_state_.UpdateNavigationState(&logger_);
      soccer_state_.RunAllTactics();
      const double t_post_tactics = GetMonotonicTime();

      //     dss.MakeSafe({kMaxRobotAcceleration, kMaxRobotVelocity},
      //                  {kMaxRobotAcceleration, kMaxRobotVelocity},
      //                  kDSSControlPeriodTicks / kTransmitFrequency);
      dss2.MakeSafe({kMaxRobotAcceleration, kMaxRobotVelocity},
                    {kMaxRobotAcceleration, kMaxRobotVelocity},
                    &logger_);

      const double t_post_dss = GetMonotonicTime();

      logger_.SetWorldState(local_world_state_);

      const double t_post_merge = GetMonotonicTime();

      soccer_state_.GetMutableSharedState()->SetCommandTime(
          local_world_state_.world_time_);

      const RadioProtocolWrapper wrapper =
          soccer_state_.GetMutableSharedState()->ConvertToRadioWrapper(
              local_world_state_, &logger_);

      thread_safe_shared_state_queue_->Add(soccer_state_.GetSharedState());
      local_world_state_.AddSharedState(soccer_state_.GetSharedState());

      if (kDebug) {
        logger_.LogPrint("Velocity Command Info");
        logger_.Push();
        for (const RadioProtocolCommand& command : wrapper.command()) {
          logger_.LogPrint("Commanding ID %d to (%.5f, %.5f, %.5f deg/s)\n",
                          command.robot_id(), command.velocity_x(),
                          command.velocity_y(), RadToDeg(command.velocity_r()));
        }
        logger_.Pop();

        logger_.LogPrint("Kick Command Info: ");
        logger_.Push();

        for (const RadioProtocolCommand& command : wrapper.command()) {
          if (command.has_flat_kick()) {
            logger_.LogPrint("ID kicking at %f \n", command.flat_kick());
          }
        }

        logger_.Pop();
      }
      if (simulating_) {
        local_simulator_->SimulateStep(wrapper);
        thread_safe_simulator_->Write(local_simulator_);
      } else {
        if (!udp_server.SendProtobuf(wrapper)) {
          LOG(ERROR) << "Send output error" << endl;
        }
      }

      const double t_end = GetMonotonicTime();

      local_world_state_.UpdateProcessingTime(t_end - t_start);

      logger_.LogPrint("Timing Info");
      logger_.Push();
      logger_.LogPrint("Total Processing time:%.3f ms",
                      1000.0 * (t_end - t_start));
      logger_.LogPrint("Copy world state time:%.3f ms",
                      1000.0 * (t_pre_stp - t_start));
      logger_.LogPrint("STP time:%.3f ms", 1000.0 * (t_post_stp - t_pre_stp));
      logger_.LogPrint(
          "Pass Table Build:%.3f ms",
          1000.0 * (t_post_pass_table_build - t_pre_pass_table_build));
      logger_.LogPrint(
        "Defense Evaluation Time %.3f ms",
        1000.0 * (t_post_defense - t_pre_defense));
      logger_.LogPrint("Tactics time:%.3f ms",
                      1000.0 * (t_post_tactics - t_pre_tactics));
      logger_.LogPrint("DSS time:%.3f ms",
                      1000.0 * (t_post_dss - t_post_tactics));
      logger_.LogPrint("Merge time:%.3f ms",
                      1000.0 * (t_post_merge - t_post_dss));
      logger_.LogPrint("Post merge time : %.3f ms",
                      1000.0 * (t_end - t_post_merge));
      logger_.LogPrint("Frame Period:%f ms", 1000.0 * (t_start - t_last_run));
      logger_.Pop();

      if (kDebug) {
        LOG(WARNING) << "DSS TIME: " << t_post_dss - t_post_tactics;
        LOG(WARNING) << "TOTAL TIME: " << t_start - t_last_run;
      }

      t_last_run = t_start;

      double t_mergining = GetMonotonicTime();
      logger_.MergeLoggers(DefenseEvaluator::GetDefenseLogger());
      DefenseEvaluator::ResetDefenseLogger();

      logger_.LogPrintPush("Robots");
      MergeRobotMessages();
      logger_.Pop();


      Logger kalman_logger;
      thread_safe_kalman_logger_->ReadOrDefault(&kalman_logger);
      logger_.MergeLoggers(kalman_logger);
      logger_.SetMessageTime(GetWallTime());
      logger_.LogPrint("Observed Ball Pose: %f,%f",
                      local_world_state_.GetBallPosition().observed_pose.x(),
                      local_world_state_.GetBallPosition().observed_pose.y());
      logger_.AddCircle(local_world_state_.GetBallPosition().observed_pose,
                        kBallRadius,
                        1,
                        .5,
                        .02,
                        .5);
      double t_merged_all = GetMonotonicTime();
      logger_.LogPrint("Log Merge Time: %f", t_merged_all - t_mergining);
      if (soccer_state_.GetTeam() == Team::YELLOW) {
        logger_.SetTeam(true);
      } else {
        logger_.SetTeam(false);
      }
      if (!referee_state_.IsHalt()) {
        logger_.SendData();
      }
      logger_.Clear();

      if (kLogNavigation) {
      navigation_logger::NavigationLogger::LogAndResetEntry();
      }

      loop.Sleep();

      if (!kProduction) {
        ofs << std::setprecision(15) << GetMonotonicTime() << "\n";
      }
    }

    if (kLogNavigation) {
      navigation_logger::NavigationLogger::Close();
    }

    if (!kProduction) {
      ofs.close();
    }
  } catch (...) {
    *exception_ = std::current_exception();
  }
}
}  // namespace app
