// Copyright 2016 - 2019 kvedder@umass.edu, jaholtz@cs.umass.edu
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

#include <glog/logging.h>
#include <signal.h>

#include <algorithm>
#include <atomic>
#include <bitset>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>

#include "configuration_reader/reader.h"
#include "constants/constants.h"
#include "debugging/tactic_exception.h"
#include "experimental_sim/experimental_sim.h"
#include "logging/logger.h"
#include "plays/skills_tactics_plays.h"
#include "soccer/executor.h"
#include "soccer/kalmanupdate.h"
#include "state/direction.h"
#include "state/position_velocity_state.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/team.h"
#include "state/world_state.h"
#include "tactics/tactic.h"
#include "third_party/optionparser-1.4/optionparser.h"
#include "util/colorize.h"
#include "thread_safe/thread_safe_actor.h"
#include "thread_safe/thread_safe_queue.h"
#include "util/timer.h"

#include "netraw_test_message.pb.h"

using app::Executor;
using app::KalmanUpdate;
using colorize::ColorCyan;
using colorize::ColorGreen;
using direction::Direction;
using experimental_simulator::SimState;
using experimental_simulator::Simulator;
using logger::Logger;
using option::Arg;
using option::OptionIndex;
using pose_2d::Pose2Df;
using SSLVisionProto::SSL_WrapperPacket;
using state::PositionVelocityState;
using state::SharedState;
using state::SoccerState;
using state::WorldState;
using std::atomic_bool;
using std::atomic_int;
using std::bitset;
using std::condition_variable;
using std::endl;
using std::mutex;
using std::pair;
using std::string;
using std::unique_lock;
using std::vector;
using tactics::Tactic;
using tactics::TacticIndex;
using team::Team;
using threadsafe::ThreadSafeActor;
using threadsafe::ThreadSafePriorityQueue;
using threadsafe::ThreadSafeQueue;

// Used for handling the lifetime and eventual shutdown of the main RoboCup
// stack.
static atomic_bool shutdown_flag(false);
static atomic_int exit_status(0);

// TODO(kvedder): Decide on better constants for CLI parse option array sizes.
const int kMaxOptions = 1000;
const int kMaxBuffer = 1000;

void SigHandler(int signo) {
  if (signo == SIGINT) {
    shutdown_flag = true;
  }
}

// See http://optionparser.sourceforge.net/example__arg_8cc.html
// for an example. I am so sorry about the mess...
const option::Descriptor usage[] = {
    {OptionIndex::UNKNOWN,
     0,
     "",
     "",
     Arg::Unknown,
     "USAGE: soccer [options]\n\nOptions:"},
    {OptionIndex::HELP,
     0,
     "",
     "help",
     Arg::None,
     "  \t--help  \tPrint usage and exit."},
    {OptionIndex::SIM,
     0,
     "S",
     "sim",
     Arg::None,
     "  \t-S  \tStarts in Simulator mode"},
    {OptionIndex::DIRECTION,
     0,
     "d",
     "direction",
     Arg::Direction,
     "  -d[<arg>], \t--direction[=<arg>]"
     "  \tTakes Positive ('p' or 'positive') or Negative ('n' or 'negative')."},
    {OptionIndex::TACTIC,
     0,
     "",
     "tactic",
     Arg::Tactic,
     "  --tactic[=<arg>]"
     "  \tDefaults to halt. Takes ForwardBackward ('fb' or "
     "'forwardbackward')"
     " or Halt ('h' or 'halt')."},
    {OptionIndex::TEAM,
     0,
     "te",
     "team",
     Arg::Team,
     "  -t[<arg>], \t--team[=<arg>]"
     "  \tTakes Yellow ('y' or 'yellow') or Blue ('b' or 'blue')."},
    {OptionIndex::USABLE_CAMERAS,
     0,
     "c",
     "cams",
     Arg::UsableCameras,
     "  -c[<arg>], \t--cams[=<arg>]"
     "  \tTakes list of 6 comma seperated non-space delimited values."},
    {OptionIndex::BALL_PARAMS,
     0,
     "b",
     "ball",
     Arg::BallParams,
     "  -b[<arg>], \t--ball[=<arg>]"
     "  \tList of 4 ball parameters\n"
     "x position\n"
     "y position\n"
     "x velocity\n"
     "y velocity\n"},
    {OptionIndex::TESTPLAYS,
     0,
     "tp",
     "testplay",
     Arg::TestPlays,
     "  -p[<arg>], \t--testplays[=<arg>]"
     "  \tTakes list of 6 comma seperated non-space delimited values."},
    {OptionIndex::REFPORT,
     0,
     "r",
     "refport",
     Arg::Refport,
     "  -r[<arg>], \t--refport[=<arg>]"
     "  \tTakes single integer for RefBox port."},
    {OptionIndex::VISPORT,
     0,
     "v",
     "visport",
     Arg::Refport,
     "  -v[<arg>], \t--visport[=<arg>]"
     "  \tTakes single integer for Vision port."},
    {OptionIndex::CMDPORT,
     0,
     "m",
     "cmdport",
     Arg::Refport,
     "  -m[<arg>], \t--cmdport[=<arg>]"
     "  \tTakes single integer for command port."},
    {OptionIndex::FBPORT,
     0,
     "f",
     "FBPORT",
     Arg::Refport,
     "  -f[<arg>], \t--fbport[=<arg>]"
     "  \tTakes single integer for feedback port port."},
    {OptionIndex::TRACEFILE,
     0,
     "T",
     "TRACEFILE",
     Arg::BallParams,
     "  -T[<arg>], \t--tracefile[=<arg>]"
     "  \tTakes a string name for the desired trace output file."},
    {OptionIndex::UNKNOWN,
     0,
     "",
     "",
     Arg::None,
     "\nExamples:\n"
     "  soccer --direction=negative\n"
     "  soccer --direction=NEGATIVE\n"
     "  soccer --direction=NeGaTiVe\n"
     "  soccer -dn\n"
     "  soccer --team=blue\n"
     "  soccer -teb \n"
     "  soccer --tactic 8,forwardbackward \n"
     "  soccer --tactic 8,fb \n"
     "  soccer --tactic 8,fb --tactic 9,fb \n"},
    {0, 0, 0, 0, 0, 0}};

string ToLowerCase(const char* cstr) {
  string arg = string(cstr);
  std::transform(arg.begin(), arg.end(), arg.begin(), ::tolower);
  return arg;
}

void ParseCLIFlags(int argc,
                   char** argv,
                   Direction* direction,
                   Team* team,
                   bool* simulator,
                   bool* test_plays,
                   int* refport,
                   int* visport,
                   int* cmdport,
                   int* fbport,
                   string* tracefile,
                   vector<SSLVisionId>* usable_cameras,
                   vector<float>* ball_parameters,
                   vector<pair<SSLVisionId, TacticIndex>>* tactic_list) {
  // Skip program name, argv[0], if present
  if (argc > 0) {
    argc--;
    argv++;
  }

  option::Stats stats(usage, argc, argv);
  option::Option options[kMaxOptions];
  option::Option buffer[kMaxBuffer];

  option::Parser parse(usage, argc, argv, options, buffer);

  if (parse.error()) {
    LOG(FATAL) << "CLI Parse error!\n";
  }

  if (options[OptionIndex::HELP] || argc == 0) {
    int columns = getenv("COLUMNS") ? atoi(getenv("COLUMNS")) : 80;
    option::printUsage(fwrite, stdout, usage, columns);
    exit(0);
  }

  if (options[OptionIndex::SIM]) {
    *simulator = true;
  }

  bool unknown_flag = false;

  for (int i = 0; i < parse.optionsCount(); ++i) {
    option::Option& opt = buffer[i];

    string lowercase_argument;
    if (opt.index() != OptionIndex::SIM) {
      lowercase_argument = string(opt.arg);
      std::transform(lowercase_argument.begin(),
                     lowercase_argument.end(),
                     lowercase_argument.begin(),
                     ::tolower);
    }

    switch (opt.index()) {
      case OptionIndex::DIRECTION: {
        if (lowercase_argument.compare("n") == 0 ||
            lowercase_argument.compare("negative") == 0) {
          *direction = Direction::NEGATIVE;
        } else if (lowercase_argument.compare("p") == 0 ||
                   lowercase_argument.compare("positive") == 0) {
          *direction = Direction::POSITIVE;
        } else {
          LOG(ERROR) << "Unknown direction argument: " << lowercase_argument
                     << "\n";
          unknown_flag = true;
        }
      } break;
      case OptionIndex::TEAM: {
        if (lowercase_argument.compare("y") == 0 ||
            lowercase_argument.compare("yellow") == 0) {
          *team = Team::YELLOW;
        } else if (lowercase_argument.compare("b") == 0 ||
                   lowercase_argument.compare("blue") == 0) {
          *team = Team::BLUE;
        } else {
          LOG(ERROR) << "Unknown team argument: " << lowercase_argument << "\n";
          unknown_flag = true;
        }
      } break;
      case OptionIndex::USABLE_CAMERAS: {
        std::stringstream ss(lowercase_argument);
        vector<string> result;

        while (ss.good()) {
          string substr;
          std::getline(ss, substr, ',');
          if (substr.length() > 0 && substr.c_str()[0] == '=') {
            substr.erase(substr.begin());
          }
          result.push_back(substr);
        }
        if (result.size() > kNumCameras) {
          LOG(ERROR) << "Incorrect number of usable cameras: " << result.size()
                     << "\n";
          unknown_flag = true;
          break;
        } else {
          for (const string& elem : result) {
            errno = 0;
            int robot_int = std::stoi(elem);
            int errsv = errno;
            if (errsv != 0) {
              LOG(ERROR) << "Failed to parse camera ID '" << elem
                         << "' into integer from argument: '" << elem
                         << "' with the following error: (" << strerror(errsv)
                         << ")\n";
              unknown_flag = true;
              break;
            }
            usable_cameras->push_back(robot_int);
          }
        }
      } break;
      case OptionIndex::BALL_PARAMS: {
        std::stringstream ss(lowercase_argument);
        vector<string> result;

        while (ss.good()) {
          string substr;
          std::getline(ss, substr, ',');
          if (substr.length() > 0 && substr.c_str()[0] == '=') {
            substr.erase(substr.begin());
          }
          result.push_back(substr);
        }

        if (result.size() < 4) {
          LOG(ERROR) << "Incorrect number of ball params " << result.size()
                     << "\n";
          unknown_flag = true;
          break;
        } else {
          for (const string& elem : result) {
            errno = 0;
            char* end;
            float param = std::strtof(elem.c_str(), &end);
            int errsv = errno;
            if (errsv != 0) {
              LOG(ERROR) << "Failed to parse param'" << elem
                         << "' into float from argument: '" << elem
                         << "' with the following error: (" << strerror(errsv)
                         << ")\n";
              unknown_flag = true;
              break;
            }
            ball_parameters->push_back(param);
          }
        }
      } break;
      case OptionIndex::TESTPLAYS: {
        if (lowercase_argument.compare("y") == 0 ||
            lowercase_argument.compare("1") == 0) {
          *test_plays = true;
        } else {
          LOG(ERROR) << "Unknown test_plays argument: " << lowercase_argument
                     << "\n";
          unknown_flag = true;
        }
      } break;
      case OptionIndex::REFPORT: {
        *refport = std::stoi(lowercase_argument);
      } break;
      case OptionIndex::VISPORT: {
        *visport = std::stoi(lowercase_argument);
      } break;
      case OptionIndex::CMDPORT: {
        *cmdport = std::stoi(lowercase_argument);
      } break;
      case OptionIndex::FBPORT: {
        *fbport = std::stoi(lowercase_argument);
      } break;
      case OptionIndex::TRACEFILE: {
        *tracefile = lowercase_argument;
      } break;
      case OptionIndex::TACTIC: {
        size_t find_location = -1;
        if ((find_location = lowercase_argument.find(",")) == string::npos) {
          LOG(ERROR) << "Unable to find robot identifier which preceeds a ',' "
                        "in the tactic argument: '"
                     << lowercase_argument << "'\n";
          unknown_flag = true;
          break;
        } else {
          string robot_substring = lowercase_argument.substr(0, find_location);

          if (robot_substring.empty()) {
            LOG(ERROR) << "No robot specified in the tactic argument: '"
                       << lowercase_argument << "'\n";
            unknown_flag = true;
            break;
          }

          // Get the robot ID from the tactic identifier
          errno = 0;
          int robot_int = std::stoi(robot_substring);
          int errsv = errno;
          if (errsv != 0) {
            robot_int = 0;
            LOG(ERROR) << "Failed to parse robot identifier '"
                       << robot_substring
                       << "' into integer from tactic argument: '"
                       << lowercase_argument << "' with the following error: ("
                       << strerror(errsv) << ")\n";
            unknown_flag = true;
            break;
          }

          string tactic_substring =
              lowercase_argument.substr(find_location + 1);

          if (tactic_substring.compare("h") == 0 ||
              tactic_substring.compare("halt") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::HALT));
          } else if (tactic_substring.compare("fb") == 0 ||
                     tactic_substring.compare("forwardbackward") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::FORWARD_BACKWARD));
          } else if (tactic_substring.compare("rp") == 0 ||
                     tactic_substring.compare("randompoints") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::RANDOM_POINTS));
          } else if (tactic_substring.compare("t") == 0 ||
                     tactic_substring.compare("triangle") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::TRIANGLE));
          } else if (tactic_substring.compare("g") == 0 ||
                     tactic_substring.compare("goalie") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::GOALIE));
          } else if (tactic_substring.compare("pd") == 0 ||
                     tactic_substring.compare("primarydefender") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::PRIMARY_DEFENDER));
          } else if (tactic_substring.compare("j") == 0 ||
                     tactic_substring.compare("joystick") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::JOYSTICK_CONTROLLER));
          } else if (tactic_substring.compare("pa") == 0 ||
                     tactic_substring.compare("primaryattacker") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::PRIMARY_ATTACKER));
          } else if (tactic_substring.compare("sa") == 0 ||
                     tactic_substring.compare("simpleattacker") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::SIMPLE_ATTACKER));
          } else if (tactic_substring.compare("gp") == 0 ||
                     tactic_substring.compare("guardpoint") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::GUARD_POINT));
          } else {
            LOG(ERROR) << "Unknown tactic: " << tactic_substring << "\n";
            unknown_flag = true;
          }
        }
      } break;
    }
  }

  if (unknown_flag) {
    LOG(FATAL) << "Unknown flag(s) found!\n";
  }
}

void FatalSignalHandler(int signo) {
  fprintf(stderr, "Received fatal signal %s\n", strsignal(signo));
  fflush(stderr);
  PrintStackTrace();
  exit(1);
}

std::vector<Pose2Df> ReadStartPositions(const std::string& file_path) {
  std::vector<Pose2Df> positions;
  std::ifstream infile(file_path);

  if (!infile) {
    LOG(FATAL) << "Cannot open file " << file_path;
  }

  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    float x = 0, y = 0, theta = 0;
    if (!(iss >> x >> y >> theta)) {
      break;
    }
    LOG(INFO) << "Line: " << line << " x: " << x << " y: " << y
              << " theta: " << theta;

    positions.push_back({DegToRad(theta), x, y});
  }
  return positions;
}

SimState GenerateWorldState(const float& kTimeSlice,
                            const vector<float>& ball_parameters) {
  const std::vector<Pose2Df> positions =
      ReadStartPositions("scripts/simulator_positions.txt");
  SimState start_state(positions, 0.0005f, kTimeSlice);
  if (ball_parameters.size() > 0) {
    NP_CHECK_EQ(ball_parameters.size(), 4);
    const Vector2f ball_pose = {ball_parameters[0], ball_parameters[1]};
    const Vector2f ball_vel = {ball_parameters[2], ball_parameters[3]};
    start_state.SetBallPose(ball_pose);
    start_state.SetBallVelocity(ball_vel);
  }
  return start_state;
}

void SimMode(
    const Direction& direction,
    const Team& team,
    const bool& test_plays,
    const int& refport,
    const int& visport,
    const int& cmdport,
    const int& fbport,
    const vector<SSLVisionId>& usable_cameras,
    const vector<float>& ball_parameters,
    const string& trace_file,
    const vector<pair<SSLVisionId, TacticIndex>>& default_tactic_list) {
  bitset<kNumCameras> camera_mask;
  for (const unsigned int& c : usable_cameras) {
    camera_mask.set(c);
  }

  PositionVelocityState global_position_velocity_state(
      PositionVelocityState::RobotPositionVelocity(0,
                                                   Pose2Df(0, Vector2f(0, 0)),
                                                   Pose2Df(0, Vector2f(0, 0)),
                                                   Pose2Df(0, Vector2f(0, 0)),
                                                   {0, 0, 0},
                                                   0,
                                                   0));

  // Global logging for logging on kalman update thread
  Logger global_logger;

  const float kTimeSlice = kSimulatorLatency;
  SimState start_state = GenerateWorldState(kTimeSlice, ball_parameters);
  // Three identical simulators, a hack to deal with pointer copies....
  experimental_simulator::ExperimentalSim simulator(kTimeSlice, &start_state);
  experimental_simulator::ExperimentalSim kalman_local_simulator(kTimeSlice,
                                                                 &start_state);
  experimental_simulator::ExperimentalSim executor_local_simulator(
      kTimeSlice, &start_state);
  // Scope intended to destroy threading primitives.
  {
    ThreadSafeQueue<SharedState> thread_safe_shared_state_queue;
    ThreadSafeActor<PositionVelocityState> thread_safe_position_velocity_state(
        global_position_velocity_state);
    ThreadSafeActor<Logger> thread_safe_kalman_logger(global_logger);
    ThreadSafeActor<experimental_simulator::Simulator*> thread_safe_simulator(
        &simulator);
    thread_safe_simulator.Write(&simulator);

    // Sets up the threads for Kalman update.
    // RAII takes care of ensuring everything shutsdown appropriately.
    KalmanUpdate kalman_update(camera_mask,
                               DATA_STREAM_VISION_IP,
                               visport,
                               &thread_safe_position_velocity_state,
                               &thread_safe_kalman_logger,
                               &thread_safe_shared_state_queue,
                               direction,
                               team,
                               false);

    // Sets up the threads for execution of the decision portion of the stack.
    // This is the "main" soccer thread.
    // Owns all of the AI stuff.
    // RAII takes care of ensuring everything shutsdown appropriately.
    std::exception_ptr exception;
    Executor executor(DATA_STREAM_CMD_IP,
                      cmdport,
                      refport,
                      test_plays,
                      ball_parameters,
                      trace_file,
                      &thread_safe_position_velocity_state,
                      &thread_safe_kalman_logger,
                      &thread_safe_shared_state_queue,
                      default_tactic_list,
                      team,
                      direction,
                      true,
                      &exception);

    // Start all threads.
    kalman_update.SimStart(&thread_safe_simulator, &kalman_local_simulator);
    executor.SimStart(&thread_safe_simulator, &executor_local_simulator);

    // Begin waiting for SIGINT to proceed to shutdown.
    while (!shutdown_flag) {
      // Sleep of 50 ms.
      Sleep(0.05);
      if (exception) {
        try {
          std::rethrow_exception(exception);
        } catch (TacticException e) {
          LOG(ERROR) << "Caught Exception: " << e.what();
          if (strcmp(e.what(), "Attacker Success") == 0) {
            exit_status = 0;
          } else {
            exit_status = 1;
          }
          shutdown_flag = true;
        }
      }
    }

    // Kill all threads.
    kalman_update.Stop();
    executor.Stop();

    thread_safe_simulator.Shutdown();
    thread_safe_kalman_logger.Shutdown();
    thread_safe_shared_state_queue.Shutdown();
  }
  // Threads will go out of scope and shutdown here.

  // Cleanly exit the protobuf library.
  google::protobuf::ShutdownProtobufLibrary();
  // Ensures that all log files are cleanly flushed.
  google::FlushLogFiles(google::GLOG_INFO);
}

void LiveMode(
    const Direction& direction,
    const Team& team,
    const bool& test_plays,
    const int& refport,
    const int& visport,
    const int& cmdport,
    const int& fbport,
    const vector<SSLVisionId>& usable_cameras,
    const vector<float>& ball_parameters,
    const string& trace_file,
    const vector<pair<SSLVisionId, TacticIndex>>& default_tactic_list) {
  bitset<kNumCameras> camera_mask;
  for (const unsigned int& c : usable_cameras) {
    camera_mask.set(c);
  }

  PositionVelocityState global_position_velocity_state(
      PositionVelocityState::RobotPositionVelocity(0,
                                                   Pose2Df(0, Vector2f(0, 0)),
                                                   Pose2Df(0, Vector2f(0, 0)),
                                                   Pose2Df(0, Vector2f(0, 0)),
                                                   {0, 0, 0},
                                                   0,
                                                   0));

  // Global logging for logging on kalman update thread
  Logger global_logger;

  // Scope intended to destroy threading primitives.
  {
    ThreadSafeQueue<SharedState> thread_safe_shared_state_queue;
    ThreadSafeActor<PositionVelocityState> thread_safe_position_velocity_state(
        global_position_velocity_state);
    ThreadSafeActor<Logger> thread_safe_kalman_logger(global_logger);

    // Start the configuration reader thread
    configuration_reader::CreateDaemon(
        {"src/configuration_reader/config_triangle.cfg"});

    // Sets up the threads for Kalman update.
    // RAII takes care of ensuring everything shutsdown appropriately.
    KalmanUpdate kalman_update(camera_mask,
                               DATA_STREAM_VISION_IP,
                               visport,
                               &thread_safe_position_velocity_state,
                               &thread_safe_kalman_logger,
                               &thread_safe_shared_state_queue,
                               direction,
                               team,
                               false);

    // Sets up the threads for execution of the decision portion of the stack.
    // This is the "main" soccer thread.
    // Owns all of the AI stuff.
    // RAII takes care of ensuring everything shutsdown appropriately.
    std::exception_ptr exception;
    Executor executor(DATA_STREAM_CMD_IP,
                      cmdport,
                      refport,
                      test_plays,
                      ball_parameters,
                      trace_file,
                      &thread_safe_position_velocity_state,
                      &thread_safe_kalman_logger,
                      &thread_safe_shared_state_queue,
                      default_tactic_list,
                      team,
                      direction,
                      false,
                      &exception);

    // Start all threads.
    kalman_update.Start();
    executor.Start();

    // Begin waiting for SIGINT to proceed to shutdown.
    while (!shutdown_flag) {
      // Sleep of 50 ms.
      Sleep(0.05);
      if (exception) {
        try {
          std::rethrow_exception(exception);
        } catch (TacticException e) {
          LOG(ERROR) << "Caught Exception: " << e.what();
          shutdown_flag = true;
        }
      }
    }

    // Kill all threads.
    kalman_update.Stop();
    executor.Stop();
    configuration_reader::Stop();

    thread_safe_kalman_logger.Shutdown();
    thread_safe_shared_state_queue.Shutdown();
  }
  // Threads will go out of scope and shutdown here.

  // Cleanly exit the protobuf library.
  google::protobuf::ShutdownProtobufLibrary();
  // Ensures that all log files are cleanly flushed.
  google::FlushLogFiles(google::GLOG_INFO);
}

int main(int argc, char** argv) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;   // INFO level logging.
  FLAGS_colorlogtostderr = 1;  // Colored logging.
  FLAGS_logtostderr = true;    // Don't log to disk

  if (signal(SIGINT, SigHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGINT\n";
  }

  if (signal(SIGSEGV, FatalSignalHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGSEGV\n";
  }

  if (signal(SIGABRT, FatalSignalHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGABRT\n";
  }

  // Seed rand() for use throughout the codebase.
  auto seed_value = time(0);
  srand(seed_value);

  // TODO(kvedder): Look into better defaults.
  Direction direction = Direction::POSITIVE;
  Team team = Team::BLUE;
  bool test_plays = false;
  bool simulator = false;
  int refport = (DATA_STREAM_REF_PORT);
  int visport = (DATA_STREAM_VISION_PORT);
  int cmdport = (DATA_STREAM_CMD_PORT);
  int fbport = (DATA_STREAM_FEEDBACK_PORT);
  string trace_file = "";
  vector<unsigned int> usable_cameras;
  vector<float> ball_parameters;
  vector<pair<SSLVisionId, TacticIndex>> default_tactic_list;

  // Initializes the tactics array based on the CLI input.
  ParseCLIFlags(argc,
                argv,
                &direction,
                &team,
                &simulator,
                &test_plays,
                &refport,
                &visport,
                &cmdport,
                &fbport,
                &trace_file,
                &usable_cameras,
                &ball_parameters,
                &default_tactic_list);
    if (simulator) {
      SimMode(direction,
              team,
              test_plays,
              refport,
              visport,
              cmdport,
              fbport,
              usable_cameras,
              ball_parameters,
              trace_file,
              default_tactic_list);
    } else {
      LiveMode(direction,
              team,
              test_plays,
              refport,
              visport,
              cmdport,
              fbport,
              usable_cameras,
              ball_parameters,
              trace_file,
              default_tactic_list);
    }
    const int return_status = exit_status;
    return(return_status);
}
