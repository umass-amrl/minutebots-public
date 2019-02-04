// Copyright 2016 - 2017 kvedder@umass.edu, srabiee@cs.umass.edu
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
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>

#include "constants/constants.h"
#include "logging/logger.h"
#include "plays/skills_tactics_plays.h"
#include "soccer/executor.h"
#include "soccer/kalmanupdate.h"
#include "soccer/sslvisioninputhandler.h"
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
#include "thread_safe/thread_safe_priority_queue.h"
#include "thread_safe/thread_safe_queue.h"

#include "netraw_test_message.pb.h"

using app::Executor;
using app::KalmanUpdate;
using app::SSLVisionInputHandler;
using colorize::ColorCyan;
using colorize::ColorGreen;
using direction::Direction;
using logger::Logger;
using SSLVisionProto::SSL_WrapperPacket;
using state::SharedState;
using state::SoccerState;
using state::WorldState;
using state::PositionVelocityState;
using std::atomic_bool;
using std::condition_variable;
using std::endl;
using std::mutex;
using std::pair;
using std::string;
using tactics::Tactic;
using tactics::TacticIndex;
using std::unique_lock;
using std::vector;
using team::Team;
using option::Arg;
using option::OptionIndex;
using threadsafe::ThreadSafePriorityQueue;
using threadsafe::ThreadSafeQueue;
using threadsafe::ThreadSafeActor;

// Input address for Vision data from SSL Vision.
static const char kInputUDPAddress[] = DATA_STREAM_VISION_IP;
static const int kInputUDPPort = DATA_STREAM_VISION_PORT;

// Output address for Command data.
static const char kOutputUDPAddress[] = DATA_STREAM_CMD_IP;
static const int kOutputUDPPort = DATA_STREAM_CMD_PORT;

// Rate at which the thread polls for data from SSL_Vision.
static const float kReadLoopRate = 120;  // hz

// Used for handling the lifetime and eventual shutdown of the main RoboCup
// stack.
static atomic_bool shutdown_flag(false);
static mutex shutdown_mutex;
static condition_variable shutdown_access_cv;

// TODO(kvedder): Decide on better constants for CLI parse option array sizes.
const int kMaxOptions = 1000;
const int kMaxBuffer = 1000;

void SigHandler(int signo) {
  if (signo == SIGINT) {
    LOG(INFO) << ColorCyan("Recieved SIGINT, flagging for shutdown") << "\n";
    shutdown_flag = true;
    shutdown_access_cv.notify_all();
  }
}

// See http://optionparser.sourceforge.net/example__arg_8cc.html
// for an example. I am so sorry about the mess...
const option::Descriptor usage[] = {
    {OptionIndex::UNKNOWN, 0, "", "", Arg::Unknown,
     "USAGE: soccer [options]\n\nOptions:"},
    {OptionIndex::HELP, 0, "", "help", Arg::None,
     "  \t--help  \tPrint usage and exit."},
    {OptionIndex::DIRECTION, 0, "d", "direction", Arg::Direction,
     "  -d[<arg>], \t--direction[=<arg>]"
     "  \tTakes Positive ('p' or 'positive') or Negative ('n' or 'negative')."},
    {OptionIndex::TACTIC, 0, "", "tactic", Arg::Tactic,
     "  --tactic[=<arg>]"
     "  \tDefaults to stopped. Takes ForwardBackward ('fb' or "
     "'forwardbackward')"
     " or Stopped ('s' or 'stopped')."},
    {OptionIndex::TEAM, 0, "te", "team", Arg::Team,
     "  -t[<arg>], \t--team[=<arg>]"
     "  \tTakes Yellow ('y' or 'yellow') or Blue ('b' or 'blue')."},
    {OptionIndex::IDS, 0, "i", "ids", Arg::Ids,
     "  -i[<arg>], \t--ids[=<arg>]"
     "  \tTakes list of 6 comma seperated non-space delimited values."},
    {OptionIndex::NUMERIC, 0, "m", "mode", Arg::Numeric,
     "  -m[<arg>], \t--mode[=<arg>]"
     "  \tTakes the test mode for ball state estimation.\n"
     "  \t1 for manual chip kick detection test: offline (default)\n"
     "  \t2 for manual chip kick detection test: real-time\n"
     "  \t3 for in-the-game chip kick detection test (partial optimization)\n"
     "  \t4 for in-the-game chip kick detection test (full optimization)\n"
     "  \t5 for in-the-game comparison of full and partial optimization"
     "  \t6 for full optimization-based camera param tuning\n"
     "  \t7 for partial optimization-based camera param tuning\n"},
    {OptionIndex::UNKNOWN, 0, "", "", Arg::None,
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

void ParseCLIFlags(int argc, char** argv, Direction* direction, Team* team,
                   vector<SSLVisionId>* robot_hardware_ids, uint* test_mode,
                   uint* tuning_mode,
                   vector<pair<SSLVisionId, TacticIndex>>* tactic_list) {
  // Skip program name, argv[0], if present
  argc -= (argc > 0);
  argv += (argc > 0);

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

  bool unknown_flag = false;

  for (int i = 0; i < parse.optionsCount(); ++i) {
    option::Option& opt = buffer[i];

    string lowercase_argument = string(opt.arg);
    std::transform(lowercase_argument.begin(), lowercase_argument.end(),
                   lowercase_argument.begin(), ::tolower);

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
      case OptionIndex::IDS: {
        std::stringstream ss(lowercase_argument);
        vector<string> result;

        while (ss.good()) {
          string substr;
          std::getline(ss, substr, ',');
          result.push_back(substr);
        }

        for (const string& str : result) {
          LOG(INFO) << str << "\n";
        }

        if (result.size() != kMaxTeamRobots) {
          LOG(ERROR) << "Incorrect number of IDs: " << result.size() << "\n";
          unknown_flag = true;
          break;
        } else {
          for (const string& elem : result) {
            errno = 0;
            int robot_int = std::atoi(elem.c_str());
            int errsv = errno;
            if (errsv != 0) {
              LOG(ERROR) << "Failed to parse robot ID '" << elem
                         << "' into integer from tactic argument: '" << elem
                         << "' with the following error: (" << strerror(errsv)
                         << ")\n";
              unknown_flag = true;
              break;
            }
            robot_hardware_ids->push_back(robot_int);
          }
        }
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
          int robot_int = std::atoi(robot_substring.c_str());
          int errsv = errno;
          if (errsv != 0) {
            LOG(ERROR) << "Failed to parse robot identifier '"
                       << robot_substring
                       << "' into integer from tactic argument: '"
                       << lowercase_argument << "' with the following error: ("
                       << strerror(errsv) << ")\n";
            unknown_flag = true;
            break;
          }

          LOG(INFO) << "Robot Int: " << robot_int << "\n";

          string tactic_substring =
              lowercase_argument.substr(find_location + 1);

          if (tactic_substring.compare("s") == 0 ||
              tactic_substring.compare("stopped") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::STOPPED));
          } else if (tactic_substring.compare("fb") == 0 ||
                     tactic_substring.compare("forwardbackward") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::FORWARD_BACKWARD));
          } else if (tactic_substring.compare("t") == 0 ||
                     tactic_substring.compare("triangle") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::TRIANGLE));
          } else if (tactic_substring.compare("g") == 0 ||
                     tactic_substring.compare("goalie") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::GOALIE));
          } else if (tactic_substring.compare("sa") == 0 ||
                     tactic_substring.compare("simpleattacker") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::SIMPLE_ATTACKER));
          } else if (tactic_substring.compare("pd") == 0 ||
                     tactic_substring.compare("primarydefender") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::PRIMARY_DEFENDER));
          } else if (tactic_substring.compare("a") == 0 ||
                     tactic_substring.compare("attacker") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::ATTACKER));
          } else if (tactic_substring.compare("pd") == 0 ||
                     tactic_substring.compare("passtodeflect") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::PASS_TO_DEFLECT));
          } else if (tactic_substring.compare("d") == 0 ||
                     tactic_substring.compare("deflection") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::DEFLECTION));
          } else if (tactic_substring.compare("bb") == 0 ||
                     tactic_substring.compare("bodyblock") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::BODY_BLOCK));
          } else {
            LOG(ERROR) << "Unknown tactic: " << tactic_substring << "\n";
            unknown_flag = true;
          }
        }
      } break;

      case OptionIndex::NUMERIC: {
        if (lowercase_argument.compare("1") == 0) {
          *test_mode = 1;
        } else if (lowercase_argument.compare("2") == 0) {
          *test_mode = 2;
        } else if (lowercase_argument.compare("3") == 0) {
          *test_mode = 3;
        } else if (lowercase_argument.compare("4") == 0) {
          *test_mode = 4;
        } else if (lowercase_argument.compare("5") == 0) {
          *test_mode = 5;
        } else if (lowercase_argument.compare("6") == 0) {
          *tuning_mode = 1;
          *test_mode = 0;
        } else if (lowercase_argument.compare("7") == 0) {
          *tuning_mode = 2;
          *test_mode = 0;
        } else {
          LOG(ERROR) << "Unknown test_mode argument: " << lowercase_argument
                     << "\n";
          unknown_flag = true;
        }
      } break;
    }
  }

  for (int i = 0; i < parse.nonOptionsCount(); ++i) {
    LOG(INFO) << "Non-option argument #" << i << " is "
              << string(parse.nonOption(i)) << "\n";
  }

  if (unknown_flag) {
    LOG(FATAL) << "Unknown flag(s) found!\n";
  }

  if (robot_hardware_ids->size() != kMaxTeamRobots) {
    LOG(FATAL) << "Missing robot IDs\n";
  }
}

int main(int argc, char** argv) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;   // INFO level logging.
  FLAGS_colorlogtostderr = 1;  // Colored logging.
  FLAGS_logtostderr = true;  // Don't log to disk.

  if (signal(SIGINT, SigHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGINT\n";
  }

  // Seed rand() for use throughout the codebase.
  auto seed_value = time(0);
  LOG(INFO) << "Global RNG Seed: " << seed_value << "\n";
  srand(seed_value);

  // TODO(kvedder): Look into better defaults.
  Direction direction = Direction::POSITIVE;
  Team team = Team::BLUE;
  vector<SSLVisionId> robot_ids;
  vector<pair<SSLVisionId, TacticIndex>> default_tactic_list;
  uint test_mode = 1;
  uint tuning_mode = 0;

  // Initializes the tactics array based on the CLI input.
  ParseCLIFlags(argc, argv, &direction, &team, &robot_ids, &test_mode,
                &tuning_mode, &default_tactic_list);

  LOG(INFO) << ColorGreen("Starting RoboCup SSL Stack") << "\n";

  ThreadSafePriorityQueue<SSL_WrapperPacket, double>
      thread_safe_ssl_vision_queue;

  WorldState global_world_state(team, robot_ids);
  ThreadSafeActor<WorldState> thread_safe_world_state(global_world_state);

  Logger global_logger;
  ThreadSafeActor<Logger> thread_safe_kalman_logger(global_logger);

  ThreadSafeQueue<SharedState> thread_safe_shared_state_queue;

  //   // Holds all information regarding the state of the world.
  //   // Must only be accessed via its mutex.
  //   WorldState global_world_state(team, robot_ids);
  //   // Mutex protecting the internal state.
  //   mutex world_state_mutex;
  //   // Condition variable to alert waiting threads that the queue is not
  //   empty.
  //   condition_variable world_state_cv;
  //   // Bool for ensuring wakeups from world_state_cv aren't spurrious.
  //   atomic_bool world_state_lock_read(false);

  {
    // Sets up the networking threads for I/O.
    // RAII takes care of ensuring everything shutsdown appropriately.
    SSLVisionInputHandler input_handler(kInputUDPAddress, kInputUDPPort,
                                        kReadLoopRate,
                                        &thread_safe_ssl_vision_queue);

    // Sets up the threads for Kalman update.
    // RAII takes care of ensuring everything shutsdown appropriately.
    KalmanUpdate kalman_update(
        &thread_safe_ssl_vision_queue, &thread_safe_world_state,
        &thread_safe_kalman_logger, &thread_safe_shared_state_queue,
        global_world_state, direction);

    // Sets up the threads for execution of the decision portion of the stack.
    // This is the "main" soccer thread.
    // Owns all of the AI stuff.
    // RAII takes care of ensuring everything shutsdown appropriately.
    Executor executor(kOutputUDPAddress, kOutputUDPPort, false,
                      &thread_safe_world_state, &thread_safe_kalman_logger,
                      &thread_safe_shared_state_queue, global_world_state,
                      default_tactic_list);

    // Start all threads.
    input_handler.Start();
    kalman_update.Start();
    // Set the type of test
    // Manual Chip kick Estimation Test (offline): 1
    // Manual Chip kick Estimation Test (real-time): 2
    kalman_update.state_estimation_test_type = test_mode;
    kalman_update.camera_param_tuning_mode = tuning_mode;
    // Disable navigation and tactic stuff
    //     executor.Start();

    // Begin waiting for SIGINT to proceed to shutdown.
    {
      // Transfer from the output queue to the local queue.
      unique_lock<mutex> guard(shutdown_mutex);

      // Block until woken up.
      // Check to make sure that the wakeup isn't spurrious.
      while (!shutdown_flag) {
        shutdown_access_cv.wait(guard);
      }
    }  // Lock loses scope here.
  }
  // Threads will go out of scope and shutdown here.

  LOG(INFO) << ColorGreen("Shutdown all threads") << "\n";

  // Cleanly exit the protobuf library.
  google::protobuf::ShutdownProtobufLibrary();
  LOG(INFO) << ColorGreen("Shutdown protobuf library!") << endl;
  // Ensures that all log files are cleanly flushed.
  google::FlushLogFiles(google::GLOG_INFO);
  LOG(INFO) << ColorGreen("Flushed log files!") << endl;
  LOG(INFO) << ColorGreen("Exiting...") << endl;

  return 0;
}
