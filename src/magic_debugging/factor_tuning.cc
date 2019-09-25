// Copyright 2016 - 2019 kvedder@umass.edu
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
#include <bitset>
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
#include "experimental_sim/experimental_sim.h"
#include "experimental_sim/objects/ball.h"

#include "netraw_test_message.pb.h"

using app::Executor;
using app::KalmanUpdate;
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
using std::bitset;
using std::condition_variable;
using std::endl;
using std::mutex;
using pose_2d::Pose2Df;
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

// Used for handling the lifetime and eventual shutdown of the main RoboCup
// stack.
static atomic_bool shutdown_flag(false);

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
    {OptionIndex::USABLE_CAMERAS, 0, "c", "cams", Arg::UsableCameras,
     "  -c[<arg>], \t--cams[=<arg>]"
     "  \tTakes list of 6 comma seperated non-space delimited values."},
    {OptionIndex::ATTACKER_PARAMS, 0, "a", "attacker", Arg::AttackerParams,
     "  -a[<arg>], \t--attacker[=<arg>]"
     "  \tList of 7 parameters which control test attacker transitions.\n"
     "maximum angle error\n"
     "maximum distance from robot \n"
     "maximum alignment error with robot center \n"
     "maximum y_prime vel\n"
     "maximum angular vel\n"
     "maximum relative vel\n"
     "number of timesteps to timeout kick\n"},
    {OptionIndex::TESTPLAYS, 0, "tp", "testplay", Arg::TestPlays,
     "  -p[<arg>], \t--testplays[=<arg>]"
     "  \tTakes list of 6 comma seperated non-space delimited values."},
    {OptionIndex::REFPORT, 0, "r", "refport", Arg::Refport,
     "  -r[<arg>], \t--refport[=<arg>]"
     "  \tTakes single integer for RefBox port."},
    {OptionIndex::VISPORT, 0, "v", "visport", Arg::Refport,
     "  -v[<arg>], \t--visport[=<arg>]"
     "  \tTakes single integer for Vision port."},
    {OptionIndex::CMDPORT, 0, "m", "cmdport", Arg::Refport,
     "  -m[<arg>], \t--cmdport[=<arg>]"
     "  \tTakes single integer for command port."},
    {OptionIndex::FBPORT, 0, "f", "FBPORT", Arg::Refport,
     "  -f[<arg>], \t--fbport[=<arg>]"
     "  \tTakes single integer for feedback port port."},
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
                   bool* test_plays, int* refport, int* visport, int* cmdport,
                   int* fbport, vector<SSLVisionId>* usable_cameras,
                   vector<float>* attacker_params,
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
      case OptionIndex::ATTACKER_PARAMS: {
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
          LOG(ERROR) << "Incorrect number of attacker params " << result.size()
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
            attacker_params->push_back(param);
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

          if (tactic_substring.compare("s") == 0 ||
              tactic_substring.compare("stopped") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::STOPPED));
          } else if (tactic_substring.compare("bt") == 0 ||
                     tactic_substring.compare("basictest") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::BASIC_TEST_TACTIC));
          } else if (tactic_substring.compare("tc") == 0 ||
                     tactic_substring.compare("testcircle") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::CIRCLE_TEST));
          } else if (tactic_substring.compare("tp") == 0 ||
                     tactic_substring.compare("testpivot") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::PIVOT_TEST));
          } else if (tactic_substring.compare("toa") == 0 ||
                     tactic_substring.compare("testoptimalangle") == 0) {
            tactic_list->push_back(std::make_pair(
                robot_int, TacticIndex::TIME_OPTIMAL_ANGLE_TEST_TACTIC));
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
          } else if (tactic_substring.compare("ptd") == 0 ||
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
          } else if (tactic_substring.compare("j") == 0 ||
                     tactic_substring.compare("joystick") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::JOYSTICK_CONTROLLER));
          } else if (tactic_substring.compare("sua") == 0 ||
                     tactic_substring.compare("supportattacker") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::SUPPORT_ATTACKER));
          } else if (tactic_substring.compare("gtb") == 0 ||
                     tactic_substring.compare("gotoball") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::GO_TO_BALL));
          } else if (tactic_substring.compare("tbi") == 0 ||
                     tactic_substring.compare("testinterception") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::TEST_BALL_INTERCEPTION));
          } else if (tactic_substring.compare("ma") == 0 ||
                     tactic_substring.compare("mainattacker") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::MAIN_ATTACKER));
          } else if (tactic_substring.compare("gp") == 0 ||
                     tactic_substring.compare("guardpoint") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::GUARD_POINT));
          } else if (tactic_substring.compare("vc") == 0 ||
                     tactic_substring.compare("velocity_configuration") == 0) {
            tactic_list->push_back(
                std::make_pair(robot_int, TacticIndex::VELOCITY_CONFIGURATION));
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
  bool test_plays = true;
  int refport = (DATA_STREAM_REF_PORT);
  int visport = (DATA_STREAM_VISION_PORT);
  int cmdport = (DATA_STREAM_CMD_PORT);
  int fbport = (DATA_STREAM_FEEDBACK_PORT);
  vector<unsigned int> usable_cameras;
  vector<float> attacker_params;
  vector<pair<SSLVisionId, TacticIndex>> default_tactic_list;

  // Initializes the tactics array based on the CLI input.
  ParseCLIFlags(argc, argv, &direction, &team, &test_plays, &refport, &visport,
                &cmdport, &fbport, &usable_cameras, &attacker_params,
                &default_tactic_list);

  bitset<kNumCameras> camera_mask;
  for (const unsigned int& c : usable_cameras) {
    camera_mask.set(c);
  }

  PositionVelocityState global_position_velocity_state(
      PositionVelocityState::RobotPositionVelocity(
          0, Pose2Df(0, Vector2f(0, 0)), Pose2Df(0, Vector2f(0, 0)),
          Pose2Df(0, Vector2f(0, 0)), 0, 0));

  // Global logging for logging on kalman update thread
  Logger global_logger;

  // Scope intended to destroy threading primitives.
  {
    ThreadSafeQueue<SharedState> thread_safe_shared_state_queue;
    ThreadSafeActor<PositionVelocityState> thread_safe_position_velocity_state(
        global_position_velocity_state);
    ThreadSafeActor<Logger> thread_safe_kalman_logger(global_logger);

    // Sets up the threads for Kalman update.
    // RAII takes care of ensuring everything shutsdown appropriately.
    KalmanUpdate kalman_update(
        camera_mask,
        DATA_STREAM_VISION_IP,
        visport,
        &thread_safe_position_velocity_state,
        &thread_safe_kalman_logger,
        &thread_safe_shared_state_queue,
        direction,
        team);

    // Sets up the threads for execution of the decision portion of the stack.
    // This is the "main" soccer thread.
    // Owns all of the AI stuff.
    // RAII takes care of ensuring everything shutsdown appropriately.
    Executor executor(
        DATA_STREAM_CMD_IP, cmdport, refport, test_plays, attacker_params,
        &thread_safe_position_velocity_state, &thread_safe_kalman_logger,
        &thread_safe_shared_state_queue, default_tactic_list, team);

    const float kTimeSlice = 1.0f/60.0f;
    experimental_simulator::SimState start_state(12, 0.0005f, kTimeSlice);
    const Eigen::Vector2f ball_pose = {attacker_params[0], attacker_params[1]};
    const Eigen::Vector2f ball_vel = {attacker_params[2], attacker_params[3]};
    std::cout << "Ball Velocity: " << ball_vel << std::endl;
    experimental_simulator::Ball ball(start_state,
                                      {0, ball_pose},
                                      ball_vel,
                                      0.08f,
                                      kTimeSlice,
                                      0.0005f);
    start_state.SetBall(ball);
    experimental_simulator::ExperimentalSim simulator(kTimeSlice, &start_state);
    ThreadSafeActor<experimental_simulator::ExperimentalSim>
        thread_safe_simulator(simulator);
    // Start all threads.
    kalman_update.SimStart(&thread_safe_simulator, simulator);
    executor.SimStart(&thread_safe_simulator, simulator);

    // Begin waiting for SIGINT to proceed to shutdown.
    while (!shutdown_flag) {
      // Sleep of 50 ms.
      Sleep(0.05);
    }

    // Kill all threads.
    kalman_update.Stop();
    executor.Stop();

    thread_safe_kalman_logger.Shutdown();
    thread_safe_shared_state_queue.Shutdown();
    thread_safe_simulator.Shutdown();
  }
  // Threads will go out of scope and shutdown here.

  // Cleanly exit the protobuf library.
  google::protobuf::ShutdownProtobufLibrary();
  // Ensures that all log files are cleanly flushed.
  google::FlushLogFiles(google::GLOG_INFO);

  return 0;
}
