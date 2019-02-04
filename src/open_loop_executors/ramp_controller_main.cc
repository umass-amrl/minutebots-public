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

#include <signal.h>

#include <atomic>
#include <iostream>
#include <thread>

#include "constants/constants.h"
#include "constants/includes.h"
#include "constants/typedefs.h"
#include "open_loop_executors/camera_reader.h"
#include "open_loop_executors/ramp_controller.h"
#include "state/team.h"

STANDARD_USINGS;
using open_loop::CameraReader;
using open_loop::Observation;
using open_loop::RampController;
using team::Team;
using threadsafe::ThreadSafeActor;
using std::atomic_bool;
using std::cout;
using std::endl;

static atomic_bool shutdown_flag(false);

void SigHandler(int signo) {
  if (signo == SIGINT) {
    shutdown_flag = true;
  }
}

void PrintUsage(char** argv) {
  cout << "Usage: " << argv[0] << " team (y or b) robot_id cam_id translate(y ";
  cout << "or n) hardware_lab(y or n)" << endl;
  exit(0);
}

int main(int argc, char** argv) {
  Team team = Team::BLUE;
  SSLVisionId robot_id = 1;
  unsigned int cam_id = 4;
  bool is_translation = true;

  if (argc < 5 || argc > 5) {
    PrintUsage(argv);
  }
  //   else {
  //     if (argv[1]->compare("y")) {
  //     } else if (argv[1]->compare("b")) {
  //     } else {
  //       PrintUsage(argv);
  //     }
  //   }

  Observation observation;
  ThreadSafeActor<Observation> thread_safe_observation(observation);

  CameraReader reader(team, robot_id, cam_id, &thread_safe_observation);
  RampController controller(robot_id, is_translation, &thread_safe_observation);

  reader.Start();
  controller.Start();

  while (!shutdown_flag) {
    // Sleep of 50 ms.
    Sleep(0.05);
  }

  // Kill all threads.
  reader.Stop();
  controller.Stop();

  thread_safe_observation.Shutdown();
}
