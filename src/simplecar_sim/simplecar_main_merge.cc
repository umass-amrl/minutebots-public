// Copyright 2019 jaholtz@cs.umass.edu
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

#include "simplecar_sim/simplecar_sim.h"
#include <signal.h>
#include "util/timer.h"
#include "logging/logger.h"
#include "simplecar_sim/simplecar_control.h"
#include "simplecar_sim/simplecar_merge.h"

using simplecar::Simplecar;
using simplecar::SimplecarSim;
using simplecar::SimplecarState;
using simplecar::SimplecarController;
using simplecar::SimplecarMerge;
using simplecar::SimplecarCommand;
using std::vector;
using logger::NetLogger;

const float kLoopRate = 62.503125;

bool is_running_ = true;

void SigHandler(int signo) {
  if (signo == SIGINT) {
    is_running_ = false;
  }
}

void SetupRoads(NetLogger* logger) {
  const float right = 1500;
  const float left = -1500;
  const float middle = 0.0;
  const float lower = -50000.0;
  const float upper = 50000.0;
  const float step_size = 500.0;
  logger->AddLine({left, lower}, {left, upper}, 1.0, 1.0, 1.0, 1.0);
  logger->AddLine({right, lower}, {right, 0.0}, 1.0, 1.0, 1.0, 1.0);
  vector<Eigen::Vector2f> points;
  for (float i = lower; i <= upper; i += step_size) {
    points.push_back({middle, i});
  }
  logger->AddPoints(points, 1.0, 1.0, 1.0, 1.0);
}

void DrawCars(const vector<Simplecar>& cars, NetLogger* logger) {
  for (Simplecar car : cars) {
    const Vector2f translation = car.GetPose().translation;
    const float angle = car.GetPose().angle;
    logger->AddEllipse(translation,
                       kCarLength,
                       kCarWidth,
                       angle,
                       0.0,
                       0.0,
                       1.0,
                       1.0);
  }
}

static const float kGoodSpeed = 1.0;
bool BadDriving(const Simplecar& player, const Simplecar& car) {
  // Is simplecar ever too close to other car.
  // This is a heuristic, it isn't perfect.
  const Eigen::Vector2f distance =
    player.GetPose().translation - car.GetPose().translation;

  // This is the minmium radius of the car
//   std::cout << "Collision Dist: " << distance.norm() << endl;
  if (distance.norm() < 1200.0) {
    return true;
  }

  // Too fast period?
  if (player.GetSpeed() > kGoodSpeed) {
    return true;
  }
  return false;
}

static int timer = 0;
static const int timeout = 5000;
bool CheckFailure(const int& player_id,
                  const vector<Simplecar>& cars) {
  const Simplecar player = cars[player_id];

  if (timer > timeout) {
    return true;
  }

  // Check all the cars for collision
  for (Simplecar car : cars) {
    if (car.GetId() != player_id) {
      const bool bad =  BadDriving(player, car);
      if (bad) {
        return true;
      }
    }
  }

  // Track the number of frames and kill it if it's been too long.
  timer++;
  return false;
}

int main(int argc, char** argv) {
  is_running_ = true;
  SimplecarSim sim(kLoopRate, "src/simplecar_sim/car_setup.txt");
  NetLogger net_logger(NetLogger("127.0.0.1", 10108));

  RateLoop loop(120.0);
  SimplecarMerge controller = SimplecarMerge("SimplecarMerge", 0);
  while (is_running_) {
    // Used to maintain a constant transmit rate.
    SimplecarState world = sim.GetWorldState();
    SetupRoads(&net_logger);
    const vector<Simplecar> cars = world.GetCars();
    // Visualize the world state.
    DrawCars(cars, &net_logger);
    // Send a message off to the viewer
    net_logger.SendData();
    net_logger.Clear();

    // Do some car stuff?
    controller.UpdateState(world);
    controller.Run();
    SimplecarCommand command = controller.GetControls();
    // Simulate the new command
    sim.SimulateStep(command);
    net_logger.AddStateMachineData(controller.GetTransitionLog());
    if (CheckFailure(0, cars)) {
      is_running_ = false;
      std::cout << "Bad Driving" << std::endl;
      return 1;
    }
    if (controller.FreeFlying()) {
      is_running_ = false;
      std::cout << "Flying Free" << std::endl;
      return 0;
    }
    // Sleep to make this visible. Remove if we need to run this
    // a million times.
    loop.Sleep();
  }
}
