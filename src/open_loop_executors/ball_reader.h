// Copyright 2018 - 2019 kvedder@umass.edu
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

#ifndef SRC_OPEN_LOOP_EXECUTORS_BALL_READER_H_
#define SRC_OPEN_LOOP_EXECUTORS_BALL_READER_H_

#include <atomic>
#include <thread>
#include <utility>
#include <vector>

#include "constants/typedefs.h"
#include "math/poses_2d.h"
#include "net/netraw.h"
#include "state/team.h"
#include "thread_safe/thread_safe_actor.h"

namespace open_loop {
struct BallObservation {
  BallObservation() : observations(), time(0) {}
  std::vector<std::pair<Eigen::Vector2f, float>> observations;
  double time;
};

class BallReader {
 public:
  BallReader(
      const unsigned int camera_id,
      threadsafe::ThreadSafeActor<BallObservation>* thread_safe_observation);
  void Start();
  void Stop();
  void Run();

 private:
  net::UDPMulticastServer vision_client_;
  std::thread update_thread_;
  unsigned int camera_id_;
  threadsafe::ThreadSafeActor<BallObservation>* thread_safe_observation_;
  std::atomic_bool is_running_;
};
}  // namespace open_loop

#endif  // SRC_OPEN_LOOP_EXECUTORS_BALL_READER_H_
