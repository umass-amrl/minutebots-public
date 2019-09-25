// Copyright 2019 jspitzer@cs.umass.edu, kvedder@umass.edu
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
#include <atomic>
#include <thread>

#include "constants/constants.h"
#include "logging/logger.h"
#include "math/poses_2d.h"
#include "obstacles/obstacle_flag.h"
#include "safety/dss2.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/ntoc_controller.h"
#include "tactics/tactic.h"
#include "tactics/three_kick.h"

#ifndef SRC_TACTICS_MARIONETTE_H_
#define SRC_TACTICS_MARIONETTE_H_

namespace tactics {
class Marionette : public Tactic {
 public:
  Marionette(const state::WorldState& world_state,
             TacticArray* tactic_list,
             state::SharedState* shared_state,
             OurRobotIndex our_robot_index,
             state::SoccerState* soccer_state);

  ~Marionette();

  const char* Name() const override { return "marionette"; }
  void Init() override;
  void Run() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  void Reset() override;
  bool ShouldKick();

  struct PuppetData {
    float x;
    float y;
    float theta;
    bool kick;
    bool halt;

    PuppetData() noexcept : x(0), y(0), theta(0), kick(false), halt(true) {}
    PuppetData(const float& x, const float& y, const float& theta,
      const bool& kick, const bool& halt)
        : x(x), y(y), theta(theta), kick(kick), halt(halt) {}
  };
  static_assert(std::is_trivially_copyable<PuppetData>::value,
                "Not trivially copyable.");

 private:
  net::UDPMulticastServer udp_server_;
  std::atomic<PuppetData> shared_puppet_data_;
  std::atomic_bool needs_shutdown_;
  std::atomic_uint ssl_vision_id_;
  std::thread recieve_thread_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_MARIONETTE_H_
