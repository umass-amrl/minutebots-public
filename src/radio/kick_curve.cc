// Copyright 2018 - 2019 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//========================================================================
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
//========================================================================

#include "radio/kick_curve.h"

#include <fstream>
#include <limits>
#include <string>

#include "math/math_util.h"

namespace radio {

static constexpr float kMinimumKickPower = 53.0f;

std::string GetFileName(const SSLVisionId& ssl_vision_id) {
  return "src/configs/kick_curve_id_" + std::to_string(ssl_vision_id) + ".txt";
}

KickCurve::KickCurve() : a_(0), b_(0), c_(0), initialized_(false) {}

KickCurve::KickCurve(const SSLVisionId& ssl_vision_id) : KickCurve() {
  const auto file_name = GetFileName(ssl_vision_id);
  std::fstream file(file_name);
  if (!file) {
    LOG(INFO) << "Cannot find kick curve for robot " << ssl_vision_id << " ("
              << file_name << ")";
    return;
  }

  file >> c_ >> b_ >> a_;
  initialized_ = true;
}

int8_t ClampToValidKickRange(const float& f) {
  return static_cast<int8_t>(
      math_util::Clamp(f,
                       kMinimumKickPower,
                       static_cast<float>(std::numeric_limits<int8_t>::max())));
}

int8_t KickCurve::GetKickPower(const float& x) const {
  if (!initialized_) {
    const auto result = ClampToValidKickRange(x);
    LOG(INFO) << "Kick power not initialized: " << static_cast<int>(result);
    return result;
  }
  const float fp_solution = a_ * math_util::Sq(x) + b_ * x + c_;
  const auto result = ClampToValidKickRange(fp_solution);
  //   LOG(INFO) << "Kick power computed " << x << " => "
  //             << static_cast<int>(result);
  return result;
}

}  // namespace radio
