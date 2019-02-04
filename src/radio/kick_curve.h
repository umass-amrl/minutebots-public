// Copyright 2018 kvedder@umass.edu
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

#ifndef SRC_RADIO_KICK_CURVE_H_
#define SRC_RADIO_KICK_CURVE_H_

#include "constants/constants.h"

namespace radio {

class KickCurve {
 public:
  KickCurve();
  explicit KickCurve(const SSLVisionId& ssl_vision_id);
  ~KickCurve() = default;

  int8_t GetKickPower(const float& kick_speed) const;

 private:
  // ax^2 + bx + c
  float a_, b_, c_;
  bool initialized_;
};

}  // namespace radio

#endif  // SRC_RADIO_KICK_CURVE_H_
