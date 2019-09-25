// Copyright 2019 kvedder@umass.edu
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

#ifndef SRC_TITTL_SIM_READER_H_
#define SRC_TITTL_SIM_READER_H_

#include <utility>

#include "messages_robocup_ssl_wrapper.pb.h"

#include "tittls/defines.h"

namespace tittls {
namespace reader {

class Reader {
  std::pair<SSLVisionProto::SSL_WrapperPacket, Time> ReadVision();
};

}  // namespace reader
}  // namespace tittls

#endif  // SRC_TITTL_SIM_READER_H_
