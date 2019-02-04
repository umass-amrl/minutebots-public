// Copyright 2018 ikhatri@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
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
#ifndef SRC_CONFIGURATION_READER_CONFIG_TYPES_CONFIG_INT_H_
#define SRC_CONFIGURATION_READER_CONFIG_TYPES_CONFIG_INT_H_

#include "configuration_reader/config_types/config_interface.h"
#include <string>

namespace configuration_reader {
namespace config_types {
class ConfigInt : public ConfigInterface {
 public:
  explicit ConfigInt(std::string key_name);
  ConfigInt(std::string key_name, int upper_bound, int lower_bound);
  int& GetVal();
  bool SetVal(LuaScript* script);

 protected:
  int upper_bound_;
  int lower_bound_;
  int val_;
};
}  // namespace config_types
}  // namespace configuration_reader
#endif  // SRC_CONFIGURATION_READER_CONFIG_TYPES_CONFIG_INT_H_
