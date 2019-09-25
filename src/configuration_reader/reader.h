// Copyright 2018 - 2019 ikhatri@umass.edu
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
#ifndef SRC_CONFIGURATION_READER_READER_H_
#define SRC_CONFIGURATION_READER_READER_H_

extern "C" {
#include <sys/inotify.h>
#include <sys/epoll.h>
#include <unistd.h>
}

#include <atomic>
#include <fstream>
#include <memory>
#include <thread>
#include <unordered_map>
#include <string>
#include <vector>

#include "config_types/config_double.h"
#include "config_types/config_float.h"
#include "config_types/config_int.h"
#include "config_types/config_interface.h"
#include "config_types/config_string.h"
#include "config_types/config_unsigned_int.h"
#include "config_types/config_vector2f.h"

namespace configuration_reader {
// Define constants
const char kDefaultFileName[] = "config.lua";

// Define macros for creating new config vars
#define CFG_INT(name, key) const int& CONFIG_##name = InitInt(key)
#define CFG_UINT(name, key) \
  const unsigned int& CONFIG_##name = InitUnsignedInt(key)
#define CFG_DOUBLE(name, key) const double& CONFIG_##name = InitDouble(key)
#define CFG_FLOAT(name, key) const float& CONFIG_##name = InitFloat(key)
#define CFG_STRING(name, key) const std::string& CONFIG_##name = InitString(key)
#define CFG_VECTOR2F(name, key) \
  \
const Eigen::Vector2f& CONFIG_##name = InitVector2f(key)

void LuaRead(std::vector<std::string> files);
const int& InitInt(std::string key);
const unsigned int& InitUnsignedInt(std::string key);
const double& InitDouble(std::string key);
const float& InitFloat(std::string key);
const std::string& InitString(std::string key);
const Eigen::Vector2f& InitVector2f(std::string key);
void HelpText();
void InitDaemon(const std::vector<std::string>& files);
void CreateDaemon(const std::vector<std::string>& files);
void Stop();

extern const Eigen::Vector2f& CONFIG_triangle_test;
extern const double& CONFIG_kHardwareLagTranslation;
extern const double& CONFIG_kLatency;
extern const double& CONFIG_kSimulatorLatency;
extern const double& CONFIG_t_reg_ratio;
extern const double& CONFIG_min_v_cost_coef;
extern const double& CONFIG_linear_activation_noise;
extern const double& CONFIG_angular_activation_noise;
extern const double& CONFIG_angular_deviation_noise;
extern const double& CONFIG_activation_drift_angle;
extern const double& CONFIG_drift_angle_weight;
extern const double& CONFIG_kDefaultRobotAcceleration;
extern const double& CONFIG_k1;
extern const double& CONFIG_k2;
extern const double& CONFIG_kSimulatorStepSize;
extern const unsigned int& CONFIG_kSimulatorControlQueueSize;

extern const double& CONFIG_ntoc_pd_position_threshold;
extern const double& CONFIG_ntoc_pd_velocity_threshold;

extern const double& CONFIG_pd_translation_proportional;
extern const double& CONFIG_pd_translation_derivative;

extern const double& CONFIG_tsocs_xf;
extern const double& CONFIG_tsocs_yf;
extern const double& CONFIG_tsocs_vxf;
extern const double& CONFIG_tsocs_vyf;

}  // namespace configuration_reader
#endif  // SRC_CONFIGURATION_READER_READER_H_
