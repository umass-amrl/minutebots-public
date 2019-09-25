// Copyright 2018 - 2019 kvedder@umass.edu, joydeepb@cs.umass.edu
//
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Constants for UMass RoboCup SSL robots.
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
#ifndef SRC_CONSTANTS_CONSTANTS_H_
#define SRC_CONSTANTS_CONSTANTS_H_

#include <cmath>
#include <string>
#include <vector>

#include "constants/includes.h"
#include "constants/typedefs.h"
#include "math/math_util.h"

// Used to define if debug behavior should be allowed.
// Example of debug behavior: LOG(FATAL) and quitting upon error versus allow
// undefined behavior. Compile time constant to allow the compiler to optimize
// away undesired checks.
static constexpr bool kProduction = false;

enum FieldSetup { LAB, B_LEAGUE, A_LEAGUE };

// Bool to indicate whether this is set to run in
// the lab. This needs to get set to false at RoboCup!!!
const FieldSetup kFieldSetup = LAB;

extern const char DATA_STREAM_VISION_IP[];
extern const int DATA_STREAM_VISION_PORT;

extern const char DATA_STREAM_CMD_IP[];
extern const int DATA_STREAM_CMD_PORT;

extern const char DATA_STREAM_REF_IP[];
extern const int DATA_STREAM_REF_PORT;

extern const char DATA_STREAM_DEBUG_IP[];
extern const int DATA_STREAM_DEBUG_PORT;

extern const char DATA_STREAM_FEEDBACK_IP[];
extern const int DATA_STREAM_FEEDBACK_PORT;

extern const char DATA_STREAM_VIEWER_IP[];
extern const int DATA_STREAM_VIEWER_PORT;

#define STANDARD_USINGS  \
  using Eigen::Vector2f; \
  using std::cout;       \
  using std::endl;       \
  using std::string;     \
  using std::vector;

#define NP_CHECK(exp) \
  if (!kProduction) { \
    CHECK(exp);       \
  }

#define NP_CHECK_MSG(exp, message) \
  if (!kProduction) {              \
    CHECK(exp) << ": " << message; \
  }

#define NP_NOT_NAN(exp)      \
  if (!kProduction) {        \
    CHECK(!std::isnan(exp)); \
  }

#define NP_NOT_NULL(exp) \
  if (!kProduction) {    \
    CHECK_NOTNULL(exp);  \
  }

#define NP_CHECK_EQ(exp1, exp2) \
  if (!kProduction) {           \
    CHECK_EQ(exp1, exp2);       \
  }

#define NP_CHECK_EQ_MSG(exp1, exp2, message) \
  if (!kProduction) {                        \
    CHECK_EQ(exp1, exp2) << ": " << message; \
  }

namespace team_management {
// Their team robot timeout in seconds.
extern const float kTheirTeamBotTimeout;  // Seconds.

// Our team robot timeout in seconds.
extern const float kOurTeamBotTimeout;  // Seconds.
}  // namespace team_management

extern const float kEpsilon;

// Hack to fix the fact that C++11 decided that std::numeric_limits<int>::max()
// is not a constexpr... This is fixed in C++14...
static constexpr int kIntMaxHack = 2147483647;

const unsigned int kMaxTeamRobots = 8;
const unsigned int kNumTeams = 2;
const unsigned int kNumSSLVisionIds = 12;

static constexpr float kSqrtTwo = 1.41421356237f;

extern const bool kLogNavigation;

extern const unsigned int kFrameQueueSize;
extern const unsigned int kLatencyCalibrationQueueSize;

extern const float kRobotRadius;
extern const float kBallRadius;
extern const float kMediumBallRadius;
extern const float kLargeBallRadius;
extern const float kRobotFaceRadius;
extern const float kHalfKickerWidth;
extern const float kAlignmentThreshold;
extern const float kAttackerFieldMargin;

extern const double kBallZeroHeight;

// Max velocity of the robots in mm/s.
extern const float kMaxRobotVelocity;

// Default velocity of the robots in mm/s
extern const float kDefaultRobotVelocity;

// Max velocity of the robots in mm/s the minutebot radio is willing to sent.
extern const float kMaxRobotRadioVelocityCommand;

// Max acceleration of the robots in mm/s^2
extern const float kMaxRobotAcceleration;

// Default acceleration of the robots in mm/s^2
extern float kDefaultRobotAcceleration;

// Max rotational velocity of the robots in rad/s
extern const float kMaxRobotRotVel;

// Max rotational velocity of the robots in rad/s the minutebot radio
// is willing to sent.
extern const float kMaxRobotRadioRotVelCommand;

// Max rotational acceleration of the robots in rad/s^2
extern const float kMaxRobotRotAccel;

// Constants to limit rotational velocity when travelling quickly
// translationally
extern const float kRotSpeedLimitThreshold;
extern const float kLowerRobotRotVel;

// Temporary values, must measure precisely
extern const float kBallAcceleration;  // in mm/s^2
extern const float kCollisionDamping;
extern const float kKickBoost;  // in mm/s
extern const float kKickDetectionThreshold;

// Estimated time used for forward prediction in seconds
// Should be the time between an image being captured and a command executing
// Set this to something low in simulation
// Set this to 0.14 on real hardware
extern double kLatency;
extern double kSimulatorLatency;
extern double kSimulatorStepSize;
extern unsigned int kSimulatorControlQueueSize;
extern const double kBallLatency;

// Lag in seconds that the hardware has in activation.
// Sample usecase: Extending DSS control period to account for the hardware
// activation lag.
extern double kHardwareLagTranslation;
extern const double kHardwareLagRotation;

// Percent of individual robot updates the simulator should randomly drop.
// On a scale from 0 (no update drop) to 1 (drop all updates).
extern const double kSimulatorPacketLossPercent;

// Standard deviation of the guassian used to fit over the position of the
// robots.
extern const double kSimulatorRobotNoiseStdDev;
extern const double kSimulatorRobotAngleNoiseStdDev;

extern const float kDefaultSafetyMargin;
extern const float kDefaultRulesSafetyMargin;

constexpr unsigned int kNumCameras =
    (kFieldSetup == LAB) ? 4 : ((kFieldSetup == A_LEAGUE) ? 8 : 4);
constexpr unsigned int kNumRulesObstacles = 8;
constexpr unsigned int kNumStaticObstacles = 8;
constexpr unsigned int kNumRobotObstacles = kMaxTeamRobots * kNumTeams;
constexpr unsigned int kNumBallObstacles = 3;
constexpr const unsigned int kNumObstacles =
    kMaxTeamRobots * kNumTeams + kNumBallObstacles + kNumRulesObstacles +
    kNumStaticObstacles;

// a robot state is the x-y-theta pose + the time derivatives
// DimSize refers to the number of dimensions, without including derivatives
const unsigned int kRobotStateSize = 6;
const unsigned int kRobotStateDimSize = 3;

extern const float kImprobabilityRejectionThreshold;
extern const float kConfidenceThreshold;

// Transmit frequency of the radio to the robot.
extern const float kTransmitFrequency;  // Hz
// Time delay between successive radio commands.
extern const float kTransmitPeriod;  // milliseconds
extern const float kControlPeriodTranslation;
extern const float kControlPeriodRotation;
extern const float kTransmitPeriodSeconds;  // seconds

// How far ahead should we set the angular waypoint.
// Used in control. kControlPeriod is multiplied by this.
extern const float kAnglePlannerLookahead;

// How far ahead of the translational control should we arrive at the angular
// target (s)
extern const float kAnglePlannerDelta;

extern const int kMaxSearchDepth;  // Max search depth for STOx planner

extern const float kSearchMargin;  // Search margin for PRM

extern const unsigned int kNumVertices;  // Default number of PRM vertices
// Default PRM scaffold connect radius
extern const float kScaffoldConnectRadius;

extern const float kMaxDeltaV;
extern const float kMaxDeltaRotV;

// constants for the circle / pivot tactic
extern const float kRadiusBuffer;
extern const float kRotationRadius;
extern const float kRotationRadiusMargin;

// constants for ball interception
extern const float kInterceptionRadius;
extern const float kInterceptionMargin;

// tactic wide constants
extern const float kDistanceThreshold;
extern const float kAngleThreshold;
extern const float kLinearVelocityThreshold;
extern const float kAngularVelocitythreshold;
extern const float kPullRightAdjustment;

namespace field_dimensions {
extern const float kFieldLength;
extern const float kFieldWidth;
extern const float kHalfFieldLength;
extern const float kHalfFieldWidth;
extern const float kGoalWidth;
extern const float kGoalDepth;
extern const float kBoundaryWidth;
extern const float kFieldLineWidth;
extern const float kDefenseStretch;
extern const float kCenterCircleRadius;
extern const float kFieldBoundary;
}  // namespace field_dimensions

extern const float kFieldXMax;
extern const float kFieldYMax;

// Field maxes plus robot diameter
extern const float kInflatedFieldXMax;
extern const float kInflatedFieldYMax;

extern const Eigen::Vector2f kOurGoalL;
extern const Eigen::Vector2f kOurGoalR;
extern const Eigen::Vector2f kOurGoalCenter;
extern const Eigen::Vector2f kTheirGoalL;
extern const Eigen::Vector2f kTheirGoalR;
extern const Eigen::Vector2f kTheirGoalCenter;

// Obstacle size constants (mostly based on field dimensions
extern const float kWallObstacleThickness;
extern const float kGoalObstacleThickness;

// ID of the joystick used for joystick controller
extern const int kJoystickID;

// Defense Evaluation Constants
extern const Eigen::Vector2f kGoalieLineL;
extern const Eigen::Vector2f kGoalieLineR;
extern const float kGoalieOvershoot;

extern const float kGoalieVelocityThreshold;  // 250 mm/s squared

// Ratios between the distance of the ball to the defender and the defender to
// the goal. These may bot be constant in the future.
extern const float kGoalieRatio;

// Maximum distance of the goalie from the goal
extern const float kGoalieMaxDistance;

extern const float kStoppedRadius;

// Minimum distance that primary defenders can be from each other
extern const float kStoppedDefenderMargin;

// Minumum angle robots can be from each other in the stopped state while around
// the ball
extern const float kStoppedMinimumAngle;

extern const float kDefenseBoundary;

extern const float kInflatedDefenseRadius;

// Default navigation thresholds
extern const float kNavigationLocationThreshold;         // mm
extern const float kNavigationAngularThreshold;          // rad (1 degree)
extern const float kNavigationLinearVelocityThreshold;   // mm/s
extern const float kNavigationAngularVelocityThreshold;  // rad/s

// Minimum forward distance for STOx Navigation
extern const float kNavigationForwardMovement;

// Margin that navigation uses for robot obstacles by default
extern const float kNavigationRobotMargin;

extern const float kNavigationBallMargin;

extern const float kNavigationBallPad;

// Reduction in margin around robot paths to reduce problems with histerisis
extern const float kNavigationRobotMarginOldPath;

// Minimum margin
extern const float kMinMargin;

// Maximum number of times closes free point can recurse
extern const int kMaxClosestPointRecursions;

// Maximum number of tangent points that tangent smooth can search
extern const int kMaxTangentRecursions;

// Tangents a little further out than the path planning itself
extern const float kTangentSmoothPadding;

// DSS Safety margin in mm.
extern const float kDSSSafetyMargin;
extern const float kDSSSafetySmallerMargin;

// Speed in mm/s we must be under to use smaller margin.
extern const float kDSSSafetyMarginSwapSpeed;
extern const float kDSSBallRadius;

// Hysteresis value for STP. Should be < 1.
extern const float kSTPHysteresis;

// SSL_Index of robot to graph stats from.
extern const int kGraphRobot;

extern const bool kNetworkSplitLarge;

extern const float kBallObstacleAngleRequirement;

// Minimum confidence required for adding a new robot
extern const float kMinConfidence;

extern const float kKickAcceleration;

// How long should the robot that was passed to stay as main attacker
extern const int kWasPassedToThreshold;

extern const float kBallMovedDistance;
extern const double kBallMovedTimeThreshold;

extern const bool kUseEightGrid;
extern const float kEightGridSquareSize;

// Rehsape the wheel commands at the radio level given the trained motion model
extern const bool kUseCmdCorrection;

// Switch to use the actual value passed to the radio vs a hardcoded threshold.
extern const bool kRadioUsePassedPower;

// Whether or not to dump kalman data to CSV
// DO NOT USE THIS WITH MORE THAN ONE ROBOT
extern const bool kDumpKalmanData;

// The filename you want to dump the data to
extern const char kDumpFileName[];

// The flag to dump command and observation data to file
// DO NOT USE THIS WITH MORE THAN ONE ROBOT
extern const bool kDumpCmdData;

// The trajectory number to be played back
extern const int kTrajectoryNum;

// SSL vision Id of the robot that is being analyzed for model learning
extern const int kRobotIdUnderTest;

// The file name for the mixed commnad and observation data
extern const char kDumpCmdObsDirectory[];
extern const char kDumpCmdObsFileName[];

// The file name for the command data
extern const char kDumpCmdDirectory[];
extern const char kDumpCmdFileName[];

extern const bool kCollectTSOCSData;

extern const double kExtendedKalmanFilterCovarianceTimeout;

extern double min_v_cost_coef;

extern const float kChipKickPadding;

void ReadConfigurationConstants();

#endif  // SRC_CONSTANTS_CONSTANTS_H_
