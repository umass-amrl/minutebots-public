// Copyright 2018 joydeepb@cs.umass.edu
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

#include "src/constants/constants.h"

#include <cmath>
#include <string>
#include <vector>

#include "constants/includes.h"
#include "constants/typedefs.h"
#include "math/math_util.h"

const char DATA_STREAM_VISION_IP[] = "224.5.23.2";
const int DATA_STREAM_VISION_PORT = 10006;

const char DATA_STREAM_CMD_IP[] = "127.0.0.1";
const int DATA_STREAM_CMD_PORT = 10007;

const char DATA_STREAM_REF_IP[] = "224.5.23.1";
const int DATA_STREAM_REF_PORT = 10003;

const char DATA_STREAM_DEBUG_IP[] = "127.0.0.1";
const int DATA_STREAM_DEBUG_PORT = 10008;

const char DATA_STREAM_FEEDBACK_IP[] = "127.0.0.1";
const int DATA_STREAM_FEEDBACK_PORT = 10009;

const char DATA_STREAM_VIEWER_IP[] = "127.0.0.1";
const int DATA_STREAM_VIEWER_PORT = 12000;

namespace team_management {
// Their team robot timeout in seconds.
const float kTheirTeamBotTimeout = 3;  // Seconds.

// Our team robot timeout in seconds.
const float kOurTeamBotTimeout = 3;  // Seconds.
}

const float kEpsilon = 1e-6;

extern const bool kLogNavigation = true;

const unsigned int kFrameQueueSize = 20;
const unsigned int kLatencyCalibrationQueueSize = 100;

const float kRobotRadius = 90;
const float kBallRadius = 21;
const float kMediumBallRadius = 71;
const float kLargeBallRadius = 1000;
const float kRobotFaceRadius = 67.12;

const double kBallZeroHeight = 30.0;

// Max velocity of the robots in mm/s.
const float kMaxRobotVelocity = 3000;

// Max velocity of the robots in mm/s.
const float kDefaultRobotVelocity = 3000;

// Max velocity of the robots in mm/s the minutebot radio is willing to sent.
const float kMaxRobotRadioVelocityCommand = 5110;

// const float kMaxRobotVelocity = 2800;
// Max acceleration of the robots in mm/s^2
const float kMaxRobotAcceleration = 3000;

const float kDefaultRobotAcceleration = 3000;

// Max rotational velocity of the robots in rad/s
constexpr const float kMaxRobotRotVel = 4.0 * M_PI;

// Max rotational velocity of the robots in rad/s the minutebot radio
// is willing to sent.
const float kMaxRobotRadioRotVelCommand = 10.0 * M_PI;

// Max rotational acceleration of the robots in rad/s^2
constexpr const float kMaxRobotRotAccel = 4.0 * M_PI;

// Constants to limit rotational velocity when travelling quickly
// translationally
const float kRotSpeedLimitThreshold = 1000.0f;
constexpr const float kLowerRobotRotVel = 3.0 * M_PI / 4.0;

// Temporary values, must measure precisely
const float kBallAcceleration = 343.7;  // in mm/s^2
const float kCollisionDamping = 0.4;
const float kKickBoost = 5000;  // in mm/s
const float kKickDetectionThreshold = 500;

// Estimated time used for forward prediction in seconds
// Should be the time between an image being captured and a command executing
// Set this to something low in simulation
const double kLatency = 0.115;
const double kSimulatorLatency = 1.0f / 60.0f;
const double kBallLatency = 0.115;

// Lag in seconds that the hardware has in activation.
// Sample usecase: Extending DSS control period to account for the hardware
// activation lag.
const double kHardwareLagTranslation = 0.05;
const double kHardwareLagRotation = 0.03;

const double kSimulatorPacketLossPercent = 0.0;
const double kSimulatorRobotNoiseStdDev = 0.0;
const double kSimulatorRobotAngleNoiseStdDev = 0.0;

const float kDefaultSafetyMargin = kRobotRadius + 5;
const float kDefaultRulesSafetyMargin = kRobotRadius + 10;

const float kImprobabilityRejectionThreshold = 0.25f;
const float kConfidenceThreshold = 0.5f;

// Transmit frequency of the radio to the robot.
const float kTransmitFrequency = 62.503125;  // Hz
// Time delay between successive radio commands.
const float kTransmitPeriod = 1000.0 / kTransmitFrequency;  // milliseconds
const float kControlPeriodTranslation =
    1.0 / kTransmitFrequency + kHardwareLagTranslation;
const float kControlPeriodRotation =
    1.0 / kTransmitFrequency + kHardwareLagRotation;
const float kTransmitPeriodSeconds = 1.0 / kTransmitFrequency;

// How far ahead should we set the angular waypoint.
// Used in control. kControlPeriod is multiplied by this.
const float kAnglePlannerLookahead = 3.0;

// How far ahead of the translational control should we arrive at the angular
// target (s)
const float kAnglePlannerDelta =
    kAnglePlannerLookahead * kTransmitPeriodSeconds;

const int kMaxSearchDepth = 4;  // Max search depth for STOx planner

const float kSearchMargin = 500.0f;  // Search margin for PRM

const unsigned int kNumVertices = 2000;  // Default number of PRM vertices
// Default PRM scaffold connect radius
const float kScaffoldConnectRadius = 500;

const float kMaxDeltaV = kMaxRobotAcceleration / kTransmitFrequency;
const float kMaxDeltaRotV = kMaxRobotRotAccel / kTransmitFrequency;

// constants for the circle / pivot tactic
// buffer is extra space between ball and robot
// margin is how close the robot must be to the radius to be considered
//     at the target radius, should be less than buffer
const float kRadiusBuffer = 45.0;
const float kRotationRadius = kRobotRadius + kBallRadius + kRadiusBuffer;
const float kRotationRadiusMargin = 10.0;

// constants for ball interception
const float kInterceptBuffer = 50.0;
const float kInterceptionRadius =
    kRobotFaceRadius + kBallRadius + kInterceptBuffer;
const float kInterceptionMargin = 1.0;

// tactic wide constants
const float kDistanceThreshold = 100.0;
const float kAngleThreshold = math_util::DegToRad(1.0);
// const float kAngleThreshold = 0.1;
const float kLinearVelocityThreshold = 150;
const float kAngularVelocitythreshold = math_util::DegToRad(10.0);
const float kPullRightAdjustment = math_util::DegToRad(0.0);

namespace field_dimensions {
const float kFieldLength = (kFieldSetup == LAB)
                               ? 8800.0
                               : ((kFieldSetup == A_LEAGUE) ? 12000.0 : 9000.0);
const float kFieldWidth = (kFieldSetup == LAB)
                              ? 5600.0
                              : ((kFieldSetup == A_LEAGUE) ? 9000.0 : 6000.0);
const float kHalfFieldLength = 0.5 * kFieldLength;
const float kHalfFieldWidth = 0.5 * kFieldWidth;
const float kGoalWidth = (kFieldSetup == B_LEAGUE) ? 1000.0 : 1200.0;
const float kGoalDepth = 180.0;
const float kBoundaryWidth = 250.0;
const float kFieldLineWidth = (kFieldSetup == LAB) ? 50.0 : 20.0;
const float kDefenseStretch = 1200.0;
const float kCenterCircleRadius = 500.0;
const float kFieldBoundary = 300.0;
}  // namespace field_dimensions

const float kFieldXMax = field_dimensions::kHalfFieldLength;
const float kFieldYMax = field_dimensions::kHalfFieldWidth;

extern const float kInflatedFieldXMax = kFieldXMax + 2.0f * kRobotRadius;
extern const float kInflatedFieldYMax = kFieldXMax + 2.0f * kRobotRadius;

const Eigen::Vector2f kOurGoalL(-kFieldXMax,
                                -field_dimensions::kGoalWidth / 2.0f);
const Eigen::Vector2f kOurGoalR(-kFieldXMax,
                                field_dimensions::kGoalWidth / 2.0f);
const Eigen::Vector2f kOurGoalCenter(-kFieldXMax, 0);
const Eigen::Vector2f kTheirGoalL(kFieldXMax,
                                  field_dimensions::kGoalWidth / 2.0f);
const Eigen::Vector2f kTheirGoalR(kFieldXMax,
                                  -field_dimensions::kGoalWidth / 2.0f);
const Eigen::Vector2f kTheirGoalCenter(kFieldXMax, 0);

// Obstacle size constants (mostly based on field dimensions
const float kWallObstacleThickness = 400;
const float kGoalObstacleThickness = 20;

const int kJoystickID = 0;  // ID of the joystick used for joystick controller

// Defense Evaluation Constants
const float kHalfKickerWidth = 50.0f;

const float kAlignmentThreshold = 9;
const float kAttackerFieldMargin = -kRobotRadius;

const Eigen::Vector2f kGoalieLineL(kOurGoalL.x() + kDefaultSafetyMargin + 50,
                                   kOurGoalL.y() + kDefaultSafetyMargin + +5 +
                                       kEpsilon);
const Eigen::Vector2f kGoalieLineR(kOurGoalR.x() + kDefaultSafetyMargin + 50,
                                   kOurGoalR.y() - kDefaultSafetyMargin - -5 -
                                       kEpsilon);

// const Eigen::Vector2f kGoalieLineL(kOurGoalL.x() + kDefaultSafetyMargin,
//                                    kOurGoalL.y() + kHalfKickerWidth);
// const Eigen::Vector2f kGoalieLineR(kOurGoalR.x() + kDefaultSafetyMargin,
//                                    kOurGoalR.y() - kHalfKickerWidth);

extern const float kGoalieOvershoot = 100;

const float kGoalieVelocityThreshold = 62500;  // 250 mm/s squared

// Ratios between the distance of the ball to the defender and the defender to
// the goal. These may bot be constant in the future.
const float kGoalieRatio = 0.1;

// Maximum distance of the goalie from the goal in mm
const float kGoalieMaxDistance = 1000.0f;

const float kStoppedRadius = 530 + kRobotRadius + kEpsilon;

// Minimum distance that primary defenders can be from each other
extern const float kStoppedDefenderMargin = 2.0f * kRobotRadius + 30 + kEpsilon;

// Minumum angle robots can be from each other in the stopped state while around
// the ball
extern const float kStoppedMinimumAngle =
    std::asin(kStoppedDefenderMargin / kStoppedRadius);

// const float kDefenseBoundary = field_dimensions::kDefenseRadius +
//                                field_dimensions::kDefenseStretch / 2 +
//                                kRobotRadius + kDefaultSafetyMargin;

// Default navigation thresholds
const float kNavigationLocationThreshold = 8;                 // mm
const float kNavigationAngularThreshold = 0.025;              // rad (1 degree)
const float kNavigationLinearVelocityThreshold = 50;          // mm/s
const float kNavigationAngularVelocityThreshold = 0.0872665;  // rad/s

// Minimum forward distance for STOx Navigation
const float kNavigationForwardMovement = 200;

// Margin that navigation uses for robot obstacles
const float kNavigationRobotMargin = kRobotRadius;

const float kNavigationBallMargin = kRobotRadius;

const float kNavigationBallPad = 20;

const float kNavigationRobotMarginOldPath = kRobotRadius;

// Minimum margin
const float kMinMargin = kRobotRadius + 1;

// Maximum number of times closes free point can recurse
const int kMaxClosestPointRecursions = 10;

// Whether or not the Smooth Path function should smoot using the tangent or
// just remove path segments.
const int kMaxTangentRecursions = 6;

// Tangents a little further out than the path planning itself
const float kTangentSmoothPadding = 20.0f;

// DSS Safety margin in mm.
const float kDSSSafetyMargin = 5;
const float kDSSSafetySmallerMargin = -kRobotRadius / 4;
const float kDSSSafetyMarginSwapSpeed = 750;
const float kDSSBallRadius = 50;

// Hysteresis value for STP. Should be < 1.
const float kSTPHysteresis = .9;

const int kGraphRobot = 0;

const bool kNetworkSplitLarge = true;

const float kBallObstacleAngleRequirement = 1.13446;  // 65 degrees in radians

// Minimum confidence required to add a robot
// This number was selected randomly as it seemed reasonable at the time
const float kMinConfidence = 0.92;

const float kKickAcceleration = 1500;

const int kWasPassedToThreshold = 10;

const float kBallMovedDistance = 100;
const double kBallMovedTimeThreshold = 10.0;

const bool kUseEightGrid = false;
const float kEightGridSquareSize = (kRobotRadius + 13.0f);

// Rehsape the wheel commands at the radio level given the trained motion model
const bool kUseCmdCorrection = false;

const bool kRadioUsePassedPower = true;
// Whether or not to dump kalman data to CSV
const bool kDumpKalmanData = false;

// Whether to log their robot or ours and its ssl vision id
const char kDumpFileName[] = "robota_point10.csv";

// The flag to dump command and observation data to file (for model learning)
const bool kDumpCmdData = false;

// The trajectory number to be played back
const int kTrajectoryNum = 12;

// SSL vision Id of the robot that is being analyzed for model learning
const int kRobotIdUnderTest = 4;

// The file name for the mixed commnad and observation data
const char kDumpCmdObsDirectory[] = "model_learning/logs/cmd_obs/";
const char kDumpCmdObsFileName[] = "cmd_obs_data.csv";

// The file name for the command data
const char kDumpCmdDirectory[] = "model_learning/logs/cmd/";
const char kDumpCmdFileName[] = "cmd_data.csv";

const bool kCollectTSOCSData = false;

const double kExtendedKalmanFilterCovarianceTimeout = 0.1;

double min_v_cost_coef = 0.001;

const float kChipKickPadding = 45;
