// Copyright 2017 srabiee@cs.umass.edu
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
#ifndef SRC_STATE_ESTIMATION_CHIP_KICK_DETECTION_H_
#define SRC_STATE_ESTIMATION_CHIP_KICK_DETECTION_H_

#include <utility>
#include <vector>

#include "state/direction.h"
#include "state_estimation/chip_kick_tester.h"
#include "logging/logger.h"

using Eigen::Vector2f;
using Eigen::Vector3f;

class ChipKickDetection{
 public:
  ChipKickDetection();
  ~ChipKickDetection();

  //  Convert the points from the world coordinate to the camera coordinate
  static int Worl2Cam(const ChipKickTester::CameraParams& cameras,
                    const Vector3f& pos_world,
                    Vector3f* pos_cam);

  //  Fit a projectile model to the input SSL-vision data
  //  (from all four cameras)
  static int FitParabola(Vector3f* initial_pos,
            Vector3f* initial_vel,
            const double& time_init,
            const vector<double>& time_input,
            const vector<float>& ball_pos_x,
            const vector<float>& ball_pos_y,
            const vector<float>& ball_pos_z,
            const vector<uint>& camera_id_input,
            const vector<ChipKickTester::CameraParams>& cameras,
            const Vector3f& g);

  //  Fit a projectile model to the input SSL-vision data
  //  (from all four cameras). Partial optimization is performed, i.e., initial
  //  position is assumed to be known and is performed and the problem is
  //  solved only for initial velocity
  static int FitParabolaPartialOptimization(Vector3f* initial_vel,
            double* time_init_optimized,
            double* chip_rmse,
            double* flat_kick_rmse,
            const Vector3f& initial_pos,
            const double& time_init,
            const vector<double>& time_input,
            const vector<float>& ball_pos_x,
            const vector<float>& ball_pos_y,
            const vector<float>& ball_pos_z,
            const vector<uint>& camera_id_input,
            const vector<ChipKickTester::CameraParams>& cameras,
            const Vector3f& g);

  //  Predict the impact point of the ball with the ground and its time
  static int PredictBouncing(const Vector3f& initial_pos,
     const Vector3f& initial_vel,
     const double& time_init,
     const float& ball_zero_height,
     const Vector3f& g,
     Vector3f* impact_point,
     double* impact_time);

  static int Project2GroundPlane(
                              const ChipKickTester::CameraParams& cameras,
                              const Vector3f& pos3d,
                              const float& ball_zero_height,
                              Vector2f* pos2d);

  // Sends out visualization messages to the viewer for testing
  // chip kick detector results
  static logger::Logger VisualizeResults(const Vector3f& ball_pos_3d,
                        const Vector3f& ball_impact_point,
                        const vector<float>& ball_raw_pos_x_queue,
                        const vector<float>& ball_raw_pos_y_queue,
                        const vector<float>& ball_raw_pos_x_offline_queue,
                        const vector<float>& ball_raw_pos_y_offline_queue,
                        const vector<float>& ball_pose2d_x_queue,
                        const vector<float>& ball_pose2d_y_queue,
                        const vector<uint>& camera_id_queue,
                        const vector<float>& ball_impact_point_x_queue,
                        const vector<float>& ball_impact_point_y_queue,
                        const vector<double>& ball_impact_time_queue,
                        const vector<float>& chip_kick_rmse_queue,
                        const vector<double>& ball_time_queue,
                        const vector<double>& computation_time_queue,
                        const vector<double>& computation_time_mean_queue,
                        const bool& detailed_visualization,
                        const direction::Direction& game_direction,
                        logger::Logger* the_log);

  // Sends out visualization messages to the viewer for testing
  // chip kick detector results (for comparing two different optimization
  // methods)
  static int VisualizeResultsCompare(const Vector3f& ball_pos_3d,
                        const Vector3f& ball_impact_point,
                        const vector<float>& ball_raw_pos_x_queue,
                        const vector<float>& ball_raw_pos_y_queue,
                        const vector<float>& ball_raw_pos_x_offline_queue,
                        const vector<float>& ball_raw_pos_y_offline_queue,
                        const vector<float>& ball_pose2d_x_queue,
                        const vector<float>& ball_pose2d_y_queue,
                        const vector<uint>& camera_id_queue,
                        const vector<float>& ball_impact_point_x_queue,
                        const vector<float>& ball_impact_point_y_queue,
                        const vector<double>& ball_impact_time_queue,
                        const vector<float>& ball_impact_point_x_queue_2nd,
                        const vector<float>& ball_impact_point_y_queue_2nd,
                        const vector<double>& ball_impact_time_queue_2nd,
                        const vector<float>& chip_kick_rmse_queue,
                        const vector<double>& ball_time_queue,
                        const vector<double>& computation_time_queue,
                        const vector<double>& computation_time_mean_queue,
                        logger::Logger* the_log);

  static int CalculateError(const vector<float>& ball_raw_pos_x_queue,
                          const vector<float>& ball_raw_pos_y_queue,
                          const vector<float>& ball_pose2d_x_queue,
                          const vector<float>& ball_pose2d_y_queue,
                          float* rmse);
};
#endif  //  SRC_STATE_ESTIMATION_CHIP_KICK_DETECTION_H_
