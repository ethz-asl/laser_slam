#ifndef LASER_SLAM_SLIDING_WINDOW_ESTIMATOR_HPP_
#define LASER_SLAM_SLIDING_WINDOW_ESTIMATOR_HPP_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "laser_slam/common.hpp"
#include "laser_slam/laser_track.hpp"
#include "laser_slam/parameters.hpp"

namespace laser_slam {

struct OptimizationResult;

/// \brief Sliding window estimator class.
class SlidingWindowEstimator {

 public:
  SlidingWindowEstimator() {};
  /// \brief Constructor.
  explicit SlidingWindowEstimator(const OnlineEstimatorParams& parameters);

  ~SlidingWindowEstimator() {};

  /// \brief Process a new pose measurement in world frame.
  void processPose(const Pose& pose);

  /// \brief Process a new laser scan in laser frame and call an optimization.
  void processPoseAndLaserScan(const Pose& pose, const LaserScan& scan);

  /// \brief Process a new loop closure.
  void processLoopClosure(const RelativePose& loop_closure);

  /// \brief Process a vector of loop closure.
  void processLoopClosures(const std::vector<RelativePose>& loop_closures);

  /// \brief Append the scans which are out of the sliding window estimation and
  /// fixed into world frame then clear the local copy.
  void appendFixedScans(DataPoints* out_point_cloud);

  /// \brief Get the trajectory.
  void getTrajectory(Trajectory* out_trajectory) const;

  /// \brief Get the trajectory based only on odometry data.
  void getOdometryTrajectory(Trajectory* out_trajectory) const;

  /// \brief Get the current estimate.
  Pose getCurrentPose() const { return laser_track_.getCurrentPose(); };

  /// \brief Get the position covariances.
  void getCovariances(std::vector<Covariance>* out_covariances) const;

  Pose findNearestPose(const Time& timestamp_ns) const {
    return laser_track_.findNearestPose(timestamp_ns);
  };


 private:
  // Build the factor graph and estimate the trajectory.
  OptimizationResult buildProblemAndOptimize();

  /// \brief Build the factor graph including loop closures and estimate the trajectory.
  OptimizationResult buildLoopClosureProblemAndOptimize();

  /// \brief Optimize the factor graph.
  OptimizationResult optimize(const gtsam::NonlinearFactorGraph& factor_graph);

  // Underlying laser track.
  LaserTrack laser_track_;

  // Odometry noise model.
  gtsam::noiseModel::Base::shared_ptr odometry_noise_model_;

  // ICP noise model.
  gtsam::noiseModel::Base::shared_ptr icp_noise_model_;

  // Loop closure noise model
  gtsam::noiseModel::Base::shared_ptr loop_closure_noise_model_;

  // Scans which are out of the sliding window estimation and fixed into world frame.
  std::vector<DataPoints> fixed_scans_;

  // Parameters.
  OnlineEstimatorParams params_;
}; // SlidingWindowEstimator

/// \brief Optimization result structure.
struct OptimizationResult {
  /// \brief Number of optimizer iterations performed.
  size_t num_iterations = 0;
  /// \brief Number of intermediate steps performed within the optimization.
  /// For L-M, this is the number of lambdas tried.
  size_t num_intermediate_steps = 0;
  /// \brief Number of variables.
  size_t num_variables = 0;
  /// \brief Initial factor graph error.
  double initial_error = 0;
  /// \brief Final factor graph error.
  double final_error = 0;
  /// \brief Optimization duration
  long duration_ms = 0;
  /// \brief CPU optimization duration.
  long durationCpu_ms = 0;
};

}  // namespace laser_slam

#endif /* LASER_SLAM_SLIDING_WINDOW_ESTIMATOR_HPP_ */
