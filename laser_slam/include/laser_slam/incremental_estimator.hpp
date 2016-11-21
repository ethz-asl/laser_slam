#ifndef LASER_SLAM_INCREMENTAL_ESTIMATOR_HPP_
#define LASER_SLAM_INCREMENTAL_ESTIMATOR_HPP_

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "laser_slam/common.hpp"
#include "laser_slam/laser_track.hpp"
#include "laser_slam/parameters.hpp"

namespace laser_slam {

/// \brief Sliding window estimator class.
class IncrementalEstimator {

 public:
  IncrementalEstimator() {};
  /// \brief Constructor.
  explicit IncrementalEstimator(const EstimatorParams& parameters);

  ~IncrementalEstimator() {};

  /// \brief Process a new laser scan in laser frame and call an optimization.
  void processPoseAndLaserScan(const Pose& pose, const LaserScan& scan);

  /// \brief Process a new loop closure.
  void processLoopClosure(const RelativePose& loop_closure);

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
  void estimate(const gtsam::NonlinearFactorGraph& newFactors,
                const gtsam::Values& newValues);

  /// \brief Build the factor graph including loop closures and estimate the trajectory.
  //OptimizationResult buildLoopClosureProblemAndOptimize();

  /// \brief Optimize the factor graph.
  //OptimizationResult optimize(const gtsam::NonlinearFactorGraph& factor_graph);

  // Underlying laser track.
  LaserTrack laser_track_;

  // Scans which are out of the sliding window estimation and fixed into world frame.
  std::vector<DataPoints> fixed_scans_;

  gtsam::ISAM2 isam2_;

  // ICP algorithm object.
  PointMatcher::ICP icp_;

  gtsam::noiseModel::Base::shared_ptr loop_closure_noise_model_;

  // Parameters.
  EstimatorParams params_;
}; // IncrementalEstimator

}  // namespace laser_slam

#endif /* LASER_SLAM_INCREMENTAL_ESTIMATOR_HPP_ */
