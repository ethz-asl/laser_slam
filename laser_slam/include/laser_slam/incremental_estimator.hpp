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
  explicit IncrementalEstimator(const EstimatorParams& parameters,
                                unsigned int n_laser_slam_workers);

  ~IncrementalEstimator() {};

  /// \brief Process a new loop closure.
  void processLoopClosure(const RelativePose& loop_closure);

  // TODO remove these functions. Can be done by the LaserSlamWorker.
  /// \brief Get the trajectory.
  void getTrajectory(Trajectory* out_trajectory,
                     unsigned int laser_track_id = 1u) const;

  /// \brief Get the trajectory based only on odometry data.
  void getOdometryTrajectory(Trajectory* out_trajectory,
                             unsigned int laser_track_id = 1u) const;

  /// \brief Get the current estimate.
  Pose getCurrentPose(unsigned int laser_track_id) const {
    // TODO rm -1.
    return laser_tracks_[laser_track_id - 1]->getCurrentPose();
  };

  std::shared_ptr<LaserTrack> getLaserTrack(unsigned int laser_track_id);

  // Build the factor graph and estimate the trajectory.
  gtsam::Values estimate(const gtsam::NonlinearFactorGraph& new_factors,
                         const gtsam::Values& new_values);

 private:
  unsigned int n_laser_slam_workers_;

  // Underlying laser tracks.
  std::vector<std::shared_ptr<LaserTrack> > laser_tracks_;

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
