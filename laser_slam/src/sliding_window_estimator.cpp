#include "laser_slam/sliding_window_estimator.hpp"

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

using namespace gtsam;

namespace laser_slam {

SlidingWindowEstimator::SlidingWindowEstimator(const OnlineEstimatorParams& parameters) :
    params_(parameters), laser_track_(parameters.laser_track_params) {

  // Create the noise models.
  using namespace gtsam::noiseModel;
  if (params_.add_m_estimator_on_odom) {
    LOG(INFO) << "Creating odometry noise model with cauchy.";
    odometry_noise_model_  = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Sigmas(params_.odometry_noise_model));
  } else {
    odometry_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(params_.odometry_noise_model);
  }

  if (params_.add_m_estimator_on_icp) {
    LOG(INFO) << "Creating ICP noise model with cauchy.";
    icp_noise_model_  = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Sigmas(params_.icp_noise_model));
  } else {
    icp_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(params_.icp_noise_model);
  }

  if (params_.add_m_estimator_on_loop_closures) {
    LOG(INFO) << "Creating loop closure noise model with cauchy.";
    loop_closure_noise_model_  = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Sigmas(params_.loop_closure_noise_model));
  } else {
    icp_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(params_.loop_closure_noise_model);
  }
}

void SlidingWindowEstimator::processPose(const Pose& pose) {
  laser_track_.processPose(pose);
}

void SlidingWindowEstimator::processLoopClosure(const RelativePose& loop_closure) {
  laser_track_.processLoopClosure(loop_closure);
  buildLoopClosureProblemAndOptimize();
}

void SlidingWindowEstimator::processLoopClosures(const std::vector<RelativePose>& loop_closures) {
  for (size_t i = 0u; i < loop_closures.size(); ++i) {
    laser_track_.processLoopClosure(loop_closures[i]);
  }
  buildLoopClosureProblemAndOptimize();
}

void SlidingWindowEstimator::processPoseAndLaserScan(const Pose& pose, const LaserScan& scan) {
  if (pose.time_ns != scan.time_ns) {
    LOG(WARNING) << "The time of the pose to add (" << pose.time_ns << ") does not match the " <<
        "time of the scan to add (" << scan.time_ns << ").";
  }
  laser_track_.processPose(pose);
  laser_track_.processLaserScan(scan);

  buildProblemAndOptimize();
}

void SlidingWindowEstimator::appendFixedScans(DataPoints* out_point_cloud) {
  CHECK_NOTNULL (out_point_cloud);
  if (!fixed_scans_.empty()) {
    if (out_point_cloud->getNbPoints() > 0) {
      for (size_t i = 0u; i < fixed_scans_.size(); ++i) {
        out_point_cloud->concatenate(fixed_scans_[i]);
      }
    } else {
      *out_point_cloud = fixed_scans_[0];
      for (size_t i = 1u; i < fixed_scans_.size(); ++i) {
        out_point_cloud->concatenate(fixed_scans_[i]);
      }
    }
    fixed_scans_.clear();
  }
}

void SlidingWindowEstimator::getTrajectory(Trajectory* out_trajectory) const {
  laser_track_.getTrajectory(out_trajectory);
}

void SlidingWindowEstimator::getOdometryTrajectory(Trajectory* out_trajectory) const {
  laser_track_.getOdometryTrajectory(out_trajectory);
}

void SlidingWindowEstimator::getCovariances(std::vector<Covariance>* out_covariances) const {
  laser_track_.getCovariances(out_covariances);
}

OptimizationResult SlidingWindowEstimator::buildProblemAndOptimize() {
  Clock clock;

  OptimizationResult optimization_result;

  if (laser_track_.getNumScans() > 1) {
    gtsam::NonlinearFactorGraph factor_graph;

    // Find the estimation min and max times. estimation_min_time_ns should fall on
    // the first node of the sliding window (decided from parameters).
    curves::Time estimation_min_time_ns;
    std::vector<curves::Time> laser_scan_times_ns;
    laser_track_.getLaserScansTimes(&laser_scan_times_ns);
    if (laser_scan_times_ns.size() > params_.sliding_window_size) {
      estimation_min_time_ns = laser_scan_times_ns[laser_scan_times_ns.size() - 1 -
                                                   params_.sliding_window_size];
    } else {
      estimation_min_time_ns = laser_track_.getMinTime();
    }

    curves::Time estimation_max_time_ns = laser_track_.getMaxTime();

    // Add the priors.
    laser_track_.appendPriorFactors(estimation_min_time_ns, &factor_graph);

    // Add the odometry factors.
    laser_track_.appendOdometryFactors(estimation_min_time_ns, estimation_max_time_ns,
                                       odometry_noise_model_, &factor_graph);

    // Add the ICP match factors.
    if (params_.laser_track_params.use_icp_factors) {
      laser_track_.appendICPFactors(estimation_min_time_ns, estimation_max_time_ns,
                                    icp_noise_model_, &factor_graph);
    }

    clock.takeTime();
    LOG(INFO) << "Took " << clock.getRealTime() << "ms to build the problem.";
    optimization_result = optimize(factor_graph);

    // If a scan was in the sliding window for the last time, save it as a fixed scan.
    if (laser_scan_times_ns.size() >= params_.sliding_window_size) {
      DataPoints local_cloud;
      laser_track_.getLocalCloudInWorldFrame(estimation_min_time_ns, &local_cloud);
      fixed_scans_.push_back(local_cloud);
    }
  }

  return optimization_result;
}

OptimizationResult SlidingWindowEstimator::buildLoopClosureProblemAndOptimize() {
  Clock clock;

  OptimizationResult optimization_result;

  if (laser_track_.getNumScans() > 1) {
    gtsam::NonlinearFactorGraph factor_graph;

    curves::Time estimation_min_time_ns = laser_track_.getMinTime();
    curves::Time estimation_max_time_ns = laser_track_.getMaxTime();

    // Add the priors.
    laser_track_.appendPriorFactors(estimation_min_time_ns, &factor_graph);

    // Add the odometry factors.
    laser_track_.appendOdometryFactors(estimation_min_time_ns, estimation_max_time_ns,
                                       odometry_noise_model_, &factor_graph);

    // Add the ICP match factors.
    if (params_.laser_track_params.use_icp_factors) {
      laser_track_.appendICPFactors(estimation_min_time_ns, estimation_max_time_ns,
                                    icp_noise_model_, &factor_graph);
    }

    // Add the loop closure factors.
    laser_track_.appendLoopClosureFactors(estimation_min_time_ns, estimation_max_time_ns,
                                          loop_closure_noise_model_, &factor_graph);

    clock.takeTime();
    LOG(INFO) << "Took " << clock.getRealTime() << "ms to build the problem.";
    optimization_result = optimize(factor_graph);
  }

  return optimization_result;
}

OptimizationResult
SlidingWindowEstimator::optimize(const gtsam::NonlinearFactorGraph& factor_graph) {
  Clock clock;

  // Make initial values.
  gtsam::KeySet keys = factor_graph.keys();
  Values initials;
  laser_track_.initializeGTSAMValues(keys, &initials);

  // Create the optimizer
  gtsam::GaussNewtonParams optimizer_params;
  gtsam::GaussNewtonOptimizer optimizer(factor_graph, initials, optimizer_params);

  // Optimize.
  Values result = optimizer.optimize();
  clock.takeTime();

  // Update the curve values.
  laser_track_.updateFromGTSAMValues(result);

  // Update covariance matrices.
  if (params_.publish_covariances) {
    laser_track_.updateCovariancesFromGTSAMValues(factor_graph, result);
  }

  // Update optimization results.
  OptimizationResult optimization_result;
  optimization_result.initial_error = optimizer.state().error;
  optimization_result.duration_ms = clock.getRealTime();
  optimization_result.durationCpu_ms = clock.getCPUTime();
  optimization_result.num_variables = optimizer.state().values.size();
  optimization_result.final_error = optimizer.state().error;
  optimization_result.num_intermediate_steps = 0u;
  optimization_result.num_iterations = optimizer.state().iterations;

  LOG(INFO) << "Took " << clock.getRealTime() << "ms to optimize.";

  return optimization_result;
}

} // namespace laser_slam
