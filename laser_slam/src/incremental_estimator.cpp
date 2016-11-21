#include "laser_slam/incremental_estimator.hpp"

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

using namespace gtsam;

namespace laser_slam {

IncrementalEstimator::IncrementalEstimator(const EstimatorParams& parameters) : params_(
    parameters), laser_track_(parameters.laser_track_params) {
  ISAM2Params isam2_params;
  isam2_params.relinearizeSkip = 1;
  isam2_ = ISAM2(isam2_params);

  // Create the loop closure noise model.
  using namespace gtsam::noiseModel;
  if (params_.add_m_estimator_on_loop_closures) {
    LOG(INFO) << "Creating loop closure noise model with cauchy.";
    loop_closure_noise_model_  = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Sigmas(params_.loop_closure_noise_model));
  } else {
    loop_closure_noise_model_ =
        gtsam::noiseModel::Diagonal::Sigmas(params_.loop_closure_noise_model);
  }

  // Load the ICP configurations for adjusting the loop closure transformations.
  // TODO now using the same configuration as for the lidar odometry.
  std::ifstream ifs_icp_configurations(params_.laser_track_params.icp_configuration_file.c_str());
  if (ifs_icp_configurations.good()) {
    LOG(INFO) << "Loading ICP configurations from: " <<
        params_.laser_track_params.icp_configuration_file;
    icp_.loadFromYaml(ifs_icp_configurations);
  } else {
    LOG(WARNING) << "Could not open ICP configuration file. Using default configuration.";
    icp_.setDefault();
  }
}

void IncrementalEstimator::processPoseAndLaserScan(const Pose& pose, const LaserScan& scan) {
  if (pose.time_ns != scan.time_ns) {
    LOG(WARNING) << "The time of the pose to add (" << pose.time_ns << ") does not match the " <<
        "time of the scan to add (" << scan.time_ns << ").";
  }

  NonlinearFactorGraph newFactors;
  Values newValues;

  laser_track_.processPoseAndLaserScan(pose, scan, &newFactors, &newValues);

  estimate(newFactors, newValues);
}

void IncrementalEstimator::processLoopClosure(const RelativePose& loop_closure) {
  CHECK_LT(loop_closure.time_a_ns, loop_closure.time_b_ns) << "Loop closure has invalid time.";
  CHECK_GE(loop_closure.time_a_ns, laser_track_.getMinTime()) << "Loop closure has invalid time.";
  CHECK_LE(loop_closure.time_a_ns, laser_track_.getMaxTime()) << "Loop closure has invalid time.";
  CHECK_GE(loop_closure.time_b_ns, laser_track_.getMinTime()) << "Loop closure has invalid time.";
  CHECK_LE(loop_closure.time_b_ns, laser_track_.getMaxTime()) << "Loop closure has invalid time.";

  // Apply an ICP step if desired.
  RelativePose updated_loop_closure = loop_closure;
  if (params_.do_icp_step_on_loop_closures) {
    // Get the initial guess.
    PointMatcher::TransformationParameters initial_guess =
        loop_closure.T_a_b.getTransformationMatrix().cast<float>();

    // Create the sub maps.
    Clock clock;
    DataPoints sub_map_a;
    DataPoints sub_map_b;
    laser_track_.buildSubMapAroundTime(loop_closure.time_a_ns,
                                       params_.loop_closures_sub_maps_radius, &sub_map_a);
    laser_track_.buildSubMapAroundTime(loop_closure.time_b_ns,
                                       params_.loop_closures_sub_maps_radius, &sub_map_b);
    clock.takeTime();
    LOG(INFO) << "Took " << clock.getRealTime() << " ms to create loop closures sub maps.";

    // Compute the ICP solution.
    clock.start();
    PointMatcher::TransformationParameters icp_solution = icp_.compute(sub_map_b, sub_map_a,
                                                                       initial_guess);
    clock.takeTime();
    LOG(INFO) << "Took " << clock.getRealTime() <<
        " ms to compute the icp_solution for the loop closure.";

    updated_loop_closure.T_a_b = convertTransformationMatrixToSE3(icp_solution);
  }

  // Create the loop closure factor.
  NonlinearFactorGraph newFactors;
  Expression<SE3> T_w_b(laser_track_.getValueExpression(updated_loop_closure.time_b_ns));
  Expression<SE3> T_w_a(laser_track_.getValueExpression(updated_loop_closure.time_a_ns));
  Expression<SE3> T_a_w(kindr::minimal::inverse(T_w_a));
  Expression<SE3> relative(kindr::minimal::compose(T_a_w, T_w_b));
  ExpressionFactor<SE3> new_factor(loop_closure_noise_model_, updated_loop_closure.T_a_b,
                                   relative);
  newFactors.push_back(new_factor);

  // Estimate the graph.
  Values newValues;
  estimate(newFactors, newValues);
}

void IncrementalEstimator::appendFixedScans(DataPoints* out_point_cloud) {
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

void IncrementalEstimator::getTrajectory(Trajectory* out_trajectory) const {
  laser_track_.getTrajectory(out_trajectory);
}

void IncrementalEstimator::getOdometryTrajectory(Trajectory* out_trajectory) const {
  laser_track_.getOdometryTrajectory(out_trajectory);
}

void IncrementalEstimator::getCovariances(std::vector<Covariance>* out_covariances) const {
  laser_track_.getCovariances(out_covariances);
}

void IncrementalEstimator::estimate(const gtsam::NonlinearFactorGraph& newFactors,
                                    const gtsam::Values& newValues) {
  Clock clock;

  isam2_.update(newFactors, newValues).print();;
  Values result(isam2_.calculateEstimate());

  laser_track_.updateFromGTSAMValues(result);

  clock.takeTime();
  LOG(INFO) << "Took " << clock.getRealTime() << "ms to estimate and update trajectory.";

  //todo this assumes that once optimized once a scan is fixed.
  if (!newValues.empty()) {
    DataPoints local_cloud;
    laser_track_.getLocalCloudInWorldFrame(laser_track_.getMaxTime(), &local_cloud);
    fixed_scans_.push_back(local_cloud);
  }
}

} // namespace laser_slam
