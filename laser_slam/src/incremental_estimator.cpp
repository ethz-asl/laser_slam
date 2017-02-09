#include "laser_slam/incremental_estimator.hpp"

#include <algorithm>
#include <utility>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

using namespace gtsam;

namespace laser_slam {

IncrementalEstimator::IncrementalEstimator(const EstimatorParams& parameters,
                                           unsigned int n_laser_slam_workers) : params_(
                                               parameters),
                                               n_laser_slam_workers_(n_laser_slam_workers) {
  // Create the iSAM2 object.
  ISAM2Params isam2_params;
  isam2_params.setRelinearizeSkip(1);
  isam2_params.setRelinearizeThreshold(0.001);
  isam2_ = ISAM2(isam2_params);

  // Create the laser tracks.
  for (size_t i = 0u; i < n_laser_slam_workers_; ++i) {
    std::shared_ptr<LaserTrack> laser_track(new LaserTrack(parameters.laser_track_params, i));
    laser_tracks_.push_back(std::move(laser_track));
  }

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

void IncrementalEstimator::processLoopClosure(const RelativePose& loop_closure) {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  CHECK_GE(loop_closure.time_a_ns, laser_tracks_[loop_closure.track_id_a]->getMinTime()) <<
      "Loop closure has invalid time.";
  CHECK_LE(loop_closure.time_a_ns, laser_tracks_[loop_closure.track_id_a]->getMaxTime()) <<
      "Loop closure has invalid time.";
  CHECK_GE(loop_closure.time_b_ns, laser_tracks_[loop_closure.track_id_b]->getMinTime()) <<
      "Loop closure has invalid time.";
  CHECK_LE(loop_closure.time_b_ns, laser_tracks_[loop_closure.track_id_b]->getMaxTime()) <<
      "Loop closure has invalid time.";

  RelativePose updated_loop_closure = loop_closure;

  // Convert the reference frame of the loop closure transformation.
  // When applying the transformation w_T_a_b to the source cloud, it will align it with the
  // target cloud.
  SE3 w_T_a_b = loop_closure.T_a_b;
  SE3 T_w_a = laser_tracks_[loop_closure.track_id_a]->evaluate(loop_closure.time_a_ns);
  SE3 T_w_b = laser_tracks_[loop_closure.track_id_b]->evaluate(loop_closure.time_b_ns);
  SE3 a_T_a_b = T_w_a.inverse() * w_T_a_b * T_w_b;
  updated_loop_closure.T_a_b = a_T_a_b;

  // Apply an ICP step if desired.
  if (params_.do_icp_step_on_loop_closures) {
    // Get the initial guess.
    PointMatcher::TransformationParameters initial_guess =
        updated_loop_closure.T_a_b.getTransformationMatrix().cast<float>();

    LOG(INFO) << "Creating the submaps for loop closure ICP.";
    Clock clock;
    DataPoints sub_map_a;
    DataPoints sub_map_b;
    laser_tracks_[updated_loop_closure.track_id_a]->buildSubMapAroundTime(
        loop_closure.time_a_ns, params_.loop_closures_sub_maps_radius, &sub_map_a);
    laser_tracks_[updated_loop_closure.track_id_b]->buildSubMapAroundTime(
        loop_closure.time_b_ns, params_.loop_closures_sub_maps_radius, &sub_map_b);
    clock.takeTime();
    LOG(INFO) << "Took " << clock.getRealTime() << " ms to create loop closures sub maps.";

    LOG(INFO) << "Creating loop closure ICP.";
    clock.start();
    PointMatcher::TransformationParameters icp_solution = icp_.compute(sub_map_b, sub_map_a,
                                                                       initial_guess);
    clock.takeTime();
    LOG(INFO) << "Took " << clock.getRealTime() <<
        " ms to compute the icp_solution for the loop closure.";

    updated_loop_closure.T_a_b = convertTransformationMatrixToSE3(icp_solution);
  }

  LOG(INFO) << "Creating loop closure factor.";

  NonlinearFactorGraph new_factors;
  Expression<SE3> exp_T_w_b(laser_tracks_[loop_closure.track_id_b]->getValueExpression(
      updated_loop_closure.time_b_ns));
  Expression<SE3> exp_T_w_a(laser_tracks_[loop_closure.track_id_a]->getValueExpression(
      updated_loop_closure.time_a_ns));
  Expression<SE3> exp_T_a_w(kindr::minimal::inverse(exp_T_w_a));
  Expression<SE3> exp_relative(kindr::minimal::compose(exp_T_a_w, exp_T_w_b));
  ExpressionFactor<SE3> new_factor(loop_closure_noise_model_, updated_loop_closure.T_a_b,
                                   exp_relative);
  new_factors.push_back(new_factor);

  LOG(INFO) << "Estimating the trajectories.";
  std::vector<unsigned int> affected_worker_ids;
  affected_worker_ids.push_back(loop_closure.track_id_a);
  affected_worker_ids.push_back(loop_closure.track_id_b);
  Values new_values;
  Values result = estimateAndRemove(new_factors, new_values, affected_worker_ids);

  LOG(INFO) << "Updating the trajectories after LC.";
  for (auto& track: laser_tracks_) {
    track->updateFromGTSAMValues(result);
  }
  LOG(INFO) << "Updating the trajectories after LC done.";
}

Values IncrementalEstimator::estimate(const gtsam::NonlinearFactorGraph& new_factors,
                                      const gtsam::Values& new_values) {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  Clock clock;
  // Update and force relinearization.
  isam2_.update(new_factors, new_values).print();
  // TODO Investigate why these two subsequent update calls are needed.
  isam2_.update();
  isam2_.update();

  Values result(isam2_.calculateEstimate());

  clock.takeTime();
  LOG(INFO) << "Took " << clock.getRealTime() << "ms to estimate the trajectory.";
  return result;
}

Values IncrementalEstimator::estimateAndRemove(
    const gtsam::NonlinearFactorGraph& new_factors,
    const gtsam::Values& new_values,
    const std::vector<unsigned int>& affected_worker_ids) {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  
  Clock clock;
  CHECK_EQ(affected_worker_ids.size(), 2u);

  // Find and update the factor indices to remove.
  std::vector<size_t> factor_indices_to_remove;
  if (affected_worker_ids.at(0u) != affected_worker_ids.at(1u)) {
    // Remove the prior of the worker with the largest ID if not already removed.
    unsigned int worker_id_to_remove = std::max(affected_worker_ids.at(0u),
                                                affected_worker_ids.at(1u));
    if (std::find(worker_ids_with_removed_prior_.begin(),
                  worker_ids_with_removed_prior_.end(), worker_id_to_remove) !=
                      worker_ids_with_removed_prior_.end()) {
      worker_id_to_remove = std::min(affected_worker_ids.at(0u),
                                     affected_worker_ids.at(1u));
    }

    CHECK_LT(factor_indices_to_remove_.count(worker_id_to_remove), 2u);
    if (factor_indices_to_remove_.count(worker_id_to_remove) == 1u) {
      factor_indices_to_remove.push_back(factor_indices_to_remove_.at(worker_id_to_remove));
      factor_indices_to_remove_.erase(worker_id_to_remove);
      worker_ids_with_removed_prior_.push_back(worker_id_to_remove);
    }
  }

  //isam2_.saveGraph("/tmp/graph_before");

  isam2_.update(new_factors, new_values, factor_indices_to_remove).print();
  // TODO Investigate why these two subsequent update calls are needed.
  isam2_.update();
  isam2_.update();

  Values result(isam2_.calculateEstimate());

  //isam2_.saveGraph("/tmp/graph_after");

  clock.takeTime();
  LOG(INFO) << "Took " << clock.getRealTime() << "ms to estimate the trajectory.";
  return result;
}

gtsam::Values IncrementalEstimator::registerPrior(const gtsam::NonlinearFactorGraph& new_factors,
                                                  const gtsam::Values& new_values,
                                                  const unsigned int worker_id) {

  ISAM2Result update_result = isam2_.update(new_factors, new_values);

  CHECK_EQ(update_result.newFactorsIndices.size(), 1u);
  if (worker_id > 0u) {
    factor_indices_to_remove_.insert(
        std::make_pair(worker_id, update_result.newFactorsIndices.at(0u)));
  }
  // TODO Investigate why these two subsequent update calls are needed.
  isam2_.update();
  isam2_.update();
  Values result(isam2_.calculateEstimate());
  return result;
}

std::shared_ptr<LaserTrack> IncrementalEstimator::getLaserTrack(unsigned int laser_track_id) {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  CHECK_GE(laser_track_id, 0u);
  CHECK_LT(laser_track_id, laser_tracks_.size());
  return laser_tracks_[laser_track_id];
}

std::vector<std::shared_ptr<LaserTrack> > IncrementalEstimator::getAllLaserTracks() {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  return laser_tracks_;
}

} // namespace laser_slam
