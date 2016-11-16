#include "laser_slam/laser_track.hpp"

#include <gtsam/nonlinear/Marginals.h>

using namespace gtsam;
using namespace curves;

namespace laser_slam {

LaserTrack::LaserTrack(const LaserTrackParams& parameters) : params_(parameters) {
  // Load the ICP configurations.
  std::ifstream ifs_icp_configurations(params_.icp_configuration_file.c_str());
  if (ifs_icp_configurations.good()) {
    LOG(INFO) << "Loading ICP configurations from: " << params_.icp_configuration_file;
    icp_.loadFromYaml(ifs_icp_configurations);
  } else {
    LOG(WARNING) << "Could not open ICP configuration file. Using default configuration.";
    icp_.setDefault();
  }

  // Load the ICP input filters configurations.
  std::ifstream ifs_input_filters(params_.icp_input_filters_file.c_str());
  if (ifs_input_filters.good()) {
    LOG(INFO) << "Loading ICP input filters from: " << params_.icp_input_filters_file;
    input_filters_ = PointMatcher::DataPointsFilters(ifs_input_filters);
  } else {
    LOG(FATAL) << "Could not open ICP input filters configuration file.";
  }

  // Create a rigid transformation.
  rigid_transformation_ = PointMatcher::get().REG(Transformation).create("RigidTransformation");
  CHECK_NOTNULL(rigid_transformation_);
}

void LaserTrack::processPose(const Pose& pose) {
  if (pose_measurements_.empty() && pose.time_ns != 0) {
    LOG(WARNING) << "First pose had timestamp different than 0 (" << pose.time_ns << ".";
  }
  pose_measurements_.push_back(pose);
}

void LaserTrack::processLaserScan(const LaserScan& in_scan) {
  LaserScan scan = in_scan;

  // Apply the input filters.
  Clock clock;
  input_filters_.apply(scan.scan);
  clock.takeTime();
  LOG(INFO) << "Took " << clock.getRealTime() << " ms to filter the input scan.";

  // Compute the relative pose measurement, extend the trajectory and
  // compute the ICP transformations.
  if (trajectory_.isEmpty()) {
    scan.key = extendTrajectory(scan.time_ns, getPoseMeasurement(scan.time_ns));
  } else {
    // Evaluate the pose measurement at the last trajectory node.
    SE3 last_pose_measurement = getPoseMeasurement(trajectory_.getMaxTime());

    // Evaluate the pose measurement at the new trajectory node.
    SE3 new_pose_measurement = getPoseMeasurement(scan.time_ns);

    // Create the relative pose measurement.
    RelativePose relative_measurement;
    relative_measurement.T_a_b = last_pose_measurement.inverse()*new_pose_measurement;
    relative_measurement.time_a_ns = trajectory_.getMaxTime();
    relative_measurement.key_a = getPoseKey(trajectory_.getMaxTime());
    relative_measurement.time_b_ns = scan.time_ns;

    // Extend the trajectory with the new node position.
    scan.key =  extendTrajectory(scan.time_ns, trajectory_.evaluate(trajectory_.getMaxTime()) *
                                 relative_measurement.T_a_b);

    // Complete and save the relative_measurement.
    relative_measurement.key_b = scan.key;
    odometry_measurements_.push_back(relative_measurement);

    // Compute the ICP transformations.
    if (params_.use_icp_factors) {
      computeICPTransformations();
    }
  }

  // Update the pose key and save the scan.
  setPoseKey(scan.time_ns, scan.key);
  laser_scans_.push_back(scan);
}

void LaserTrack::processLoopClosure(const RelativePose& loop_closure) {
  CHECK_LT(loop_closure.time_a_ns, loop_closure.time_b_ns) << "Loop closure has invalid time.";
  CHECK_GE(loop_closure.time_a_ns, trajectory_.getMinTime()) << "Loop closure has invalid time.";
  CHECK_LE(loop_closure.time_a_ns, trajectory_.getMaxTime()) << "Loop closure has invalid time.";
  CHECK_GE(loop_closure.time_b_ns, trajectory_.getMinTime()) << "Loop closure has invalid time.";
  CHECK_LE(loop_closure.time_b_ns, trajectory_.getMaxTime()) << "Loop closure has invalid time.";
  loop_closures_.push_back(loop_closure);
}

void LaserTrack::getLastPointCloud(DataPoints* out_point_cloud) const {
  CHECK_NOTNULL(out_point_cloud);
  // todo
}

void LaserTrack::getPointCloudOfTimeInterval(const std::pair<Time, Time>& times_ns,
                                             DataPoints* out_point_cloud) const {
  CHECK_NOTNULL(out_point_cloud);
  *out_point_cloud = DataPoints();
  // todo
}

void LaserTrack::getLocalCloudInWorldFrame(const Time& timestamp_ns,
                                           DataPoints* out_point_cloud) const {
  CHECK_NOTNULL(out_point_cloud);

  // Find an iterator to the local scan.
  std::vector<LaserScan>::const_iterator it = laser_scans_.end();
  do {
    --it;
  } while (it != laser_scans_.begin() && it->time_ns != timestamp_ns);
  CHECK(it->time_ns == timestamp_ns) << "The requested local scan could not be found:";

  // Get the rigid transformation from the trajectory to transform the scan in world frame.
  PointMatcher::TransformationParameters transformation_matrix =
      trajectory_.evaluate(timestamp_ns).getTransformationMatrix().cast<float>();
  correctTransformationMatrix(&transformation_matrix);

  // Transform the scan in world frame.
  *out_point_cloud = rigid_transformation_->compute(it->scan,transformation_matrix);
}

void LaserTrack::getTrajectory(Trajectory* trajectory) const {
  CHECK_NOTNULL(trajectory)->clear();

  std::vector<Time> trajectory_times_ns;
  trajectory_.getCurveTimes(&trajectory_times_ns);

  for (auto time_ns: trajectory_times_ns) {
    trajectory->emplace(time_ns, trajectory_.evaluate(time_ns));
  }
}

void LaserTrack::getCovariances(std::vector<Covariance>* out_covariances) const {
  CHECK_NOTNULL(out_covariances)->clear();
  *out_covariances = covariances_;
}

Pose LaserTrack::getCurrentPose() const {
  Pose current_pose;
  current_pose.time_ns = getMaxTime();
  current_pose.T_w = trajectory_.evaluate(current_pose.time_ns);
  return current_pose;
}

void LaserTrack::getOdometryTrajectory(Trajectory* trajectory) const {
  CHECK_NOTNULL(trajectory)->clear();
  for (const auto& pose: pose_measurements_) {
    trajectory->emplace(pose.time_ns, pose.T_w);
  }
}

Time LaserTrack::getMinTime() const {
  return trajectory_.getMinTime();
}

Time LaserTrack::getMaxTime() const {
  return trajectory_.getMaxTime();
}

void LaserTrack::getLaserScansTimes(std::vector<curves::Time>* out_times_ns) const {
  CHECK_NOTNULL(out_times_ns)->clear();
  for (size_t i = 0u; i < laser_scans_.size(); ++i) {
    out_times_ns->push_back(laser_scans_[i].time_ns);
  }
}

void LaserTrack::appendPriorFactors(const Time& prior_time_ns, NonlinearFactorGraph* graph) const {
  CHECK_NOTNULL(graph);

  trajectory_.addPriorFactors(graph, prior_time_ns);
}

void LaserTrack::appendOdometryFactors(const curves::Time& optimization_min_time_ns,
                                       const curves::Time& optimization_max_time_ns,
                                       noiseModel::Base::shared_ptr noise_model,
                                       NonlinearFactorGraph* graph) const {
  CHECK_NOTNULL(graph);

  //TODO(Renaud): this can be optimized but let's see when we fix issue #27.
  for (const auto& odometry_measurement: odometry_measurements_) {
    if (odometry_measurement.time_a_ns >= optimization_min_time_ns &&
        odometry_measurement.time_b_ns <= optimization_max_time_ns) {
      graph->push_back(makeRelativeMeasurementFactor(odometry_measurement, noise_model));
    }
  }
}

void LaserTrack::appendICPFactors(const curves::Time& optimization_min_time_ns,
                                  const curves::Time& optimization_max_time_ns,
                                  noiseModel::Base::shared_ptr noise_model,
                                  NonlinearFactorGraph* graph) const {
  CHECK_NOTNULL(graph);

  for (size_t i = 0u; i < icp_transformations_.size(); ++i) {
    // If the second node falls within the optimization window.
    if (icp_transformations_[i].time_b_ns >= optimization_min_time_ns &&
        icp_transformations_[i].time_b_ns <= optimization_max_time_ns) {

      // If the first node also falls within the optimization window.
      if (icp_transformations_[i].time_a_ns >= optimization_min_time_ns &&
          icp_transformations_[i].time_a_ns <= optimization_max_time_ns) {
        graph->push_back(makeRelativeMeasurementFactor(icp_transformations_[i], noise_model));
      } else {
        graph->push_back(makeRelativeMeasurementFactor(icp_transformations_[i],
                                                       noise_model, true));
      }
    }
  }
}

//TODO(renaud): That's basically the same as above, just with a different variable. Can we combine?
void LaserTrack::appendLoopClosureFactors(const curves::Time& optimization_min_time_ns,
                                          const curves::Time& optimization_max_time_ns,
                                          noiseModel::Base::shared_ptr noise_model,
                                          NonlinearFactorGraph* graph) const {
  CHECK_NOTNULL(graph);

  for (size_t i = 0u; i < loop_closures_.size(); ++i) {
    // If the second node falls within the optimization window.
    if (loop_closures_[i].time_b_ns >= optimization_min_time_ns &&
        loop_closures_[i].time_b_ns <= optimization_max_time_ns) {

      // If the first node also falls within the optimization window.
      if (loop_closures_[i].time_a_ns >= optimization_min_time_ns &&
          loop_closures_[i].time_a_ns <= optimization_max_time_ns) {
        graph->push_back(makeRelativeMeasurementFactor(loop_closures_[i], noise_model));
      } else {
        graph->push_back(makeRelativeMeasurementFactor(loop_closures_[i],
                                                       noise_model, true));
      }
    }
  }
}

void LaserTrack::initializeGTSAMValues(const KeySet& keys, Values* values) const {
  trajectory_.initializeGTSAMValues(keys, values);
}

void LaserTrack::updateFromGTSAMValues(const Values& values) {
  trajectory_.updateFromGTSAMValues(values);
}

void LaserTrack::updateCovariancesFromGTSAMValues(const gtsam::NonlinearFactorGraph& factor_graph,
                                                  const gtsam::Values& values) {
  gtsam::KeySet keys = factor_graph.keys();
  gtsam::Marginals marginals(factor_graph, values);
  for (const auto& key: keys) {
    covariances_.push_back(marginals.marginalCovariance(key));
  }
}

ExpressionFactor<SE3>
LaserTrack::makeRelativeMeasurementFactor(const RelativePose& relative_pose_measurement,
                                          noiseModel::Base::shared_ptr noise_model,
                                          const bool fix_first_node) const {
  Expression<SE3> T_w_b(trajectory_.getValueExpression(relative_pose_measurement.time_b_ns));

  // If fix_first_node is true, a constant SE3 expression is used for T_w_a. That is, this
  // node will be fixed and not be part of the factor.
  // TODO(Renaud): is there a cleaner way of doing this?
  if (fix_first_node) {
    Expression<SE3> T_w_a(trajectory_.evaluate(relative_pose_measurement.time_a_ns));
    Expression<SE3> T_a_w(kindr::minimal::inverse(T_w_a));
    Expression<SE3> relative(kindr::minimal::compose(T_a_w, T_w_b));
    return ExpressionFactor<SE3>(noise_model,relative_pose_measurement.T_a_b, relative);
  } else {
    Expression<SE3> T_w_a(trajectory_.getValueExpression(relative_pose_measurement.time_a_ns));
    Expression<SE3> T_a_w(kindr::minimal::inverse(T_w_a));
    Expression<SE3> relative(kindr::minimal::compose(T_a_w, T_w_b));
    return ExpressionFactor<SE3>(noise_model,relative_pose_measurement.T_a_b, relative);
  }
}

void LaserTrack::computeICPTransformations() {
  if (getNumScans() > 1u) {
    Clock clock;
    if (params_.strategy_for_icp_transformations == "local_scan_to_sub_map") {
      local_scan_to_sub_map();
    } else if (params_.strategy_for_icp_transformations == "local_scan_to_local_scans") {
      local_scan_to_local_scans();
    } else {
      CHECK(false) << "This strategy for computing ICP transformations does not exist.";
    }
    clock.takeTime();
    LOG(INFO) << "Took " << clock.getRealTime() << " ms to compute the ICP transformations.";
  }
}

void LaserTrack::local_scan_to_sub_map() {
  LaserScan last_scan = laser_scans_.back();
  RelativePose icp_transformation;
  icp_transformation.time_b_ns = last_scan.time_ns;
  icp_transformation.time_a_ns = laser_scans_[getNumScans() - 2u].time_ns;

  // Transform the last (parameters_.nscan_in_sub_map - 1) scans
  // in the frame of the second last scan.
  Clock clock;
  const SE3 T_w_to_second_last_scan = trajectory_.evaluate(
      laser_scans_[getNumScans() - 2u].time_ns);
  DataPoints sub_map = laser_scans_[getNumScans() - 2u].scan;
  PointMatcher::TransformationParameters transformation_matrix;
  for (size_t i = 0u; i < std::min(getNumScans() - 2u, size_t(params_.nscan_in_sub_map - 1u)); ++i) {
    LaserScan previous_scan = laser_scans_[getNumScans() - 3u - i];
    transformation_matrix = (T_w_to_second_last_scan.inverse() *
        trajectory_.evaluate(previous_scan.time_ns)).getTransformationMatrix().cast<float>();

    correctTransformationMatrix(&transformation_matrix);

    sub_map.concatenate(rigid_transformation_->compute(previous_scan.scan,transformation_matrix));
  }
  clock.takeTime();
  LOG(INFO) << "Took " << clock.getRealTime() << " ms to build the submap.";

  // Obtain the initial guess from the trajectory.
  SE3 initial_guess = trajectory_.evaluate(icp_transformation.time_a_ns).inverse() *
      trajectory_.evaluate(icp_transformation.time_b_ns);
  transformation_matrix = initial_guess.getTransformationMatrix().cast<float>();

  // Compute the ICP solution.
  PointMatcher::TransformationParameters icp_solution = icp_.compute(last_scan.scan, sub_map,
                                                                     transformation_matrix);

  if (params_.save_icp_results) {
    last_scan.scan.save("/tmp/last_scan.vtk");
    sub_map.save("/tmp/sub_map.vtk");
    correctTransformationMatrix(&transformation_matrix);
    rigid_transformation_->compute(last_scan.scan,transformation_matrix).save(
        "/tmp/last_scan_alligned_by_initial_guess.vtk");
    correctTransformationMatrix(&icp_solution);
    rigid_transformation_->compute(last_scan.scan,icp_solution).save(
        "/tmp/last_scan_alligned_by_solution.vtk");
  }

  icp_transformation.T_a_b = convertTransformationMatrixToSE3(icp_solution);
  icp_transformation.key_a = getPoseKey(icp_transformation.time_a_ns);
  icp_transformation.key_b = getPoseKey(icp_transformation.time_b_ns);
  icp_transformations_.push_back(icp_transformation);
}

void LaserTrack::local_scan_to_local_scans() {
  // Transformations are computed in the frame of the previous nodes towards the newest node.
  RelativePose icp_transformation;
  LaserScan last_scan = laser_scans_.back();

  if (params_.save_icp_results) {
    last_scan.scan.save("/tmp/last_scan.vtk");
  }

  icp_transformation.time_b_ns = last_scan.time_ns;
  for (size_t i = 0u; i < std::min(getNumScans() - 1u, size_t(params_.nscan_to_match)); ++i) {
    LaserScan previous_scan = laser_scans_[getNumScans() - 2u - i];
    icp_transformation.time_a_ns = previous_scan.time_ns;

    // Obtain the initial guess from the trajectory.
    SE3 initial_guess = trajectory_.evaluate(icp_transformation.time_a_ns).inverse() *
        trajectory_.evaluate(icp_transformation.time_b_ns);

    PointMatcher::TransformationParameters initial_guess_matrix;
    initial_guess_matrix = initial_guess.getTransformationMatrix().cast<float>();

    // Compute the ICP solution.
    PointMatcher::TransformationParameters icp_solution = icp_.compute(last_scan.scan,
                                                                       previous_scan.scan,
                                                                       initial_guess_matrix);

    if (params_.save_icp_results) {
      // Save the clouds when matching to the furthest node (in time).
      if (i + 1u == std::min(getNumScans() - 1u, size_t(params_.nscan_to_match))) {
        previous_scan.scan.save("/tmp/previous_scan_" + std::to_string(i) + ".vtk");
        correctTransformationMatrix(&initial_guess_matrix);
        rigid_transformation_->compute(last_scan.scan,initial_guess_matrix).save(
            "/tmp/last_scan_alligned_by_initial_guess_" + std::to_string(i) + ".vtk");
        correctTransformationMatrix(&icp_solution);
        rigid_transformation_->compute(last_scan.scan,icp_solution).save(
            "/tmp/last_scan_alligned_by_solution" + std::to_string(i) + ".vtk");
      }
    }

    icp_transformation.T_a_b = convertTransformationMatrixToSE3(icp_solution);
    icp_transformation.key_a = getPoseKey(icp_transformation.time_a_ns);
    icp_transformation.key_b = getPoseKey(icp_transformation.time_b_ns);
    icp_transformations_.push_back(icp_transformation);
  }
}

Pose* LaserTrack::findPose(const Time& timestamp_ns) {
  CHECK(!pose_measurements_.empty()) << "Cannot register the scan as no pose was registered.";
  CHECK_LE(timestamp_ns, pose_measurements_.back().time_ns) << "The requested time ("
      << timestamp_ns << ") does not exist in the pose measurements. Last pose time is "
      << pose_measurements_.back().time_ns << ".";

  // Find an iterator to the pose measurement at the requested time.
  PoseVector::iterator it = pose_measurements_.end();
  do {
    --it;
  } while (it != pose_measurements_.begin() && it->time_ns != timestamp_ns);

  CHECK_EQ(it->time_ns, timestamp_ns) 
  << "The requested time does not exist in the pose measurements.";

  return &(*it);
}

Pose LaserTrack::findPose(const Time& timestamp_ns) const {
  CHECK(!pose_measurements_.empty()) << "Cannot register the scan as no pose was registered.";
  CHECK_LE(timestamp_ns, pose_measurements_.back().time_ns) << "The requested time ("
      << timestamp_ns << ") does not exist in the pose measurements. Last pose time is "
      << pose_measurements_.back().time_ns << ".";

  // Find an iterator to the pose measurement at the requested time.
  PoseVector::const_iterator it = pose_measurements_.end();
  do {
    --it;
  } while (it != pose_measurements_.begin() && it->time_ns != timestamp_ns);

  CHECK_EQ(it->time_ns, timestamp_ns)
  << "The requested time does not exist in the pose measurements.";

  return *it;
}

Pose LaserTrack::findNearestPose(const Time& timestamp_ns) const {
  CHECK(!pose_measurements_.empty()) << "Cannot find nearest pose as no pose was registered.";
  CHECK_LE(timestamp_ns, pose_measurements_.back().time_ns) << "The requested time ("
      << timestamp_ns << ") is later than the latest pose time. Latest pose time is "
      << pose_measurements_.back().time_ns << ".";

  Pose pose;
  pose.time_ns = timestamp_ns;
  pose.T_w = trajectory_.evaluate(timestamp_ns);
  // Not used.
  pose.key = Key();

  return pose;
}

Key LaserTrack::extendTrajectory(const Time& timestamp_ns, const SE3& value) {
  std::vector<Time> times_ns;
  std::vector<SE3> values;
  std::vector<Key> keys;
  times_ns.push_back(timestamp_ns);
  values.push_back(value);
  trajectory_.extend(times_ns, values, &keys);
  CHECK_EQ(keys.size(), 1u);
  return keys[0];
}

SE3 LaserTrack::convertTransformationMatrixToSE3(
    const PointMatcher::TransformationParameters& transformation_matrix) const {
  SO3 rotation = SO3::constructAndRenormalize(
      transformation_matrix.cast<double>().topLeftCorner<3,3>());
  SE3::Position position = transformation_matrix.cast<double>().topRightCorner<3,1>();
  return SE3(rotation, position);
}

} // namespace laser_slam
