#ifndef LASER_SLAM_ROS_COMMON_HPP_
#define LASER_SLAM_ROS_COMMON_HPP_

#include <laser_slam/parameters.hpp>

namespace laser_slam_ros {

static laser_slam::LaserTrackParams getLaserTrackParams(const ros::NodeHandle& nh,
                                                        const std::string& prefix) {
  laser_slam::LaserTrackParams params;
  const std::string ns = prefix + "/LaserTrack";

  std::vector<float> odometry_noise_model, icp_noise_model;
  constexpr unsigned int kNoiseModelDimension = 6u;
  nh.getParam(ns + "/odometry_noise_model", odometry_noise_model);
  CHECK_EQ(odometry_noise_model.size(), kNoiseModelDimension);
  for (size_t i = 0u; i < 6u; ++i) {
    params.odometry_noise_model[i] = odometry_noise_model.at(i);
  }
  nh.getParam(ns + "/icp_noise_model", icp_noise_model);
  CHECK_EQ(icp_noise_model.size(), kNoiseModelDimension);
  for (size_t i = 0u; i < 6u; ++i) {
    params.icp_noise_model[i] = icp_noise_model.at(i);
  }
  nh.getParam(ns + "/add_m_estimator_on_odom", params.add_m_estimator_on_odom);
  nh.getParam(ns + "/add_m_estimator_on_icp", params.add_m_estimator_on_icp);

  // TODO move loading of icp_configuration_file and icp_input_filters_file to here.
  nh.getParam(ns + "/use_icp_factors", params.use_icp_factors);
  nh.getParam(ns + "/strategy_for_icp_transformations", params.strategy_for_icp_transformations);
  nh.getParam(ns + "/nscan_to_match", params.nscan_to_match);
  nh.getParam(ns + "/nscan_in_sub_map", params.nscan_in_sub_map);
  nh.getParam(ns + "/save_icp_results", params.save_icp_results);


  return params;
}

static laser_slam::EstimatorParams getOnlineEstimatorParams(const ros::NodeHandle& nh,
                                                            const std::string& prefix) {
  laser_slam::EstimatorParams params;
  const std::string ns = prefix + "/OnlineEstimator";

  std::vector<float>  loop_closure_noise_model;
  constexpr unsigned int kNoiseModelDimension = 6u;
  nh.getParam(ns + "/loop_closure_noise_model", loop_closure_noise_model);
  CHECK_EQ(loop_closure_noise_model.size(), kNoiseModelDimension);
  for (size_t i = 0u; i < 6u; ++i) {
    params.loop_closure_noise_model[i] = loop_closure_noise_model.at(i);
  }
  nh.getParam(ns + "/add_m_estimator_on_loop_closures", params.add_m_estimator_on_loop_closures);

  nh.getParam(ns + "/do_icp_step_on_loop_closures", params.do_icp_step_on_loop_closures);
  nh.getParam(ns + "/loop_closures_sub_maps_radius", params.loop_closures_sub_maps_radius);

  nh.getParam(ns + "/sliding_window_size", params.sliding_window_size);
  nh.getParam(ns + "/add_intermediate_poses", params.add_intermediate_poses);
  nh.getParam(ns + "/publish_covariances", params.publish_covariances);

  params.laser_track_params = getLaserTrackParams(nh, ns);

  return params;
}

} // namespace segmatch_ros

#endif // LASER_SLAM_ROS_COMMON_HPP_
