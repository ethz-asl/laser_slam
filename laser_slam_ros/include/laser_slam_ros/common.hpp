#ifndef LASER_SLAM_ROS_COMMON_HPP_
#define LASER_SLAM_ROS_COMMON_HPP_

#include <laser_slam/parameters.hpp>
#include <ros/ros.h>

namespace laser_slam_ros {

struct LaserSlamWorkerParams {
  // Map creation & filtering parameters.
  double distance_to_consider_fixed;
  bool separate_distant_map;
  bool create_filtered_map;
  double minimum_distance_to_add_pose;
  double voxel_size_m;
  int minimum_point_number_per_voxel;

  // Frames.
  std::string odom_frame;
  std::string sensor_frame;
  std::string world_frame;

  // Topics.
  std::string assembled_cloud_sub_topic;
  std::string trajectory_pub_topic;
  std::string odometry_trajectory_pub_topic;
  std::string full_map_pub_topic;
  std::string local_map_pub_topic;
  std::string distant_map_pub_topic;

  // Map publication.
  bool publish_local_map;
  bool publish_full_map;
  bool publish_distant_map;
  double map_publication_rate_hz;
}; // struct LaserSlamWorkerParams

static LaserSlamWorkerParams getLaserSlamWorkerParams(const ros::NodeHandle& nh,
                                                      const std::string& prefix) {
  LaserSlamWorkerParams params;
  const std::string ns = prefix + "/LaserSlamWorker";

  nh.getParam(ns + "/distance_to_consider_fixed", params.distance_to_consider_fixed);
  nh.getParam(ns + "/separate_distant_map", params.separate_distant_map);
  nh.getParam(ns + "/create_filtered_map", params.create_filtered_map);
  nh.getParam(ns + "/minimum_distance_to_add_pose", params.minimum_distance_to_add_pose);
  nh.getParam(ns + "/voxel_size_m", params.voxel_size_m);
  nh.getParam(ns + "/minimum_point_number_per_voxel", params.minimum_point_number_per_voxel);

  nh.getParam(ns + "/odom_frame", params.odom_frame);
  nh.getParam(ns + "/sensor_frame", params.sensor_frame);
  nh.getParam(ns + "/world_frame", params.world_frame);

  nh.getParam(ns + "/publish_local_map", params.publish_local_map);
  nh.getParam(ns + "/publish_full_map", params.publish_full_map);
  nh.getParam(ns + "/publish_distant_map", params.publish_distant_map);
  nh.getParam(ns + "/map_publication_rate_hz", params.map_publication_rate_hz);

  nh.getParam(ns + "/assembled_cloud_sub_topic", params.assembled_cloud_sub_topic);
  nh.getParam(ns + "/trajectory_pub_topic", params.trajectory_pub_topic);
  nh.getParam(ns + "/odometry_trajectory_pub_topic", params.odometry_trajectory_pub_topic);
  nh.getParam(ns + "/full_map_pub_topic", params.full_map_pub_topic);
  nh.getParam(ns + "/local_map_pub_topic", params.local_map_pub_topic);
  nh.getParam(ns + "/distant_map_pub_topic", params.distant_map_pub_topic);

  return params;
}

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
