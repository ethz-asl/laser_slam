#ifndef LASER_SLAM_PARAMETERS_HPP_
#define LASER_SLAM_PARAMETERS_HPP_

#include <Eigen/Dense>

namespace laser_slam {

struct LaserTrackParams {
  std::string icp_configuration_file;
  std::string icp_input_filters_file;
  bool use_icp_factors;
  std::string strategy_for_icp_transformations;
  int nscan_to_match;
  int nscan_in_sub_map;
  bool save_icp_results;
}; // struct LaserTrackParams

struct OnlineEstimatorParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double,6,1> odometry_noise_model;
  Eigen::Matrix<double,6,1> icp_noise_model;
  Eigen::Matrix<double,6,1> loop_closure_noise_model;
  bool add_m_estimator_on_odom;
  bool add_m_estimator_on_icp;
  bool add_m_estimator_on_loop_closures;
  int sliding_window_size;
  bool add_intermediate_poses;
  bool publish_covariances;
  LaserTrackParams laser_track_params;
}; // struct OnlineEstimatorParams

} // namespace laser_slam

#endif // LASER_SLAM_PARAMETERS_HPP_
