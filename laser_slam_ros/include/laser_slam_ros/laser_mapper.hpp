#ifndef LASER_SLAM_ROS_LASER_MAPPER_HPP_
#define LASER_SLAM_ROS_LASER_MAPPER_HPP_

#include <string>
#include <vector>

#include <laser_slam/parameters.hpp>
#include <laser_slam/incremental_estimator.hpp>
#include <laser_slam_ros/laser_slam_worker.hpp>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>

#include "laser_slam_ros/SaveMap.h"

namespace laser_slam_ros {

class LaserMapper {

 public:
  explicit LaserMapper(ros::NodeHandle &n);
  ~LaserMapper();

  /// \brief A thread function for handling map publishing.
  void publishMapThread();

  /// \brief A thread function for mapping.
  void MappingThread();

 protected:
  /// \brief Call back of the save_map service.
  bool saveMapServiceCall(laser_slam_ros::SaveMap::Request &request,
                          laser_slam_ros::SaveMap::Response &response);

  /// \brief Call back of the save_local_map service.
  bool saveLocalMapServiceCall(laser_slam_ros::SaveMap::Request &request,
                               laser_slam_ros::SaveMap::Response &response);

 private:
  // Get ROS parameters.
  void getParameters();

  // Node handle.
  ros::NodeHandle &nh_;
  std::mutex local_map_mutex_;

  // Publishers of the map
  ros::Publisher map_pub_;
  ros::Publisher local_map_pub_;
  static constexpr unsigned int kPublisherQueueSize = 50u;

  // Services.
  ros::ServiceServer save_distant_map_;
  ros::ServiceServer save_map_;
  ros::ServiceServer save_local_map_;
  ros::ServiceServer show_statistics_;

  // Incremental estimator.
  std::shared_ptr<laser_slam::IncrementalEstimator> incremental_estimator_;
  static constexpr double kSleepTime_s = 0.01;

  // laser_slam objects.
  laser_slam_ros::LaserSlamWorker laser_slam_worker_;
  laser_slam_ros::LaserSlamWorkerParams laser_slam_worker_params_;

  bool first_points_received_;
  unsigned int skip_counter_;

  // Pose of the robot when localization occured. Used to compute statistics on dead-reckoning
  // distances.
  laser_slam::SE3 pose_at_last_localization_;
  bool pose_at_last_localization_set_ = false;

  static constexpr laser_slam::Time kHeadDurationToExport_ns = 60000000000u;
  laser_slam::EstimatorParams online_estimator_params_;

  // ICP configurations.
  std::string icp_configuration_file_;
  std::string icp_input_filters_file_;
}; // LaserMapper

}
#endif /* LASER_SLAM_ROS_LASER_MAPPER_HPP_ */