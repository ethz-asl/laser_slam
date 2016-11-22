#ifndef LASER_SLAM_ROS_LASER_SLAM_WORKER_HPP_
#define LASER_SLAM_ROS_LASER_SLAM_WORKER_HPP_

#include <mutex>

#include <laser_slam/common.hpp>
#include <laser_slam/incremental_estimator.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
//TODO fix this dependency.
#include <segmatch/common.hpp>
#include <tf/transform_listener.h>

#include "laser_slam_ros/common.hpp"

namespace laser_slam_ros {

class LaserSlamWorker {

 public:
  LaserSlamWorker();
  ~LaserSlamWorker();

  void init(ros::NodeHandle& nh, const LaserSlamWorkerParams& params,
            laser_slam::IncrementalEstimator* incremental_estimator,
            std::mutex* incremental_estimator_mutex);

  /// \brief Register the local scans to the sliding window estimator.
  void scanCallback(const sensor_msgs::PointCloud2& cloud_msg_in);

  /// \brief Publish the robot trajectory (as path) in ROS.
  void publishTrajectory(const laser_slam::Trajectory& trajectory,
                         const ros::Publisher& publisher) const;

  /// \brief Publish the map.
  void publishMap();

  /// \brief Publish the estimated trajectory and the odometry only based trajectory.
  void publishTrajectories();

  void getLocalMapFiltered(segmatch::PointCloud* local_map_filtered);

  void clearLocalMap();

 private:
  // Convert a tf::StampedTransform to a laser_slam::Pose.
  laser_slam::Pose tfTransformToPose(const tf::StampedTransform& tf_transform);
  // TODO: common.hpp?
  laser_slam::SE3 geometryMsgTransformToSE3(const geometry_msgs::Transform& transform);
  geometry_msgs::Transform SE3ToGeometryMsgTransform(const laser_slam::SE3& transform);

  // Standardize the time so that the trajectory starts at time 0.
  laser_slam::Time rosTimeToCurveTime(const laser_slam::Time& timestamp_ns);

  // Convert time from trajectory base back to ROS base.
  laser_slam::Time curveTimeToRosTime(const laser_slam::Time& timestamp_ns) const;

  // Get a filtered map and apply map separation if desired.
  void getFilteredMap(segmatch::PointCloud* filtered_map);

  // TODO(renaud) : using ros::Time(0) means "use the latest available transform". Might solve your problem in relocalizer?
  bool getTransform(const std::string& first_frame,
                    const std::string& second_frame,
                    tf::StampedTransform* transform_ptr,
                    ros::Time transform_time = ros::Time(0));
 private:
  LaserSlamWorkerParams params_;
  // Subscribers.
  ros::Subscriber scan_sub_;

  // Publishers.
  ros::Publisher trajectory_pub_;
  ros::Publisher local_map_pub_;
  //  ros::Publisher odometry_trajectory_pub_;
  //  ros::Publisher point_cloud_pub_;
  //  ros::Publisher distant_map_pub_;
  //  ros::Publisher new_fixed_cloud_pub_;

  tf::TransformListener tf_listener_;

  // Pointers to the incremental estimator.
  laser_slam::IncrementalEstimator* incremental_estimator_;
  std::mutex* incremental_estimator_mutex_;

  // Contains the map which is estimated by the sliding window.
  segmatch::PointCloud local_map_;
  std::mutex local_map_mutex_;

  segmatch::PointCloud local_map_filtered_;
  std::mutex local_map_filtered_mutex_;

  // Contains the map which is distant from sensor and assumed to be fixed.
  // If the robot revisits the same environment, the distant_map_and local_map_ will be one
  // above each other, each with same density.
  segmatch::PointCloud distant_map_;
  std::mutex distant_map_mutex_;

  // Timestamp to be subtracted to each measurement time so that the trajectory starts at time 0.
  laser_slam::Time base_time_ns_ = 0;

  // Indicates whether the base time was set.
  bool base_time_set_ = false;

  laser_slam::SE3 last_pose_;
  bool last_pose_set_ = false;

  pcl::VoxelGrid<segmatch::PclPoint> voxel_filter_;

  // Indicates whether a new source cloud is ready for localization or loop-closure.
  bool source_cloud_ready_ = false;
  std::mutex source_cloud_ready_mutex_;

  static constexpr double kTimeout_s = 0.2;
  static constexpr unsigned int kScanSubscriberMessageQueueSize = 1u;
  static constexpr unsigned int kPublisherQueueSize = 50u;
}; // LaserSlamWorker

} // namespace laser_slam_ros

#endif /* LASER_SLAM_ROS_LASER_SLAM_WORKER_HPP_ */
