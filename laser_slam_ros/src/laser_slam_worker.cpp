#include "laser_slam_ros/laser_slam_worker.hpp"

//TODO clean
#include <Eigen/Eigenvalues>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <laser_slam/common.hpp>
#include <laser_slam_ros/common.hpp>
#include <laser_slam_ros/laser_slam_worker.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace laser_slam_ros {

using namespace laser_slam;

LaserSlamWorker::LaserSlamWorker() { }

LaserSlamWorker::~LaserSlamWorker() { }

void LaserSlamWorker::init(
    ros::NodeHandle& nh, const LaserSlamWorkerParams& params,
    std::shared_ptr<laser_slam::IncrementalEstimator> incremental_estimator,
    unsigned int worker_id) {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  params_ = params;
  incremental_estimator_ = incremental_estimator;
  worker_id_ = worker_id;

  // Get the LaserTrack object from the IncrementalEstimator.
  laser_track_ = incremental_estimator_->getLaserTrack(worker_id);

  // Setup subscriber.
  scan_sub_ = nh.subscribe(params_.assembled_cloud_sub_topic, kScanSubscriberMessageQueueSize,
                           &LaserSlamWorker::scanCallback, this);

  // Setup publishers.
  trajectory_pub_ = nh.advertise<nav_msgs::Path>(params_.trajectory_pub_topic,
                                                 kPublisherQueueSize, true);

  if (params_.publish_local_map) {
    local_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(params_.local_map_pub_topic,
                                                            kPublisherQueueSize);
  }

  // Setup services.
  get_laser_track_srv_ = nh.advertiseService(
        params_.get_laser_track_srv_topic,
        &LaserSlamWorker::getLaserTracksServiceCall, this);

  voxel_filter_.setLeafSize(params_.voxel_size_m, params_.voxel_size_m,
                            params_.voxel_size_m);
  voxel_filter_.setMinimumPointsNumberPerVoxel(params_.minimum_point_number_per_voxel);

  // Set the first world to odom transform.
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix;
  matrix.resize(4, 4);
  matrix = Eigen::Matrix<float, 4,4>::Identity();
  world_to_odom_ = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
      matrix, params_.world_frame, params_.odom_frame, ros::Time::now());

  // TODO reactivate or rm.
  //  odometry_trajectory_pub_ = nh_.advertise<nav_msgs::Path>(params_.odometry_trajectory_pub_topic,
  //
  //  if (params_.publish_distant_map) {
  //    distant_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(params_.distant_map_pub_topic,
  //                                                               kPublisherQueueSize);
  //  }
  //  if (params_.publish_full_map) {
  //    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(params_.full_map_pub_topic,
  //                                                               kPublisherQueueSize);
  //  }
  //  new_fixed_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("new_fixed_cloud",
  //                                                               kPublisherQueueSize);
}

void LaserSlamWorker::scanCallback(const sensor_msgs::PointCloud2& cloud_msg_in) {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  if (tf_listener_.waitForTransform(params_.odom_frame, params_.sensor_frame,
                                    cloud_msg_in.header.stamp, ros::Duration(kTimeout_s))) {
    // Get the tf transform.
    tf::StampedTransform tf_transform;
    tf_listener_.lookupTransform(params_.odom_frame, params_.sensor_frame,
                                 cloud_msg_in.header.stamp, tf_transform);

    bool process_scan = false;
    SE3 current_pose;

    if (!last_pose_set_) {
      process_scan = true;
      last_pose_set_ = true;
      last_pose_ = tfTransformToPose(tf_transform).T_w;
    } else {
      current_pose = tfTransformToPose(tf_transform).T_w;
      float dist_m = distanceBetweenTwoSE3(current_pose, last_pose_);
      if (dist_m > params_.minimum_distance_to_add_pose) {
        process_scan = true;
        last_pose_ = current_pose;
      }
    }

    if (process_scan) {
      // Convert input cloud to laser scan.
      LaserScan new_scan;
      new_scan.scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in);
      new_scan.time_ns = rosTimeToCurveTime(cloud_msg_in.header.stamp.toNSec());

      // Process the new scan and get new values and factors.
      gtsam::NonlinearFactorGraph new_factors;
      gtsam::Values new_values;
      bool is_prior;
      laser_track_->processPoseAndLaserScan(tfTransformToPose(tf_transform), new_scan,
                                            &new_factors, &new_values, &is_prior);

      // Process the new values and factors.
      gtsam::Values result;
      if (is_prior) {
        result = incremental_estimator_->registerPrior(new_factors, new_values, worker_id_);
      } else {
        result = incremental_estimator_->estimate(new_factors, new_values);
      }

      // Update the trajectory.
      laser_track_->updateFromGTSAMValues(result);

      // Adjust the correction between the world and odom frames.
      Pose current_pose = laser_track_->getCurrentPose();
      SE3 T_odom_sensor = tfTransformToPose(tf_transform).T_w;
      SE3 T_w_sensor = current_pose.T_w;
      SE3 T_w_odom = T_w_sensor * T_odom_sensor.inverse();

      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix;

      // TODO resize needed?
      matrix.resize(4, 4);
      matrix = T_w_odom.getTransformationMatrix().cast<float>();

      world_to_odom_ = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
          matrix, params_.world_frame, params_.odom_frame, cloud_msg_in.header.stamp);

      publishTrajectories();

      // Get the last cloud in world frame.
      DataPoints new_fixed_cloud;
      laser_track_->getLocalCloudInWorldFrame(laser_track_->getMaxTime(), &new_fixed_cloud);


      // Transform the cloud in sensor frame
      //TODO(Renaud) move to a transformPointCloud() fct.
      //      laser_slam::PointMatcher::TransformationParameters transformation_matrix =
      //          T_w_sensor.inverse().getTransformationMatrix().cast<float>();
      //
      //      laser_slam::correctTransformationMatrix(&transformation_matrix);
      //
      //      laser_slam::PointMatcher::Transformation* rigid_transformation =
      //          laser_slam::PointMatcher::get().REG(Transformation).create("RigidTransformation");
      //      CHECK_NOTNULL(rigid_transformation);
      //
      //      laser_slam::PointMatcher::DataPoints fixed_cloud_in_sensor_frame =
      //          rigid_transformation->compute(new_fixed_cloud,transformation_matrix);
      //
      //
      //      new_fixed_cloud_pub_.publish(
      //          PointMatcher_ros::pointMatcherCloudToRosMsg<float>(fixed_cloud_in_sensor_frame,
      //                                                             params_.sensor_frame,
      //                                                             cloud_msg_in.header.stamp));

      PointCloud new_fixed_cloud_pcl = lpmToPcl(new_fixed_cloud);

      // Add the local scans to the full point cloud.
      if (params_.create_filtered_map) {
        if (new_fixed_cloud_pcl.size() > 0u) {
          if (local_map_.size() > 0u) {
            local_map_ += new_fixed_cloud_pcl;
          } else {
            local_map_ = new_fixed_cloud_pcl;
          }
        }
      }

    }
  } else {
    ROS_WARN_STREAM("[SegMapper] Timeout while waiting between " + params_.odom_frame  +
                    " and " + params_.sensor_frame  + ".");
  }
}

bool LaserSlamWorker::getLaserTracksServiceCall(
    laser_slam_ros::GetLaserTrackSrv::Request& request,
    laser_slam_ros::GetLaserTrackSrv::Response& response) {
  std::vector<std::shared_ptr<LaserTrack> > laser_tracks =
      incremental_estimator_->getAllLaserTracks();
  Trajectory trajectory;
  ros::Time scan_stamp;
  tf::StampedTransform tf_transform;
  geometry_msgs::TransformStamped ros_transform;
  for (const auto& track: laser_tracks) {
    track->getTrajectory(&trajectory);
    for (const auto& scan: track->getLaserScans()) {
      // Get data.
      scan_stamp.fromNSec(curveTimeToRosTime(scan.time_ns));
      // Fill response.
      response.laser_scans.push_back(
              PointMatcher_ros::pointMatcherCloudToRosMsg<float>(scan.scan,
                                                                 params_.sensor_frame,
                                                                 scan_stamp));
      tf_transform = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
          trajectory.at(scan.time_ns).getTransformationMatrix().cast<float>(),
          params_.world_frame,
          params_.sensor_frame,
          scan_stamp);
      tf::transformStampedTFToMsg(tf_transform, ros_transform);
      response.transforms.push_back(ros_transform);
    }
  }
  return true;
}

void LaserSlamWorker::publishTrajectory(const Trajectory& trajectory,
                                        const ros::Publisher& publisher) const {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  nav_msgs::Path traj_msg;
  traj_msg.header.frame_id = params_.world_frame;
  Time traj_time = curveTimeToRosTime(trajectory.rbegin()->first);
  traj_msg.header.stamp.fromNSec(traj_time);

  for (const auto& timePose : trajectory) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = traj_msg.header;
    pose_msg.header.stamp.fromNSec(curveTimeToRosTime(timePose.first));

    //TODO functionize
    pose_msg.pose.position.x = timePose.second.getPosition().x();
    pose_msg.pose.position.y = timePose.second.getPosition().y();
    pose_msg.pose.position.z = timePose.second.getPosition().z();
    pose_msg.pose.orientation.w = timePose.second.getRotation().w();
    pose_msg.pose.orientation.x = timePose.second.getRotation().x();
    pose_msg.pose.orientation.y = timePose.second.getRotation().y();
    pose_msg.pose.orientation.z = timePose.second.getRotation().z();
    traj_msg.poses.push_back(pose_msg);
  }
  publisher.publish(traj_msg);
}

void LaserSlamWorker::publishMap() {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  // TODO make thread safe.
  if (local_map_.size() > 0) {
    PointCloud filtered_map;
    getFilteredMap(&filtered_map);

    // Indicate that a new source cloud is ready to be used for localization and loop-closure.
    source_cloud_ready_ = true;

    //maximumNumberPointsFilter(&filtered_map);
    //    if (params_.publish_full_map) {
    //      sensor_msgs::PointCloud2 msg;
    //      convert_to_point_cloud_2_msg(filtered_map, params_.world_frame, &msg);
    //      point_cloud_pub_.publish(msg);
    //    }
    if (params_.publish_local_map) {
      sensor_msgs::PointCloud2 msg;
      convert_to_point_cloud_2_msg(local_map_filtered_, params_.world_frame, &msg);
      local_map_pub_.publish(msg);
    }
    //    if (params_.publish_distant_map) {
    //      sensor_msgs::PointCloud2 msg;
    //      convert_to_point_cloud_2_msg(distant_map_, params_.world_frame, &msg);
    //      distant_map_pub_.publish(msg);
    //    }
  }
}

void LaserSlamWorker::publishTrajectories() {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  Trajectory trajectory;
  laser_track_->getTrajectory(&trajectory);
  publishTrajectory(trajectory, trajectory_pub_);
  //  incremental_estimator_->getOdometryTrajectory(&trajectory);
  //  publishTrajectory(trajectory, odometry_trajectory_pub_);
}

// TODO can we move?
Pose LaserSlamWorker::tfTransformToPose(const tf::StampedTransform& tf_transform) {
  // Register new pose.
  Pose pose;
  SE3::Position pos(tf_transform.getOrigin().getX(), tf_transform.getOrigin().getY(),
                    tf_transform.getOrigin().getZ());
  SE3::Rotation::Implementation rot(tf_transform.getRotation().getW(),
                                    tf_transform.getRotation().getX(),
                                    tf_transform.getRotation().getY(),
                                    tf_transform.getRotation().getZ());
  pose.T_w = SE3(pos, rot);
  pose.time_ns = rosTimeToCurveTime(tf_transform.stamp_.toNSec());

  return pose;
}

Time LaserSlamWorker::rosTimeToCurveTime(const Time& timestamp_ns) {
  if (!base_time_set_) {
    base_time_ns_ = timestamp_ns;
    base_time_set_ = true;
  }
  return timestamp_ns - base_time_ns_;
}

Time LaserSlamWorker::curveTimeToRosTime(const Time& timestamp_ns) const {
  CHECK(base_time_set_);
  return timestamp_ns + base_time_ns_;
}

// TODO one shot of cleaning.
void LaserSlamWorker::getFilteredMap(PointCloud* filtered_map) {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  laser_slam::Pose current_pose = laser_track_->getCurrentPose();

  PclPoint current_position;
  current_position.x = current_pose.T_w.getPosition()[0];
  current_position.y = current_pose.T_w.getPosition()[1];
  current_position.z = current_pose.T_w.getPosition()[2];

  // Apply the cylindrical filter on the local map and get a copy.
  PointCloud local_map = local_map_;
  applyCylindricalFilter(current_position, params_.distance_to_consider_fixed,
                         40, false, &local_map_);

  // Apply a voxel filter.
  laser_slam::Clock clock;

  PointCloudPtr local_map_ptr(new PointCloud());
  pcl::copyPointCloud<PclPoint, PclPoint>(local_map, *local_map_ptr);

  PointCloud local_map_filtered;

  voxel_filter_.setInputCloud(local_map_ptr);
  voxel_filter_.filter(local_map_filtered);

  clock.takeTime();

  if (params_.separate_distant_map) {
    // If separating the map is enabled, the distance between each point in the local_map_ will
    // be compared to the current robot position. Points which are far from the robot will
    // be transfered to the distant_map_. This is helpful for publishing (points in distant_map_
    // need to be filtered only once) and for any other processing which needs to be done only
    // when a map is distant from robot and can be assumed as static (until loop closure).

    // TODO(renaud) Is there a way to separate the cloud without having to transform in sensor
    // frame by setting the position to compute distance from?
    // Transform local_map_ in sensor frame.
    clock.start();

    // Save before removing points.
    PointCloud new_distant_map = local_map_filtered;

    applyCylindricalFilter(current_position, params_.distance_to_consider_fixed,
                           40, false, &local_map_filtered);

    applyCylindricalFilter(current_position, params_.distance_to_consider_fixed,
                           40, true, &new_distant_map);

    local_map_filtered_ = local_map_filtered;

    // Add the new_distant_map to the distant_map_.
    if (distant_map_.size() > 0u) {
      distant_map_ += new_distant_map;
    } else {
      distant_map_ = new_distant_map;
    }

    *filtered_map = local_map_filtered;
    *filtered_map += distant_map_;

    clock.takeTime();
    // LOG(INFO) << "new_local_map.size() " << local_map.size();
    // LOG(INFO) << "new_distant_map.size() " << new_distant_map.size();
    // LOG(INFO) << "distant_map_.size() " << distant_map_.size();
    // LOG(INFO) << "Separating done! Took " << clock.getRealTime() << " ms.";
  } else {
    *filtered_map = local_map;
  }
}

void LaserSlamWorker::getLocalMapFiltered(PointCloud* local_map_filtered) {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  CHECK_NOTNULL(local_map_filtered);
  *local_map_filtered = local_map_filtered_;
}

void LaserSlamWorker::clearLocalMap() {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  local_map_.clear();
}

tf::StampedTransform LaserSlamWorker::getWorldToOdom() {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  tf::StampedTransform world_to_odom = world_to_odom_;
  return world_to_odom;
}

void LaserSlamWorker::getTrajectory(Trajectory* out_trajectory) const {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  laser_track_->getTrajectory(out_trajectory);
}

void LaserSlamWorker::getOdometryTrajectory(Trajectory* out_trajectory) const {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  laser_track_->getOdometryTrajectory(out_trajectory);
}

void LaserSlamWorker::updateLocalMap(const SE3& last_pose_before_update,
                                     const laser_slam::Time last_pose_before_update_timestamp_ns) {
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  Trajectory new_trajectory;
  laser_track_->getTrajectory(&new_trajectory);
  SE3 new_last_pose = new_trajectory.at(last_pose_before_update_timestamp_ns);

  const Eigen::Matrix4f transform_matrix = (new_last_pose * last_pose_before_update.inverse()).
      getTransformationMatrix().cast<float>();
  pcl::transformPointCloud(local_map_, local_map_, transform_matrix);
}

} // namespace laser_slam_ros
