#include "laser_slam_ros/laser_mapper.hpp"

#include <stdlib.h>

#include <laser_slam/benchmarker.hpp>
#include <laser_slam/common.hpp>
#include <laser_slam_ros/common.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace laser_slam;
using namespace laser_slam_ros;

namespace laser_slam_ros {

LaserMapper::LaserMapper(ros::NodeHandle &n) : nh_(n) {
  // Load ROS parameters from server.
  getParameters();

  // Create local map publisher
  if (laser_slam_worker_params_.publish_local_map) {
    local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        laser_slam_worker_params_.local_map_pub_topic,
        kPublisherQueueSize);
  }

  // Setup the laser_slam workers.
    // Adjust the topics and frames for that laser_slam worker.
    LaserSlamWorkerParams params = laser_slam_worker_params_;

  std::shared_ptr<IncrementalEstimator> incremental_estimator(
      new IncrementalEstimator(online_estimator_params_, 1));

  incremental_estimator_ = incremental_estimator;
      // Subscribers.

      params.assembled_cloud_sub_topic = "/" + laser_slam_worker_params_
          .assembled_cloud_sub_topic;

      // TF frames.
      params.odom_frame =  "/" + laser_slam_worker_params_.odom_frame;
      params.sensor_frame =  "/" + laser_slam_worker_params_.sensor_frame;

      // Publishers.
      params.trajectory_pub_topic = "/" +
              laser_slam_worker_params_.trajectory_pub_topic;

      params.local_map_pub_topic = "/" +
              laser_slam_worker_params_.local_map_pub_topic;

    LOG(INFO) << "Robot subscribes to "
              << params.assembled_cloud_sub_topic << " "
              << params.odom_frame << " and " << params.sensor_frame;

    LOG(INFO) << "Robot publishes to "
              << params.trajectory_pub_topic << " and "
              << params.local_map_pub_topic;

    laser_slam_worker_.init(nh_, params, incremental_estimator_, 0u);

  // Advertise the save_map service.
  save_map_ =
      nh_.advertiseService("save_map", &LaserMapper::saveMapServiceCall, this);
  save_local_map_ = nh_.advertiseService("save_local_map",
                                         &LaserMapper::saveLocalMapServiceCall,
                                         this);

    skip_counter_ = 0u;
    first_points_received_ = false;
}

LaserMapper::~LaserMapper() {}

void LaserMapper::publishMapThread() {
  // Check if map publication is required.
  if (!laser_slam_worker_params_.publish_local_map)
    return;

  ros::Rate thread_rate(laser_slam_worker_params_.map_publication_rate_hz);
  while (ros::ok()) {
    LOG(INFO) << "publishing local maps ";
    laser_slam_ros::PointCloud filtered_map;
    laser_slam_worker_.getFilteredMap(&filtered_map);
    sensor_msgs::PointCloud2 msg;
    laser_slam_ros::convert_to_point_cloud_2_msg(
        filtered_map,
        laser_slam_worker_params_.world_frame, &msg);
    local_map_pub_.publish(msg);
    thread_rate.sleep();
  }
}

bool LaserMapper::saveMapServiceCall(laser_slam_ros::SaveMap::Request &request,
                                   laser_slam_ros::SaveMap::Response &response) {
//  try {
//    pcl::io::savePCDFileASCII(request.filename.data,
//                              local_maps_.front().getFilteredPoints());
//  }
//  catch (const std::runtime_error &e) {
//    ROS_ERROR_STREAM("Unable to save: " << e.what());
//    return false;
//  }
  return true;
}

bool LaserMapper::saveLocalMapServiceCall(laser_slam_ros::SaveMap::Request &request,
                                        laser_slam_ros::SaveMap::Response &response) {
  // TODO this is saving only the local map of worker ID 0.
//  std::unique_lock<std::mutex> map_lock(local_maps_mutexes_[0]);
//  MapCloud local_map;
//  local_map += local_maps_[0].getFilteredPoints();
//  map_lock.unlock();
//  try {
//    pcl::io::savePCDFileASCII(request.filename.data,
//                              mapPoint2PointCloud(local_map));
//  }
//  catch (const std::runtime_error &e) {
//    ROS_ERROR_STREAM("Unable to save: " << e.what());
//    return false;
//  }
  return true;
}

void LaserMapper::getParameters() {
  // LaserMapper parameters.
  const std::string ns = "/LaserMapper";

  // laser_slam worker parameters.
  laser_slam_worker_params_ = laser_slam_ros::getLaserSlamWorkerParams(nh_, ns);
  nh_.getParam(ns + "/world_frame",
               laser_slam_worker_params_.world_frame);
  // Online estimator parameters.
  online_estimator_params_ = laser_slam_ros::getOnlineEstimatorParams(nh_, ns);
  nh_.getParam("icp_configuration_file",
               online_estimator_params_.laser_track_params
               .icp_configuration_file);
  nh_.getParam("icp_input_filters_file",
               online_estimator_params_.laser_track_params
               .icp_input_filters_file);
}
}