#include <thread>

#include <ros/ros.h>

#include "laser_slam_ros/laser_mapper.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "LaserMapper");
  ros::NodeHandle node_handle("~");

  laser_slam_ros::LaserMapper mapper(node_handle);

  std::thread publish_map_thread
  (&laser_slam_ros::LaserMapper::publishMapThread, &mapper);

  try {
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception");
    return 1;
  }

  publish_map_thread.join();

  return 0;
}