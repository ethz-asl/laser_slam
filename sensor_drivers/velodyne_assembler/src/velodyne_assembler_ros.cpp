// VelodyneAssembler was originally developed by Philipp Kruesi.

#include <velodyne_assembler/velodyne_assembler_ros.hpp>

using namespace std;

VelodyneAssemblerRos::VelodyneAssemblerRos() :
        naive_assembling_(true),
        transformation_(PM::get().REG(Transformation).create("RigidTransformation")),
        last_azimuth_rad_(0.0),
        last_stamp_(0.0),
        initialized_(false),
        T_sensor_base_(PM::TransformationParameters::Identity(4,4)),
        T_base_sensor_(PM::TransformationParameters::Identity(4,4)),
        T_fixed_basePrevious_(PM::TransformationParameters::Identity(4,4)),
        T_sensorStart_sensorCurrent_(PM::TransformationParameters::Identity(4,4)) {
}

VelodyneAssemblerRos::~VelodyneAssemblerRos() {}

bool VelodyneAssemblerRos::init() {
  if (!getParams()) {
    return false;
  }

  ros::Duration(1.0).sleep();		// Let tf buffer fill up, avoid error messages.

  // Setup subscribers.
  pcl_sub_ = nh_.subscribe(raw_pcl_, kScanSubscriberMessageQueueSize,
                           &VelodyneAssemblerRos::pclCallback, this);
  // Setup publishers.
  assembled_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(assembled_pcl_, 1);

  // Get the static transform sensor --> vehicle_cor.
  ros::Time currentTime = ros::Time::now();
  tf::StampedTransform T_sensor_vehicleCor_tf;
  try {
    tf_listener_.waitForTransform(sensor_frame_id_, vehicle_base_frame_id_,
                                  currentTime, ros::Duration(20.0));
    tf_listener_.lookupTransform( sensor_frame_id_, vehicle_base_frame_id_,
                                  currentTime, T_sensor_vehicleCor_tf);
  }
  catch (tf::TransformException ex) {
    ROS_WARN("[velodyne_assembler] Unable to get transform from vehicleCor frame to sensor frame."
        "Terminating. %s", ex.what());
    return false;
  }

  Eigen::Affine3d eigen_transform;
  tf::transformTFToEigen(T_sensor_vehicleCor_tf, eigen_transform);
  T_sensor_base_ = eigen_transform.matrix().cast<float>();
  T_base_sensor_ = T_sensor_base_.inverse();

  return true;
}

unique_ptr<VelodyneAssemblerRos::DataPoints> VelodyneAssemblerRos::convertToFixedSensorFrame(
                                          const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg) {
  ros::Time stamp_in = msg->header.stamp;
  // Create the DataPoints object.
  unique_ptr<DataPoints> cloud(new DataPoints(
            PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*msg)));

  // get tf from rotating sensor frame to fixed sensor frame
  tf::StampedTransform T_target_rotating_tf;
  try {
    tf_listener_.waitForTransform(sensor_fixed_frame_id_, sensor_frame_id_,
                                    stamp_in, ros::Duration(0.1));

    tf_listener_.lookupTransform(sensor_fixed_frame_id_, sensor_frame_id_,
                                    stamp_in, T_target_rotating_tf);
    }
    catch (tf::TransformException ex) {
      ROS_WARN("[PointcloudConverter] Unable to get transform to fixed sensor frame. "
          "Setting to identity. %s", ex.what());
      T_target_rotating_tf.setIdentity();
    }
  // get PM representation of the trafo
  Eigen::Affine3d eigen_conv;
  tf::transformTFToEigen(T_target_rotating_tf, eigen_conv);
  eigen_conv = eigen_conv.rotate(Eigen::AngleAxisd(1.0*M_PI, Eigen::Vector3d::UnitZ()));
  PM::TransformationParameters T_target_rotating_tf_current = eigen_conv.matrix().cast<float>();
    
  // convert pcl to fixed sensor frame
  *cloud = transformation_->compute(
        *cloud, T_target_rotating_tf_current);

  return cloud;
}

void VelodyneAssemblerRos::pclCallback(
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg) {
  ros::Time stamp_in = msg->header.stamp;
  if (msg->header.frame_id != sensor_frame_id_ && msg->header.frame_id != "/" + sensor_frame_id_) {
    ROS_ERROR("[velodyne_assembler] Incoming point cloud is represented in an unknown frame. "
        "Expected: %s, received: %s",
              sensor_frame_id_.c_str(), msg->header.frame_id.c_str());
    return;
  }

  // Create the DataPoints object.
  unique_ptr<DataPoints> cloud_in;

  // Transform pcl to fixed sensor frame if primal sensor frame is rotating
  if (rotating_sensor_frame_) {
    cloud_in = std::move(convertToFixedSensorFrame(msg));
  } else {
    unique_ptr<DataPoints> cloud_tmp(new DataPoints(
      PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*msg)));
    cloud_in = std::move(cloud_tmp);
  }

  // Do nothing if the cloud contains no points.
  if (cloud_in->features.cols() > 0) {
    // Compute the transform to the origin of the last point cloud (rotation only):
    // T_previous_current.
    tf::StampedTransform T_fixed_base_tf;
    if (naive_assembling_) {
      T_fixed_base_tf.setIdentity();
    } else {
      try {
        tf_listener_.waitForTransform(fixed_frame_id_, vehicle_base_frame_id_,
                                      stamp_in, ros::Duration(0.1));

        tf_listener_.lookupTransform(fixed_frame_id_, vehicle_base_frame_id_,
                                     stamp_in, T_fixed_base_tf);
      }
      catch (tf::TransformException ex) {
        ROS_WARN("[velodyne_assembler] Unable to get transform to inertial frame. "
            "Setting to identity. %s", ex.what());
        T_fixed_base_tf.setIdentity();
      }
    }
    Eigen::Affine3d eigen_tr;
    tf::transformTFToEigen(T_fixed_base_tf, eigen_tr);
    PM::TransformationParameters T_fixed_base_current = eigen_tr.matrix().cast<float>();
    PM::TransformationParameters T_basePrevious_baseCurrent = T_fixed_basePrevious_.inverse() *
        T_fixed_base_current;
    T_fixed_basePrevious_ = T_fixed_base_current;

    // Make the assembled cloud (one revolution) start on the -y axis (sensor frame).
    constexpr double kStartAngleRad = M_PI / 2.0;
    double current_azimuth_rad = atan2(cloud_in->features(1,0), cloud_in->features(0,0));
    if ((last_azimuth_rad_ > kStartAngleRad && current_azimuth_rad <= kStartAngleRad) ||
        !initialized_) {
      // Publish point cloud and initialize new one when we've collected one full turn.
      if (initialized_) {
        // Transform the points (want to have timestamp at the end).
        *current_assembled_cloud_ = transformation_->compute(
            *current_assembled_cloud_, T_sensorStart_sensorCurrent_.inverse());
        // Publish the assembled cloud.
        if (rotating_sensor_frame_) {
          assembled_cloud_publisher_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
            *current_assembled_cloud_, sensor_fixed_frame_id_, last_stamp_));
        } else {
          assembled_cloud_publisher_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
            *current_assembled_cloud_, sensor_frame_id_, last_stamp_));
        }
        // Set and send the updated transform to tf.
        // Add the static offset of the sensor wrt the vehicle cor.
        PM::TransformationParameters T_sensorPrevious_sensorCurrent = T_sensor_base_ *
            T_basePrevious_baseCurrent * T_base_sensor_;
        // Compute the transform to the start of the assembled cloud.
        T_sensorStart_sensorCurrent_ = T_sensorStart_sensorCurrent_ *
            T_sensorPrevious_sensorCurrent;
      }

      current_assembled_cloud_ = unique_ptr<DataPoints>(new DataPoints(*cloud_in));
      initialized_ = true;

      T_sensorStart_sensorCurrent_ = PM::TransformationParameters::Identity(4,4);
    } else {
      // Add the static offset of the sensor wrt the vehicle cor.
      PM::TransformationParameters T_sensorPrevious_sensorCurrent = T_sensor_base_ *
          T_basePrevious_baseCurrent * T_base_sensor_;

      // Compute the transform to the start of the assembled cloud.
      T_sensorStart_sensorCurrent_ = T_sensorStart_sensorCurrent_ * T_sensorPrevious_sensorCurrent;

      // Transform the points.
      *cloud_in = transformation_->compute(*cloud_in, T_sensorStart_sensorCurrent_);

      // Concatenate.
      current_assembled_cloud_->concatenate(*cloud_in);
    }
    last_azimuth_rad_ = current_azimuth_rad;
    last_stamp_ = stamp_in;
  }
}

bool VelodyneAssemblerRos::getParams() {
  ros::NodeHandle private_nh("~");

  private_nh.param<bool>("naive_assembling", naive_assembling_, false);
  private_nh.param<bool>("rotating_sensor_frame", rotating_sensor_frame_, false);
  private_nh.param<string>("fixed_frame_id", fixed_frame_id_, "/odom");
  private_nh.param<string>("vehicle_base_frame_id", vehicle_base_frame_id_, "/base_link");
  private_nh.param<string>("sensor_frame_id", sensor_frame_id_, "/velodyne");
  private_nh.param<string>("sensor_frame_fixed_id", sensor_fixed_frame_id_, "/velodyne_axis_aligned");
  private_nh.param<string>("input_points", raw_pcl_, "/velodyne_points");
  private_nh.param<string>("assembled_cloud", assembled_pcl_, "/velodyne_assembled_cloud");

  ROS_INFO_STREAM("[velodyne_assembler] Param 'naive_assembling_' = " << naive_assembling_);
  ROS_INFO_STREAM("[velodyne_assembler] Param 'rotating_sensor_frame_' = " << rotating_sensor_frame_);
  ROS_INFO_STREAM("[velodyne_assembler] Param 'fixed_frame_id_' = " << fixed_frame_id_);
  ROS_INFO_STREAM("[velodyne_assembler] Param 'vehicle_base_frame_id_' = " << vehicle_base_frame_id_);
  ROS_INFO_STREAM("[velodyne_assembler] Param 'sensor_frame_id_' = " << sensor_frame_id_);
  ROS_INFO_STREAM("[velodyne_assembler] Param 'sensor_fixed_frame_id_' = " << sensor_fixed_frame_id_);
  ROS_INFO_STREAM("[velodyne_assembler] Param 'raw_pcl_' = " << raw_pcl_);
  ROS_INFO_STREAM("[velodyne_assembler] Param 'assembled_pcl_' = " << assembled_pcl_);

  return true;
}
