#ifndef LASER_SLAM_COMMON_HPP_
#define LASER_SLAM_COMMON_HPP_

#include <fstream>
#include <limits>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <mincurves/DiscreteSE3Curve.hpp>
#include <pcl/common/transforms.h>
#include <pcl/correspondence.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointmatcher/PointMatcher.h>

namespace laser_slam {

typedef PointMatcher<float> PointMatcher;
typedef typename PointMatcher::DataPoints DataPoints;

typedef kindr::minimal::QuatTransformationTemplate<double> SE3;
typedef SE3::Rotation SO3;

typedef curves::Time Time;

/// \brief Timer helper class.
class Clock {

 public:
  Clock() { start(); }

  /// \brief Start clock timer.
  void start(){
    gettimeofday(&real_time_start_, NULL);
    cpu_start_ = clock();
  }

  /// \brief Sample clock timer.
  void takeTime(){
    struct timeval end;
    gettimeofday(&end, NULL);
    cpu_time_ms_ = double(clock() - cpu_start_) / CLOCKS_PER_SEC * kSecondsToMiliseconds;

    long seconds, useconds;

    seconds  = end.tv_sec  - real_time_start_.tv_sec;
    useconds = end.tv_usec - real_time_start_.tv_usec;
    real_time_ms_ = (seconds * kSecondsToMiliseconds +
        useconds * kMicrosecondsToMiliseconds) + 0.5;
  }

  /// \brief Return elapsed physical time.
  double getRealTime() { return real_time_ms_; }

  /// \brief Return elapsed CPU time.
  double getCPUTime() { return cpu_time_ms_; }

  double takeRealTime() { takeTime(); return getRealTime(); }

 private:
  struct timeval real_time_start_;
  double real_time_ms_, cpu_time_ms_;
  clock_t cpu_start_;

  static constexpr double kSecondsToMiliseconds = 1000.0;
  static constexpr double kMicrosecondsToMiliseconds = 0.001;
};

static double multiplyVectorsImplementation(Eigen::Vector3d a,
                                            Eigen::Vector3d b,
                                            gtsam::OptionalJacobian<1,3> Ha,
                                            gtsam::OptionalJacobian<1,3> Hb) {
  if(Ha)
    *Ha = b.transpose();

  if(Hb)
    *Hb = a.transpose();

  return a.transpose() * b;
}

static gtsam::Expression<double> multiplyVectors(const gtsam::Expression<Eigen::Vector3d>& C1,
                                                 const gtsam::Expression<Eigen::Vector3d>& C2) {
  return gtsam::Expression<double>(&multiplyVectorsImplementation, C1, C2);
}

/// \brief Key type.
typedef size_t Key;

/// \brief Pose type including absolute transformation and time stamp.
struct Pose {
  /// \brief Absolute transform.
  SE3 T_w;
  /// \brief Time stamp.
  curves::Time time_ns;
  /// \brief Node key.
  Key key;
};

/// \brief RelativePose type including relative transformation and interval time stamps.
struct RelativePose {
  /// \brief Relative transform.
  SE3 T_a_b;
  /// \brief Time stamp at frame A.
  curves::Time time_a_ns;
  /// \brief Time stamp at frame B.
  curves::Time time_b_ns;
  /// \brief Prior node key.
  Key key_a;
  /// \brief Posterior node key.
  Key key_b;
  unsigned int track_id_a;
  unsigned int track_id_b;
};

/// \brief LaserScan type including point cloud and time stamp.
struct LaserScan {
  /// \brief Local point cloud scan.
  DataPoints scan;
  /// \brief Time stamp.
  curves::Time time_ns;
  /// \brief Node key.
  Key key;
};

typedef Eigen::MatrixXd Covariance;


// The Aligned declaration should be used when one wishes
// to have a std::vector<> containing Eigen fixed types.
template<template<typename, typename> class Container, typename Type>
using Aligned = Container<Type, Eigen::aligned_allocator<Type> >;

typedef Aligned<std::vector, Pose> PoseVector;
typedef Aligned<std::vector, RelativePose> RelativePoseVector;

typedef std::map<Time, SE3> Trajectory;

// Correct transformation matrix.
static void correctTransformationMatrix(
    PointMatcher::TransformationParameters* transformation_matrix) {
  CHECK_NOTNULL(transformation_matrix);

  PointMatcher::Transformation* rigid_transformation =
      PointMatcher::get().REG(Transformation).create("RigidTransformation");
  CHECK_NOTNULL(rigid_transformation);

  if (!rigid_transformation->checkParameters(*transformation_matrix)) {
    LOG(WARNING) << "The transformation matrix does not represent a valid rigid "
        << "transformation. Projecting onto an orthogonal basis.";
    *transformation_matrix = rigid_transformation->correctParameters(*transformation_matrix);
  }
}

typedef std::vector<std::string> StringRow;
typedef std::vector<std::vector<std::string> > StringMatrix;

// Helper function to write 'matrix' of strings to a CSV file.
static void writeCSV(const StringMatrix& string_matrix, const std::string& filename) {
  CHECK_GE(string_matrix.size(), 1) << "Provided matrix of strings had no entries.";
  std::ofstream out_file_stream;
  out_file_stream.open(filename.c_str());

  // Iterate over the rows of the string matrix and write comma-separated fields.
  for (StringMatrix::const_iterator it = string_matrix.begin(); it != string_matrix.end(); ++it) {
    CHECK_GE(it->size(), 1) << "String matrix row has no entries.";
    out_file_stream << it->at(0u);
    for (size_t i = 1u; i < it->size(); ++i) {
      out_file_stream << "," << it->at(i);
    }
    out_file_stream << std::endl;
  }
  out_file_stream.close();
}

// Helper function to write an Eigen::MatrixXd to a CSV file.
static void writeEigenMatrixXdCSV(const Eigen::MatrixXd& matrix, const std::string& filename) {
  StringMatrix string_matrix;
  string_matrix.reserve(matrix.rows());
  StringRow string_row;
  string_row.reserve(matrix.cols());
  for (size_t i = 0u; i < matrix.rows(); ++i) {
    string_row.clear();
    for (size_t j = 0u; j < matrix.cols(); ++j) {
      string_row.push_back(std::to_string(matrix(i,j)));
    }
    string_matrix.push_back(string_row);
  }
  writeCSV(string_matrix, filename);
}

// Helper function to read CSV files into 'matrix' of strings.
static StringMatrix loadCSV(std::string file_name) {
  // Open file stream and check that it is error free.
  std::ifstream in_file_stream(file_name.c_str());
  CHECK(in_file_stream.good()) << "error opening input file " << file_name;
  StringMatrix string_matrix;
  // Loop over lines (rows) of CSV file.
  std::string line;
  while(std::getline(in_file_stream, line)) {
    std::istringstream ss(line);
    StringRow str_row;
    // Loop over comma separated fields of CSV line.
    std::string field;
    while (getline(ss, field,',')) {
      str_row.push_back(field);
    }
    string_matrix.push_back(str_row);
  }
  in_file_stream.close();
  return string_matrix;
}

// Helper function to read a CSV file into an Eigen::MatrixXd.
static void loadEigenMatrixXdCSV(std::string file_name, Eigen::MatrixXd* matrix) {
  CHECK_NOTNULL(matrix);

  // Load CSV to matrix of strings
  StringMatrix string_matrix = loadCSV(file_name);
  CHECK_GE(string_matrix.size(), 1) << "CSV " << file_name << "was empty.";

  // Iterate over the rows of the CSV and use comma-separated fields to populate outputs.
  const unsigned n_rows = string_matrix.size();
  const unsigned n_cols = string_matrix[0].size();
  matrix->resize(n_rows, n_cols);

  for (size_t i = 0u; i < n_rows; ++i) {
    for (size_t j = 0u; j < n_cols; ++j) {
      (*matrix)(i,j) = atof(string_matrix[i][j].c_str());
    }
  }
  LOG(INFO) << "Loaded " << file_name << " with " << n_rows << " rows and " <<
      n_cols << " cols.";
}

/// \brief Optimization result structure.
struct OptimizationResult {
  /// \brief Number of optimizer iterations performed.
  size_t num_iterations = 0;
  /// \brief Number of intermediate steps performed within the optimization.
  /// For L-M, this is the number of lambdas tried.
  size_t num_intermediate_steps = 0;
  /// \brief Number of variables.
  size_t num_variables = 0;
  /// \brief Initial factor graph error.
  double initial_error = 0;
  /// \brief Final factor graph error.
  double final_error = 0;
  /// \brief Optimization duration
  long duration_ms = 0;
  /// \brief CPU optimization duration.
  long durationCpu_ms = 0;
};

static SE3 convertTransformationMatrixToSE3(
    const PointMatcher::TransformationParameters& transformation_matrix) {
  SO3 rotation = SO3::constructAndRenormalize(
      transformation_matrix.cast<double>().topLeftCorner<3,3>());
  SE3::Position position = transformation_matrix.cast<double>().topRightCorner<3,1>();
  return SE3(rotation, position);
}

// PCL objects and functions definitions.
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointICloud;
typedef std::pair<PointICloud, PointICloud> PointICloudPair;
typedef PointICloud::Ptr PointICloudPtr;
typedef std::pair<PointI, PointI> PointIPair;
typedef std::vector<PointIPair> PointIPairs;

typedef pcl::PointXYZ PclPoint;
typedef pcl::PointCloud<PclPoint> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef std::pair<PclPoint, PclPoint> PointPair;
typedef std::vector<PointPair> PointPairs;

typedef pcl::Normal PclNormal;
typedef pcl::PointCloud<PclNormal> PointNormals;
typedef pcl::PointCloud<PclNormal>::Ptr PointNormalsPtr;

typedef pcl::PointCloud<pcl::FPFHSignature33> PointDescriptors;
typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr PointDescriptorsPtr;

static void loadCloud(const std::string& filename, PointICloud* cloud) {
  LOG(INFO) <<"Loading cloud: " << filename << ".";
  CHECK_NE(pcl::io::loadPCDFile(filename, *cloud), -1) <<
      "Failed to load cloud: " << filename << ".";
}

struct Translation {
  Translation(double x_in, double y_in, double z_in) :
    x(x_in), y(y_in), z(z_in) {}
  double x;
  double y;
  double z;
};

static void translateCloud(const Translation& translation, PointICloud* cloud) {
  for (size_t i = 0u; i < cloud->size(); ++i) {
    cloud->points[i].x += translation.x;
    cloud->points[i].y += translation.y;
    cloud->points[i].z += translation.z;
  }
}

static PointPair findLimitPoints(const PointICloud& cloud) {
  PclPoint min_point, max_point;
  min_point.x = std::numeric_limits<float>::max();
  min_point.y = std::numeric_limits<float>::max();
  min_point.z = std::numeric_limits<float>::max();

  max_point.x = std::numeric_limits<float>::min();
  max_point.y = std::numeric_limits<float>::min();
  max_point.z = std::numeric_limits<float>::min();

  for (size_t i = 0u; i < cloud.size(); ++i) {
    if (cloud.points[i].x < min_point.x) {
      min_point.x = cloud.points[i].x;
    }
    if (cloud.points[i].y < min_point.y) {
      min_point.y = cloud.points[i].y;
    }
    if (cloud.points[i].z < min_point.z) {
      min_point.z = cloud.points[i].z;
    }
    if (cloud.points[i].x > max_point.x) {
      max_point.x = cloud.points[i].x;
    }
    if (cloud.points[i].y > max_point.y) {
      max_point.y = cloud.points[i].y;
    }
    if (cloud.points[i].z > max_point.z) {
      max_point.z = cloud.points[i].z;
    }
  }
  return PointPair(min_point, max_point);
}

static void extractBox(const PointPair& limit_points, float margin, PointICloud* cloud) {
  CHECK_NOTNULL(cloud);
  PointICloud filtered_cloud;
  for (size_t i = 0u; i < cloud->size(); ++i) {
    if (cloud->points[i].x > limit_points.first.x - margin &&
        cloud->points[i].x < limit_points.second.x + margin &&
        cloud->points[i].y > limit_points.first.y - margin &&
        cloud->points[i].y < limit_points.second.y + margin &&
        cloud->points[i].z > limit_points.first.z - margin &&
        cloud->points[i].z < limit_points.second.z + margin) {
      filtered_cloud.points.push_back(cloud->points[i]);
    }
  }
  LOG(INFO) << "Extracting box from " << cloud->size() << " points to " << filtered_cloud.size() << " points.";
  *cloud = filtered_cloud;
}

static void applyCylindricalFilter(const PclPoint& center, double radius_m,
                                   double height_m, PointICloud* cloud) {
  CHECK_NOTNULL(cloud);
  PointICloud filtered_cloud;

  const double radius_squared = pow(radius_m, 2.0);
  const double height_halved_m = height_m / 2.0;

  for (size_t i = 0u; i < cloud->size(); ++i) {
    if ((pow(cloud->points[i].x - center.x, 2.0)
        + pow(cloud->points[i].y - center.y, 2.0)) <= radius_squared &&
        abs(cloud->points[i].z - center.z) <= height_halved_m) {
      filtered_cloud.points.push_back(cloud->points[i]);
    }
  }

  filtered_cloud.width = 1;
  filtered_cloud.height = filtered_cloud.points.size();

  ROS_INFO_STREAM("Applied cylindrical filter from " << cloud->size()
                  << " points to " << filtered_cloud.size() << " points.");
  *cloud = filtered_cloud;
}

// Find the (index of the) nearest neighbour of point in target_cloud.
static bool findNearestNeighbour(const PclPoint& point, const PointCloud& target_cloud,
                                 size_t* index, float* squared_distance = NULL) {
  CHECK_NOTNULL(index);

  // Set up nearest neighbour search.
  pcl::KdTreeFLANN<PclPoint> kdtree;
  PointCloudPtr target_cloud_copy_ptr(new PointCloud);
  pcl::copyPointCloud(target_cloud, *target_cloud_copy_ptr);
  kdtree.setInputCloud(target_cloud_copy_ptr);
  std::vector<int> nearest_neighbour_indices(1);
  std::vector<float> nearest_neighbour_squared_distances(1);

  // Find the nearest neighbours in target.
  if (kdtree.nearestKSearch(point, 1, nearest_neighbour_indices,
                            nearest_neighbour_squared_distances) <= 0) {
    LOG(ERROR) << "Nearest neighbour search failed.";
    return false;
  }

  // Return values.
  *index = nearest_neighbour_indices.at(0);
  if (squared_distance != NULL) { *squared_distance = nearest_neighbour_squared_distances.at(0); }
  return true;
}

static bool findNearestNeighbour(const PointI& point, const PointICloud& target_cloud,
                                 size_t* index, float* squared_distance = NULL) {
  // Convert inputs.
  // TODO: templates would be better.
  PclPoint point_converted;
  point_converted.x = point.x;
  point_converted.y = point.y;
  point_converted.z = point.z;
  PointCloud target_cloud_converted;
  pcl::copyPointCloud(target_cloud, target_cloud_converted);
  // Use pre-existing function.
  return findNearestNeighbour(point_converted, target_cloud_converted, index, squared_distance);
}

static PointCloud lpmToPcl(const laser_slam::PointMatcher::DataPoints& cloud_in) {
  PointCloud cloud_out;
  cloud_out.width = cloud_in.getNbPoints();
  cloud_out.height = 1;
  for (size_t i = 0u; i < cloud_in.getNbPoints(); ++i) {
    PclPoint point;
    point.x = cloud_in.features(0,i);
    point.y = cloud_in.features(1,i);
    point.z = cloud_in.features(2,i);
    cloud_out.push_back(point);
  }
  return cloud_out;
}

static void applyCylindricalFilter(const PclPoint& center, double radius_m,
                                   double height_m, bool remove_point_inside,
                                   PointCloud* cloud) {
  CHECK_NOTNULL(cloud);
  PointCloud filtered_cloud;

  const double radius_squared = pow(radius_m, 2.0);
  const double height_halved_m = height_m / 2.0;

  for (size_t i = 0u; i < cloud->size(); ++i) {
    if (remove_point_inside) {
      if ((pow(cloud->points[i].x - center.x, 2.0)
          + pow(cloud->points[i].y - center.y, 2.0)) >= radius_squared ||
          abs(cloud->points[i].z - center.z) >= height_halved_m) {
        filtered_cloud.points.push_back(cloud->points[i]);
      }
    } else {
      if ((pow(cloud->points[i].x - center.x, 2.0)
          + pow(cloud->points[i].y - center.y, 2.0)) <= radius_squared &&
          abs(cloud->points[i].z - center.z) <= height_halved_m) {
        filtered_cloud.points.push_back(cloud->points[i]);
      }
    }
  }

  filtered_cloud.width = 1;
  filtered_cloud.height = filtered_cloud.points.size();

  *cloud = filtered_cloud;
}

static PclPoint laserSlamPoseToPclPoint(const laser_slam::Pose& pose) {
  PclPoint point;
  point.x = pose.T_w.getPosition()(0);
  point.y = pose.T_w.getPosition()(1);
  point.z = pose.T_w.getPosition()(2);
  return point;
}

static PclPoint se3ToPclPoint(const laser_slam::SE3& transform) {
  PclPoint point;
  point.x = transform.getPosition()(0);
  point.y = transform.getPosition()(1);
  point.z = transform.getPosition()(2);
  return point;
}

static void transformPointCloud(const SE3& transform, PointICloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  const Eigen::Matrix4f transform_matrix = transform.getTransformationMatrix().cast<float>();
  pcl::transformPointCloud(*point_cloud, *point_cloud, transform_matrix);
}

static void transformPclPoint(const SE3& transform, PclPoint* point) {
  CHECK_NOTNULL(point);
  Eigen::Matrix<double, 3, 1> eigen_point;
  eigen_point << point->x, point->y, point->z;
  eigen_point = transform.transform(eigen_point);
  point->x = eigen_point(0);
  point->y = eigen_point(1);
  point->z = eigen_point(2);
}

static void applyRandomFilterToCloud(double ratio_of_points_to_keep,
                                     PointICloud* point_cloud) {
  if (ratio_of_points_to_keep != 1.0) {
    CHECK_NOTNULL(point_cloud);
    PointICloud filtered_cloud;
    // Manual filtering as pcl::RandomSample seems to be broken.
    for (size_t i = 0u; i < point_cloud->size(); ++i) {
      if (double(std::rand()) / double(RAND_MAX) < ratio_of_points_to_keep) {
        filtered_cloud.points.push_back(point_cloud->points[i]);
    }

    filtered_cloud.width = 1;
    filtered_cloud.height = filtered_cloud.points.size();
    LOG(INFO) << "Filtering cloud from " << point_cloud->points.size() <<
        " to " << filtered_cloud.points.size() << " points.";
    *point_cloud = filtered_cloud;
  }
}

} // namespace laser_slam

#endif /* LASER_SLAM_COMMON_HPP_ */
