#ifndef LASER_SLAM_COMMON_HPP_
#define LASER_SLAM_COMMON_HPP_

#include <fstream>

#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <mincurves/DiscreteSE3Curve.hpp>
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

static void toEigenMatrixXd(std::map<Time, double> map_in, Eigen::MatrixXd* matrix_out) {
  CHECK_NOTNULL(matrix_out);
  matrix_out->resize(map_in.size(),2);

  unsigned int i = 0u;
  for (const auto& elem: map_in) {
    (*matrix_out)(i,0) = elem.first;
    (*matrix_out)(i,1) = elem.second;
    ++i;
  }
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

static double distanceBetweenTwoSE3(const SE3& pose1, const SE3& pose2) {
  return std::sqrt(
      (pose1.getPosition()(0) - pose2.getPosition()(0)) *
      (pose1.getPosition()(0) - pose2.getPosition()(0)) +
      (pose1.getPosition()(1) - pose2.getPosition()(1)) *
      (pose1.getPosition()(1) - pose2.getPosition()(1)) +
      (pose1.getPosition()(2) - pose2.getPosition()(2)) *
      (pose1.getPosition()(2) - pose2.getPosition()(2)));
}

static void getMeanAndSigma(std::vector<double> values, double* mean_out,
                            double* sigma_out) {
  CHECK_NOTNULL(mean_out);
  CHECK_NOTNULL(sigma_out);
  //LOG(INFO) << "values";
  double sum = 0.0;
  const double n = double(values.size());
  for (const auto& value: values) {
    //LOG(INFO) << "value " << value;
    sum += value;
  }
  const double mean = sum / n;

  double sum_of_squared_diff = 0.0;
  for (const auto& value: values) {
    sum_of_squared_diff += (mean - value) * (mean - value);
  }
  *mean_out = mean;
  *sigma_out = std::sqrt(sum_of_squared_diff / n);
}

} // namespace laser_slam

#endif /* LASER_SLAM_COMMON_HPP_ */
