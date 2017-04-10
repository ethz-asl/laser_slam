#ifndef LASER_SLAM_BENCHMARKER_HPP_
#define LASER_SLAM_BENCHMARKER_HPP_

#include <chrono>
#include <mutex>
#include <string>
#include <map>
#include <vector>

namespace laser_slam {

/// TODO(Mattia): Move this to the preprocessor definitions:
#define BENCHMARK_ENABLE
#define BENCHMARK_ENABLE_LIVE_OUTPUT

#ifdef BENCHMARK_ENABLE
/// \brief Measure the time elapsed from this line to the end of the scope.
#define BENCHMARK_BLOCK(unique_name) \
  laser_slam::ScopedTimer unique_name(#unique_name);
/// \brief Start a timer called unique_name.
#define BENCHMARK_START(unique_name) \
  auto __start ## unique_name = laser_slam::Benchmarker::Clock::now();
/// \brief Stop the timer called unique_name and commit the results
/// to the benchmarker.
#define BENCHMARK_STOP(unique_name) \
  laser_slam::Benchmarker::addMeasurement( \
      #unique_name, \
      __start ## unique_name, \
      laser_slam::Benchmarker::Clock::now());
#else
#define BENCHMARK_BLOCK(unique_name)
#define BENCHMARK_START(unique_name)
#define BENCHMARK_STOP(unique_name)
#endif

/// \brief Benchmark helper class. Allows collecting execution times
/// of parts of the code and computing their statistics.
/// \note This class is thread-safe.
class Benchmarker {

 public:
  /// \brief The clock used for timing operations
  typedef std::chrono::high_resolution_clock Clock;

  /// \brief Add a time measurement for a region of code with a specific name.
  static void addMeasurement(
      const std::string& name,
      const std::chrono::time_point<Clock>& start,
      const std::chrono::time_point<Clock>& end);

  /// \brief Print the statistics to the specified file.
  static void saveStatistics(const std::string& file_name);

  /// \brief Print the statistics to the logger.
  static void logStatistics();

 private:
  /// \brief Prevent static class from being instantiated.
  Benchmarker();

  /// \brief Helper class for collecting statistics over a series of measurements.
  class MeasurementStatistics_ {

   public:
    /// \brief Add a measurement.
    void addMeasurement(const double& value);

    /// \brief Compute the mean of the measurements.
    double getMean() const;

    /// \brief Compute the standard deviation of the measurements.
    double getStandardDeviation() const;

   private:
    /// \brief Sum of the measurements, needed for computing mean and standard deviations.
    double sum_ { 0.0 };

    /// \brief Sum of squares of the measurements, needed for computing the standard deviation.
    double sum_of_squares_ { 0.0 };

    /// \brief The number of measurements recorded.
    uint64_t measurements_count_ { 0 };
  };

  /// \brief Mutex for synchronizing accesses to the benchmarker.
  static std::mutex mutex_;

  /// \brief Map containing all the measurements for each named region of code.
  /// Measurements are sorted in alphabetical order.
  static std::map<std::string, MeasurementStatistics_> statistics_;
};

/// \brief A timer that measures the time elapsed between its creation
/// and destruction. This is useful for timing whole functions or any
/// scoped block of code. The measurement is automatically committed to
/// the benchmarker.
class ScopedTimer {

 public:

  /// \brief Default constructor. Starts the timer.
  ScopedTimer(const std::string& name);

  /// \brief Destructor. Stops the timer and commits the result to the
  /// benchmarker.
  ~ScopedTimer();

 private:
  /// \brief The time point when the timer was started.
  std::chrono::time_point<Benchmarker::Clock> start_;

  /// \brief Name of the timed block.
  std::string name_;
};

} // namespace laser_slam

#endif /* LASER_SLAM_BENCHMARKER_HPP_ */
