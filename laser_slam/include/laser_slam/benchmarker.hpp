#ifndef LASER_SLAM_BENCHMARKER_HPP_
#define LASER_SLAM_BENCHMARKER_HPP_

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace laser_slam {

// In order to use the benchmarker define the following macro in your project.
// #define BENCHMARK_ENABLE

#ifdef BENCHMARK_ENABLE
/// \brief Measure the time elapsed from this line to the end of the scope and record it to the
/// \c topic_name topic.
#define BENCHMARK_BLOCK(topic_name) laser_slam::ScopedTimer __timer_ ## __LINE__(topic_name);
/// \brief Starts a measurement for the \c topic_name topic.
#define BENCHMARK_START(topic_name) laser_slam::Benchmarker::startMeasurement(topic_name);
/// \brief Stops a measurement for the \c topic_name topic.
#define BENCHMARK_STOP(topic_name) laser_slam::Benchmarker::stopMeasurement(topic_name);
/// \brief Records a value in the \c topic_name topic. Only values of type \c double are supported.
#define BENCHMARK_RECORD_VALUE(topic_name, value) \
    laser_slam::Benchmarker::addValue(topic_name, value);
/// \brief Reset the topics with name starting with \c topic_prefix.
#define BENCHMARK_RESET(topic_prefix) laser_slam::Benchmarker::resetTopic(topic_prefix);
/// \brief Reset all topics.
#define BENCHMARK_RESET_ALL() laser_slam::Benchmarker::resetTopic("");
#else
#define BENCHMARK_BLOCK(topic_name)
#define BENCHMARK_START(topic_name)
#define BENCHMARK_STOP(topic_name)
#define BENCHMARK_RECORD_VALUE(topic_name, value)
#define BENCHMARK_RESET(topic_prefix)
#define BENCHMARK_RESET_ALL()
#endif

struct BenchmarkerParams {
  bool save_statistics_only;
  bool enable_live_output;
  std::string results_directory;
};

/// \brief Benchmark helper class. Allows collecting data and statistics about execution times and
/// metrics (values).
/// \remark This class is thread-safe, but simultaneously collecting measurements for the same
/// topic from multiple threads is not supported.
class Benchmarker {

 public:
  /// \brief The clock used by the benchmarker.
  typedef std::chrono::high_resolution_clock Clock;
  typedef Clock::time_point TimePoint;
  typedef Clock::duration Duration;

  /// \brief Prevent static class from being instantiated.
  Benchmarker() = delete;

  /// \brief Starts a measurement for the \c topic_name topic.
  static void startMeasurement(const std::string& topic_name);

  /// \brief Stops a measurement for the \c topic_name topic.
  static void stopMeasurement(const std::string& topic_name);

  /// \brief Add a measurement for the \c topic_name topic.
  static void addMeasurement(const std::string& topic_name, const TimePoint& start,
                             const TimePoint& end);

  /// \brief Add a value for the \c topic_name topic.
  static void addValue(const std::string& topic_name, double value);

  /// \brief Reset all collected data for all the topics with the given prefix. If \e topic_prefix
  /// is an empty string, all data will be reset.
  static void resetTopic(const std::string& topic_prefix);

  /// \brief Save the recorded data for all the topics.
  static void saveData();

  /// \brief Print the statistics to the logger.
  static void logStatistics();

  /// \brief Set the parameters of the benchmarker.
  /// \param parameters The new parameters for the benchmarker.
  static inline void setParameters(const BenchmarkerParams& parameters) {
    params_ = parameters;
  }

  /// \brief Get the parameters of the benchmarker.
  /// \return The parameters of the benchmarker.
  static inline const BenchmarkerParams& getParameters() {
    return params_;
  }

 private:
  /// \brief Helper class for collecting data and statistics about values.
  class ValueTopic {

   public:
    /// \brief Adds a value.
    void addValue(const TimePoint& timestamp, double value);

    /// \brief Computes the mean of the measurements.
    double getMean() const;

    /// \brief Computes the standard deviation of the values.
    double getStandardDeviation() const;

    /// \brief Gets a reference to the stored data.
    inline const std::vector<std::pair<TimePoint, double>>& getValues() const {
      return values_;
    }

   private:
    /// \brief Sum of the values, needed for computing mean and standard deviations.
    double sum_ = 0.0;

    /// \brief Sum of squares of the values, needed for computing the standard deviation.
    double sum_of_squares_ = 0.0;

    /// \brief The number of values recorded.
    uint64_t values_count_ = 0;

    /// \brief The values_ collected.
    std::vector<std::pair<TimePoint, double>> values_;
  };

  // Helper functions.
  static std::string setupAndGetResultsRootDirectory();
  static TimePoint getFirstValueTimepoint();
  static double durationToMilliseconds(const Duration& duration);

  // Mutexes for synchronizing accesses to the benchmarker.
  static std::mutex value_topics_mutex_;
  static std::mutex started_measurements_mutex_;

  /// \brief Map containing the values sorted in alphabetical order.
  static std::map<std::string, ValueTopic> value_topics_;

  /// \brief Starting time points of the measurements currently in progress.
  static std::unordered_map<std::string, TimePoint> started_mesurements_;

  /// \brief Parameters of the benchmarker.
  static BenchmarkerParams params_;
};

/// \brief A timer that measures the time elapsed between its creation and destruction. This is
/// useful for timing whole functions or any scoped block of code. The measurement is automatically
/// committed to the benchmarker.
class ScopedTimer {

 public:
  /// \brief Default constructor. Starts the timer.
  ScopedTimer(const std::string& topic_name)
   : topic_name_(topic_name)
   , start_(Benchmarker::Clock::now()) {
  }

  /// \brief Destructor. Stops the timer and commits the result to the benchmarker.
  ~ScopedTimer();

 private:
  /// \brief Name of the timed block.
  const std::string topic_name_;

  /// \brief The time point when the timer was started.
  const std::chrono::time_point<Benchmarker::Clock> start_;
};

} // namespace laser_slam

#endif /* LASER_SLAM_BENCHMARKER_HPP_ */
