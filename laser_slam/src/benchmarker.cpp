#include "laser_slam/benchmarker.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <glog/logging.h>

namespace laser_slam {

#define ALIGN_MODIFIERS std::setw(40) << std::setfill(' ') << std::left
#define FLOAT_MODIFIERS std::fixed << std::setprecision(2)

void Benchmarker::addMeasurement(
    const std::string& name,
    const std::chrono::time_point<Clock>& start,
    const std::chrono::time_point<Clock>& end) {

  constexpr double mus_to_ms = 1.0 / 1000.0;
  auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  double milliseconds = static_cast<double>(microseconds) * mus_to_ms;

  #ifdef BENCHMARK_ENABLE_LIVE_OUTPUT
  LOG(INFO) << name << " took " << FLOAT_MODIFIERS << milliseconds << "ms.";
  #endif

  std::lock_guard<std::mutex> lock(mutex_);
  statistics_[name].addMeasurement(milliseconds);
}

void Benchmarker::saveStatistics(const std::string& file_name) {

  std::ofstream out_file;
  out_file.open(file_name);

  if (out_file.is_open()) {

    out_file << "Benchmark results:" << std::endl;
    out_file << " " << std::endl
        << ALIGN_MODIFIERS << "Name: " << "Mean (SD)" << std::endl;

    for (const auto& code_block : statistics_) {
      out_file << " "
          << ALIGN_MODIFIERS << (code_block.first + ": ")
          << FLOAT_MODIFIERS << code_block.second.getMean() << "ms ("
          << FLOAT_MODIFIERS << code_block.second.getStandardDeviation()
          << "ms)"  << std::endl;
    }

    out_file.close();
    LOG(INFO) << "Benchmark results saved to " << file_name;
  }
  else {
    LOG(INFO) << "Failed to save benchmark results.";
  }
}

void Benchmarker::logStatistics() {

  LOG(INFO) << "";
  LOG(INFO) << "Benchmark results:";
  LOG(INFO) << " "
      << ALIGN_MODIFIERS << "Name: " << "Mean (SD)";

  for (const auto& code_block : statistics_) {
    LOG(INFO) << " "
        << ALIGN_MODIFIERS << (code_block.first + ": ")
        << FLOAT_MODIFIERS << code_block.second.getMean() << "ms ("
        << FLOAT_MODIFIERS << code_block.second.getStandardDeviation() << "ms)";
  }

  LOG(INFO) << "";
}

Benchmarker::Benchmarker() { }

std::mutex Benchmarker::mutex_;
std::map<std::string, Benchmarker::MeasurementStatistics_> Benchmarker::statistics_;

void Benchmarker::MeasurementStatistics_::addMeasurement(const double& value) {
  sum_ += value;
  sum_of_squares_ += value * value;
  measurements_count_++;
}

double Benchmarker::MeasurementStatistics_::getMean() const {
  return sum_ / static_cast<double>(measurements_count_);
}

double Benchmarker::MeasurementStatistics_::getStandardDeviation() const {
  return sqrt(
      sum_of_squares_ / static_cast<double>(measurements_count_) -
      pow(sum_ / static_cast<double>(measurements_count_), 2.0));
}

ScopedTimer::ScopedTimer(const std::string& name)
  : name_(name)
  , start_(Benchmarker::Clock::now()) { }

ScopedTimer::~ScopedTimer() {
  Benchmarker::addMeasurement(name_, start_, Benchmarker::Clock::now());
}

}
