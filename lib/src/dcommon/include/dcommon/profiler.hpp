#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "dcommon/timer.hpp"

namespace dcommon {

struct Record {
  float time{0};
  int count{0};
};

class Profiler {
 public:
  /**
   * @brief Construct a profiler
   * @param name Name of the profiler
   * @param src_profilers Optionally initialize profiler with other profilers
   */
  explicit Profiler(
      const std::string& name,
      const std::vector<Profiler>& src_profilers = std::vector<Profiler>());

  /**
   * @brief Record the execution time of specific module
   * @param module_name name of module
   * @param ms execution time in millisecond
   */
  void RecordModuleTime(const std::string& module_name, float ms);
  friend std::ostream& operator<<(std::ostream& out, const Profiler& value);

  inline void StartTimer() { timer_.restart(); }
  inline void LogAndRestartTimer(const std::string& module_name) {
    RecordModuleTime(module_name, timer_.elapsedMsec());
  }
  inline void LogElapsedTime(const std::string& module_name) {
    RecordModuleTime(module_name, timer_.getElapsedMsec());
  }

 private:
  std::string name_;
  std::map<std::string, Record> profile_;
  Timer timer_;
};

}  // namespace dcommon
