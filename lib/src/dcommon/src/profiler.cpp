#include "dcommon/profiler.hpp"
#include <iomanip>

namespace dcommon {

Profiler::Profiler(const std::string& name,
                   const std::vector<Profiler>& src_profilers)
    : name_(name) {
  for (const auto& src_profiler : src_profilers) {
    for (const auto& rec : src_profiler.profile_) {
      auto it = profile_.find(rec.first);

      if (it == profile_.end()) {
        profile_.insert(rec);
      } else {
        it->second.time += rec.second.time;
        it->second.count += rec.second.count;
      }
    }
  }
}

void Profiler::RecordModuleTime(const std::string& module_name, float ms) {
  profile_[module_name].count++;
  profile_[module_name].time += ms;
}

std::ostream& operator<<(std::ostream& out, const Profiler& value) {
  out << "========== " << value.name_ << " profile ==========" << std::endl;
  float totalTime = 0;
  std::string layerNameStr = "Operation";

  int maxLayerNameLength = static_cast<int>(layerNameStr.size());
  for (const auto& elem : value.profile_) {
    totalTime += elem.second.time;
    maxLayerNameLength =
        std::max(maxLayerNameLength, static_cast<int>(elem.first.size()));
  }

  auto old_settings = out.flags();
  auto old_precision = out.precision();

  // print header
  {
    out << std::left;
    out << std::setw(maxLayerNameLength) << layerNameStr << " ";
    out << std::right;
    out << std::setw(12) << "Runtime, "
        << "%"
        << " ";
    out << std::setw(12) << "Invocations"
        << " ";
    out << std::setw(12) << "Runtime, ms"
        << " ";
    out << std::setw(16) << "Avg Runtime, ms" << std::endl;
  }

  // print content
  for (const auto& elem : value.profile_) {
    out << std::left;
    out << std::setw(maxLayerNameLength) << elem.first << " ";
    out << std::right;
    out << std::setw(12) << std::fixed << std::setprecision(1)
        << (elem.second.time * 100.0F / totalTime) << "%"
        << " ";
    out << std::setw(12) << elem.second.count << " ";
    out << std::setw(12) << std::fixed << std::setprecision(2)
        << elem.second.time << " ";
    out << std::setw(12) << std::fixed << std::setprecision(2)
        << elem.second.time / elem.second.count << std::endl;
  }
  out.flags(old_settings);
  out.precision(old_precision);

  // print footer
  out << "========== " << value.name_ << " total runtime = " << totalTime
      << " ms ==========" << std::endl;

  return out;
}

}  // namespace dcommon
