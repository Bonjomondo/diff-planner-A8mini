#pragma once
#include <vector>
#include <cmath>
#include <stdexcept>

// Bounds describe the allowed vehicle-center volume, already inset from nets.
struct MissionBox {
  bool enabled = false;
  std::vector<double> bounds;
  void configure(bool active, const std::vector<double>& values) {
    if (active) {
      if (values.size() != 6) throw std::runtime_error("indoor mission fence requires six bounds");
      for (int i = 0; i < 6; ++i)
        if (!std::isfinite(values[i])) throw std::runtime_error("non-finite indoor fence");
      for (int i = 0; i < 3; ++i)
        if (values[i] >= values[i+3]) throw std::runtime_error("invalid indoor fence bounds");
    }
    enabled = active; bounds = active ? values : std::vector<double>();
  }
  bool excludes(double x, double y, double z) const {
    if (!enabled) return false;
    const double point[] = {x,y,z};
    for (int i = 0; i < 3; ++i)
      if (!std::isfinite(point[i]) || point[i] <= bounds[i] || point[i] >= bounds[i+3]) return true;
    return false;
  }
};
