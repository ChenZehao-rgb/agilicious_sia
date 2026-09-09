#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace agi::hardware {
// Measured TOTAL vehicle thrust in N on a rectangular [voltage][PWM] grid.
// Caller loads audited calibration data; no guessed hover throttle/defaults.
class ThrustTable {
 public:
  ThrustTable(std::vector<double> volts, std::vector<double> pwm,
              std::vector<std::vector<double>> newtons)
    : volts_(std::move(volts)), pwm_(std::move(pwm)), force_(std::move(newtons)) {
    const auto increasing = [](const std::vector<double>& values) {
      if (values.size() < 2) return false;
      for (size_t i = 0; i < values.size(); ++i)
        if (!std::isfinite(values[i]) || (i && values[i] <= values[i - 1]))
          return false;
      return true;
    };
    if (!increasing(volts_) || !increasing(pwm_) || volts_.front() <= 0 ||
        pwm_.front() < 1000 || pwm_.back() > 2000 || force_.size() != volts_.size())
      throw std::invalid_argument("invalid thrust calibration grid");
    for (const auto& row : force_)
      if (row.size() != pwm_.size() || !increasing(row) || row.front() < 0)
        throw std::invalid_argument("thrust must be finite and strictly increasing");
  }
  // Acceleration units match Command::collective_thrust (m/s^2).
  // Out-of-calibration commands fail; never extrapolate or silently saturate.
  uint16_t collectiveThrustToRc(double acceleration, double mass, double voltage) const {
    if (!std::isfinite(acceleration) || !std::isfinite(mass) ||
        !std::isfinite(voltage) || acceleration < 0 || mass <= 0 ||
        voltage < volts_.front() || voltage > volts_.back())
      throw std::out_of_range("invalid thrust/mass or uncalibrated battery voltage");
    size_t high = std::upper_bound(volts_.begin(), volts_.end(), voltage) - volts_.begin();
    high = std::min(high, volts_.size() - 1);
    const size_t low = high - 1;
    const double mix = (voltage - volts_[low]) / (volts_[high] - volts_[low]);
    const auto force = [&](size_t i) {
      return force_[low][i] * (1 - mix) + force_[high][i] * mix;
    };
    const double target = mass * acceleration;
    if (!std::isfinite(target) || target < force(0) || target > force(pwm_.size() - 1))
      throw std::out_of_range("requested thrust outside measured envelope");
    for (size_t i = 1; i < pwm_.size(); ++i) {
      if (target <= force(i)) {
        const double x = (target - force(i - 1)) / (force(i) - force(i - 1));
        return static_cast<uint16_t>(std::lround(pwm_[i - 1] + x * (pwm_[i] - pwm_[i - 1])));
      }
    }
    throw std::out_of_range("unreachable thrust table lookup");
  }
 private:
  std::vector<double> volts_, pwm_;
  std::vector<std::vector<double>> force_;
};
}
