#pragma once
#include <cmath>
#include <string>

namespace agi::hardware {
enum class Mode { Boot, SensorCheck, ReadyManual, AutoStandby, AutoActive,
                  ManualFallback };

// All times are CLOCK_MONOTONIC seconds, stamped at acquisition, not receipt.
// Unknown evidence is false. A driver must not equate AHRS initialization with
// convergence or a GPS arrival timestamp with PPS synchronization.
struct Evidence {
  double now{NAN}, imu_time{NAN}, rtk_time{NAN}, rc_time{NAN};
  double command_time{NAN}, solve_seconds{NAN};
  bool rtk_fixed{false}, heading_valid{false}, accuracy_ok{false};
  bool imu_calibrated{false}, synchronized{false}, converged{false};
  bool config_verified{false}, thrust_calibrated{false}, geofence_ok{false};
  bool msp_healthy{false}, command_valid{false}, controller_warm{false};
  bool armed{false}, auto_switch{false}, kill{true}, rc_link{false};
};

class SafetyGate {
 public:
  static bool inputsHealthy(const Evidence& e) {
    return fresh(e.now, e.imu_time, 0.010) &&
      fresh(e.now, e.rtk_time, 0.300) && fresh(e.now, e.rc_time, 0.10) &&
      e.rc_link && e.rtk_fixed && e.heading_valid && e.accuracy_ok &&
      e.imu_calibrated && e.synchronized && e.converged &&
      e.config_verified && e.thrust_calibrated && e.geofence_ok && e.msp_healthy;
  }
  bool update(const Evidence& e) {
    const bool fresh_rc = fresh(e.now, e.rc_time, 0.10) && e.rc_link;
    const bool healthy = inputsHealthy(e) && e.controller_warm;
    const bool command_ok = fresh(e.now, e.command_time, 0.025) &&
      e.command_valid && std::isfinite(e.solve_seconds) &&
      e.solve_seconds >= 0 && e.solve_seconds <= 0.008;
    // Require a *healthy observed low* after boot or any fault. A switch held
    // high through process restart or a sensor outage cannot re-enter AUTO.
    if (fresh_rc && !e.auto_switch && !e.kill && healthy) low_seen_ = true;
    if (!fresh_rc || e.kill || !healthy || !command_ok) {
      low_seen_ = false;
      mode_ = mode_ == Mode::AutoActive || mode_ == Mode::ManualFallback
                ? Mode::ManualFallback : Mode::SensorCheck;
      reason_ = !healthy ? "health/configuration" : "command/authority";
      return false;
    }
    if (!e.auto_switch) {
      mode_ = e.armed ? Mode::AutoStandby : Mode::ReadyManual;
      reason_ = "physical AUTO is low";
      return false;
    }
    if (!e.armed) {
      low_seen_ = false;
      mode_ = Mode::ReadyManual;
      reason_ = "physical ARM is low";
      return false;
    }
    if (mode_ != Mode::AutoActive && !low_seen_) {
      reason_ = "cycle physical AUTO low then high";
      return false;
    }
    low_seen_ = false;
    mode_ = Mode::AutoActive;
    reason_ = "authorized";
    return true;
  }
  Mode mode() const { return mode_; }
  const std::string& reason() const { return reason_; }
  static const char* name(Mode mode) {
    switch (mode) {
      case Mode::Boot: return "BOOT";
      case Mode::SensorCheck: return "SENSOR_CHECK";
      case Mode::ReadyManual: return "READY_MANUAL";
      case Mode::AutoStandby: return "AUTO_STANDBY";
      case Mode::AutoActive: return "AUTO_ACTIVE";
      case Mode::ManualFallback: return "MANUAL_FALLBACK";
    }
    return "UNKNOWN";
  }
  static bool fresh(double now, double sample, double limit) {
    return std::isfinite(now) && std::isfinite(sample) &&
           now >= sample && now - sample <= limit;
  }
 private:
  Mode mode_{Mode::Boot};
  bool low_seen_{false};
  std::string reason_{"boot"};
};
}
