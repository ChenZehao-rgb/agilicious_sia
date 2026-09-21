#pragma once
#include <cmath>
#include <string>

namespace agi::hardware {
enum class Mode { Boot, SensorCheck, ReadyManual, AutoStandby, AutoActive, ManualFallback };
enum class NavigationPolicy { Rtk, Gnss };

// Times share CLOCK_MONOTONIC on hardware, or the explicitly selected SITL clock.
// Unknown evidence is false. GNSS clock alignment is not RTK/PPS synchronization.
struct Evidence {
	bool timing_checks{true};  // Local policy; never taken from a received command.
	double now{NAN}, imu_time{NAN}, rtk_time{NAN}, rc_time{NAN};
	double command_time{NAN}, solve_seconds{NAN};
	bool rtk_fixed{false}, heading_valid{false}, accuracy_ok{false};
	bool imu_calibrated{false}, synchronized{false}, converged{false};
	bool imu_ready{false}, estimator_ready{false}, navigation_ready{false};
	bool clock_aligned{false}, accuracy_known{false};
	bool config_verified{false}, thrust_calibrated{false}, geofence_ok{false};
	bool msp_healthy{false}, command_valid{false}, controller_warm{false};
	bool armed{false}, auto_switch{false}, kill{true}, rc_link{false};
};

class SafetyGate {
public:
	explicit SafetyGate(NavigationPolicy policy = NavigationPolicy::Rtk) : _policy(policy) {}
	static const char* inputFailure(const Evidence& e, NavigationPolicy policy = NavigationPolicy::Rtk) {
		if (!fresh(e.now, e.imu_time, .010, e.timing_checks)) return "IMU stale/future";
		if (!fresh(e.now, e.rtk_time, .300, e.timing_checks)) return "navigation stale/future";
		if (!fresh(e.now, e.rc_time, .100, e.timing_checks) || !e.rc_link) return "RC timeout/future/link unavailable";
		if (!e.heading_valid) return "heading unavailable";
		if (!e.accuracy_ok) return "navigation accuracy rejected";
		if (policy == NavigationPolicy::Gnss) {
			if (!e.clock_aligned) return "GNSS clock not aligned";
			if (!e.accuracy_known) return "GNSS accuracy unknown";
			if (!e.navigation_ready) return "GNSS not ready";
			if (!e.imu_ready) return "IMU not ready";
			if (!e.estimator_ready) return "estimator not ready";
		} else {
			if (!e.rtk_fixed || !e.synchronized) return "RTK fix/PPS unavailable";
			if (!e.imu_calibrated || !e.converged) return "IMU calibration/estimator convergence unavailable";
		}
		if (!e.config_verified) return "flight-controller configuration rejected";
		if (!e.thrust_calibrated) return "thrust calibration unavailable";
		if (!e.geofence_ok) return "geofence rejected";
		if (!e.msp_healthy) return "output transport unavailable";
		return nullptr;
	}
	static bool inputsHealthy(const Evidence& e, NavigationPolicy policy = NavigationPolicy::Rtk) {
		return inputFailure(e, policy) == nullptr;
	}
	bool canEnterAuto(const Evidence& e) const {
		return _low_seen && e.armed && e.auto_switch && !e.kill && e.controller_warm && inputsHealthy(e, _policy);
	}
	bool update(const Evidence& e) {
		const char* failure = inputFailure(e, _policy);
		const bool command_ok = fresh(e.now, e.command_time, .025, e.timing_checks) && e.command_valid &&
		                        std::isfinite(e.solve_seconds) && e.solve_seconds >= 0 &&
		                        (!e.timing_checks || e.solve_seconds <= .008);
		if (failure || e.kill || !e.controller_warm || !command_ok) {
			_low_seen = false;
			_mode = _mode == Mode::AutoActive || _mode == Mode::ManualFallback ? Mode::ManualFallback : Mode::SensorCheck;
			_reason = failure              ? failure
			          : e.kill             ? "receiver KILL"
			          : !e.controller_warm ? "MPC warming"
			                               : "command invalid/expired";
			return false;
		}
		if (!e.auto_switch) {
			_low_seen = true;
			_mode = e.armed ? Mode::AutoStandby : Mode::ReadyManual;
			_reason = "physical AUTO is low";
			return false;
		}
		if (!e.armed) {
			_low_seen = false;
			_mode = Mode::ReadyManual;
			_reason = "physical ARM is low";
			return false;
		}
		if (_mode != Mode::AutoActive && !_low_seen) {
			_reason = "cycle physical AUTO low then high";
			return false;
		}
		_low_seen = false;
		_mode = Mode::AutoActive;
		_reason = "authorized";
		return true;
	}
	Mode mode() const { return _mode; }
	const std::string& reason() const { return _reason; }
	static const char* name(Mode mode) {
		switch (mode) {
			case Mode::Boot:
				return "BOOT";
			case Mode::SensorCheck:
				return "SENSOR_CHECK";
			case Mode::ReadyManual:
				return "READY_MANUAL";
			case Mode::AutoStandby:
				return "AUTO_STANDBY";
			case Mode::AutoActive:
				return "AUTO_ACTIVE";
			case Mode::ManualFallback:
				return "MANUAL_FALLBACK";
		}
		return "UNKNOWN";
	}
	static bool fresh(double now, double sample, double limit, bool timing_checks = true) {
		return std::isfinite(now) && std::isfinite(sample) && (!timing_checks || (now >= sample && now - sample <= limit));
	}

private:
	const NavigationPolicy _policy;
	Mode _mode{Mode::Boot};
	bool _low_seen{false};
	std::string _reason{"boot"};
};
}  // namespace agi::hardware
