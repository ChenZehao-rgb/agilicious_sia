#pragma once
#include <cmath>
#include <stdexcept>
#include <string>

namespace agi::hardware {
enum class Mode { Boot, SensorCheck, ReadyManual, AutoStandby, AutoActive, ManualFallback };
enum class NavigationPolicy { Rtk, Gnss };
enum class ReceiverPolicy { FreshSamples, LatchedSwitches };

// Times share CLOCK_MONOTONIC on hardware, or the explicitly selected SITL clock.
// Unknown evidence is false. GNSS clock alignment is not RTK/PPS synchronization.
struct Evidence {
	bool timing_checks{true};  // Local policy; never taken from a received command.
	double now{NAN}, imu_time{NAN}, rtk_time{NAN}, rc_time{NAN};
	double command_time{NAN}, solve_seconds{NAN};
	// Original source sample/receipt times, distinct from accepted rtk_time.
	double navigation_sample_time{NAN}, navigation_receive_time{NAN};
	bool rtk_fixed{false}, heading_valid{false}, accuracy_ok{false};
	bool imu_calibrated{false}, synchronized{false}, converged{false};
	bool imu_ready{false}, estimator_ready{false}, navigation_ready{false};
	bool clock_aligned{false}, accuracy_known{false};
	bool config_verified{false}, thrust_calibrated{false}, geofence_ok{false};
	bool thrust_mapping_ready{false};  // A usable mapping may still be an uncalibrated estimate.
	bool msp_healthy{false}, command_valid{false}, controller_warm{false};
	bool receiver_valid{false};
	bool armed{false}, auto_switch{false}, kill{true}, rc_link{false};
};

class SafetyGate {
public:
	static constexpr double kNavigationSourceMaxAge = 0.300;
	static constexpr double kStateMaxAge = 0.015;
	// Hardware rechecks a command between control ticks and before UART writes.
	// Its source IMU must outlive the 25 ms command lease plus state/solve delay.
	static constexpr double kHardwareImuMaxAge = 0.050;
	static double acceptedNavigationMaxAge(double observation_delay) {
		if (!std::isfinite(observation_delay) || observation_delay < 0 || observation_delay > 0.250) return NAN;
		return kNavigationSourceMaxAge + observation_delay;
	}
	explicit SafetyGate(NavigationPolicy policy = NavigationPolicy::Rtk, double observation_delay = 0.0,
	                    ReceiverPolicy receiver_policy = ReceiverPolicy::FreshSamples)
	        : _policy(policy), _observation_delay(observation_delay), _receiver_policy(receiver_policy) {
		if (!std::isfinite(acceptedNavigationMaxAge(observation_delay)))
			throw std::invalid_argument("Invalid navigation observation delay");
	}
	static const char* inputFailure(const Evidence& e, NavigationPolicy policy = NavigationPolicy::Rtk, double observation_delay = 0.0,
	                                ReceiverPolicy receiver_policy = ReceiverPolicy::FreshSamples) {
		const double imu_max_age = receiver_policy == ReceiverPolicy::LatchedSwitches ? kHardwareImuMaxAge : .010;
		if (!fresh(e.now, e.imu_time, imu_max_age, e.timing_checks)) return "IMU stale/future";
		const double accepted_max_age = acceptedNavigationMaxAge(observation_delay);
		if (!std::isfinite(accepted_max_age)) return "navigation timing policy invalid";
		if (policy == NavigationPolicy::Gnss || observation_delay > 0) {
			if (!fresh(e.now, e.navigation_sample_time, kNavigationSourceMaxAge, e.timing_checks))
				return "navigation source sample stale/future";
			if (!fresh(e.now, e.navigation_receive_time, kNavigationSourceMaxAge, e.timing_checks))
				return "navigation source receive stale/future";
		}
		if (!fresh(e.now, e.rtk_time, accepted_max_age, e.timing_checks)) return "navigation stale/future";
		if (receiver_policy == ReceiverPolicy::LatchedSwitches) {
			if (!e.receiver_valid) return "receiver mode unavailable";
			if (!e.rc_link) return "receiver reports RX failsafe";
		} else if (!fresh(e.now, e.rc_time, .100, e.timing_checks) || !e.rc_link) {
			return "RC timeout/future/link unavailable";
		}
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
		if (!e.thrust_mapping_ready) return "thrust mapping unavailable";
		if (!e.geofence_ok) return "geofence rejected";
		if (!e.msp_healthy) return "output transport unavailable";
		return nullptr;
	}
	static bool inputsHealthy(const Evidence& e, NavigationPolicy policy = NavigationPolicy::Rtk, double observation_delay = 0.0,
	                          ReceiverPolicy receiver_policy = ReceiverPolicy::FreshSamples) {
		return inputFailure(e, policy, observation_delay, receiver_policy) == nullptr;
	}
	bool canEnterAuto(const Evidence& e) const {
		return _low_seen && e.armed && e.auto_switch && !e.kill && e.controller_warm &&
		       inputsHealthy(e, _policy, _observation_delay, _receiver_policy);
	}
	bool update(const Evidence& e) {
		const char* failure = inputFailure(e, _policy, _observation_delay, _receiver_policy);
		const bool command_ok = fresh(e.now, e.command_time, .025, e.timing_checks) && e.command_valid &&
		                        std::isfinite(e.solve_seconds) && e.solve_seconds >= 0 &&
		                        (!e.timing_checks || e.solve_seconds <= .008);
		if (failure || e.kill || !e.controller_warm || !command_ok) {
			_low_seen = false;
			_mode = _mode == Mode::AutoActive || _mode == Mode::ManualFallback ? Mode::ManualFallback : Mode::SensorCheck;
			_reason = failure              ? failure
			          : e.kill             ? "receiver KILL"
			          : !e.controller_warm ? "controller warming"
			                               : "command invalid/expired";
			return false;
		}
		if (_receiver_policy == ReceiverPolicy::LatchedSwitches && !e.armed) {
			_low_seen = false;
			_mode = Mode::ReadyManual;
			_reason = "physical ARM is low";
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
	const double _observation_delay;
	const ReceiverPolicy _receiver_policy;
	Mode _mode{Mode::Boot};
	bool _low_seen{false};
	std::string _reason{"boot"};
};
}  // namespace agi::hardware
