#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace agi::hardware {
enum class Mode { Boot, SensorCheck, ReadyManual, AutoStandby, AutoActive, ManualFallback };
enum class NavigationPolicy { Rtk, Gnss, FcFused };
enum class ReceiverPolicy { FreshSamples, LatchedSwitches };
// Selected locally by the caller; never accepted from a received message.
enum class FcStateCheck { Input, Hold, CommandHold };

// Times share CLOCK_MONOTONIC on hardware, or the explicitly selected SITL clock.
// Unknown evidence is false. GNSS clock alignment is not RTK/PPS synchronization.
struct Evidence {
	// FC epochs remain independent of legacy IMU/GNSS/PPS evidence.
	bool fc_valid{false};
	double fc_sample_time{NAN}, fc_receive_time{NAN}, fc_publish_time{NAN};
	double fc_health_sample_time{NAN}, fc_health_receive_time{NAN};
	double fc_xy_observation_time{NAN}, fc_z_observation_time{NAN};
	uint16_t fc_flags{0};
	uint64_t fc_generation{0};
	uint32_t fc_reset_counter{0};
	std::string fc_session;
	std::array<double, 3> fc_position_variance{{NAN, NAN, NAN}}, fc_velocity_variance{{NAN, NAN, NAN}};
	bool timing_checks{true};  // Local policy; never taken from a received command.
	double now{NAN}, imu_time{NAN}, rtk_time{NAN}, rc_time{NAN};
	// Mapped FC update epoch, distinct from fusion callback receipt. The FC
	// snapshot does not by itself establish the sensor's physical exposure time.
	double imu_sample_time{NAN}, status_time{NAN};
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
	// Total source-sample age during normal command hold, not an extra lease.
	static constexpr double kFcHoldMaxAge = kStateMaxAge + 0.010;
	// Hardware rechecks a command between control ticks and before UART writes.
	// Its source IMU must outlive the 25 ms command lease plus state/solve delay.
	static constexpr double kHardwareImuMaxAge = 0.050;
	static constexpr double kReceiverEvidenceMaxAge = 0.100;
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
	                                ReceiverPolicy receiver_policy = ReceiverPolicy::FreshSamples,
	                                FcStateCheck fc_check = FcStateCheck::Input) {
		if (policy == NavigationPolicy::FcFused) {
			if (const char* failure = fcFailure(e, fc_check)) return failure;
		} else {
			const double imu_max_age = receiver_policy == ReceiverPolicy::LatchedSwitches ? kHardwareImuMaxAge : .010;
			if (!fresh(e.now, e.imu_time, imu_max_age, e.timing_checks)) return "IMU stale/future";
			if (!fresh(e.now, e.imu_sample_time, imu_max_age, e.timing_checks)) return "IMU source sample stale/future";
			const double accepted_max_age = acceptedNavigationMaxAge(observation_delay);
			if (!std::isfinite(accepted_max_age)) return "navigation timing policy invalid";
			if (policy == NavigationPolicy::Gnss || observation_delay > 0) {
				if (!fresh(e.now, e.navigation_sample_time, kNavigationSourceMaxAge, e.timing_checks))
					return "navigation source sample stale/future";
				if (!fresh(e.now, e.navigation_receive_time, kNavigationSourceMaxAge, e.timing_checks))
					return "navigation source receive stale/future";
			}
			if (!fresh(e.now, e.rtk_time, accepted_max_age, e.timing_checks)) return "navigation stale/future";
		}
		if (receiver_policy == ReceiverPolicy::LatchedSwitches) {
			if (!e.receiver_valid) return "receiver mode unavailable";
			if (!fresh(e.now, e.rc_time, kReceiverEvidenceMaxAge, e.timing_checks)) return "RC request stale/future";
			if (!fresh(e.now, e.status_time, kReceiverEvidenceMaxAge, e.timing_checks)) return "STATUS request stale/future";
			if (!e.rc_link) return "receiver reports RX failsafe";
		} else if (!fresh(e.now, e.rc_time, .100, e.timing_checks) || !e.rc_link) {
			return "RC timeout/future/link unavailable";
		}
		if (policy != NavigationPolicy::FcFused && !e.heading_valid) return "heading unavailable";
		if (!e.accuracy_ok) return "navigation accuracy rejected";
		if (policy == NavigationPolicy::Gnss) {
			if (!e.clock_aligned) return "GNSS clock not aligned";
			if (!e.accuracy_known) return "GNSS accuracy unknown";
			if (!e.navigation_ready) return "GNSS not ready";
			if (!e.imu_ready) return "IMU not ready";
			if (!e.estimator_ready) return "estimator not ready";
		} else if (policy == NavigationPolicy::Rtk) {
			if (!e.rtk_fixed || !e.synchronized) return "RTK fix/PPS unavailable";
			if (!e.imu_calibrated || !e.converged) return "IMU calibration/estimator convergence unavailable";
		}
		if (!e.config_verified) return "flight-controller configuration rejected";
		if (!e.thrust_mapping_ready) return "thrust mapping unavailable";
		if (!e.geofence_ok) return "geofence rejected";
		if (!e.msp_healthy) return "output transport unavailable";
		return nullptr;
	}
	static const char* fcFailure(const Evidence& e, FcStateCheck check = FcStateCheck::Input) {
		// Timing cannot be disabled for FC fused state, including in shadow mode.
		if (!e.fc_valid || e.fc_session.empty()) return "FC state invalid/session unavailable";
		if (!e.clock_aligned) return "FC clock not aligned";
		const double max_age = check == FcStateCheck::Input ? kStateMaxAge : kFcHoldMaxAge;
		if (!fresh(e.now, e.fc_sample_time, max_age)) return "FC state sample stale/future";
		if (!fresh(e.now, e.fc_receive_time, max_age)) return "FC state receive stale/future";
		if (!fresh(e.now, e.fc_publish_time, max_age)) return "FC state publish stale/future";
		if (check == FcStateCheck::CommandHold) {
			if (!fresh(e.now, e.command_time, .025)) return "FC command time stale/future";
			if (!fresh(e.command_time, e.fc_sample_time, kStateMaxAge) ||
			    !fresh(e.command_time, e.fc_receive_time, kStateMaxAge) ||
			    !fresh(e.command_time, e.fc_publish_time, kStateMaxAge))
				return "FC state stale/future at command creation";
		}
		if (e.fc_receive_time < e.fc_sample_time || e.fc_publish_time < e.fc_receive_time) return "FC timestamp ordering invalid";
		if (!fresh(e.now, e.fc_health_sample_time, .060) || !fresh(e.now, e.fc_health_receive_time, .060))
			return "FC estimator health stale/future";
		// FC heading validity/freshness is diagnostic; retain XY, Z, fusion and snapshot requirements.
		if ((e.fc_flags & 0xc3) != 0xc3) return "FC XY/Z/fusion/snapshot unavailable";
		if (!fresh(e.now, e.fc_xy_observation_time, .300)) return "FC XY observation stale/future";
		if (!fresh(e.now, e.fc_z_observation_time, .250)) return "FC Z observation stale/future";
		// Initial limits retain the existing hardware readiness standard (1m, 0.3m/s).
		for (double v : e.fc_position_variance)
			if (!std::isfinite(v) || v < 0 || v > 1.0) return "FC position variance unknown/exceeded";
		for (double v : e.fc_velocity_variance)
			if (!std::isfinite(v) || v < 0 || v > .09) return "FC velocity variance unknown/exceeded";
		return nullptr;
	}
	static bool inputsHealthy(const Evidence& e, NavigationPolicy policy = NavigationPolicy::Rtk, double observation_delay = 0.0,
	                          ReceiverPolicy receiver_policy = ReceiverPolicy::FreshSamples,
	                          FcStateCheck fc_check = FcStateCheck::Input) {
		return inputFailure(e, policy, observation_delay, receiver_policy, fc_check) == nullptr;
	}
	bool canEnterAuto(const Evidence& e) const {
		return _low_seen && e.armed && e.auto_switch && !e.kill && e.controller_warm &&
		       inputsHealthy(e, _policy, _observation_delay, _receiver_policy);
	}
	bool update(const Evidence& e, FcStateCheck fc_check = FcStateCheck::Input) {
		if (_policy == NavigationPolicy::FcFused &&
		    (e.fc_session != _fc_session || e.fc_generation != _fc_generation || e.fc_reset_counter != _fc_reset_counter)) {
			_fc_session = e.fc_session;
			_fc_generation = e.fc_generation;
			_fc_reset_counter = e.fc_reset_counter;
			_low_seen = false;
			_mode = Mode::ManualFallback;
			_reason = "FC state session/reset changed; cycle physical AUTO";
			return false;
		}
		const char* failure = inputFailure(e, _policy, _observation_delay, _receiver_policy, fc_check);
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
	std::string _fc_session;
	uint64_t _fc_generation{0};
	uint32_t _fc_reset_counter{0};
	const NavigationPolicy _policy;
	const double _observation_delay;
	const ReceiverPolicy _receiver_policy;
	Mode _mode{Mode::Boot};
	bool _low_seen{false};
	std::string _reason{"boot"};
};
}  // namespace agi::hardware
