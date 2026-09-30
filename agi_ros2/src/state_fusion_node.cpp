#include "agi_ros2/state_fusion_node.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include "agi_ros2/node_common.h"
#include "agi_ros2/runtime_config.h"
#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include "agilib/bridge/betaflight/hardware_safety.hpp"
#include "agilib/types/imu_sample.hpp"

namespace agi_ros2 {
namespace {
constexpr double kUnknownTime = std::numeric_limits<double>::quiet_NaN();
using agi::hardware::monotonicSeconds;
using agi::hardware::SafetyGate;

std::string navigationUpdateReason(agi::EkfImu::NavigationUpdateReason reason) {
	using Reason = agi::EkfImu::NavigationUpdateReason;
	switch (reason) {
		case Reason::kNotAttempted:
			return "not_attempted";
		case Reason::kAccepted:
			return "accepted";
		case Reason::kInvalidInput:
			return "invalid_input";
		case Reason::kTimestamp:
			return "timestamp";
		case Reason::kInnovation:
			return "innovation";
		case Reason::kNumericalFailure:
			return "numerical_failure";
	}
	return "unknown";
}
}  // namespace

StateFusionNode::StateFusionNode()
        : Node("state_fusion"), _clock_id(readClockId()), _rtk_receive_time(kUnknownTime), _last_rtk_time(kUnknownTime) {
	rcl_interfaces::msg::ParameterDescriptor descriptor;
	descriptor.read_only = true;
	const auto mode = declare_parameter<std::string>("mode", "sitl", descriptor);
	if (mode != "sitl" && mode != "hardware") throw std::invalid_argument("mode must be sitl or hardware");
	if (mode == "hardware" && get_parameter("use_sim_time").as_bool())
		throw std::invalid_argument("Hardware fusion requires use_sim_time=false");
	const auto profile = loadRuntimeConfig(*this, mode);
	const auto configured = [this, &descriptor, &profile](const char* name, auto fallback) {
		using Value = decltype(fallback);
		if (profile) {
			const auto value = profile->section("fusion")[name];
			if (!value.isNull()) {
				if constexpr (std::is_same_v<Value, std::vector<double>>) {
					if (!value.isSequence()) throw std::invalid_argument(name);
					fallback.clear();
					for (size_t i = 0; i < value.size(); ++i)
						fallback.push_back(value[static_cast<int>(i)].as<double>());
				} else {
					fallback = value.as<Value>();
				}
			}
		}
		return declare_parameter<Value>(name, fallback, descriptor);
	};
	const bool delay_test = declare_parameter<bool>(
	        "sitl_delay_test", profile ? profile->section("flight")["sitl_delay_test"].as<bool>() : false, descriptor);
	if (delay_test && (mode != "sitl" || !get_parameter("use_sim_time").as_bool())) {
		throw std::invalid_argument("sitl_delay_test requires mode=sitl and use_sim_time=true");
	}
	_timing_checks = !delay_test;
	const auto positive = [&configured](const char* name, double value) {
		const double result = configured(name, value);
		if (!std::isfinite(result) || result <= 0) throw std::invalid_argument(name);
		return result;
	};
	_rtk_position_variance.setConstant(positive("rtk_position_variance", 0.0004));
	_rtk_velocity_variance.setConstant(positive("rtk_velocity_variance", 0.0025));
	_rtk_heading_variance = positive("rtk_heading_variance", 0.0001);
	_ekf_parameters = std::make_shared<agi::EkfImuParameters>();
	_ekf_parameters->R_acc.setConstant(positive("imu_acceleration_variance", 0.1));
	_ekf_parameters->R_omega.setConstant(positive("imu_angular_velocity_variance", 0.0001));
	const auto variances = [&configured](const char* name, auto& destination, double value) {
		const auto values = configured(name, std::vector<double>(destination.size(), value));
		if (values.size() != static_cast<size_t>(destination.size())) throw std::invalid_argument(name);
		for (size_t i = 0; i < values.size(); ++i) {
			if (!std::isfinite(values[i]) || values[i] < 0) throw std::invalid_argument(name);
			destination(i) = values[i];
		}
	};
	variances("ekf_process_position_variance", _ekf_parameters->Q_pos, 1e-6);
	variances("ekf_process_attitude_variance", _ekf_parameters->Q_att, 1e-6);
	variances("ekf_process_velocity_variance", _ekf_parameters->Q_vel, 1e-3);
	variances("ekf_process_gyro_bias_variance", _ekf_parameters->Q_bome, 1e-9);
	variances("ekf_process_acceleration_bias_variance", _ekf_parameters->Q_bacc, 1e-9);
	variances("ekf_initial_attitude_variance", _ekf_parameters->Q_init_att, 0.01);
	variances("ekf_initial_gyro_bias_variance", _ekf_parameters->Q_init_bome, 0.001);
	variances("ekf_initial_acceleration_bias_variance", _ekf_parameters->Q_init_bacc, 0.01);
	_navigation_source = declare_parameter<std::string>("navigation_source", mode == "hardware" ? "gnss" : "rtk", descriptor);
	if (_navigation_source != "rtk" && _navigation_source != "gnss")
		throw std::invalid_argument("navigation_source must be rtk or gnss");
	ImuInitialization::Params initialization;
	initialization.duration = positive("imu_initialization_duration", 3.0);
	initialization.minimum_samples = configured("imu_initialization_samples", 1000);
	if (initialization.minimum_samples < 2 || initialization.minimum_samples > 10000)
		throw std::invalid_argument("imu_initialization_samples must be between 2 and 10000");
	initialization.max_gyro_bias = positive("imu_max_gyro_bias", 0.15);
	initialization.max_gyro_stddev = positive("imu_max_gyro_stddev", 0.02);
	initialization.max_acceleration_stddev = positive("imu_max_acceleration_stddev", 0.2);
	initialization.gravity_tolerance = positive("imu_gravity_tolerance", 0.5);
	_reference_gravity_tolerance = initialization.gravity_tolerance;
	_reference_max_angular_speed = initialization.max_gyro_bias;
	_imu_initialization = std::make_unique<ImuInitialization>(initialization);
	_initialization_max_speed = positive("imu_initialization_max_speed", 0.3);
	_navigation_ready_updates = configured("navigation_ready_updates", 30);
	if (_navigation_ready_updates < 1) throw std::invalid_argument("navigation_ready_updates must be positive");
	_navigation_nis_threshold = positive("navigation_nis_threshold", 24.322);
	_navigation_horizontal_nis_threshold = positive("navigation_horizontal_nis_threshold", 20.515);
	_navigation_height_nis_threshold = positive("navigation_height_nis_threshold", 10.828);
	_navigation_vertical_velocity_nis_threshold = positive("navigation_vertical_velocity_nis_threshold", 10.828);
	_gnss_height_variance_floor = positive("gnss_height_variance_floor", 18.0);
	_gnss_height_variance_scale = positive("gnss_height_variance_scale", 4.5);
	_gnss_vertical_velocity_variance_floor = positive("gnss_vertical_velocity_variance_floor", 0.09);
	_gnss_vertical_velocity_variance_scale = positive("gnss_vertical_velocity_variance_scale", 2.25);
	if (_gnss_height_variance_scale < 1 || _gnss_vertical_velocity_variance_scale < 1)
		throw std::invalid_argument("GNSS variance scales must be at least one");
	const auto limit = [&configured](const char* name) {
		const double value = configured(name, 0.0);
		if (!std::isfinite(value) || value < 0) throw std::invalid_argument(name);
		return value;
	};
	_max_horizontal_position_stddev = limit("max_horizontal_position_stddev");
	_max_vertical_position_stddev = limit("max_vertical_position_stddev");
	_max_velocity_stddev = limit("max_velocity_stddev");
	_max_heading_stddev = limit("max_heading_stddev");
	_baro_enabled = configured("baro_enabled", false);
	_gnss_use_baro_height = configured("gnss_use_baro_height", false);
	_height_fusion_mode = configured("height_fusion_mode", std::string(_gnss_use_baro_height ? "baro_primary" : "legacy_full3d"));
	if (_height_fusion_mode != "legacy_full3d" && _height_fusion_mode != "baro_primary" && _height_fusion_mode != "baro_gnss_weighted")
		throw std::invalid_argument("height_fusion_mode must be legacy_full3d, baro_primary or baro_gnss_weighted");
	_weighted_height_mode = _navigation_source == "gnss" && _height_fusion_mode == "baro_gnss_weighted";
	if (_navigation_source == "gnss") _gnss_use_baro_height = _height_fusion_mode != "legacy_full3d";
	_observation_delay = configured("observation_delay", _baro_enabled ? 0.20 : 0.0);
	if (!std::isfinite(_observation_delay) || _observation_delay < 0 || _observation_delay > 0.25)
		throw std::invalid_argument("observation_delay must be between 0 and 0.25 seconds");
	_baro_max_age = positive("baro_max_age", 0.25);
	_baro_pressure_variance = positive("baro_pressure_variance_pa2", 4.0);
	_baro_min_pressure = positive("baro_min_pressure_pa", 30000.0);
	_baro_max_pressure = positive("baro_max_pressure_pa", 120000.0);
	_baro_model_variance = positive("baro_model_variance", 0.25);
	_baro_nis_threshold = positive("baro_nis_threshold", 10.828);
	if (_baro_max_pressure <= _baro_min_pressure || _baro_max_age > 1.0)
		throw std::invalid_argument("Invalid barometer pressure range or maximum age");
	_ekf_parameters->baro_bias_random_walk = configured("baro_bias_random_walk", _gnss_use_baro_height ? 0.0 : 0.01);
	if (!std::isfinite(_ekf_parameters->baro_bias_random_walk) || _ekf_parameters->baro_bias_random_walk < 0 ||
	    (_gnss_use_baro_height && _navigation_source == "gnss" && _ekf_parameters->baro_bias_random_walk != 0))
		throw std::invalid_argument("baro_bias_random_walk must be nonnegative, and zero for relative barometer height modes");
	_ekf_parameters->baro_relative_reference = _weighted_height_mode;
	BarometerReference::Params baro_reference;
	baro_reference.duration = positive("baro_reference_duration", 2.0);
	const int reference_samples = configured("baro_reference_min_samples", 40);
	if (reference_samples < 2 || reference_samples > 10000 || baro_reference.duration > 30)
		throw std::invalid_argument("Invalid barometer reference sample count or duration");
	baro_reference.minimum_samples = static_cast<size_t>(reference_samples);
	baro_reference.maximum_gap = _baro_max_age;
	baro_reference.maximum_pressure_stddev = positive("baro_reference_max_stddev_pa", 15.0);
	baro_reference.maximum_height_stddev = positive("baro_reference_max_vertical_stddev", 0.3);
	baro_reference.alignment_variance = positive("baro_reference_variance", 4.0);
	_baro_reference = std::make_unique<BarometerReference>(baro_reference);
	reset();
	_fused_pub = create_publisher<msg::FusedState>("fused_state", 1);
	_state_pub = create_publisher<nav_msgs::msg::Odometry>("state", 1);
	_authority_sub = create_subscription<msg::Authority>("authority", 1, [this](msg::Authority::ConstSharedPtr message) {
		_authority = *message;
		_authority_receive_time = monotonicSeconds();
	});
	if (_navigation_source == "rtk") {
		_rtk_sub =
		        create_subscription<msg::Rtk>("sensors/rtk", 10, std::bind(&StateFusionNode::onRtk, this, std::placeholders::_1));
	} else {
		_navigation_sub =
		        create_subscription<msg::LocalNavigation>("sensors/local_navigation", rclcpp::SensorDataQoS(),
			                                          std::bind(&StateFusionNode::onNavigation, this, std::placeholders::_1));
	}
	_imu_sub = create_subscription<sensor_msgs::msg::Imu>("sensors/imu", rclcpp::SensorDataQoS().keep_last(256),
	                                                      std::bind(&StateFusionNode::onImu, this, std::placeholders::_1));
	_baro_status_pub = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("fusion/baro/status", 10);
	_baro_status_timer = create_wall_timer(std::chrono::milliseconds(200), [this] { publishBarometerStatus(); });
	if (_baro_enabled) {
		_baro_sub = create_subscription<msg::Barometer>("sensors/baro/sample", rclcpp::QoS(100).reliable(),
		                                                std::bind(&StateFusionNode::onBarometer, this, std::placeholders::_1));
	}
}

void StateFusionNode::reset() {
	_ahrs = std::make_unique<agi::CompanionAhrs>(agi::CompanionAhrs::Params{});
	auto params = std::make_shared<agi::EkfImuParameters>(*_ekf_parameters);
	params->Q_init_pos = _rtk_position_variance;
	params->Q_init_vel = _rtk_velocity_variance;
	_ekf = std::make_unique<agi::EkfImu>(params);
	_state.setZero();
	_state.t = kUnknownTime;
	_last_rtk_time = kUnknownTime;
	_last_navigation_attempt = kUnknownTime;
	_gnss_raw_height_variance = _gnss_effective_height_variance = kUnknownTime;
	_gnss_raw_vertical_velocity_variance = _gnss_effective_vertical_velocity_variance = kUnknownTime;
	_last_imu_time = kUnknownTime;
	_observations.clear();
	_reference_imus.clear();
	_observation_watermark = kUnknownTime;
	resetBarometer("EKF reset; waiting for stationary pressure reference");
	_gnss_vertical_recovery_pending = false;
	_imu_ready = false;
	_accepted_navigation_updates = 0;
	_imu_initialization->reset();
	_readiness_reason = "Waiting for stationary IMU initialization";
	++_reset_counter;
}

void StateFusionNode::onRtk(msg::Rtk::ConstSharedPtr message) {
	if (stampSeconds(message->header.stamp) >= stampSeconds(_rtk.header.stamp)) {
		_rtk = *message;
		_rtk_receive_time = monotonicSeconds();
	}
	enqueueNavigation(*message);
}

void StateFusionNode::onNavigation(msg::LocalNavigation::ConstSharedPtr message) {
	if (message->header.frame_id != "odom" || message->session_id.empty() || message->source_session.empty()) return;
	const bool changed = _navigation.session_id != message->session_id || _navigation.source_session != message->source_session;
	const bool newest = changed || stampSeconds(message->header.stamp) > stampSeconds(_navigation.header.stamp);
	if (!message->observation_valid || !message->clock_aligned || message->fix_type < 3 || message->fix_type > 6 ||
	    !message->heading_valid || !std::isfinite(message->heading)) {
		if (!newest) return;
		const double previous_imu_time = _state.t;
		if (changed) {
			reset();
			_state.t = previous_imu_time;
		}
		_navigation = *message;
		_observations.clear();
		resetBarometer("Navigation revoked; waiting for stationary pressure reference");
		_rtk.heading_valid = false;
		_rtk.accuracy_ok = false;
		_rtk_receive_time = kUnknownTime;
		_accepted_navigation_updates = 0;
		_readiness_reason = message->reason.empty() ? "Navigation source explicitly revoked its solution" : message->reason;
		// Do not refresh IMU or last accepted navigation acquisition times.
		publishState(_last_imu_receive_time);
		return;
	}
	if (message->altitude_reference != "ellipsoid" && message->altitude_reference != "msl") return;
	for (double variance : message->position_variance)
		if (!std::isfinite(variance) || variance <= 0) return;
	for (double variance : message->velocity_variance)
		if (!std::isfinite(variance) || variance <= 0) return;
	if (!std::isfinite(message->heading_variance) || message->heading_variance <= 0) return;
	if (!std::isfinite(message->position.x) || !std::isfinite(message->position.y) || !std::isfinite(message->position.z) ||
	    !std::isfinite(message->velocity.x) || !std::isfinite(message->velocity.y) || !std::isfinite(message->velocity.z) ||
	    !message->heading_valid || !std::isfinite(message->heading))
		return;
	const bool initialize_parameters = !_navigation.observation_valid && !_ekf->healthy();
	if (newest) {
		_navigation = *message;
		for (int i = 0; i < 3; ++i) {
			_rtk_position_variance(i) = message->position_variance[i];
			_rtk_velocity_variance(i) = message->velocity_variance[i];
		}
		_rtk_heading_variance = message->heading_variance;
	}
	if (changed || initialize_parameters) {
		_rtk = msg::Rtk();
		reset();
	}
	// Internal observation storage only: never manufacture fixed/PPS evidence.
	msg::Rtk navigation;
	navigation.header = message->header;
	navigation.position = message->position;
	navigation.velocity = message->velocity;
	navigation.heading = message->heading;
	navigation.heading_valid = message->heading_valid;
	navigation.accuracy_ok = message->accuracy_known && message->accuracy_ok && std::isfinite(message->horizontal_accuracy) &&
	                         message->horizontal_accuracy > 0 && std::isfinite(message->vertical_accuracy) &&
	                         message->vertical_accuracy > 0 && std::isfinite(message->velocity_accuracy) &&
	                         message->velocity_accuracy > 0;
	if (newest) {
		_rtk = navigation;
		_rtk_receive_time = monotonicSeconds();
	}
	Observation observation;
	observation.time = stampSeconds(message->header.stamp);
	observation.navigation = navigation;
	for (int i = 0; i < 3; ++i) {
		observation.position_variance(i) = message->position_variance[i];
		observation.velocity_variance(i) = message->velocity_variance[i];
	}
	observation.heading_variance = message->heading_variance;
	enqueueObservation(observation);
}

void StateFusionNode::enqueueNavigation(const msg::Rtk& navigation) {
	if (navigation.header.frame_id != "odom" || !navigation.fixed || !navigation.accuracy_ok) return;
	Observation observation;
	observation.time = stampSeconds(navigation.header.stamp);
	observation.navigation = navigation;
	observation.position_variance = _rtk_position_variance;
	observation.velocity_variance = _rtk_velocity_variance;
	observation.heading_variance = _rtk_heading_variance;
	enqueueObservation(observation);
}

void StateFusionNode::enqueueObservation(const Observation& observation) {
	const double age = now().seconds() - observation.time;
	if (!std::isfinite(observation.time) || (_timing_checks && (age < -0.010 || age > (observation.barometer ? _baro_max_age : 0.3))) ||
	    (std::isfinite(_observation_watermark) && observation.time <= _observation_watermark)) {
		if (observation.barometer)
			++_late_barometer;
		else
			++_late_navigation;
		return;
	}
	// At a shared timestamp GNSS runs first, then the scalar pressure update.
	const auto less = [](const Observation& left, const Observation& right) {
		return left.time < right.time || (left.time == right.time && left.barometer < right.barometer);
	};
	const auto position = std::lower_bound(_observations.begin(), _observations.end(), observation, less);
	if (position != _observations.end() && position->time == observation.time && position->barometer == observation.barometer) return;
	if (_observations.size() >= 1024) {
		++_observation_overflows;
		return;
	}
	_observations.insert(position, observation);
}

void StateFusionNode::resetBarometer(const std::string& reason) {
	if (_weighted_height_mode && _ekf) _ekf->clearBarometerReference();
	if (_gnss_baro_height_active) {
		_accepted_navigation_updates = 0;
		_gnss_vertical_recovery_pending = true;
	}
	_gnss_baro_height_active = false;
	_baro_stream_stale = false;
	_observations.erase(std::remove_if(_observations.begin(), _observations.end(),
	                                   [](const Observation& observation) { return observation.barometer; }),
	                    _observations.end());
	_baro_reference->reset();
	_baro_last_time = _baro_receive_time = kUnknownTime;
	_baro_last_height = _baro_last_height_variance = kUnknownTime;
	_baro_reference_bias_variance = kUnknownTime;
	_baro_reason = _baro_enabled ? reason : "Barometer disabled";
	++_baro_reference_resets;
}

void StateFusionNode::onBarometer(msg::Barometer::ConstSharedPtr message) {
	if (!_baro_enabled || message->source_session.empty()) return;
	if (_baro_session != message->source_session) {
		resetBarometer("Pressure source session changed; waiting for stationary reference");
		_baro_session = message->source_session;
	}
	if (!message->valid || !message->clock_aligned) {
		resetBarometer(message->reason.empty() ? "Pressure source invalid or clock unaligned" : message->reason);
		return;
	}
	if (_navigation_source == "gnss" && _baro_session != _navigation.source_session) {
		resetBarometer("Pressure and navigation source sessions differ");
		return;
	}
	const double time = stampSeconds(message->header.stamp);
	const double age = now().seconds() - time;
	if (message->header.frame_id != "baro_link" || !std::isfinite(message->pressure_pa) || message->pressure_pa < _baro_min_pressure ||
	    message->pressure_pa > _baro_max_pressure || !std::isfinite(message->pressure_variance) || message->pressure_variance < 0 ||
	    (_timing_checks && (age < -0.010 || age > _baro_max_age))) {
		++_baro_invalid_samples;
		return;
	}
	Observation observation;
	observation.time = time;
	observation.barometer = true;
	observation.pressure = message->pressure_pa;
	observation.pressure_variance = message->pressure_variance > 0 ? message->pressure_variance : _baro_pressure_variance;
	enqueueObservation(observation);
	// Duplicate or out-of-order arrivals never renew stream freshness.
	if (!std::isfinite(_baro_last_time) || time > _baro_last_time) {
		_baro_last_time = time;
		_baro_receive_time = monotonicSeconds();
	}
}

void StateFusionNode::processBarometer(const Observation& observation, double received) {
	if (!_baro_reference->ready()) {
		agi::QuadState aligned;
		const auto quality = _ekf->navigationQuality();
		const bool disarmed = SafetyGate::fresh(received, _authority_receive_time, 0.1) &&
		                      SafetyGate::fresh(now().seconds(), stampSeconds(_authority.header.stamp), 0.1) &&
		                      _authority.rc_link && !_authority.armed;
		const auto reference_imu = std::lower_bound(_reference_imus.begin(), _reference_imus.end(), observation.time,
		                                            [](const agi::ImuSample& imu, double time) { return imu.t < time; });
		const bool imu_covered = reference_imu != _reference_imus.end() && reference_imu->t - observation.time <= 0.025;
		std::string reference_blocker;
		if (!_ekf->getAt(observation.time, &aligned) || !aligned.valid() || !quality.valid)
			reference_blocker = "Reference requires a valid predicted state and covariance";
		else if (!disarmed)
			reference_blocker = "Reference requires fresh disarmed authority";
		else if (!_imu_ready || !_rtk.heading_valid || !SafetyGate::fresh(observation.time, _last_rtk_time, 0.3, _timing_checks))
			reference_blocker = "Reference requires initialized IMU and fresh navigation with heading";
		else if (!imu_covered)
			reference_blocker = "Reference has no IMU coverage at the pressure sampling time";
		else if (aligned.v.norm() > _initialization_max_speed)
			reference_blocker = "Reference velocity exceeds imu_initialization_max_speed";
		else if ((reference_imu->omega - aligned.bw).norm() > _reference_max_angular_speed)
			reference_blocker = "Reference angular speed exceeds imu_max_gyro_bias";
		else if ((agi::GVEC + aligned.R() * (reference_imu->acc - aligned.ba)).norm() > _reference_gravity_tolerance)
			reference_blocker = "Reference gravity residual exceeds imu_gravity_tolerance";
		const bool stationary = reference_blocker.empty();
		if (_baro_reference->addSample(observation.time, observation.pressure, observation.pressure_variance,
		                               stationary ? aligned.p.z() : NAN, stationary)) {
			double reference_height = NAN;
			if (!_ekf->alignBarometerReference(observation.time, _baro_reference->independentVariance(), &reference_height)) {
				resetBarometer("Pressure bias initialization failed");
				return;
			}
			_baro_reference->alignHeight(reference_height);
			_baro_reference_bias_variance = _ekf->barometerQuality().bias_variance;
			_baro_reason = "Pressure reference aligned; waiting for a new observation";
		} else {
			_baro_reason = stationary ? "Collecting stable pressure and local height reference" : reference_blocker;
		}
		return;
	}
	if (!_baro_reference->heightObservation(observation.pressure, observation.pressure_variance, _baro_model_variance,
	                                        &_baro_last_height, &_baro_last_height_variance))
		return;
	if (_ekf->addBaro(observation.time, _baro_last_height, _baro_last_height_variance, _baro_nis_threshold))
		_baro_reason = "Fusing IMU, navigation and barometer";
	else
		_baro_reason = "Barometer update rejected by innovation, timing or numerical checks";
}

void StateFusionNode::processObservations(double time, double received) {
	const double started = monotonicSeconds();
	// A current pressure sample can precede an IMU callback that is catching up.
	// Check stream age against the ROS clock; the IMU stamp only advances the queue.
	const double current_time = now().seconds();
	const bool pressure_time_fresh =
	        SafetyGate::fresh(current_time, _baro_last_time, _baro_max_age, _timing_checks) ||
	        (std::isfinite(_baro_last_time) && _baro_last_time > current_time && _baro_last_time - current_time <= 0.010);
	if (_baro_enabled && std::isfinite(_baro_receive_time) &&
	    (!SafetyGate::fresh(received, _baro_receive_time, _baro_max_age) || !pressure_time_fresh)) {
		if (_weighted_height_mode) {
			if (!_baro_stream_stale && _gnss_baro_height_active) {
				_accepted_navigation_updates = 0;
				_gnss_vertical_recovery_pending = true;
			}
			_baro_stream_stale = true;
			_gnss_baro_height_active = false;
			_baro_reason = "Pressure stream stale; retaining same-session reference for recovery";
		} else {
			resetBarometer("Pressure stream stale; using IMU and navigation");
		}
	} else {
		_baro_stream_stale = false;
	}
	const double cutoff = time - _observation_delay;
	while (!_observations.empty() && _observations.front().time <= cutoff) {
		const Observation observation = _observations.front();
		_observations.pop_front();
		if (observation.barometer) {
			processBarometer(observation, received);
			continue;
		}
		const auto& navigation = observation.navigation;
		_last_navigation_attempt = observation.time;
		const agi::Vector<3> position(navigation.position.x, navigation.position.y, navigation.position.z);
		const agi::Vector<3> velocity(navigation.velocity.x, navigation.velocity.y, navigation.velocity.z);
		const auto barometer = _ekf->barometerQuality();
		// Select the height source at the observation time, not the newest queued pressure time.
		// A stream of rejected pressure outliers must not suppress GNSS height indefinitely.
		const bool baro_height = _navigation_source == "gnss" && _gnss_use_baro_height && _baro_enabled && !_baro_stream_stale &&
		                         _baro_reference->ready() && barometer.valid &&
		                         SafetyGate::fresh(observation.time, barometer.stamp, _baro_max_age);
		if (_weighted_height_mode) {
			if (_gnss_baro_height_active != baro_height) _accepted_navigation_updates = 0;
			_gnss_baro_height_active = baro_height;
			agi::Vector<3> position_variance = observation.position_variance;
			agi::Vector<3> velocity_variance = observation.velocity_variance;
			_gnss_raw_height_variance = position_variance.z();
			_gnss_raw_vertical_velocity_variance = velocity_variance.z();
			if (baro_height)
				position_variance.z() =
				        std::max(_gnss_height_variance_floor, _gnss_height_variance_scale * position_variance.z());
			velocity_variance.z() = std::max(_gnss_vertical_velocity_variance_floor,
			                                 _gnss_vertical_velocity_variance_scale * velocity_variance.z());
			_gnss_effective_height_variance = position_variance.z();
			_gnss_effective_vertical_velocity_variance = velocity_variance.z();
			const auto result = _ekf->addNavigation(
			        observation.time, position, velocity, navigation.heading, navigation.heading_valid, position_variance,
			        velocity_variance, observation.heading_variance, _navigation_horizontal_nis_threshold,
			        _navigation_height_nis_threshold, _navigation_vertical_velocity_nis_threshold);
			const bool horizontal_accepted = result.committed && (result.accepted_groups & agi::EkfImu::kHorizontalGroup);
			const auto vertical_groups = agi::EkfImu::kHeightGroup | agi::EkfImu::kVerticalVelocityGroup;
			const bool vertical_accepted = result.committed && (result.accepted_groups & vertical_groups) == vertical_groups;
			_gnss_vertical_recovery_pending = !baro_height && !vertical_accepted;
			if (horizontal_accepted) _last_rtk_time = observation.time;
			if (horizontal_accepted && navigation.accuracy_ok && !_gnss_vertical_recovery_pending)
				_accepted_navigation_updates = std::min(_accepted_navigation_updates + 1, _navigation_ready_updates);
			else
				_accepted_navigation_updates = 0;
			if (!horizontal_accepted)
				_readiness_reason =
				        "GNSS horizontal/heading update rejected; accepted vertical groups do not restore navigation "
				        "readiness";
			else if (_gnss_vertical_recovery_pending)
				_readiness_reason =
				        "GNSS vertical recovery unavailable; waiting for accepted height and vertical velocity or "
				        "barometer";
			else
				_readiness_reason = "Waiting for consecutive accepted navigation updates and covariance limits";
			continue;
		}
		const auto measurement_mode =
		        baro_height ? agi::EkfImu::NavigationMeasurementMode::kHorizontal : agi::EkfImu::NavigationMeasurementMode::kFull3d;
		const double innovation_limit = _navigation_source == "gnss"
		                                        ? (baro_height ? _navigation_horizontal_nis_threshold : _navigation_nis_threshold)
		                                        : std::numeric_limits<double>::infinity();
		if (_gnss_baro_height_active != baro_height) _accepted_navigation_updates = 0;
		_gnss_baro_height_active = baro_height;
		bool accepted = _ekf->addRtk(observation.time, position, velocity, navigation.heading, navigation.heading_valid,
		                             observation.position_variance, observation.velocity_variance, observation.heading_variance,
		                             innovation_limit, measurement_mode);
		_gnss_vertical_recovery_pending = false;
		if (!accepted && !baro_height && _gnss_use_baro_height && _navigation_source == "gnss") {
			// Height sources may disagree after an outage. Keep usable XY/heading observations, but do not
			// advertise navigation readiness until a complete vertical update or pressure fusion recovers.
			_gnss_vertical_recovery_pending = true;
			accepted = _ekf->addRtk(observation.time, position, velocity, navigation.heading, navigation.heading_valid,
			                        observation.position_variance, observation.velocity_variance, observation.heading_variance,
			                        _navigation_horizontal_nis_threshold, agi::EkfImu::NavigationMeasurementMode::kHorizontal);
		}
		if (accepted) {
			_last_rtk_time = observation.time;
			if (navigation.accuracy_ok && !_gnss_vertical_recovery_pending)
				_accepted_navigation_updates = std::min(_accepted_navigation_updates + 1, _navigation_ready_updates);
			else
				_accepted_navigation_updates = 0;
			_readiness_reason = "Waiting for consecutive accepted navigation updates and covariance limits";
		} else {
			_accepted_navigation_updates = 0;
			_readiness_reason = "Navigation update rejected by EKF timing, innovation or numerical checks";
		}
		if (_gnss_vertical_recovery_pending)
			_readiness_reason = "GNSS vertical recovery unavailable; waiting for accepted 3D navigation or barometer";
	}
	// Without barometer or an explicit reorder delay, preserve the original
	// delayed-navigation contract: only a committed posterior closes history.
	const double committed = _baro_enabled || _observation_delay > 0 ? cutoff : _ekf->navigationQuality().stamp;
	if (!std::isfinite(_observation_watermark) || committed > _observation_watermark) _observation_watermark = committed;
	_observation_processing_seconds = monotonicSeconds() - started;
}

void StateFusionNode::publishBarometerStatus() {
	const auto quality = _ekf->barometerQuality();
	const auto navigation = _ekf->navigationQuality();
	diagnostic_msgs::msg::DiagnosticArray out;
	out.header.stamp = now();
	diagnostic_msgs::msg::DiagnosticStatus status;
	status.name = "height_fusion";
	status.hardware_id = _baro_session;
	const double age = now().seconds() - _baro_last_time;
	const double accepted_age = now().seconds() - quality.stamp;
	const bool healthy = _baro_enabled && _baro_reference->ready() && quality.valid &&
	                     SafetyGate::fresh(monotonicSeconds(), _baro_receive_time, _baro_max_age) && age >= -0.010 &&
	                     age <= _baro_max_age && accepted_age >= 0 && accepted_age <= _baro_max_age + _observation_delay;
	status.level = !_baro_enabled || healthy ? status.OK : status.WARN;
	status.message = _baro_reason;
	if (_baro_enabled && !healthy && _baro_reference->ready())
		status.message = "No recent accepted barometer update; using IMU and navigation: " + _baro_reason;
	const bool vertical_degraded = _weighted_height_mode && healthy && std::isfinite(navigation.height.observation_stamp) &&
	                               (!navigation.height.accepted || !navigation.vertical_velocity.accepted);
	if (vertical_degraded) {
		status.level = status.WARN;
		status.message += "; GNSS height or vertical velocity rejected; pressure retains vertical readiness support";
	}
	if (_weighted_height_mode && std::isfinite(navigation.horizontal.observation_stamp) && !navigation.horizontal.accepted) {
		status.level = status.WARN;
		status.message += "; GNSS horizontal/heading rejected; navigation readiness revoked";
	}
	const auto add = [&status](const std::string& key, const auto& value) {
		diagnostic_msgs::msg::KeyValue item;
		item.key = key;
		if constexpr (std::is_convertible_v<decltype(value), std::string>)
			item.value = value;
		else
			item.value = std::to_string(value);
		status.values.push_back(item);
	};
	add("enabled", _baro_enabled);
	add("healthy", healthy);
	add("height_fusion_mode", _height_fusion_mode);
	add("weighted_height_mode_active", _weighted_height_mode);
	add("gnss_use_baro_height", _gnss_use_baro_height);
	add("gnss_baro_height_active", _gnss_baro_height_active);
	add("gnss_vertical_recovery_pending", _gnss_vertical_recovery_pending);
	add("gnss_vertical_degraded", vertical_degraded);
	add("baro_bias_random_walk_m2_s", _ekf_parameters->baro_bias_random_walk);
	add("reference_valid", _baro_reference->ready());
	add("ekf_reference_valid", quality.reference_valid);
	add("relative_reference_active", quality.relative_reference_active);
	add("reference_samples", _baro_reference->sampleCount());
	add("reference_pressure_pa", _baro_reference->pressure());
	add("reference_height_m", _baro_reference->height());
	add("reference_bias_variance_m2", _baro_reference_bias_variance);
	add("reference_independent_variance_m2", _baro_reference->independentVariance());
	add("sample_age_s", age);
	add("last_accepted_stamp", quality.stamp);
	add("height_m", _baro_last_height);
	add("height_variance_m2", _baro_last_height_variance);
	add("bias_m", quality.bias);
	add("bias_variance_m2", quality.bias_variance);
	add("relative_height_variance_m2", quality.relative_height_variance);
	add("innovation_m", quality.innovation);
	add("nis", quality.innovation_squared);
	add("accepted_updates", quality.accepted_updates);
	add("rejected_updates", quality.rejected_updates);
	add("late_navigation", _late_navigation);
	add("late_barometer", _late_barometer);
	add("queue_size", _observations.size());
	add("queue_overflows", _observation_overflows);
	add("queue_capacity", 1024);
	add("queue_payload_bytes", _observations.size() * sizeof(Observation));
	add("last_processing_seconds", _observation_processing_seconds);
	add("reference_resets", _baro_reference_resets);
	add("invalid_samples", _baro_invalid_samples);
	add("observation_delay_s", _observation_delay);
	add("navigation_nis_dimensions", navigation.observation_dimensions);
	add("absolute_height_variance_m2", navigation.position_variance.z());
	add("gnss_raw_height_variance_m2", _gnss_raw_height_variance);
	add("gnss_effective_height_variance_m2", _gnss_effective_height_variance);
	add("gnss_raw_vertical_velocity_variance_m2_s2", _gnss_raw_vertical_velocity_variance);
	add("gnss_effective_vertical_velocity_variance_m2_s2", _gnss_effective_vertical_velocity_variance);
	const auto add_group = [&add, this](const std::string& name, const agi::EkfImu::NavigationGroupQuality& group,
	                                    double nis_threshold) {
		add(name + "_nis", group.innovation_squared);
		add(name + "_nis_threshold", nis_threshold);
		add(name + "_dimensions", group.observation_dimensions);
		add(name + "_accepted", group.accepted);
		add(name + "_reason", navigationUpdateReason(group.reason));
		add(name + "_observation_stamp", group.observation_stamp);
		add(name + "_last_accepted_stamp", group.stamp);
		add(name + "_accepted_age_s", now().seconds() - group.stamp);
		add(name + "_accepted_updates", group.accepted_updates);
		add(name + "_rejected_updates", group.rejected_updates);
	};
	add_group("gnss_horizontal", navigation.horizontal, _navigation_horizontal_nis_threshold);
	add_group("gnss_height", navigation.height, _navigation_height_nis_threshold);
	add_group("gnss_vertical_velocity", navigation.vertical_velocity, _navigation_vertical_velocity_nis_threshold);
	const std::vector<std::string> horizontal_rows{"position_x_m2", "position_y_m2", "velocity_x_m2_s2", "velocity_y_m2_s2",
	                                               "heading_rad2"};
	for (size_t i = 0; i < horizontal_rows.size(); ++i)
		add("gnss_horizontal_variance_" + horizontal_rows[i], navigation.horizontal.measurement_variance(i));
	add("gnss_height_variance_m2", navigation.height.measurement_variance(0));
	add("gnss_vertical_velocity_variance_m2_s2", navigation.vertical_velocity.measurement_variance(0));
	add("prediction_span_s", _state.t - navigation.stamp);
	out.status.push_back(status);
	_baro_status_pub->publish(out);
}

bool StateFusionNode::navigationFresh(double time, double received) const {
	const double fix_time = stampSeconds(_rtk.header.stamp);
	// IMU and navigation travel on different DDS streams. Hold a slightly newer
	// navigation sample until IMU coverage reaches it, without revoking the last update.
	const bool aligned = SafetyGate::fresh(time, fix_time, 0.3, _timing_checks) ||
	                     (std::isfinite(time) && fix_time > time && fix_time - time <= 0.010);
	return (get_parameter("use_sim_time").as_bool() || SafetyGate::fresh(received, _rtk_receive_time, 0.3)) && aligned &&
	       _rtk.header.frame_id == "odom" &&
	       (_navigation_source == "gnss" ? _navigation.clock_aligned : (_rtk.fixed && _rtk.accuracy_ok));
}

bool StateFusionNode::covarianceReady(const agi::EkfImu::NavigationQuality& quality) const {
	return quality.valid && _max_horizontal_position_stddev > 0 && _max_vertical_position_stddev > 0 && _max_velocity_stddev > 0 &&
	       _max_heading_stddev > 0 && quality.position_variance.head<2>().maxCoeff() <= std::pow(_max_horizontal_position_stddev, 2) &&
	       quality.position_variance.z() <= std::pow(_max_vertical_position_stddev, 2) &&
	       quality.velocity_variance.maxCoeff() <= std::pow(_max_velocity_stddev, 2) &&
	       quality.heading_variance <= std::pow(_max_heading_stddev, 2);
}

void StateFusionNode::onImu(sensor_msgs::msg::Imu::ConstSharedPtr message) {
	const double received = monotonicSeconds();
	const double time = stampSeconds(message->header.stamp);
	if (message->header.frame_id != "base_link") return;
	const agi::ImuSample imu(time, {message->linear_acceleration.x, message->linear_acceleration.y, message->linear_acceleration.z},
	                         {message->angular_velocity.x, message->angular_velocity.y, message->angular_velocity.z});
	if (!imu.valid()) return;
	if (std::isfinite(_last_imu_time)) {
		if (time == _last_imu_time) return;
		if (time < _last_imu_time || (_timing_checks && time - _last_imu_time > 0.025)) {
			if (time < _last_imu_time) {
				_rtk = msg::Rtk();
				_rtk_receive_time = kUnknownTime;
			}
			reset();
		}
	}
	_last_imu_time = time;
	_last_imu_receive_time = received;
	if (_baro_enabled) {
		_reference_imus.push_back(imu);
		while (_reference_imus.size() > 1 &&
		       (_reference_imus.front().t < time - _observation_delay - _baro_max_age - 0.025 || _reference_imus.size() > 1024))
			_reference_imus.pop_front();
	}
	const double fix_time = stampSeconds(_rtk.header.stamp);
	const bool navigation_fresh = navigationFresh(time, received);
	const bool valid_fix =
	        navigation_fresh && fix_time <= time && (!std::isfinite(_last_navigation_attempt) || fix_time > _last_navigation_attempt);
	const agi::Vector<3> position(_rtk.position.x, _rtk.position.y, _rtk.position.z);
	const agi::Vector<3> velocity(_rtk.velocity.x, _rtk.velocity.y, _rtk.velocity.z);
	const bool heading_valid = _rtk.heading_valid && std::isfinite(_rtk.heading);
	if (!navigation_fresh || (_navigation_source == "gnss" && !_rtk.accuracy_ok)) _accepted_navigation_updates = 0;

	if (!_ekf->healthy()) {
		_state.t = time;
		if (_navigation_source == "gnss") {
			const bool disarmed = SafetyGate::fresh(received, _authority_receive_time, 0.1) &&
			                      SafetyGate::fresh(now().seconds(), stampSeconds(_authority.header.stamp), 0.1) &&
			                      _authority.rc_link && !_authority.armed;
			if (!disarmed || !navigation_fresh || !heading_valid || !position.allFinite() || !velocity.allFinite() ||
			    velocity.norm() > _initialization_max_speed) {
				_imu_initialization->reset();
				_readiness_reason = "Initialization requires disarmed authority and stationary, fresh navigation";
				publishState(received);
				return;
			}
			if (!_imu_initialization->addSample(imu) || !valid_fix) {
				_readiness_reason = "Collecting stationary IMU samples; checking gravity, gyro bias and noise";
				publishState(received);
				return;
			}
			_ahrs->setHeading(_rtk.heading);
			_ahrs->addImu({time, _imu_initialization->acceleration(), _imu_initialization->gyroBias()});
		} else {
			// Preserve the existing SITL/RTK initialization contract.
			if (valid_fix && heading_valid) _ahrs->setHeading(_rtk.heading);
			_ahrs->addImu(imu);
		}
		if (!valid_fix || !heading_valid || !position.allFinite() || !velocity.allFinite() || !_ahrs->initialized()) {
			_readiness_reason = "Waiting for valid navigation and initial attitude";
			publishState(received);
			return;
		}
		_state.p = position + velocity * (time - fix_time);
		_state.v = velocity;
		_state.q(_ahrs->attitude());
		_state.q(agi::Quaternion(Eigen::AngleAxis<agi::Scalar>(_rtk.heading - _state.getYaw(), agi::Vector<3>::UnitZ())) *
		         _state.q());
		_state.bw = _navigation_source == "gnss" ? _imu_initialization->gyroBias() : _ahrs->gyroBias();
		if (!_ekf->initialize(_state) || !_ekf->addImu(imu)) {
			reset();
			_state.t = time;
			_readiness_reason = "EKF initialization failed";
			publishState(received);
			return;
		}
		_imu_ready = true;
		_last_rtk_time = fix_time;
		_last_navigation_attempt = fix_time;
		_observation_watermark = time;
		while (!_observations.empty() && _observations.front().time <= time) _observations.pop_front();
	} else {
		if (!_ekf->addImu(imu)) {
			reset();
			_state.t = time;
			_readiness_reason = "EKF rejected IMU; reinitialization required";
			publishState(received);
			return;
		}
		processObservations(time, received);
	}
	if (!_ekf->getAt(time, &_state) || !_state.valid()) {
		reset();
		_state.t = time;
		_readiness_reason = "EKF prediction failed; reinitialization required";
	}
	publishState(received);
}

void StateFusionNode::publishState(double imu_receive_time) {
	msg::FusedState out;
	out.header.stamp = rosStamp(std::isfinite(_state.t) ? _state.t : now().seconds());
	out.header.frame_id = "odom";
	out.position.x = _state.p.x();
	out.position.y = _state.p.y();
	out.position.z = _state.p.z();
	out.velocity.x = _state.v.x();
	out.velocity.y = _state.v.y();
	out.velocity.z = _state.v.z();
	out.orientation.w = _state.q().w();
	out.orientation.x = _state.q().x();
	out.orientation.y = _state.q().y();
	out.orientation.z = _state.q().z();
	out.body_rates.x = _state.w.x();
	out.body_rates.y = _state.w.y();
	out.body_rates.z = _state.w.z();
	out.acceleration.x = _state.a.x();
	out.acceleration.y = _state.a.y();
	out.acceleration.z = _state.a.z();
	out.clock_id = _clock_id;
	out.published_steady_time = monotonicSeconds();
	out.imu_receive_time = imu_receive_time;
	out.rtk_receive_time = _rtk_receive_time;
	if (std::isfinite(_last_rtk_time)) {
		out.rtk_stamp = rosStamp(_last_rtk_time);
	}
	out.navigation_source = _navigation_source;
	out.navigation_session = _navigation.session_id + ":" + _navigation.source_session;
	out.fix_type = _navigation_source == "gnss" ? _navigation.fix_type : (_rtk.fixed ? 6 : 0);
	out.clock_aligned = _navigation_source == "gnss" ? _navigation.clock_aligned : _rtk.synchronized;
	out.accuracy_known = _navigation_source == "gnss" ? _navigation.accuracy_known : _rtk.accuracy_ok;
	out.navigation_valid = std::isfinite(_last_rtk_time) && SafetyGate::fresh(_state.t, _last_rtk_time, .3, _timing_checks) &&
	                       navigationFresh(_state.t, imu_receive_time);
	out.reset_counter = _reset_counter;
	out.initialized = _ekf->healthy() && _state.valid() && std::isfinite(_last_rtk_time);
	out.rtk_fixed = _rtk.fixed;
	out.heading_valid = _rtk.heading_valid;
	out.accuracy_ok = _rtk.accuracy_ok;
	out.synchronized = _rtk.synchronized;
	out.imu_ready = _imu_ready;
	out.navigation_accuracy_ok = _rtk.accuracy_ok;
	out.navigation_ready = out.navigation_valid && out.heading_valid && out.navigation_accuracy_ok && out.clock_aligned &&
	                       _last_navigation_attempt == _last_rtk_time && !_gnss_vertical_recovery_pending;
	const auto quality = _ekf->navigationQuality();
	if (std::isfinite(quality.stamp)) out.covariance_stamp = rosStamp(quality.stamp);
	for (int i = 0; i < 3; ++i) {
		out.position_variance[i] = quality.position_variance(i);
		out.velocity_variance[i] = quality.velocity_variance(i);
	}
	out.heading_variance = quality.heading_variance;
	out.navigation_innovation_squared = _weighted_height_mode ? quality.horizontal.innovation_squared : quality.innovation_squared;
	out.navigation_rejections = quality.rejected_updates;
	out.navigation_accepted_updates = _accepted_navigation_updates;
	out.estimator_ready =
	        out.initialized && out.imu_ready && out.navigation_ready &&
	        (_navigation_source == "rtk" || (_accepted_navigation_updates >= _navigation_ready_updates && covarianceReady(quality)));
	out.readiness_reason = _readiness_reason;
	if (out.initialized) {
		if (_navigation_source == "gnss" && !_navigation.observation_valid) {
			out.readiness_reason = _readiness_reason;
		} else if (!out.navigation_valid) {
			out.readiness_reason = "Navigation unavailable, stale or clock alignment lost";
		} else if (_gnss_vertical_recovery_pending) {
			out.readiness_reason = "GNSS vertical recovery unavailable; waiting for accepted 3D navigation or barometer";
		} else if (!out.navigation_accuracy_ok) {
			out.readiness_reason = "GNSS accuracy unknown, above limits, or flight accuracy limits unconfigured";
		} else if (out.navigation_ready && _navigation_source == "gnss" && !covarianceReady(quality)) {
			out.readiness_reason = "Estimator covariance invalid, above limits, or flight covariance limits unconfigured";
		} else if (out.estimator_ready) {
			out.readiness_reason = "Navigation and estimator satisfy configured readiness checks";
		}
	}
	_fused_pub->publish(out);
	if (!out.initialized) return;

	// Keep the established evaluation topic, with standard body-frame twist.
	nav_msgs::msg::Odometry odometry;
	odometry.header = out.header;
	odometry.child_frame_id = "base_link";
	odometry.pose.pose.position = out.position;
	odometry.pose.pose.orientation = out.orientation;
	const auto velocity_body = _state.q().conjugate() * _state.v;
	odometry.twist.twist.linear.x = velocity_body.x();
	odometry.twist.twist.linear.y = velocity_body.y();
	odometry.twist.twist.linear.z = velocity_body.z();
	odometry.twist.twist.angular = out.body_rates;
	_state_pub->publish(odometry);
}

}  // namespace agi_ros2
