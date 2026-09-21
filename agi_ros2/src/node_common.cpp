#include "agi_ros2/node_common.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <thread>

#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"

namespace agi_ros2 {

std::string readClockId() {
	std::ifstream input("/proc/sys/kernel/random/boot_id");
	std::string id;
	if (!(input >> id)) {
		throw std::runtime_error("Cannot identify the host monotonic clock domain");
	}
	return id;
}

std::string evidenceClockId(const std::string& host_id, bool simulation) { return simulation ? host_id + ":ros" : host_id; }

double stampSeconds(const builtin_interfaces::msg::Time& stamp) { return stamp.sec + stamp.nanosec * 1e-9; }

builtin_interfaces::msg::Time rosStamp(double seconds) { return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9))); }

double alignedRosTime(rclcpp::Node& node, double newest) {
	double now = node.now().seconds();
	if (newest > now && newest - now <= 0.010) {
		const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(3);
		while (now < newest && std::chrono::steady_clock::now() < deadline) {
			std::this_thread::sleep_for(std::chrono::microseconds(100));
			now = node.now().seconds();
		}
	}
	return now;
}

msg::SafetyEvidence encodeEvidence(const agi::hardware::Evidence& evidence) {
	msg::SafetyEvidence message;
	message.now = evidence.now;
	message.imu_time = evidence.imu_time;
	message.rtk_time = evidence.rtk_time;
	message.rc_time = evidence.rc_time;
	message.command_time = evidence.command_time;
	message.solve_seconds = evidence.solve_seconds;
	message.rtk_fixed = evidence.rtk_fixed;
	message.heading_valid = evidence.heading_valid;
	message.accuracy_ok = evidence.accuracy_ok;
	message.imu_calibrated = evidence.imu_calibrated;
	message.synchronized = evidence.synchronized;
	message.converged = evidence.converged;
	message.imu_ready = evidence.imu_ready;
	message.estimator_ready = evidence.estimator_ready;
	message.navigation_ready = evidence.navigation_ready;
	message.clock_aligned = evidence.clock_aligned;
	message.accuracy_known = evidence.accuracy_known;
	message.config_verified = evidence.config_verified;
	message.thrust_calibrated = evidence.thrust_calibrated;
	message.geofence_ok = evidence.geofence_ok;
	message.msp_healthy = evidence.msp_healthy;
	message.command_valid = evidence.command_valid;
	message.controller_warm = evidence.controller_warm;
	message.armed = evidence.armed;
	message.auto_switch = evidence.auto_switch;
	message.kill = evidence.kill;
	message.rc_link = evidence.rc_link;
	return message;
}

agi::hardware::Evidence decodeEvidence(const msg::SafetyEvidence& message) {
	agi::hardware::Evidence evidence;
	evidence.now = message.now;
	evidence.imu_time = message.imu_time;
	evidence.rtk_time = message.rtk_time;
	evidence.rc_time = message.rc_time;
	evidence.command_time = message.command_time;
	evidence.solve_seconds = message.solve_seconds;
	evidence.rtk_fixed = message.rtk_fixed;
	evidence.heading_valid = message.heading_valid;
	evidence.accuracy_ok = message.accuracy_ok;
	evidence.imu_calibrated = message.imu_calibrated;
	evidence.synchronized = message.synchronized;
	evidence.converged = message.converged;
	evidence.imu_ready = message.imu_ready;
	evidence.estimator_ready = message.estimator_ready;
	evidence.navigation_ready = message.navigation_ready;
	evidence.clock_aligned = message.clock_aligned;
	evidence.accuracy_known = message.accuracy_known;
	evidence.config_verified = message.config_verified;
	evidence.thrust_calibrated = message.thrust_calibrated;
	evidence.geofence_ok = message.geofence_ok;
	evidence.msp_healthy = message.msp_healthy;
	evidence.command_valid = message.command_valid;
	evidence.controller_warm = message.controller_warm;
	evidence.armed = message.armed;
	evidence.auto_switch = message.auto_switch;
	evidence.kill = message.kill;
	evidence.rc_link = message.rc_link;
	return evidence;
}

}  // namespace agi_ros2
