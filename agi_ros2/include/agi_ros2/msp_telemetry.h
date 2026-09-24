#ifndef AGI_ROS2_MSP_TELEMETRY_H_
#define AGI_ROS2_MSP_TELEMETRY_H_

#include <array>
#include <vector>

#include "agi_ros2/msg/msp_event.hpp"
#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace agi_ros2 {
// Shares the output node's single-threaded serial owner. Never opens a second fd.
class MspTelemetry {
public:
	explicit MspTelemetry(rclcpp::Node& node);
	void tick(agi::hardware::BetaflightMspBridge& bridge, double write_deadline, double write_budget = .002);
	void sentRc(const std::array<uint16_t, 4>& channels, uint64_t errors);
	bool healthy() const { return _healthy; }

private:
	struct Poll {
		uint16_t code;
		double hz, next, sent{0};
		bool pending{false};
		rclcpp::Publisher<msg::MspEvent>::SharedPtr publisher;
		std::string setting;
		builtin_interfaces::msg::Time stamp;
		bool critical{false};
	};
	void emit(const std::string& event, const agi::hardware::MspFrame& frame, uint64_t errors, double latency = NAN,
	          const Poll* request = nullptr);
	void expireRequests(double now, uint64_t errors);
	rclcpp::Node& _node;
	std::vector<Poll> _polls;
	rclcpp::Publisher<msg::MspEvent>::SharedPtr _events;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _config;
	double _next_config{0};
	double _timeout;
	uint64_t _timeouts{0};
	bool _healthy{true};
	std::string _session_id;
};
}  // namespace agi_ros2

#endif  // AGI_ROS2_MSP_TELEMETRY_H_
