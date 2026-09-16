#ifndef AGI_ROS2_COMMAND_OUTPUT_NODE_H_
#define AGI_ROS2_COMMAND_OUTPUT_NODE_H_

#include <netinet/in.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "agi_ros2/msg/authority.hpp"
#include "agi_ros2/msg/control_command.hpp"
#include "agi_ros2/msg/health.hpp"
#include "agi_ros2/msg/output_status.hpp"
#include "agi_ros2/msp_telemetry.h"
#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include "agilib/bridge/betaflight/betaflight_rc_mapper.hpp"
#include "agilib/bridge/betaflight/hardware_safety.hpp"
#include "agilib/bridge/betaflight/thrust_table.hpp"
#include "agilib/bridge/betaflight_udp/betaflight_udp_bridge_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agi_ros2 {

// A single-threaded executor owns transport construction, I/O and destruction.
class CommandOutputNode final : public rclcpp::Node {
public:
	CommandOutputNode();
	~CommandOutputNode() override;

private:
	void loadThrustTable(const std::string& filename);
	void onCommand(msg::ControlCommand::ConstSharedPtr message);
	void processOutput();
	void watchdog();
	void publishStatus();
	void reportFault(const std::string& reason);
	std::array<uint16_t, 4> mapCommand() const;
	bool sendUdp(const std::array<uint16_t, 4>& channels, bool armed);

	const std::string _clock_id;
	const double _session_start;
	std::string _mode;
	bool _simulation_time = false;
	bool _timing_checks = true;
	double _mass = 0.0;
	int _socket_fd = -1;
	sockaddr_in _destination{};
	agi::BetaflightUdpBridgeParams _bridge_params;
	std::unique_ptr<agi::BetaflightRcMapper> _mapper;
	std::unique_ptr<agi::hardware::BetaflightMspBridge> _msp;
	std::unique_ptr<MspTelemetry> _telemetry;
	rclcpp::TimerBase::SharedPtr _msp_timer;
	std::unique_ptr<agi::hardware::ThrustTable> _thrust;
	agi::hardware::SafetyGate _gate;
	msg::ControlCommand _command;
	msg::Authority _authority;
	msg::Health _health;
	double _command_receive_time;
	double _authority_receive_time;
	double _health_receive_time;
	double _previous_command_time;
	double _previous_ros_time;
	bool _transport_healthy = true;
	bool _override_active = false;
	uint64_t _fault_count = 0;
	std::string _reason = "Waiting for control and authority";
	rclcpp::Subscription<msg::ControlCommand>::SharedPtr _command_sub;
	rclcpp::Subscription<msg::Authority>::SharedPtr _authority_sub;
	rclcpp::Subscription<msg::Health>::SharedPtr _health_sub;
	rclcpp::Publisher<msg::OutputStatus>::SharedPtr _status_pub;
	rclcpp::TimerBase::SharedPtr _watchdog;
};

}  // namespace agi_ros2

#endif  // AGI_ROS2_COMMAND_OUTPUT_NODE_H_
