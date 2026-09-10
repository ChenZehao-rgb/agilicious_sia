#include "agi_ros2/command_output_node.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "agi_ros2/node_common.h"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/yaml.hpp"

namespace agi_ros2 {
namespace {
constexpr double kUnknownTime = std::numeric_limits<double>::quiet_NaN();
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
constexpr double kGravity = 9.8066;
constexpr std::array<uint16_t, 4> kIdleChannels = {1500, 1500, 1000, 1500};
using agi::hardware::monotonicSeconds;
using agi::hardware::SafetyGate;
}  // namespace

CommandOutputNode::CommandOutputNode()
    : Node("command_output"),
      clock_id_(ReadClockId()),
      session_start_(monotonicSeconds()),
      command_receive_time_(kUnknownTime),
      authority_receive_time_(kUnknownTime),
      health_receive_time_(kUnknownTime),
      previous_command_time_(kUnknownTime) {
  mode_ = declare_parameter<std::string>("mode", "sitl");
  if (mode_ != "sitl" && mode_ != "hardware") {
    throw std::invalid_argument("mode must be sitl or hardware");
  }
  if (mode_ == "hardware" && get_parameter("use_sim_time").as_bool()) {
    throw std::invalid_argument("Hardware requires use_sim_time=false");
  }
  const auto params_dir = declare_parameter<std::string>("params_dir", "");
  const auto pilot_file =
      declare_parameter<std::string>("pilot_config", "pilot_ros2.yaml");
  const auto bridge_file =
      declare_parameter<std::string>("bridge_config", "betaflight_udp.yaml");
  const auto device = declare_parameter<std::string>("device", "/dev/ttyAMA0");
  const int baud = declare_parameter<int>("baud", 921600);
  const auto thrust_file = declare_parameter<std::string>("thrust_table", "");
  if (!bridge_params_.load(std::filesystem::path(params_dir) / bridge_file)) {
    throw std::invalid_argument("Invalid Betaflight channel mapping");
  }
  const agi::Yaml pilot_config(std::filesystem::path(params_dir) / pilot_file);
  const auto quad_file = pilot_config["quadrotor"].as<std::string>();
  agi::Quadrotor quad;
  if (!quad.load(std::filesystem::path(params_dir) / "quads" / quad_file) ||
      !quad.valid()) {
    throw std::invalid_argument("Invalid vehicle mass configuration");
  }
  mass_ = quad.m_;
  mapper_ = std::make_unique<agi::BetaflightRcMapper>(bridge_params_);
  if (!thrust_file.empty()) {
    LoadThrustTable(thrust_file);
  }
  if (mode_ == "hardware") {
    if (!thrust_) {
      throw std::invalid_argument("Hardware output requires a thrust_table");
    }
    msp_ = std::make_unique<agi::hardware::BetaflightMspBridge>(device, baud);
  } else {
    destination_.sin_family = AF_INET;
    destination_.sin_port = htons(bridge_params_.port);
    if (inet_pton(AF_INET, bridge_params_.host.c_str(),
                  &destination_.sin_addr) != 1) {
      throw std::invalid_argument("Invalid UDP destination address");
    }
    socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error("Cannot open SITL UDP socket");
    }
  }
  status_pub_ = create_publisher<msg::OutputStatus>("output_status", 1);
  command_sub_ = create_subscription<msg::ControlCommand>(
      "control_command", 1,
      std::bind(&CommandOutputNode::OnCommand, this, std::placeholders::_1));
  authority_sub_ = create_subscription<msg::Authority>(
      "authority", 1, [this](msg::Authority::ConstSharedPtr message) {
        const bool revoke = message->kill || !message->armed ||
                            message->auto_switch != authority_.auto_switch ||
                            !message->rc_link;
        authority_ = *message;
        authority_receive_time_ = monotonicSeconds();
        if (revoke) {
          ProcessOutput();
        }
      });
  health_sub_ = create_subscription<msg::Health>(
      "health", 1, [this](msg::Health::ConstSharedPtr message) {
        health_ = *message;
        health_receive_time_ = monotonicSeconds();
      });
  watchdog_ = create_wall_timer(std::chrono::milliseconds(5),
                                std::bind(&CommandOutputNode::Watchdog, this));
}

CommandOutputNode::~CommandOutputNode() {
  if (socket_fd_ >= 0) {
    SendUdp(kIdleChannels, false);
    close(socket_fd_);
  }
  // MSP destruction only closes the serial port. Never write ARM/AUX or an
  // invented replacement throttle to physical hardware on shutdown.
}

void CommandOutputNode::LoadThrustTable(const std::string& filename) {
  std::ifstream input(filename);
  if (!input) {
    throw std::invalid_argument("Cannot open thrust calibration table");
  }
  std::vector<double> voltages;
  std::vector<double> pwm;
  std::vector<std::vector<double>> forces;
  std::string line;
  bool first = true;
  while (std::getline(input, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row_stream(line);
    double value;
    std::vector<double> row;
    while (row_stream >> value) {
      row.push_back(value);
    }
    if (row.size() < 3) {
      throw std::invalid_argument("Invalid thrust calibration row");
    }
    if (first) {
      pwm.assign(row.begin() + 1, row.end());
      first = false;
    } else {
      voltages.push_back(row.front());
      forces.emplace_back(row.begin() + 1, row.end());
    }
  }
  thrust_ = std::make_unique<agi::hardware::ThrustTable>(voltages, pwm, forces);
}

void CommandOutputNode::OnCommand(msg::ControlCommand::ConstSharedPtr message) {
  if (message->clock_id != clock_id_ || !std::isfinite(message->evidence.now) ||
      (std::isfinite(previous_command_time_) &&
       message->evidence.now <= previous_command_time_)) {
    return;
  }
  previous_command_time_ = message->evidence.now;
  command_ = *message;
  command_receive_time_ = monotonicSeconds();
  ProcessOutput();
}

std::array<uint16_t, 4> CommandOutputNode::MapCommand() const {
  if (!std::isfinite(command_.total_thrust) || command_.total_thrust < 0 ||
      !std::isfinite(command_.body_rates.x) ||
      !std::isfinite(command_.body_rates.y) ||
      !std::isfinite(command_.body_rates.z)) {
    throw std::invalid_argument("Nonfinite or negative thrust/rates command");
  }
  std::array<uint16_t, 4> channels = kIdleChannels;
  const double sign = mode_ == "hardware" ? -1.0 : 1.0;
  channels[0] = mapper_->rateToPwm(
      mapper_->inverseActualRate(command_.body_rates.x * kRadiansToDegrees, 0),
      bridge_params_.deadband);
  // Hardware is FRD. The SITL model already flips its sensor axes.
  channels[1] = mapper_->rateToPwm(
      mapper_->inverseActualRate(
          sign * command_.body_rates.y * kRadiansToDegrees, 1),
      bridge_params_.deadband);
  channels[3] = mapper_->rateToPwm(
      mapper_->inverseActualRate(
          sign * command_.body_rates.z * kRadiansToDegrees, 2),
      bridge_params_.yaw_deadband);
  const double acceleration = command_.total_thrust / mass_;
  if (msp_) {
    channels[2] = thrust_->collectiveThrustToRc(acceleration, mass_,
                                                health_.battery_voltage);
  } else {
    const double motor =
        (bridge_params_.motor_idle +
         (1 - bridge_params_.motor_idle) * bridge_params_.hover_throttle) *
        std::sqrt(acceleration / kGravity);
    channels[2] = static_cast<uint16_t>(
        std::lround(bridge_params_.min_check +
                    (2000 - bridge_params_.min_check) *
                        std::clamp((motor - bridge_params_.motor_idle) /
                                       (1 - bridge_params_.motor_idle),
                                   0.0, 1.0)));
  }
  return channels;
}

void CommandOutputNode::ReportFault(const std::string& reason) {
  ++fault_count_;
  reason_ = reason;
}

void CommandOutputNode::ProcessOutput() {
  const double ros_time =
      AlignedRosTime(*this, std::max({StampSeconds(authority_.header.stamp),
                                      StampSeconds(health_.header.stamp),
                                      StampSeconds(command_.header.stamp)}));
  auto evidence = DecodeEvidence(command_.evidence);
  evidence.now = monotonicSeconds();
  const bool command_fresh =
      command_.clock_id == clock_id_ &&
      command_.header.frame_id == "base_link" &&
      SafetyGate::fresh(evidence.now, command_receive_time_, 0.025) &&
      SafetyGate::fresh(evidence.now, command_.evidence.now, 0.025) &&
      SafetyGate::fresh(ros_time, StampSeconds(command_.header.stamp), 0.025);
  const bool rc_fresh =
      SafetyGate::fresh(evidence.now, authority_receive_time_, 0.1) &&
      SafetyGate::fresh(ros_time, StampSeconds(authority_.header.stamp), 0.1) &&
      authority_.rc_link;
  const bool health_fresh =
      SafetyGate::fresh(evidence.now, health_receive_time_, 0.2) &&
      SafetyGate::fresh(ros_time, StampSeconds(health_.header.stamp), 0.2);
  // New authority can revoke old commands immediately, but never authorize a
  // command computed under a different ARM/AUTO state.
  const bool authority_matches = evidence.armed == authority_.armed &&
                                 evidence.auto_switch == authority_.auto_switch;
  // Authority and command travel on different DDS topics. A new AUTO edge
  // may arrive before its matching control decision (or vice versa). Wait for
  // that pair without consuming the healthy low edge. Never defer a revocation
  // while output is active, and never extend the command's original deadline.
  if (command_fresh && !authority_matches && !override_active_ && rc_fresh &&
      authority_.armed && !authority_.kill) {
    PublishStatus();
    return;
  }
  evidence.rc_link = evidence.rc_link && rc_fresh;
  evidence.kill = evidence.kill || authority_.kill;
  evidence.armed = evidence.armed && authority_.armed;
  evidence.auto_switch = authority_.auto_switch;
  evidence.command_valid =
      evidence.command_valid && command_fresh && authority_matches &&
      (!authority_.auto_switch || command_.permit_override);
  evidence.msp_healthy = evidence.msp_healthy && transport_healthy_ &&
                         health_fresh && health_.transport_healthy;
  evidence.config_verified =
      evidence.config_verified && health_fresh && health_.config_verified;
  evidence.geofence_ok =
      evidence.geofence_ok && health_fresh && health_.geofence_ok;
  evidence.thrust_calibrated =
      evidence.thrust_calibrated && health_fresh && health_.thrust_calibrated;
  bool active = gate_.update(evidence) && command_.permit_override;
  auto channels = kIdleChannels;
  if (active) {
    try {
      channels = MapCommand();
    } catch (const std::exception& error) {
      ReportFault(error.what());
      evidence.command_valid = false;
      gate_.update(evidence);
      active = false;
    }
  }
  if (override_active_ && !active && authority_.auto_switch &&
      !authority_.kill) {
    ReportFault("Output authorization or command expired");
  }
  if (authority_.auto_switch && !active) {
    evidence.command_valid = false;
  }
  if (msp_) {
    // The MSP bridge has its own gate, which must also observe healthy AUTO
    // low.
    const uint64_t previous_errors = msp_->errors();
    const bool sent =
        msp_->sendOverride(channels, evidence, monotonicSeconds() + 0.003);
    if (active && !sent) {
      ReportFault("MSP output rejected or write failed");
      if (msp_->errors() != previous_errors) {
        transport_healthy_ = false;
      }
      active = false;
    }
  } else {
    if (!active && rc_fresh && !authority_.kill && !authority_.auto_switch) {
      channels = authority_.manual_aetr;
    }
    const bool armed = rc_fresh && !authority_.kill && authority_.armed &&
                       (!authority_.auto_switch || active);
    if (!SendUdp(channels, armed)) {
      if (transport_healthy_) {
        ReportFault("UDP send failed");
      }
      transport_healthy_ = false;
      active = false;
    }
  }
  override_active_ = active;
  if (active) {
    reason_ = "Authorized output";
  }
  PublishStatus();
}

void CommandOutputNode::Watchdog() {
  // Use wall time: paused /clock or a stopped controller cannot keep output
  // alive. The original sensor/command times are never refreshed here.
  const double wall = monotonicSeconds();
  if (!SafetyGate::fresh(wall, command_receive_time_, 0.025) ||
      !SafetyGate::fresh(wall, command_.evidence.now, 0.025)) {
    ProcessOutput();
  } else {
    PublishStatus();
  }
}

void CommandOutputNode::PublishStatus() {
  msg::OutputStatus status;
  status.header.stamp = now();
  status.clock_id = clock_id_;
  status.steady_time = monotonicSeconds();
  status.transport_healthy = transport_healthy_;
  status.thrust_calibrated = mode_ == "sitl" || static_cast<bool>(thrust_);
  status.fault_count = fault_count_;
  status.session_start = session_start_;
  status.override_active = override_active_;
  status.reason = reason_;
  status_pub_->publish(status);
}

bool CommandOutputNode::SendUdp(const std::array<uint16_t, 4>& channels,
                                bool armed) {
  std::array<uint8_t, 40> bytes{};
  const double time = monotonicSeconds();
  uint64_t bits;
  static_assert(sizeof(bits) == sizeof(time));
  std::memcpy(&bits, &time, sizeof(bits));
  for (int i = 0; i < 8; ++i) {
    bytes[i] = (bits >> (8 * i)) & 255;
  }
  for (int i = 0; i < 16; ++i) {
    uint16_t value = i < 4 ? channels[i] : 1000;
    if (i == 4 && armed) {
      value = 2000;
    }
    if (value < 1000 || value > 2000) {
      value = 1000;
    }
    bytes[8 + 2 * i] = value & 255;
    bytes[9 + 2 * i] = value >> 8;
  }
  return sendto(socket_fd_, bytes.data(), bytes.size(), 0,
                reinterpret_cast<const sockaddr*>(&destination_),
                sizeof(destination_)) == static_cast<ssize_t>(bytes.size());
}

}  // namespace agi_ros2
