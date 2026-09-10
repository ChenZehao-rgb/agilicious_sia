#include "agi_ros2/control_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>

#include "agi_ros2/node_common.h"
#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include "agilib/reference/trajectory_csv.hpp"

namespace agi_ros2 {
namespace {
constexpr double kUnknownTime = std::numeric_limits<double>::quiet_NaN();
using agi::hardware::monotonicSeconds;
using agi::hardware::SafetyGate;
}  // namespace

ControlNode::ControlNode()
    : Node("flight_control"),
      clock_id_(ReadClockId()),
      previous_clock_(kUnknownTime),
      authority_receive_time_(kUnknownTime),
      health_receive_time_(kUnknownTime),
      output_receive_time_(kUnknownTime) {
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
  const auto trajectory = declare_parameter<std::string>("trajectory", "");
  params_ = std::make_unique<agi::PilotParams>(
      std::filesystem::path(params_dir) / pilot_file, params_dir);
  pilot_ = std::make_unique<agi::hardware::HardwarePilot>(
      *params_, [this] { return control_time_; }, monotonicSeconds);
  if (!trajectory.empty()) {
    const auto rows = agi::trajectory_csv::readTrajectoryRows(trajectory);
    const auto points = agi::trajectory_csv::loadTrajectory(
        rows, 0, agi::Vector<3>::Zero(), 0,
        agi::trajectory_csv::estimateSourceMass(rows),
        -std::numeric_limits<double>::infinity());
    if (!pilot_->setTrajectory(points)) {
      throw std::invalid_argument("Invalid trajectory timestamps or states");
    }
  }
  state_sub_ = create_subscription<msg::FusedState>(
      "fused_state", 1,
      std::bind(&ControlNode::OnState, this, std::placeholders::_1));
  authority_sub_ = create_subscription<msg::Authority>(
      "authority", 1, [this](msg::Authority::ConstSharedPtr message) {
        authority_ = *message;
        authority_receive_time_ = monotonicSeconds();
      });
  health_sub_ = create_subscription<msg::Health>(
      "health", 1, [this](msg::Health::ConstSharedPtr message) {
        health_ = *message;
        health_receive_time_ = monotonicSeconds();
      });
  output_sub_ = create_subscription<msg::OutputStatus>(
      "output_status", 1,
      std::bind(&ControlNode::OnOutputStatus, this, std::placeholders::_1));
  command_pub_ = create_publisher<msg::ControlCommand>("control_command", 1);
  reference_pub_ = create_publisher<nav_msgs::msg::Odometry>("reference", 1);
  diagnostic_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "control_diagnostics", 10);
  status_pub_ = create_publisher<std_msgs::msg::String>("status", 1);
  timer_ = create_wall_timer(std::chrono::milliseconds(10),
                             std::bind(&ControlNode::Tick, this));
}

void ControlNode::OnState(msg::FusedState::ConstSharedPtr message) {
  if (message->clock_id != clock_id_) {
    return;
  }
  if (state_.reset_counter != 0 &&
      message->reset_counter != state_.reset_counter) {
    output_fault_ = true;
  }
  state_ = *message;
}

void ControlNode::OnOutputStatus(msg::OutputStatus::ConstSharedPtr message) {
  if (message->clock_id != clock_id_) {
    return;
  }
  if (!output_.clock_id.empty() &&
      (output_.session_start != message->session_start ||
       output_.fault_count != message->fault_count)) {
    output_fault_ = true;
  }
  output_ = *message;
  output_receive_time_ = monotonicSeconds();
}

void ControlNode::Tick() {
  control_time_ =
      AlignedRosTime(*this, std::max({StampSeconds(state_.header.stamp),
                                      StampSeconds(authority_.header.stamp),
                                      StampSeconds(health_.header.stamp)}));
  const double wall = monotonicSeconds();
  if (output_fault_ || (std::isfinite(previous_clock_) &&
                        (control_time_ < previous_clock_ ||
                         control_time_ - previous_clock_ > 0.25))) {
    pilot_->reportOutputFault();
    output_fault_ = false;
  }
  previous_clock_ = control_time_;

  agi::QuadState state;
  state.setZero();
  state.t = kUnknownTime;
  if (state_.initialized && state_.clock_id == clock_id_ &&
      state_.header.frame_id == "odom" &&
      SafetyGate::fresh(wall, state_.published_steady_time, 0.010)) {
    state.t = StampSeconds(state_.header.stamp);
    state.p =
        agi::Vector<3>{state_.position.x, state_.position.y, state_.position.z};
    state.v =
        agi::Vector<3>{state_.velocity.x, state_.velocity.y, state_.velocity.z};
    state.q(agi::Quaternion(state_.orientation.w, state_.orientation.x,
                            state_.orientation.y, state_.orientation.z));
    state.w = agi::Vector<3>{state_.body_rates.x, state_.body_rates.y,
                             state_.body_rates.z};
    state.a = agi::Vector<3>{state_.acceleration.x, state_.acceleration.y,
                             state_.acceleration.z};
  }
  agi::hardware::Evidence evidence;
  evidence.now = wall;
  evidence.imu_time =
      state_.clock_id == clock_id_ ? state_.imu_receive_time : kUnknownTime;
  evidence.rtk_time =
      state_.initialized
          ? wall - (control_time_ - StampSeconds(state_.rtk_stamp))
          : kUnknownTime;
  evidence.rc_time =
      wall - (control_time_ - StampSeconds(authority_.header.stamp));
  evidence.rc_link = authority_.rc_link &&
                     SafetyGate::fresh(wall, authority_receive_time_, 0.1);
  evidence.armed = authority_.armed;
  evidence.auto_switch = authority_.auto_switch;
  evidence.kill = authority_.kill;
  evidence.rtk_fixed =
      state_.rtk_fixed && SafetyGate::fresh(wall, state_.rtk_receive_time, 0.3);
  evidence.heading_valid = state_.heading_valid;
  evidence.accuracy_ok = state_.accuracy_ok;
  evidence.synchronized = state_.synchronized;
  const bool health_fresh =
      SafetyGate::fresh(wall, health_receive_time_, 0.2) &&
      SafetyGate::fresh(control_time_, StampSeconds(health_.header.stamp), 0.2);
  const bool output_fresh =
      SafetyGate::fresh(wall, output_receive_time_, 0.05) &&
      SafetyGate::fresh(wall, output_.steady_time, 0.05);
  evidence.imu_calibrated = health_fresh && health_.imu_calibrated;
  evidence.converged = health_fresh && health_.converged && state_.initialized;
  evidence.config_verified = health_fresh && health_.config_verified;
  evidence.thrust_calibrated = health_fresh && health_.thrust_calibrated &&
                               output_fresh && output_.thrust_calibrated;
  evidence.geofence_ok = health_fresh && health_.geofence_ok;
  evidence.msp_healthy = health_fresh && health_.transport_healthy &&
                         output_fresh && output_.transport_healthy;
  PublishDecision(pilot_->tick(state, evidence), state);
}

void ControlNode::PublishDecision(
    const agi::hardware::ControlDecision& decision,
    const agi::QuadState& state) {
  msg::ControlCommand command;
  command.header.stamp = now();
  command.header.frame_id = "base_link";
  command.clock_id = clock_id_;
  command.sequence = ++sequence_;
  // Agilib stores collective thrust as acceleration; expose physical N on ROS.
  command.total_thrust = decision.command.collective_thrust * params_->quad_.m_;
  command.body_rates.x = decision.command.omega.x();
  command.body_rates.y = decision.command.omega.y();
  command.body_rates.z = decision.command.omega.z();
  command.permit_override = decision.permit_override;
  command.mode = static_cast<uint8_t>(decision.mode);
  command.evidence = EncodeEvidence(decision.evidence);
  command_pub_->publish(command);

  std_msgs::msg::Float64MultiArray diagnostic;
  const auto& evidence = decision.evidence;
  diagnostic.data = {control_time_,
                     evidence.now,
                     evidence.now - evidence.imu_time,
                     control_time_ - state.t,
                     control_time_ - StampSeconds(state_.rtk_stamp),
                     evidence.now - evidence.rc_time,
                     evidence.solve_seconds,
                     static_cast<double>(pilot_->warmCycles()),
                     decision.permit_override ? 1.0 : 0.0,
                     static_cast<double>(decision.mode)};
  diagnostic_pub_->publish(diagnostic);
  std_msgs::msg::String status;
  status.data =
      std::string(SafetyGate::name(decision.mode)) + ": " + decision.reason;
  status_pub_->publish(status);
  if (decision.reference.valid()) {
    const auto& reference = decision.reference;
    nav_msgs::msg::Odometry message;
    message.header.stamp = RosStamp(reference.t);
    message.header.frame_id = "odom";
    message.pose.pose.position.x = reference.p.x();
    message.pose.pose.position.y = reference.p.y();
    message.pose.pose.position.z = reference.p.z();
    message.pose.pose.orientation.w = reference.q().w();
    message.pose.pose.orientation.x = reference.q().x();
    message.pose.pose.orientation.y = reference.q().y();
    message.pose.pose.orientation.z = reference.q().z();
    reference_pub_->publish(message);
  }
}

}  // namespace agi_ros2
