#pragma once
#include <thread>
#include "agilib/pilot/pilot.hpp"
#include "agilib/bridge/betaflight/hardware_safety.hpp"

namespace agi::hardware {
struct ControlDecision {
  Command command; // Invalid unless this tick completed successfully.
  Evidence evidence;
  Mode mode{Mode::Boot};
  std::string reason{"boot"};
  bool permit_override{false};
};

// Single-owner 100 Hz coordination step: fused ENU/FLU state -> Pilot -> MPC
// -> safety FSM. No device I/O, automatic arming, takeoff or fake sensors.
// A scheduler calls tick(); the serial worker consumes its latest decision.
class HardwarePilot {
 public:
  explicit HardwarePilot(const PilotParams& params,
                         TimeFunction clock, TimeFunction steady_clock = {});
  // Relative SI setpoints, executed only after a new authorized AUTO edge.
  bool setTrajectory(const SetpointVector& relative_setpoints);
  ControlDecision tick(const QuadState& fused_state, Evidence evidence);
  // Latch an output/transport/mapping fault into the SAME FSM before the next
  // tick. Recovery requires healthy warmup and a new physical AUTO low->high.
  void reportOutputFault();
  Mode mode() const { return gate_.mode(); }
  unsigned warmCycles() const { return warm_cycles_; }
  // Deliberately no public Pilot access: callers cannot enable a physical
  // bridge or insert a takeoff reference behind the hardware supervisor.
 private:
  bool resetHover(const QuadState& state, double now);
  void checkOwner() const;
  const TimeFunction clock_;
  const TimeFunction steady_clock_;
  SetpointVector trajectory_;
  const std::thread::id owner_;
  std::unique_ptr<Pilot> pilot_;
  std::shared_ptr<BridgeBase> sink_;
  SafetyGate gate_;
  unsigned warm_cycles_{0};
  bool reference_ready_{false};
  bool previous_auto_{true}; // startup high is not a rising edge
  double previous_tick_{NAN};
};
}
