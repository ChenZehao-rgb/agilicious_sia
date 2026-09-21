#pragma once
#include <thread>

#include "agilib/bridge/betaflight/hardware_safety.hpp"
#include "agilib/pilot/pilot.hpp"
#include "agilib/reference/captured_reference.h"

namespace agi::hardware {
struct ControlDecision {
	Command command;
	QuadState reference;
	Evidence evidence;
	Mode mode{Mode::Boot};
	std::string reason{"boot"};
	bool permit_override{false};
	bool state_valid{false};
	bool trajectory_active{false};
	double reference_elapsed{0};
};

// Single-owner 100 Hz coordination, without device I/O or automatic takeoff.
class HardwarePilot {
public:
	explicit HardwarePilot(const PilotParams& params, TimeFunction clock, TimeFunction steady_clock = {},
	                       NavigationPolicy policy = NavigationPolicy::Rtk);
	bool setTrajectory(const SetpointVector& relative_setpoints);
	ControlDecision tick(const QuadState& fused_state, Evidence evidence, bool shadow_only = false, bool navigation_valid = false);
	void reportOutputFault();
	Mode mode() const { return _gate.mode(); }
	unsigned warmCycles() const { return _warm_cycles; }

private:
	void checkOwner() const;
	const TimeFunction _clock;
	const TimeFunction _steady_clock;
	const NavigationPolicy _navigation_policy;
	const std::thread::id _owner;
	std::unique_ptr<Pilot> _pilot;
	std::shared_ptr<BridgeBase> _sink;
	std::shared_ptr<CapturedReference> _reference;
	SafetyGate _gate;
	unsigned _warm_cycles{0};
	bool _reference_registered{false};
	bool _reference_active{false};
	bool _previous_auto{true};
	bool _shadow_low_seen{false};
	double _previous_tick{NAN};
	double _reference_start{NAN};
};
}  // namespace agi::hardware
