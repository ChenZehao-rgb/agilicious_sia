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
	// Empty only while the current state is ready for another online target.
	std::string goal_rejection{"AUTO reference inactive"};
	std::string goal_event;
};

// Single-owner 100 Hz coordination, without device I/O or automatic takeoff.
class HardwarePilot {
public:
	explicit HardwarePilot(const PilotParams& params, TimeFunction clock, TimeFunction steady_clock = {},
	                       NavigationPolicy policy = NavigationPolicy::Rtk, double observation_delay = 0.0,
	                       ReceiverPolicy receiver_policy = ReceiverPolicy::FreshSamples);
	bool setTrajectory(const SetpointVector& relative_setpoints);
	ControlDecision tick(const QuadState& fused_state, Evidence evidence, bool shadow_only = false, bool navigation_valid = false,
	                     bool computation_requested = true, const Vector<3>* go_to_position = nullptr);
	void reportOutputFault();
	Mode mode() const { return _gate.mode(); }
	unsigned warmCycles() const { return _warm_cycles; }

private:
	void checkOwner() const;
	std::string goalRejection(const QuadState& state, double now);
	// Actual speed and tracking error; no dwell time or additional navigation gate.
	static constexpr double kHoverSpeed = 0.2;          // m/s, Euclidean norm
	static constexpr double kHoverPositionError = 0.2;  // m, Euclidean norm
	const TimeFunction _clock;
	const TimeFunction _steady_clock;
	const NavigationPolicy _navigation_policy;
	const double _observation_delay;
	const ReceiverPolicy _receiver_policy;
	const double _go_to_pose_mean_vel;
	const std::thread::id _owner;
	std::unique_ptr<Pilot> _pilot;
	std::shared_ptr<BridgeBase> _sink;
	std::shared_ptr<CapturedReference> _reference;
	SafetyGate _gate;
	unsigned _warm_cycles{0};
	bool _reference_registered{false};
	bool _reference_active{false};
	bool _online_moving{false};
	bool _previous_auto{true};
	bool _shadow_low_seen{false};
	double _previous_tick{NAN};
	double _previous_fc_sample{NAN};
	std::string _fc_session;
	uint64_t _fc_generation{0};
	uint32_t _fc_reset_counter{0};
	double _reference_start{NAN};
};
}  // namespace agi::hardware
