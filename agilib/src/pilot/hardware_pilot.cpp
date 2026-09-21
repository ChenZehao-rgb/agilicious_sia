#include "agilib/pilot/hardware_pilot.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "agilib/estimator/feedthrough/feedthrough_estimator.hpp"

namespace agi::hardware {
namespace {
class CommandSink final : public BridgeBase {
public:
	explicit CommandSink(const TimeFunction& clock) : BridgeBase("Hardware command sink", clock, 0.0, 0, false, false) {}

protected:
	bool sendCommand(const Command& command, bool) override { return command.isRatesThrust() && command.collective_thrust >= 0; }
};
}  // namespace

HardwarePilot::HardwarePilot(const PilotParams& params, TimeFunction clock, TimeFunction steady_clock, NavigationPolicy policy)
        : _clock(std::move(clock)),
          _steady_clock(steady_clock ? std::move(steady_clock) : _clock),
          _navigation_policy(policy),
          _owner(std::this_thread::get_id()),
          _reference(std::make_shared<CapturedReference>()),
          _gate(policy) {
	const auto& cfg = params.pipeline_cfg_;
	// Check before constructing Pilot, whose general-purpose constructor may open devices.
	if (!_clock || !params.valid() || cfg.bridge_cfg.type != "External" || cfg.estimator_cfg.type != "External" ||
	    cfg.outer_controller_cfg.type != "MPC" || cfg.sampler_cfg.type != "Time" ||
	    (!cfg.inner_controller_cfg.type.empty() && cfg.inner_controller_cfg.type != "None") ||
	    (!params.guard_cfg_.type.empty() && params.guard_cfg_.type != "None") || params.velocity_in_bodyframe_ ||
	    params.outerloop_divisor_ != 1 || !std::isfinite(params.dt_min_) || std::abs(params.dt_min_ - .01) > 1e-9) {
		throw ParameterException(
		        "HardwarePilot requires External estimator/bridge, MPC, Time, no inner controller/guard, "
		        "world velocity and dt=0.01");
	}
	_pilot = std::make_unique<Pilot>(params, _clock);
	if (!_pilot->registerExternalEstimator(std::make_shared<FeedthroughEstimator>(nullptr)))
		throw std::runtime_error("could not register fused-state handoff");
	_sink = std::make_shared<CommandSink>(_clock);
	if (!_pilot->registerExternalDebugBridge(_sink)) throw std::runtime_error("could not register command sink");
	_pilot->enable(false);
}

bool HardwarePilot::setTrajectory(const SetpointVector& points) {
	checkOwner();
	if (_reference_active || points.size() < 2 || points.front().state.t != 0) return false;
	double previous = -1;
	for (const auto& point : points) {
		if (!point.state.valid() || !point.input.valid() || point.state.t <= previous) return false;
		previous = point.state.t;
	}
	_reference->setTrajectory(std::make_shared<SampledTrajectory>(points));
	return true;
}

void HardwarePilot::checkOwner() const {
	if (_owner != std::this_thread::get_id()) throw std::logic_error("HardwarePilot must have one control-thread owner");
}

void HardwarePilot::reportOutputFault() {
	checkOwner();
	Evidence failed;
	failed.now = _steady_clock();
	_gate.update(failed);
	_previous_auto = true;
	_warm_cycles = 0;
	_reference_active = false;
	_reference_start = NAN;
	_shadow_low_seen = false;
}

ControlDecision HardwarePilot::tick(const QuadState& state, Evidence evidence, bool shadow_only, bool navigation_valid) {
	checkOwner();
	ControlDecision result;
	const double now = _clock();
	evidence.now = _steady_clock();
	evidence.command_valid = false;
	evidence.command_time = NAN;
	evidence.solve_seconds = NAN;
	evidence.controller_warm = _warm_cycles >= 50;
	const bool cadence_ok =
	        !std::isfinite(_previous_tick) || (now > _previous_tick && (!evidence.timing_checks || now - _previous_tick <= .025));
	_previous_tick = now;
	const bool state_ok =
	        state.valid() && SafetyGate::fresh(now, state.t, .010, evidence.timing_checks) && std::abs(state.q().norm() - 1.0) < 1e-3;
	result.state_valid = state_ok && cadence_ok && (!shadow_only || navigation_valid);
	const bool receiver_ok =
	        evidence.rc_link && SafetyGate::fresh(evidence.now, evidence.rc_time, .1, evidence.timing_checks) && !evidence.kill;
	const bool compute_healthy = shadow_only ? navigation_valid : SafetyGate::inputsHealthy(evidence, _navigation_policy);
	const bool ready = result.state_valid && compute_healthy && !evidence.kill;
	if (!ready || !receiver_ok || !evidence.armed || !evidence.auto_switch) {
		_reference_active = false;
		_reference_start = NAN;
	}
	if (!ready || !receiver_ok) _shadow_low_seen = false;
	if (shadow_only && ready && receiver_ok && evidence.controller_warm && !evidence.auto_switch) _shadow_low_seen = true;
	const bool candidate = !_reference_active && ready && receiver_ok && evidence.armed && evidence.auto_switch && !_previous_auto &&
	                       (shadow_only ? (_shadow_low_seen && evidence.controller_warm) : _gate.canEnterAuto(evidence));
	if (evidence.auto_switch) _shadow_low_seen = false;
	std::string compute_failure;
	if (!ready) {
		_warm_cycles = 0;
	} else {
		const auto begin = std::chrono::steady_clock::now();
		try {
			_pilot->odometryCallback(state);
			// Manual warmup follows the vehicle. Only a candidate AUTO edge freezes
			// position/yaw and starts relative time; no automatic takeoff API is used.
			if (!_reference_active) _reference->capture(state, now, candidate);
			if (!_reference_registered) _reference_registered = _pilot->addReference(_reference);
			if (_reference_registered && _pilot->runPipelineChecked(now)) {
				const auto& references = _pilot->getReferenceSetpoints();
				if (!references.empty()) result.reference = references.front().state;
				const Command command = _pilot->getCommand();
				if (command.isRatesThrust() && command.collective_thrust >= 0 &&
				    SafetyGate::fresh(now, command.t, .010, evidence.timing_checks)) {
					result.command = command;
					evidence.command_valid = true;
					evidence.command_time = _steady_clock();
				}
			} else {
				compute_failure = "reference registration or control pipeline failed";
			}
		} catch (const std::exception& error) {
			compute_failure = error.what();
			evidence.command_valid = false;
		}
		evidence.solve_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
		if (evidence.command_valid && (!evidence.timing_checks || evidence.solve_seconds <= .008)) {
			_warm_cycles = std::min(_warm_cycles + 1, 50u);
		} else {
			_warm_cycles = 0;
			result.command = Command();
			evidence.command_valid = false;
		}
	}
	_previous_auto = evidence.auto_switch;
	evidence.controller_warm = _warm_cycles >= 50;
	evidence.now = _steady_clock();
	const bool authorized = _gate.update(evidence);
	result.permit_override = !shadow_only && authorized && (_reference_active || candidate);
	const bool commit = candidate && evidence.command_valid && evidence.controller_warm && (shadow_only || authorized);
	if (commit) {
		_reference_active = true;
		_reference_start = now;
	}
	if (!evidence.command_valid || (!shadow_only && !result.permit_override)) {
		_reference_active = false;
		_reference_start = NAN;
		if (!evidence.command_valid) _shadow_low_seen = false;
	}
	if (!shadow_only && !SafetyGate::inputsHealthy(evidence, _navigation_policy)) _warm_cycles = 0;
	result.trajectory_active = _reference_active;
	result.reference_elapsed = _reference_active ? now - _reference_start : 0;
	result.evidence = evidence;
	result.mode = _gate.mode();
	result.reason = _gate.reason();
	if (!compute_failure.empty()) result.reason += "; " + compute_failure;
	if (!cadence_ok) result.reason += "; cadence";
	if (!state_ok) result.reason += "; state age/validity";
	if (shadow_only && !navigation_valid) result.reason += "; navigation unavailable";
	return result;
}
}  // namespace agi::hardware
