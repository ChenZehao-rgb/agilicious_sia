#include "agilib/pilot/hardware_pilot.hpp"
#include "agilib/estimator/feedthrough/feedthrough_estimator.hpp"
#include "agilib/reference/hover_reference.hpp"
#include <chrono>
#include "agilib/reference/trajectory_reference/sampled_trajectory.hpp"
#include <algorithm>
#include <stdexcept>

namespace agi::hardware {
namespace {
class CommandSink final : public BridgeBase {
 public:
  explicit CommandSink(const TimeFunction& clock)
    : BridgeBase("Hardware command sink", clock, 0.0, 0, false) {
    voltage_watchdog_.disable();
  }
 protected:
  bool sendCommand(const Command& command, bool) override {
    return command.isRatesThrust() && command.collective_thrust >= 0;
  }
};
}
HardwarePilot::HardwarePilot(const PilotParams& params, TimeFunction clock, TimeFunction steady_clock)
  : clock_(std::move(clock)), steady_clock_(steady_clock ? std::move(steady_clock) : clock_), owner_(std::this_thread::get_id()) {
  const auto& cfg = params.pipeline_cfg_;
  // Check before constructing Pilot: its constructor may open physical
  // bridges based on YAML. Only the supervisor's serial worker may do that.
  if (!clock_ || !params.valid() || cfg.bridge_cfg.type != "External" ||
      cfg.estimator_cfg.type != "External" ||
      cfg.outer_controller_cfg.type != "MPC" ||
      cfg.sampler_cfg.type != "Time" ||
      (!cfg.inner_controller_cfg.type.empty() && cfg.inner_controller_cfg.type != "None") ||
      (!params.guard_cfg_.type.empty() && params.guard_cfg_.type != "None") ||
      params.velocity_in_bodyframe_ || params.outerloop_divisor_ != 1 ||
      !std::isfinite(params.dt_min_) || std::abs(params.dt_min_ - 0.01) > 1e-9)
    throw ParameterException("HardwarePilot requires External estimator/bridge, MPC, Time, no inner controller/guard, world velocity and dt=0.01");
  pilot_ = std::make_unique<Pilot>(params, clock_);
  // This is only a state handoff. Actual RTK/IMU fusion happens upstream.
  if (!pilot_->registerExternalEstimator(std::make_shared<FeedthroughEstimator>(nullptr)))
    throw std::runtime_error("could not register fused-state handoff");
  sink_ = std::make_shared<CommandSink>(clock_);
  if (!pilot_->registerExternalDebugBridge(sink_))
    throw std::runtime_error("could not register command sink");
  pilot_->enable(false); // computations enabled; physical bridge never enabled
}
bool HardwarePilot::setTrajectory(const SetpointVector& points) {
  checkOwner();
  if (points.size() < 2 || points.front().state.t != 0) return false;
  double previous = -1;
  for (const auto& p : points) {
    if (!p.state.valid() || !p.input.valid() || p.state.t <= previous) return false;
    previous = p.state.t;
  }
  trajectory_ = points;
  return true;
}
void HardwarePilot::checkOwner() const {
  if (owner_ != std::this_thread::get_id())
    throw std::logic_error("HardwarePilot must have one control-thread owner");
}
bool HardwarePilot::resetHover(const QuadState& state, double now) {
  pilot_->off(); // only clears references / deactivates the in-memory sink
  pilot_->enable(false);
  QuadState reference = state.getHoverState();
  reference.t = now;
  return pilot_->addReference(std::make_shared<HoverReference>(reference));
}
void HardwarePilot::reportOutputFault() {
	checkOwner();
	Evidence failed;
	failed.now = steady_clock_();
	gate_.update(failed);
	pilot_->off();
	previous_auto_ = true;
	warm_cycles_ = 0;
	reference_ready_ = false;
	_shadow_low_seen = false;
	_shadow_trajectory_active = false;
	_shadow_reference_start = NAN;
}
ControlDecision HardwarePilot::tick(const QuadState& state, Evidence evidence, bool shadow_only, bool navigation_valid) {
	checkOwner();
	ControlDecision result;
	const double now = clock_();
	evidence.now = steady_clock_();
	// Producer-provided command evidence is ignored. Only this tick can create it.
	evidence.command_valid = false;
	evidence.command_time = NAN;
	evidence.solve_seconds = NAN;
	evidence.controller_warm = false;
	const bool cadence_ok =
	        !std::isfinite(previous_tick_) || (now > previous_tick_ && (!evidence.timing_checks || now - previous_tick_ <= 0.025));
	previous_tick_ = now;
	const bool state_ok =
	        state.valid() && SafetyGate::fresh(now, state.t, 0.010, evidence.timing_checks) && std::abs(state.q().norm() - 1.0) < 1e-3;
	result.state_valid = state_ok && cadence_ok && (!shadow_only || navigation_valid);
	const bool shadow_receiver = evidence.rc_link && SafetyGate::fresh(evidence.now, evidence.rc_time, .1) && !evidence.kill;
	bool shadow_edge = false;
	if (shadow_only) {
		const bool ready = result.state_valid && shadow_receiver;
		if (!ready || !evidence.armed || !evidence.auto_switch) {
			if (_shadow_trajectory_active) {
				reference_ready_ = false;
				warm_cycles_ = 0;
			}
			_shadow_trajectory_active = false;
			_shadow_reference_start = NAN;
		}
		if (!ready) {
			_shadow_low_seen = false;
			warm_cycles_ = 0;
			reference_ready_ = false;
		}
		if (ready && !evidence.auto_switch) _shadow_low_seen = true;
		shadow_edge = ready && evidence.armed && evidence.auto_switch && !previous_auto_ && _shadow_low_seen && warm_cycles_ >= 50;
		if (evidence.auto_switch) _shadow_low_seen = false;
		if (shadow_edge) {
			_shadow_trajectory_active = true;
			_shadow_reference_start = now;
		}
	}
	const bool compute_healthy = shadow_only ? navigation_valid : SafetyGate::inputsHealthy(evidence);
	if (!cadence_ok || !state_ok || !compute_healthy || (!shadow_only && evidence.kill)) {
		warm_cycles_ = 0;
		reference_ready_ = false;
	} else {
		const auto begin = std::chrono::steady_clock::now();
		try {
			pilot_->odometryCallback(state);
			// Shadow MPC runs in manual. On the physical rising edge capture the
			// current location once, then hold it fixed throughout AUTO.
			// Never call Pilot::start(): that API may generate an automatic takeoff.
			if (!reference_ready_ || (shadow_only ? shadow_edge : (evidence.auto_switch && !previous_auto_))) {
				reference_ready_ = resetHover(state, now);
				if (reference_ready_ && (shadow_only ? shadow_edge : (evidence.auto_switch && !previous_auto_)) &&
				    !trajectory_.empty()) {
					auto points = trajectory_;
					const Quaternion alignment(Eigen::AngleAxisd(state.getYaw(), Vector<3>::UnitZ()));
					for (auto& point : points) {
						point.state.t += now;
						point.input.t += now;
						point.state.p = state.p + alignment * point.state.p;
						point.state.v = alignment * point.state.v;
						point.state.a = alignment * point.state.a;
						point.state.j = alignment * point.state.j;
						point.state.s = alignment * point.state.s;
						point.state.q(alignment * point.state.q());
					}
					// resetHover() starts at exactly `now`. Appending at that same
					// instant cannot truncate an infinite hover (truncate requires >),
					// so replace the in-memory reference list before starting the CSV.
					pilot_->off();
					pilot_->enable(false);
					reference_ready_ = pilot_->addReference(std::make_shared<SampledTrajectory>(points));
				}
			}
			if (reference_ready_ && pilot_->runPipelineChecked(now)) {
				const auto references = pilot_->getReferenceSetpoints();
				if (!references.empty()) result.reference = references.front().state;
				const Command command = pilot_->getCommand();
				if (command.isRatesThrust() && command.collective_thrust >= 0 &&
				    SafetyGate::fresh(now, command.t, 0.010, evidence.timing_checks)) {
					result.command = command;
					evidence.command_valid = true;
					evidence.command_time = steady_clock_();
				}
			}
		} catch (const std::exception&) {
			evidence.command_valid = false;
		}
		evidence.solve_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
		if (evidence.command_valid && (!evidence.timing_checks || evidence.solve_seconds <= 0.008))
			warm_cycles_ = std::min(warm_cycles_ + 1, 50u);
		else {
			warm_cycles_ = 0;
			reference_ready_ = false;
			result.command = Command();
			evidence.command_valid = false;
		}
	}
	previous_auto_ = evidence.auto_switch;
	evidence.controller_warm = warm_cycles_ >= 50;
	// Freshness is checked AFTER solving; a long solve cannot make old sensors
	// or RC appear fresh by using a timestamp captured before the solve.
	evidence.now = steady_clock_();
	const bool authorized = gate_.update(evidence);
	result.permit_override = !shadow_only && authorized;
	if (!shadow_only && !SafetyGate::inputsHealthy(evidence)) warm_cycles_ = 0;
	if (shadow_only && (!evidence.command_valid || !result.state_valid)) {
		_shadow_low_seen = false;
		_shadow_trajectory_active = false;
		_shadow_reference_start = NAN;
		reference_ready_ = false;
	}
	result.trajectory_active = shadow_only && _shadow_trajectory_active;
	result.reference_elapsed = result.trajectory_active ? now - _shadow_reference_start : 0;
	result.evidence = evidence;
	result.mode = gate_.mode();
	result.reason = gate_.reason();
	if (!cadence_ok) result.reason += "; cadence";
	if (!state_ok) result.reason += "; state age/validity";
	if (!SafetyGate::fresh(evidence.now, evidence.imu_time, 0.010, evidence.timing_checks)) result.reason += "; imu age";
	if (!SafetyGate::fresh(evidence.now, evidence.rc_time, 0.10, evidence.timing_checks)) result.reason += "; RC timeout";
	if (!evidence.rc_link) result.reason += "; RC link unavailable";
	if (!evidence.command_valid) result.reason += "; no command";
	if (!evidence.controller_warm) result.reason += "; MPC warming";
	return result;
}
}
