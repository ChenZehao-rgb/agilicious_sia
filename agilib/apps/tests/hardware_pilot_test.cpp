#include "agilib/pilot/hardware_pilot.hpp"
#include "agilib/estimator/feedthrough/feedthrough_estimator.hpp"
#include "agilib/reference/hover_reference.hpp"
#include "agilib/math/gravity.hpp"
#include <iostream>
#include <stdexcept>

using namespace agi;
using namespace agi::hardware;
namespace {
void require(bool yes, const char* message) {
  if (!yes) throw std::runtime_error(message);
}
PilotParams configuration() {
  PilotParams p;
  p.quad_ = Quadrotor(1.5, 0.25); // synthetic unit-test fixture, not flight data
  p.pipeline_cfg_.bridge_cfg.type = "External";
  p.pipeline_cfg_.estimator_cfg.type = "External";
  p.pipeline_cfg_.outer_controller_cfg.type = "MPC";
  p.pipeline_cfg_.sampler_cfg.type = "Time";
  p.velocity_in_bodyframe_ = false;
  return p;
}
Evidence health(double now, bool automatic = false) {
  Evidence e;
  e.now = e.imu_time = e.rtk_time = e.rc_time = now;
  e.rtk_fixed = e.heading_valid = e.accuracy_ok = e.imu_calibrated = true;
  e.synchronized = e.converged = e.config_verified = e.thrust_calibrated = true;
  e.geofence_ok = e.msp_healthy = e.rc_link = e.armed = true;
  e.kill = false;
  e.auto_switch = automatic;
  return e;
}
}
int main() {
  try {
    double now = 10;
    HardwarePilot runtime(configuration(), [&] { return now; });
    QuadState state; state.setZero(); state.p.z() = 0.2;
    ControlDecision d;
    const auto step = [&](bool automatic) {
      now += 0.01; state.t = now;
      return runtime.tick(state, health(now, automatic));
    };
    // Real acados MPC is executed in these cycles; no serial device is opened.
    for (int i = 0; i < 80; ++i) {
      d = step(true);
      require(!d.permit_override, "startup AUTO high entered active");
    }
    require(d.command.isRatesThrust(), "MPC did not produce rates/thrust");
    require(std::abs(d.command.collective_thrust - G) < 0.1,
            "hover generated takeoff thrust");
    for (int i = 0; i < 100 && runtime.warmCycles() < 50; ++i) step(true);
    require(runtime.warmCycles() == 50, "MPC failed 50-cycle timing warmup");
    d = step(false);
    require(d.mode == Mode::AutoStandby && !d.permit_override, "no standby");
    state.p.x() = 0.3; // capture NEW manual position, no pullback to old origin
    d = step(true);
    require(d.permit_override && d.mode == Mode::AutoActive, "AUTO edge rejected");
    require(d.command.omega.norm() < 0.15, "takeover used obsolete position");
    state.p.x() += 0.1;
    d = step(true);
    require(d.permit_override, "active hover failed");
    now += 0.01; state.t = now;
    auto e = health(now,true); e.imu_time -= 0.020;
    d = runtime.tick(state,e);
    require(!d.permit_override && !d.command.valid() &&
            d.mode == Mode::ManualFallback, "IMU fault reused prior command");
    for (int i = 0; i < 80; ++i)
      require(!step(true).permit_override, "fault automatically resumed AUTO");
    step(false); d = step(true);
    require(d.permit_override, "could not reauthorize after recovery");
    runtime.reportOutputFault();
    require(runtime.mode() == Mode::ManualFallback, "output failure not latched");
    require(!step(true).permit_override, "output failure auto recovered");
    now += 0.1;
    d = step(true);
    require(!d.permit_override && !d.command.valid(), "scheduler pause ignored");
    auto bad = configuration(); bad.pipeline_cfg_.bridge_cfg.type = "MSP";
    bool refused = false;
    try { HardwarePilot unsafe(bad, [&] { return now; }); }
    catch (const ParameterException&) { refused = true; }
    require(refused, "physical bridge YAML not rejected");

    // Regression: runPipelineChecked must surface estimator failure rather
    // than allowing a caller to infer success from a cached getCommand().
    Pilot pilot(configuration(), [&] { return now; });
    auto handoff = std::make_shared<FeedthroughEstimator>(nullptr);
    require(pilot.registerExternalEstimator(handoff), "estimator registration");
    pilot.enable(false); state.t = now;
    pilot.odometryCallback(state);
    require(pilot.addReference(std::make_shared<HoverReference>(state)), "hover reference");
    require(pilot.runPipelineChecked(now), "initial pipeline solve");
    pilot.odometryCallback(QuadState());
    require(!pilot.runPipelineChecked(now + 0.01), "estimator failure hidden");
    require(!pilot.runPipelineChecked(NAN), "NaN pipeline time accepted");
    std::cout << "PASS: actual Pilot/MPC, hover takeover, FSM, failure propagation\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
