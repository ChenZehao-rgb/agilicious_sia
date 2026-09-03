#include "AerodynamicsModel.hh"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace agilicious::aero {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRpmPerRadS = 60.0 / (2.0 * kPi);
constexpr double kGravity = 9.80665;

double clampFinite(double value, double low, double high) {
  if (!std::isfinite(value)) return low;
  return std::clamp(value, low, high);
}
}  // namespace

RotorConfig AerodynamicsModel::defaultRotorConfig() {
  RotorConfig config;
  // Qianfeng 5136 three-blade, supplied bench data. Efficiency is in g/W.
  constexpr std::array<double, 10> rpm{
    4972, 9301, 12487, 15618, 18688, 21256, 23587, 25793, 27955, 29280};
  constexpr std::array<double, 10> thrustG{
    44.2, 162.1, 296.6, 478.6, 700.7, 920.8, 1135.9, 1368.1, 1568.8, 1724.5};
  constexpr std::array<double, 10> efficiency{
    4.6, 4.6, 4.0, 3.6, 3.2, 2.9, 2.7, 2.4, 2.1, 1.9};
  for (std::size_t i = 0; i < rpm.size(); ++i) {
    config.bench.push_back(
      {rpm[i], thrustG[i] * kGravity / 1000.0, thrustG[i] / efficiency[i]});
  }
  return config;
}

AerodynamicsModel::AerodynamicsModel(RotorConfig config)
    : config_(std::move(config)) {
  if (config_.bench.size() < 2 || config_.radius <= config_.hubRadius ||
      config_.pitch <= 0.0 || config_.blades < 1 ||
      config_.radialElements < 4 || config_.azimuthElements < 8) {
    throw std::invalid_argument("invalid rotor aerodynamics configuration");
  }
  std::sort(config_.bench.begin(), config_.bench.end(),
            [](const BenchPoint &a, const BenchPoint &b) { return a.rpm < b.rpm; });
}

AerodynamicsModel::RawLoads AerodynamicsModel::integrate(
    double omega, double horizontalSpeed, double verticalSpeedFrd,
    double inducedVelocity) const {
  RawLoads result;
  if (omega < 1.0) return result;

  const double dr = (config_.radius - config_.hubRadius) / config_.radialElements;
  const double dpsi = 2.0 * kPi / config_.azimuthElements;
  const double mu = horizontalSpeed / (omega * config_.radius);
  const double prefactor = config_.blades * config_.airDensity / (4.0 * kPi);

  for (int ir = 0; ir < config_.radialElements; ++ir) {
    const double r = config_.hubRadius + (ir + 0.5) * dr;
    const double radialFraction = (r - config_.hubRadius) /
                                  (config_.radius - config_.hubRadius);
    const double chord = config_.chordRoot +
                         radialFraction * (config_.chordTip - config_.chordRoot);
    // A constant-pitch propeller is a helix; its geometric blade angle varies
    // with radius even though the advertised pitch is a single number.
    const double theta = std::atan2(config_.pitch, 2.0 * kPi * r);

    for (int ip = 0; ip < config_.azimuthElements; ++ip) {
      const double psi = (ip + 0.5) * dpsi;
      const double ut = omega * (r + config_.radius * mu * std::sin(psi));
      const double up = verticalSpeedFrd - inducedVelocity;
      const double phi = std::atan2(up, ut);
      const double alpha = theta + phi;
      const double speed2 = ut * ut + up * up;
      const double mach = std::sqrt(speed2) / config_.speedOfSound;

      // Bounded subsonic correction plus a smooth drag rise. This is not a
      // transonic CFD model, but keeps the 50-100 m/s trend conservative.
      const double beta = std::sqrt(std::max(0.36, 1.0 - mach * mach));
      double cl = config_.liftSlope *
                  (std::sin(alpha) * std::cos(alpha) + config_.liftBias) / beta;
      cl = std::clamp(cl, -1.6, 1.6);
      const double excessMach = std::max(0.0, mach - config_.dragRiseMach);
      const double cd = config_.profileDrag +
                        config_.dragCoefficient * std::sin(alpha) * std::sin(alpha) +
                        config_.dragRiseGain * excessMach * excessMach;

      const double sinPhi = std::sin(phi);
      const double cosPhi = std::cos(phi);
      const double sinPhiAbs = std::max(0.03, std::abs(sinPhi));
      const double fTip = 2.0 / kPi * std::acos(std::clamp(
        std::exp(-0.5 * config_.blades * (config_.radius - r) /
                 (r * sinPhiAbs)), 0.0, 1.0));
      const double fRoot = 2.0 / kPi * std::acos(std::clamp(
        std::exp(-0.5 * config_.blades * (r - config_.hubRadius) /
                 (config_.hubRadius * sinPhiAbs)), 0.0, 1.0));
      const double loss = std::max(0.05, fTip * fRoot);
      const double lift = speed2 * cl * chord * loss;
      const double drag = speed2 * cd * chord * loss;

      result.thrust += (lift * cosPhi + drag * sinPhi) * dr * dpsi;
      const double inPlane = -lift * sinPhi + drag * cosPhi;
      result.torque += r * inPlane * dr * dpsi;
      result.hForce += inPlane * std::sin(psi) * dr * dpsi;
    }
  }
  result.thrust *= prefactor;
  result.torque *= prefactor;
  result.hForce *= prefactor;
  return result;
}

double AerodynamicsModel::solveInflow(double omega, double horizontalSpeed,
                                      double verticalSpeedFrd,
                                      bool *converged) const {
  const double area = kPi * config_.radius * config_.radius;
  auto residual = [&](double vi) {
    const double thrust = integrate(omega, horizontalSpeed, verticalSpeedFrd, vi).thrust;
    const double axial = verticalSpeedFrd - vi;
    const double momentum = 2.0 * config_.airDensity * area * vi *
      std::sqrt(horizontalSpeed * horizontalSpeed + axial * axial);
    return thrust - momentum;
  };

  double low = 0.0;
  double high = 80.0;
  double fLow = residual(low);
  double fHigh = residual(high);
  if (!std::isfinite(fLow) || !std::isfinite(fHigh) || fLow * fHigh > 0.0) {
    *converged = false;
    const double staticThrust = std::max(0.0, interpolateBench(omega * kRpmPerRadS).thrustN);
    return std::sqrt(staticThrust /
                     std::max(1e-9, 2.0 * config_.airDensity * area));
  }
  for (int i = 0; i < 24; ++i) {
    const double middle = 0.5 * (low + high);
    const double fMiddle = residual(middle);
    if (fLow * fMiddle <= 0.0) {
      high = middle;
      fHigh = fMiddle;
    } else {
      low = middle;
      fLow = fMiddle;
    }
  }
  *converged = true;
  return 0.5 * (low + high);
}

BenchPoint AerodynamicsModel::interpolateBench(double rpm) const {
  if (rpm <= 0.0) return {0.0, 0.0, 0.0};
  const auto &first = config_.bench.front();
  if (rpm <= first.rpm) {
    const double ratio2 = rpm * rpm / (first.rpm * first.rpm);
    return {rpm, first.thrustN * ratio2, first.powerW * ratio2 * rpm / first.rpm};
  }
  for (std::size_t i = 1; i < config_.bench.size(); ++i) {
    if (rpm <= config_.bench[i].rpm) {
      const auto &a = config_.bench[i - 1];
      const auto &b = config_.bench[i];
      const double t = (rpm - a.rpm) / (b.rpm - a.rpm);
      return {rpm, a.thrustN + t * (b.thrustN - a.thrustN),
                   a.powerW + t * (b.powerW - a.powerW)};
    }
  }
  const auto &last = config_.bench.back();
  // Do not invent motor capability above the measured maximum.
  return {rpm, last.thrustN, last.powerW};
}

RotorResult AerodynamicsModel::evaluateRotor(
    double omegaRadS, const Vec3 &airVelocityBody) const {
  RotorResult result;
  const double omega = std::abs(omegaRadS);
  if (omega < 1.0) return result;
  result.omegaRadS = omega;
  const double horizontal = std::hypot(airVelocityBody.x, airVelocityBody.y);
  const double verticalFrd = -airVelocityBody.z;
  result.advanceRatio = horizontal / (omega * config_.radius);
  result.inducedVelocity = solveInflow(
    omega, horizontal, verticalFrd, &result.inflowConverged);
  result.tipMach = std::hypot(horizontal + omega * config_.radius,
                              verticalFrd - result.inducedVelocity) /
                   config_.speedOfSound;

  const RawLoads dynamic = integrate(
    omega, horizontal, verticalFrd, result.inducedVelocity);
  const BenchPoint bench = interpolateBench(omega * kRpmPerRadS);
  const double diskArea = kPi * config_.radius * config_.radius;
  const double staticInflow = std::sqrt(
    bench.thrustN / std::max(1e-9, 2.0 * config_.airDensity * diskArea));
  const RawLoads stationary = integrate(omega, 0.0, 0.0, staticInflow);
  const double thrustRatio = dynamic.thrust / std::max(1e-6, stationary.thrust);
  const double torqueRatio = std::abs(dynamic.torque) /
                             std::max(1e-6, std::abs(stationary.torque));
  result.thrust = bench.thrustN * clampFinite(thrustRatio, -0.25, 1.6);
  result.torque = bench.powerW / omega * clampFinite(torqueRatio, 0.2, 3.0);
  // The factor of three in the original Agilicious BEM was identified below
  // 18 m/s. Fade that residual out instead of extrapolating it to 100 m/s.
  const double hScale = 1.0 + (config_.hForceScale - 1.0) *
                        std::exp(-std::pow(result.advanceRatio / 0.25, 2));
  result.hForce = bench.thrustN * hScale *
                  clampFinite(std::abs(dynamic.hForce) /
                              std::max(1e-6, stationary.thrust), 0.0, 3.0);
  return result;
}

Vec3 AerodynamicsModel::evaluateBodyDrag(
    const Vec3 &velocity, const BodyDragConfig &config) const {
  return {
    -0.5 * config.airDensity * config.cdArea.x * velocity.x * std::abs(velocity.x),
    -0.5 * config.airDensity * config.cdArea.y * velocity.y * std::abs(velocity.y),
    -0.5 * config.airDensity * config.cdArea.z * velocity.z * std::abs(velocity.z)};
}

}  // namespace agilicious::aero
