#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace agilicious::aero {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct BenchPoint {
  double rpm;
  double thrustN;
  double powerW;
};

struct RotorConfig {
  double airDensity{1.2041};
  double speedOfSound{343.0};
  double radius{0.06477};
  double pitch{0.09144};
  double hubRadius{0.008};
  double chordRoot{0.017};
  double chordTip{0.008};
  int blades{3};
  double liftSlope{4.797071};
  double liftBias{0.07};
  double dragCoefficient{4.168863};
  double profileDrag{0.018};
  double dragRiseMach{0.65};
  double dragRiseGain{1.5};
  double hForceScale{3.0};
  int radialElements{12};
  int azimuthElements{16};
  std::vector<BenchPoint> bench;
};

struct RotorResult {
  double omegaRadS{0.0};
  double thrust{0.0};
  double torque{0.0};
  double hForce{0.0};
  double inducedVelocity{0.0};
  double advanceRatio{0.0};
  double tipMach{0.0};
  bool inflowConverged{true};
};

struct BodyDragConfig {
  double airDensity{1.2041};
  // CdA in the body x/y/z directions [m^2].
  Vec3 cdArea{0.00733824, 0.00934752, 0.03739008};
};

class AerodynamicsModel {
 public:
  explicit AerodynamicsModel(RotorConfig config = defaultRotorConfig());

  RotorResult evaluateRotor(double omegaRadS, const Vec3 &airVelocityBody) const;
  Vec3 evaluateBodyDrag(const Vec3 &airVelocityBody,
                        const BodyDragConfig &config) const;

  const RotorConfig &config() const { return config_; }
  static RotorConfig defaultRotorConfig();

 private:
  struct RawLoads {
    double thrust{0.0};
    double torque{0.0};
    double hForce{0.0};
  };

  RawLoads integrate(double omega, double horizontalSpeed,
                     double verticalSpeedFrd, double inducedVelocity) const;
  double solveInflow(double omega, double horizontalSpeed,
                     double verticalSpeedFrd, bool *converged) const;
  BenchPoint interpolateBench(double rpm) const;

  RotorConfig config_;
};

}  // namespace agilicious::aero
