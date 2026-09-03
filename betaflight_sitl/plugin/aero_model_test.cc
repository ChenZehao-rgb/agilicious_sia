#include "AerodynamicsModel.hh"

#include <cassert>
#include <cmath>
#include <iostream>

using agilicious::aero::AerodynamicsModel;
using agilicious::aero::Vec3;

int main() {
  AerodynamicsModel model;
  constexpr double rpmToRadS = 2.0 * 3.14159265358979323846 / 60.0;
  constexpr double rpm[] = {4972, 9301, 12487, 15618, 18688,
                            21256, 23587, 25793, 27955, 29280};
  constexpr double thrustG[] = {44.2, 162.1, 296.6, 478.6, 700.7,
                                920.8, 1135.9, 1368.1, 1568.8, 1724.5};
  for (int i = 0; i < 10; ++i) {
    const auto point = model.evaluateRotor(rpm[i] * rpmToRadS, {});
    assert(point.inflowConverged);
    assert(std::abs(point.thrust - thrustG[i] * 9.80665 / 1000.0) < 0.03);
  }
  const auto staticMax = model.evaluateRotor(29280.0 * rpmToRadS, {});
  assert(staticMax.inflowConverged);
  assert(std::abs(staticMax.thrust - 16.911568) < 0.02);
  assert(staticMax.torque > 0.0);

  const auto stopped = model.evaluateRotor(0.0, {100.0, 0.0, 0.0});
  assert(stopped.thrust == 0.0 && stopped.hForce == 0.0);

  double previousDrag = 0.0;
  for (double speed = 0.0; speed <= 100.0; speed += 5.0) {
    const auto result = model.evaluateRotor(29280.0 * rpmToRadS, {speed, 0, 0});
    const auto drag = model.evaluateBodyDrag({speed, 0, 0}, {});
    assert(std::isfinite(result.thrust));
    assert(std::isfinite(result.hForce));
    assert(result.hForce >= 0.0);
    assert(-drag.x >= previousDrag);
    previousDrag = -drag.x;
    std::cout << speed << ',' << result.thrust << ',' << result.hForce << ','
              << -drag.x << ',' << result.advanceRatio << ',' << result.tipMach
              << ',' << result.inflowConverged << '\n';
  }
  return 0;
}
