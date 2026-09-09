#include "agilib/bridge/betaflight/betaflight_rc_mapper.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
  try {
    agi::BetaflightUdpBridgeParams params;
    for (double expo : {0.0, 35.0, 100.0}) {
      params.expo_percent.setConstant(expo);
      agi::BetaflightRcMapper mapper(params);
      for (int axis = 0; axis < 3; ++axis) {
        for (double stick : {-1.0,-0.8,-0.1,0.0,0.1,0.8,1.0}) {
          const double x = std::abs(stick);
          const double forward = std::copysign(
            params.center_rate_deg_s(axis) * x +
            (params.max_rate_deg_s(axis) - params.center_rate_deg_s(axis)) *
            ((1 - expo / 100) * x * x + expo / 100 * std::pow(x,6)), stick);
          const double inverse = mapper.inverseActualRate(forward, axis);
          if (std::abs(stick - inverse) > 1e-10)
            throw std::runtime_error("ACTUAL forward/inverse mismatch");
          for (int deadband : {0,5,20}) {
            const double pwm = mapper.rateToPwm(inverse, deadband);
            const double magnitude = std::max(0.0, std::abs(pwm - 1500) - deadband);
            const double recovered = std::copysign(magnitude / (500 - deadband), pwm - 1500);
            if (std::abs(stick - recovered) > 0.0011)
              throw std::runtime_error("PWM/deadband mismatch");
          }
        }
      }
      bool caught = false;
      try { mapper.inverseActualRate(NAN, 0); }
      catch (const agi::ParameterException&) { caught = true; }
      if (!caught) throw std::runtime_error("NaN accepted");
    }
    std::cout << "PASS: ACTUAL mapping, all axes, expo, deadband and NaN\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
