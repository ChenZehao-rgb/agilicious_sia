#include "agilib/bridge/betaflight/betaflight_rc_mapper.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace agi {
Scalar BetaflightRcMapper::inverseActualRate(const Scalar rate_deg_s,
                                              const size_t axis) const {
  if (!std::isfinite(rate_deg_s) || axis >= 3)
    throw ParameterException("Invalid rate or axis");
  const Scalar sign = std::signbit(rate_deg_s) ? -1.0 : 1.0;
  const Scalar target =
    std::clamp(std::abs(rate_deg_s), 0.0, params_.max_rate_deg_s(axis));
  if (target <= std::numeric_limits<Scalar>::epsilon()) return 0.0;

  const Scalar center = params_.center_rate_deg_s(axis);
  const Scalar movement = params_.max_rate_deg_s(axis) - center;
  const Scalar expo = params_.expo_percent(axis) / 100.0;

  // Betaflight 2026.6 ACTUAL rates for a positive normalized stick x:
  // center*x + (max-center)*((1-expo)*x^2 + expo*x^6).
  Scalar lower = 0.0;
  Scalar upper = 1.0;
  for (int i = 0; i < 48; ++i) {
    const Scalar x = 0.5 * (lower + upper);
    const Scalar x2 = x * x;
    const Scalar x6 = x2 * x2 * x2;
    const Scalar value =
      center * x + movement * ((1.0 - expo) * x2 + expo * x6);
    if (value < target)
      lower = x;
    else
      upper = x;
  }
  return sign * 0.5 * (lower + upper);
}

uint16_t BetaflightRcMapper::rateToPwm(const Scalar normalized_stick,
                                        const int deadband) const {
  if (!std::isfinite(normalized_stick) || deadband < 0 || deadband >= 500)
    throw ParameterException("Invalid stick or deadband");
  const Scalar x = std::clamp(normalized_stick, -1.0, 1.0);
  if (std::abs(x) <= std::numeric_limits<Scalar>::epsilon()) return PWM_MID;

  // Undo Betaflight's fapplyDeadband() and subsequent division by
  // (500-deadband), so the ACTUAL-rate inverse receives precisely x.
  const Scalar magnitude = deadband + std::abs(x) * (500.0 - deadband);
  const Scalar pwm = PWM_MID + std::copysign(magnitude, x);
  return static_cast<uint16_t>(std::clamp(
    std::lround(pwm), static_cast<long>(PWM_LOW),
    static_cast<long>(PWM_HIGH)));
}

}
