#pragma once
#include <cstdint>
#include "agilib/bridge/betaflight_udp/betaflight_udp_bridge_params.hpp"
namespace agi {
// Shared ACTUAL rate inverse. Hardware thrust MUST use a measured map;
// the UDP hover/idle square-law model deliberately stays in the SITL bridge.
class BetaflightRcMapper {
 public:
  explicit BetaflightRcMapper(const BetaflightUdpBridgeParams& params)
    : params_(params) {
    if (!params.valid()) throw ParameterException("Invalid RC mapping");
  }
  Scalar inverseActualRate(Scalar rate_deg_s, size_t axis) const;
  uint16_t rateToPwm(Scalar normalized_stick, int deadband) const;
 private:
  static constexpr uint16_t PWM_LOW = 1000, PWM_MID = 1500, PWM_HIGH = 2000;
  const BetaflightUdpBridgeParams params_;
};
}
