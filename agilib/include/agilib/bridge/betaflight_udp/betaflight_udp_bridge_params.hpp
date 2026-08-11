#pragma once

#include <iosfwd>
#include <string>

#include "agilib/base/parameter_base.hpp"

namespace agi {

/** Parameters for Betaflight's SITL UDP RC input (port 9004 by default). */
struct BetaflightUdpBridgeParams : public ParameterBase {
  bool load(const fs::path& filename) override;
  bool load(const Yaml& node) override;
  bool valid() const override;

  friend std::ostream& operator<<(std::ostream& os,
                                  const BetaflightUdpBridgeParams& params);

  std::string host{"127.0.0.1"};
  int port{9004};

  Scalar timeout{0.10};
  int n_timeouts_for_lock{4};

  // Betaflight ACTUAL rates, in degrees per second. These correspond to
  // center sensitivity and max rate shown by Betaflight Configurator.
  Vector<3> center_rate_deg_s{70.0, 70.0, 70.0};
  Vector<3> max_rate_deg_s{670.0, 670.0, 670.0};
  // Betaflight roll/pitch/yaw expo in percent [0, 100].
  Vector<3> expo_percent{0.0, 0.0, 0.0};

  // RC deadbands configured in Betaflight, in PWM microseconds.
  int deadband{0};
  int yaw_deadband{0};

  // At collective_thrust == g, the normalized throttle command is
  // hover_throttle. min_check is the lower endpoint of the active throttle
  // mapping; disarmed packets always use 1000 instead.
  Scalar hover_throttle{0.30};
  int min_check{1000};

  // Betaflight's `motor_idle`, as a fraction of the motor output range. Stick
  // throttle u maps to motor output motor_idle + (1 - motor_idle) * u, and
  // rotor thrust follows the square of that motor output. Leaving this at zero
  // reproduces the older hover-only sqrt mapping exactly.
  Scalar motor_idle{0.0};
};

}  // namespace agi
