#include "agilib/bridge/betaflight_udp/betaflight_udp_bridge_params.hpp"

#include <arpa/inet.h>

#include <cmath>
#include <ostream>

namespace agi {

bool BetaflightUdpBridgeParams::load(const fs::path& filename) {
  return load(Yaml(filename));
}

bool BetaflightUdpBridgeParams::load(const Yaml& node) {
  node["host"].getIfDefined(host);
  node["port"].getIfDefined(port);
  node["timeout"].getIfDefined(timeout);
  node["n_timeouts_for_lock"].getIfDefined(n_timeouts_for_lock);

  node["center_rate_deg_s"].getIfDefined(center_rate_deg_s);
  node["max_rate_deg_s"].getIfDefined(max_rate_deg_s);
  node["expo_percent"].getIfDefined(expo_percent);
  node["deadband"].getIfDefined(deadband);
  node["yaw_deadband"].getIfDefined(yaw_deadband);

  node["hover_throttle"].getIfDefined(hover_throttle);
  node["min_check"].getIfDefined(min_check);
  node["motor_idle"].getIfDefined(motor_idle);

  return valid();
}

bool BetaflightUdpBridgeParams::valid() const {
  in_addr address{};
  if (host.empty() || inet_pton(AF_INET, host.c_str(), &address) != 1)
    return false;
  if (port < 1 || port > 65535) return false;
  if (!std::isfinite(timeout) || timeout <= 0.0) return false;
  if (n_timeouts_for_lock < 1) return false;

  if (!center_rate_deg_s.allFinite() || !max_rate_deg_s.allFinite() ||
      !expo_percent.allFinite())
    return false;
  if ((center_rate_deg_s.array() <= 0.0).any() ||
      (max_rate_deg_s.array() < center_rate_deg_s.array()).any() ||
      (max_rate_deg_s.array() > 1998.0).any())
    return false;
  if ((expo_percent.array() < 0.0).any() ||
      (expo_percent.array() > 100.0).any())
    return false;

  if (deadband < 0 || deadband >= 500 || yaw_deadband < 0 ||
      yaw_deadband >= 500)
    return false;
  if (!std::isfinite(hover_throttle) || hover_throttle <= 0.0 ||
      hover_throttle > 1.0)
    return false;
  if (min_check < 1000 || min_check >= 2000) return false;
  if (!std::isfinite(motor_idle) || motor_idle < 0.0 || motor_idle >= 1.0)
    return false;
  // The hover stick position has to stay inside the usable motor range once
  // the idle offset is applied, otherwise the throttle inverse has no solution.
  if (motor_idle + (1.0 - motor_idle) * hover_throttle >= 1.0) return false;

  return true;
}

std::ostream& operator<<(std::ostream& os,
                         const BetaflightUdpBridgeParams& params) {
  os << "Betaflight UDP Bridge Settings:\n"
     << "host:                 " << params.host << '\n'
     << "port:                 " << params.port << '\n'
     << "timeout:              " << params.timeout << "s\n"
     << "timeouts before lock: " << params.n_timeouts_for_lock << '\n'
     << "center rate [deg/s]:  " << params.center_rate_deg_s.transpose()
     << '\n'
     << "max rate [deg/s]:     " << params.max_rate_deg_s.transpose() << '\n'
     << "expo [%]:             " << params.expo_percent.transpose() << '\n'
     << "deadband/yaw:         " << params.deadband << "/"
     << params.yaw_deadband << '\n'
     << "hover throttle:       " << params.hover_throttle << '\n'
     << "motor idle:           " << params.motor_idle << '\n'
     << "min check:            " << params.min_check << std::endl;
  return os;
}

}  // namespace agi
