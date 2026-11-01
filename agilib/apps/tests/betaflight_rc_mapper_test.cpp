#include "agilib/bridge/betaflight/betaflight_rc_mapper.hpp"

#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "agilib/bridge/betaflight_udp/betaflight_udp_bridge.hpp"

namespace {
class TestUdpBridge : public agi::BetaflightUdpBridge {
public:
	using BetaflightUdpBridge::BetaflightUdpBridge;
	using BetaflightUdpBridge::sendCommand;
};

void checkUdpBodyRateSigns() {
	const int receiver = socket(AF_INET, SOCK_DGRAM, 0);
	sockaddr_in address{};
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (receiver < 0 || bind(receiver, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
		throw std::runtime_error("Cannot bind loopback test receiver");
	socklen_t size = sizeof(address);
	if (getsockname(receiver, reinterpret_cast<sockaddr*>(&address), &size) != 0)
		throw std::runtime_error("Cannot get loopback test port");
	agi::BetaflightUdpBridgeParams params;
	params.port = ntohs(address.sin_port);
	TestUdpBridge bridge(params, [] { return 1.0; });
	for (double sign : {-1., 1.}) {
		agi::Command command;
		command.t = 1.;
		command.collective_thrust = 9.81;
		command.omega = agi::Vector<3>::Constant(sign * .3);
		if (!bridge.sendCommand(command, true)) throw std::runtime_error("UDP send failed");
		const auto channels = bridge.lastChannels();
		if (sign * (int(channels[0]) - 1500) <= 0 || sign * (int(channels[1]) - 1500) <= 0 || sign * (int(channels[3]) - 1500) >= 0)
			throw std::runtime_error("FLU rates must map to RC signs (+roll, +pitch, -yaw)");
	}
	close(receiver);
}
}  // namespace

int main() {
  try {
		checkUdpBodyRateSigns();
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
		std::cout << "PASS: UDP signed body rates, ACTUAL mapping, all axes, expo, deadband and NaN\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
