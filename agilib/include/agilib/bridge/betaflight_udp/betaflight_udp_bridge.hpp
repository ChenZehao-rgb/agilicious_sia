#pragma once

#include <array>
#include <cstdint>
#include <mutex>

#include <netinet/in.h>

#include "agilib/bridge/betaflight_udp/betaflight_udp_bridge_params.hpp"
#include "agilib/bridge/bridge_base.hpp"

namespace agi {

/**
 * Sends Betaflight SITL rc_packet datagrams.
 *
 * The wire format used by Betaflight 2026.6 is exactly 40 bytes:
 * little-endian double timestamp followed by 16 little-endian uint16_t RC
 * channels. Channels are ordered AETR, then AUX1..AUX12.
 */
class BetaflightUdpBridge : public BridgeBase {
 public:
  static constexpr size_t N_CHANNELS = 16;
  using Channels = std::array<uint16_t, N_CHANNELS>;

  BetaflightUdpBridge(const BetaflightUdpBridgeParams& params,
                      const TimeFunction time_function);
  ~BetaflightUdpBridge() override;

  BetaflightUdpBridge(const BetaflightUdpBridge&) = delete;
  BetaflightUdpBridge& operator=(const BetaflightUdpBridge&) = delete;

  bool isOpen() const;
  Channels lastChannels() const;

 protected:
  bool sendCommand(const Command& command, const bool active) override;

 private:
  static constexpr size_t PACKET_SIZE = sizeof(double) +
                                        N_CHANNELS * sizeof(uint16_t);
  static constexpr uint16_t PWM_LOW = 1000;
  static constexpr uint16_t PWM_MID = 1500;
  static constexpr uint16_t PWM_HIGH = 2000;

  Channels safeChannels() const;
  Channels commandChannels(const Command& command) const;
  Scalar inverseActualRate(const Scalar rate_deg_s, const size_t axis) const;
  uint16_t rateToPwm(const Scalar normalized_stick,
                     const int deadband) const;

  bool sendPacketLocked(const Channels& channels);
  static std::array<uint8_t, PACKET_SIZE> encodePacket(
    const Scalar timestamp, const Channels& channels);

  const BetaflightUdpBridgeParams params_;
  int socket_fd_{-1};
  sockaddr_in destination_{};

  mutable std::mutex send_mutex_;
  Channels last_channels_{};
};

}  // namespace agi
