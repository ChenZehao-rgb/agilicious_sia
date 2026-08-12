#include "agilib/bridge/betaflight_udp/betaflight_udp_bridge.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>

#include "agilib/math/gravity.hpp"

namespace agi {
namespace {

constexpr size_t CHANNEL_ROLL = 0;
constexpr size_t CHANNEL_PITCH = 1;
constexpr size_t CHANNEL_THROTTLE = 2;
constexpr size_t CHANNEL_YAW = 3;
constexpr size_t CHANNEL_AUX1 = 4;

}  // namespace

BetaflightUdpBridge::BetaflightUdpBridge(
  const BetaflightUdpBridgeParams& params,
  const TimeFunction time_function)
  : BridgeBase("Betaflight UDP Bridge", time_function, params.timeout,
               params.n_timeouts_for_lock, false),
    params_(params),
    last_channels_(safeChannels()) {
  voltage_watchdog_.disable();

  if (!params_.valid())
    throw ParameterException("Invalid Betaflight UDP bridge parameters!");

  destination_.sin_family = AF_INET;
  destination_.sin_port = htons(static_cast<uint16_t>(params_.port));
  if (inet_pton(AF_INET, params_.host.c_str(), &destination_.sin_addr) != 1)
    throw ParameterException("Betaflight UDP host is not a valid IPv4 address!");

  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0)
    throw ParameterException("Could not create Betaflight UDP socket: " +
                             std::string(std::strerror(errno)));

  // BridgeBase's guard can call the virtual sendCommand().  Start it only
  // after every socket and mutex dependency used by that method is ready.
  startTimeoutGuard();
}

BetaflightUdpBridge::~BetaflightUdpBridge() {
  // BridgeBase owns a watchdog thread which calls sendCommand(). Stop it before
  // destroying the socket and the mutex used by sendCommand().
  stopTimeoutGuard();

  const std::lock_guard<std::mutex> lock(send_mutex_);
  if (socket_fd_ >= 0) {
    // Best effort: always leave the simulated receiver disarmed.
    sendPacketLocked(safeChannels());
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool BetaflightUdpBridge::isOpen() const {
  const std::lock_guard<std::mutex> lock(send_mutex_);
  return socket_fd_ >= 0;
}

BetaflightUdpBridge::Channels BetaflightUdpBridge::lastChannels() const {
  const std::lock_guard<std::mutex> lock(send_mutex_);
  return last_channels_;
}

bool BetaflightUdpBridge::sendCommand(const Command& command,
                                      const bool active) {
  const bool command_valid = command.isRatesThrust() &&
                             command.collective_thrust >= 0.0;
  const Channels channels =
    active && command_valid ? commandChannels(command) : safeChannels();

  const std::lock_guard<std::mutex> lock(send_mutex_);
  const bool sent = sendPacketLocked(channels);
  // Report invalid input to the caller even though its disarm packet was sent.
  return sent && command_valid;
}

BetaflightUdpBridge::Channels BetaflightUdpBridge::safeChannels() const {
  Channels channels{};
  channels.fill(PWM_LOW);
  channels[CHANNEL_ROLL] = PWM_MID;
  channels[CHANNEL_PITCH] = PWM_MID;
  channels[CHANNEL_THROTTLE] = PWM_LOW;
  channels[CHANNEL_YAW] = PWM_MID;
  channels[CHANNEL_AUX1] = PWM_LOW;
  return channels;
}

BetaflightUdpBridge::Channels BetaflightUdpBridge::commandChannels(
  const Command& command) const {
  Channels channels{};
  channels.fill(PWM_LOW);

  constexpr Scalar RAD_TO_DEG = 180.0 / M_PI;
  // All three axes pass through with the same sign.  The Gazebo model mounts
  // its IMU with a 180 degree roll, so Betaflight already receives its gyro in
  // the frame its sticks are defined in; negating yaw here as well inverts the
  // yaw rate loop into positive feedback.  That fault is easy to miss because
  // `yaw_motors_reversed` can hide it: flipping the mixer restores the sign of
  // the commanded direction while leaving the gyro feedback inverted, so the
  // vehicle flies but its yaw rate winds up to a steady rotation opposite to
  // whatever is commanded.
  const Vector<3> rate_deg_s{command.omega.x() * RAD_TO_DEG,
                             command.omega.y() * RAD_TO_DEG,
                             command.omega.z() * RAD_TO_DEG};
  channels[CHANNEL_ROLL] =
    rateToPwm(inverseActualRate(rate_deg_s.x(), 0), params_.deadband);
  channels[CHANNEL_PITCH] =
    rateToPwm(inverseActualRate(rate_deg_s.y(), 1), params_.deadband);
  channels[CHANNEL_YAW] =
    rateToPwm(inverseActualRate(rate_deg_s.z(), 2), params_.yaw_deadband);

  if (command.collective_thrust <= 0.0) {
    // Raw throttle must be strictly below min_check while AUX1 transitions
    // high. Pilot intentionally emits zero collective thrust during pre-arm.
    channels[CHANNEL_THROTTLE] = PWM_LOW;
  } else {
    // Betaflight maps stick throttle u onto the motor output range as
    // motor = motor_idle + (1 - motor_idle) * u, and rotor thrust follows
    // motor^2.  Solve that chain for u instead of assuming thrust is
    // proportional to u^2, which only holds when motor_idle is zero.
    const Scalar hover_motor =
      params_.motor_idle + (1.0 - params_.motor_idle) * params_.hover_throttle;
    const Scalar motor =
      hover_motor * std::sqrt(command.collective_thrust / G);
    const Scalar normalized_throttle = std::clamp(
      (motor - params_.motor_idle) / (1.0 - params_.motor_idle), 0.0, 1.0);
    const Scalar throttle =
      params_.min_check +
      (PWM_HIGH - params_.min_check) * normalized_throttle;
    channels[CHANNEL_THROTTLE] = static_cast<uint16_t>(std::clamp(
      std::lround(throttle), static_cast<long>(params_.min_check),
      static_cast<long>(PWM_HIGH)));
  }
  channels[CHANNEL_AUX1] = PWM_HIGH;
  return channels;
}

Scalar BetaflightUdpBridge::inverseActualRate(const Scalar rate_deg_s,
                                              const size_t axis) const {
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

uint16_t BetaflightUdpBridge::rateToPwm(const Scalar normalized_stick,
                                        const int deadband) const {
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

bool BetaflightUdpBridge::sendPacketLocked(const Channels& channels) {
  if (socket_fd_ < 0) return false;

  const Scalar timestamp = time_function_();
  if (!std::isfinite(timestamp)) return false;
  const std::array<uint8_t, PACKET_SIZE> packet =
    encodePacket(timestamp, channels);
  const ssize_t sent = sendto(socket_fd_, packet.data(), packet.size(), 0,
                              reinterpret_cast<const sockaddr*>(&destination_),
                              sizeof(destination_));
  if (sent != static_cast<ssize_t>(packet.size())) {
    logger_.error("Could not send Betaflight UDP RC packet: %s",
                  std::strerror(errno));
    return false;
  }
  last_channels_ = channels;
  return true;
}

std::array<uint8_t, BetaflightUdpBridge::PACKET_SIZE>
BetaflightUdpBridge::encodePacket(const Scalar timestamp,
                                 const Channels& channels) {
  static_assert(sizeof(Scalar) == sizeof(uint64_t),
                "Betaflight SITL requires an IEEE-754 64-bit timestamp");

  std::array<uint8_t, PACKET_SIZE> packet{};
  uint64_t timestamp_bits = 0;
  std::memcpy(&timestamp_bits, &timestamp, sizeof(timestamp_bits));
  for (size_t byte = 0; byte < sizeof(timestamp_bits); ++byte)
    packet[byte] = static_cast<uint8_t>(timestamp_bits >> (8 * byte));

  for (size_t channel = 0; channel < channels.size(); ++channel) {
    const size_t offset = sizeof(timestamp_bits) + channel * sizeof(uint16_t);
    packet[offset] = static_cast<uint8_t>(channels[channel] & 0xffu);
    packet[offset + 1] = static_cast<uint8_t>(channels[channel] >> 8);
  }
  return packet;
}

}  // namespace agi
