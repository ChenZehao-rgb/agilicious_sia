#include "betaflight_msp_client.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace agi {
namespace {

constexpr Scalar DEG_TO_RAD = M_PI / 180.0;
/// MSP_ATTITUDE reports roll and pitch in decidegrees but yaw in whole
/// degrees, so the two share a frame but not a resolution.
constexpr Scalar DECIDEG_TO_RAD = 0.1 * DEG_TO_RAD;

/// Longest a single MSP reply may take before the poll gives up [s].
constexpr Scalar kReadTimeout = 0.030;

int16_t readInt16(const uint8_t* data) {
  return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                              (static_cast<uint16_t>(data[1]) << 8));
}

uint16_t readUint16(const uint8_t* data) {
  return static_cast<uint16_t>(static_cast<uint16_t>(data[0]) |
                               (static_cast<uint16_t>(data[1]) << 8));
}

uint32_t readUint32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

}  // namespace

BetaflightMspClient::BetaflightMspClient(std::string host, const int port,
                                        const Scalar rate_hz,
                                        std::function<Scalar()> time_function)
  : host_(std::move(host)),
    port_(port),
    period_(rate_hz > 0.0 ? 1.0 / rate_hz : 0.0),
    time_function_(std::move(time_function)) {
  if (!(rate_hz > 0.0)) {
    throw std::runtime_error("MSP poll rate must be > 0");
  }
  if (!time_function_) {
    throw std::runtime_error("MSP client needs a clock");
  }
}

BetaflightMspClient::~BetaflightMspClient() { stop(); }

void BetaflightMspClient::start() {
  if (running_.load()) return;

  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    throw std::runtime_error("could not create MSP socket: " +
                             std::string(std::strerror(errno)));
  }

  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(static_cast<uint16_t>(port_));
  if (inet_pton(AF_INET, host_.c_str(), &destination.sin_addr) != 1) {
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("MSP host is not a valid IPv4 address: " + host_);
  }

  if (connect(socket_fd_, reinterpret_cast<const sockaddr*>(&destination),
              sizeof(destination)) != 0) {
    const std::string reason = std::strerror(errno);
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("could not connect to Betaflight MSP on " + host_ +
                             ":" + std::to_string(port_) + ": " + reason);
  }

  // MSP frames are tiny and latency sensitive; Nagle would coalesce a request
  // with the following poll interval's request.
  int flag = 1;
  setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

  // Close with an RST rather than a FIN.  A graceful close leaves a TIME-WAIT
  // entry against 127.0.0.1:5761 for a minute, and the next run's port check
  // -- and Betaflight's own listener -- then cannot bind it.  Nothing is lost:
  // MSP replies are polled, so there is no pending payload worth draining.
  linger no_linger{};
  no_linger.l_onoff = 1;
  no_linger.l_linger = 0;
  setsockopt(socket_fd_, SOL_SOCKET, SO_LINGER, &no_linger, sizeof(no_linger));

  // Short receive timeout so the poll loop can notice stop() promptly.
  timeval receive_timeout{};
  receive_timeout.tv_usec = 2000;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout,
             sizeof(receive_timeout));

  rx_.clear();
  running_.store(true);
  thread_ = std::thread(&BetaflightMspClient::pollLoop, this);
}

void BetaflightMspClient::stop() {
  if (running_.exchange(false)) {
    if (thread_.joinable()) thread_.join();
  }
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

BetaflightMspClient::Snapshot BetaflightMspClient::snapshot() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

std::string BetaflightMspClient::armingDisableFlagNames(const uint32_t flags) {
  // Bit order of `armingDisableFlags_e`, taken from Betaflight's own
  // `armingDisableFlagNames[]` in fc/runtime_config.c.  A raw bitmask says
  // nothing; "BOOTGRACE" or "THROTTLE" says exactly what to do about it.
  static const char* kNames[] = {
    "NOGYRO",      "FAILSAFE",  "RXLOSS",      "NOT_DISARMED", "BOXFAILSAFE",
    "RUNAWAY",     "CRASH",     "THROTTLE",    "ANGLE",        "BOOTGRACE",
    "NOPREARM",    "LOAD",      "CALIB",       "CLI",          "CMS",
    "BST",         "MSP",       "PARALYZE",    "GPS",          "RESCUE_SW",
    "DSHOT_TELEM", "REBOOT_REQD", "DSHOT_BBANG", "NO_ACC_CAL", "MOTOR_PROTO",
    "FLIP_SWITCH", "ALT_HOLD_SW", "POS_HOLD_SW", "AUTOPILOT_SW", "ARM_SWITCH"};
  constexpr int kCount = static_cast<int>(sizeof(kNames) / sizeof(kNames[0]));

  std::string names;
  for (int bit = 0; bit < 32; ++bit) {
    if ((flags & (1u << bit)) == 0) continue;
    if (!names.empty()) names += '|';
    names += bit < kCount ? kNames[bit] : ("BIT" + std::to_string(bit));
  }
  return names;
}

bool BetaflightMspClient::healthy() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.sequence > 0;
}

uint64_t BetaflightMspClient::errorCount() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return error_count_;
}

std::string BetaflightMspClient::lastError() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

void BetaflightMspClient::noteError(const std::string& message) {
  const std::lock_guard<std::mutex> lock(mutex_);
  ++error_count_;
  last_error_ = message;
}

void BetaflightMspClient::pollLoop() {
  using SteadyClock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<Scalar>(period_));
  SteadyClock::time_point next = SteadyClock::now();

  std::array<uint8_t, MAX_FRAME> payload{};
  size_t length = 0;

  while (running_.load()) {
    next += period;

    // Request and consume one message at a time.  Betaflight answers in
    // order, but interleaving two outstanding requests would make a dropped
    // reply desynchronise the pairing for good.
    bool swept = false;
    for (const int code : {MSP_STATUS, MSP_ATTITUDE, MSP_ANALOG}) {
      if (!running_.load()) break;
      if (!request(code)) continue;
      // A loopback MSP round trip is sub-millisecond when Betaflight is idle
      // but routinely tens of milliseconds while it is busy running its rate
      // loop, so the budget cannot scale with the poll period.
      if (!readFrame(code, &payload, &length, kReadTimeout)) continue;
      switch (code) {
        case MSP_STATUS:
          decodeStatus(payload.data(), length);
          break;
        case MSP_ATTITUDE:
          decodeAttitude(payload.data(), length);
          break;
        default:
          decodeAnalog(payload.data(), length);
          break;
      }
      swept = true;
    }

    if (swept) {
      const std::lock_guard<std::mutex> lock(mutex_);
      snapshot_.feedback.t = time_function_();
      snapshot_.feedback.received = true;
      ++snapshot_.sequence;
    }

    const SteadyClock::time_point now = SteadyClock::now();
    if (next < now) {
      next = now;
    } else {
      std::this_thread::sleep_until(next);
    }
  }
}

bool BetaflightMspClient::request(const int code) {
  if (socket_fd_ < 0) return false;
  // MSP v1 request: '$' 'M' '<' <payload length> <code> <checksum>.
  // The checksum is the XOR of everything after the direction byte.
  const std::array<uint8_t, 6> frame{
    '$', 'M', '<', 0x00, static_cast<uint8_t>(code),
    static_cast<uint8_t>(0x00 ^ code)};

  size_t written = 0;
  while (written < frame.size()) {
    const ssize_t sent =
      ::send(socket_fd_, frame.data() + written, frame.size() - written,
             MSG_NOSIGNAL);
    if (sent <= 0) {
      if (errno == EINTR) continue;
      noteError("MSP send failed: " + std::string(std::strerror(errno)));
      return false;
    }
    written += static_cast<size_t>(sent);
  }
  return true;
}

bool BetaflightMspClient::readFrame(const int code,
                                    std::array<uint8_t, MAX_FRAME>* const
                                      payload,
                                    size_t* const length,
                                    const Scalar deadline_seconds) {
  using SteadyClock = std::chrono::steady_clock;
  const SteadyClock::time_point deadline =
    SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
                           std::chrono::duration<Scalar>(deadline_seconds));

  std::array<uint8_t, 512> chunk{};
  while (running_.load() && SteadyClock::now() < deadline) {
    // Try to decode a complete frame out of whatever is already buffered
    // before blocking on the socket again.
    while (rx_.size() >= 6) {
      // Resynchronise on the '$M' preamble; SITL emits stray bytes when the
      // port was previously used for something else.
      if (rx_[0] != '$' || rx_[1] != 'M') {
        rx_.erase(rx_.begin());
        continue;
      }
      const uint8_t direction = rx_[2];
      const uint8_t payload_length = rx_[3];
      const uint8_t frame_code = rx_[4];
      const size_t frame_size = static_cast<size_t>(payload_length) + 6;
      if (rx_.size() < frame_size) break;  // Need more bytes.

      uint8_t checksum = payload_length ^ frame_code;
      for (size_t i = 0; i < payload_length; ++i) checksum ^= rx_[5 + i];

      const bool checksum_ok = checksum == rx_[frame_size - 1];
      const bool is_response = direction == '>';
      if (!checksum_ok) {
        noteError("MSP checksum mismatch on code " +
                  std::to_string(frame_code));
        rx_.erase(rx_.begin());
        continue;
      }
      if (!is_response) {
        // '!' is Betaflight's error direction: the code is unsupported.
        noteError("Betaflight rejected MSP code " + std::to_string(frame_code));
        rx_.erase(rx_.begin(), rx_.begin() + frame_size);
        continue;
      }

      const bool wanted = frame_code == code;
      if (wanted) {
        *length = payload_length;
        std::memcpy(payload->data(), rx_.data() + 5, payload_length);
      }
      rx_.erase(rx_.begin(), rx_.begin() + frame_size);
      if (wanted) return true;
    }

    const ssize_t received = ::recv(socket_fd_, chunk.data(), chunk.size(), 0);
    if (received > 0) {
      rx_.insert(rx_.end(), chunk.begin(), chunk.begin() + received);
      // A desynchronised stream must not grow without bound.
      if (rx_.size() > 8 * MAX_FRAME) {
        noteError("MSP receive buffer overflow; resynchronising");
        rx_.clear();
      }
      continue;
    }
    if (received == 0) {
      noteError("Betaflight closed the MSP connection");
      return false;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
    noteError("MSP receive failed: " + std::string(std::strerror(errno)));
    return false;
  }

  noteError("timed out waiting for MSP code " + std::to_string(code));
  return false;
}

void BetaflightMspClient::decodeStatus(const uint8_t* const payload,
                                       const size_t length) {
  // Betaflight's MSP_STATUS writer, in order:
  //   u16 PID task period, u16 i2c errors, u16 sensor flags,
  //   u32 flight mode flags, u8 PID profile, u16 system load,
  //   u16 gyro cycle time, u8 extra flag bytes, <that many bytes>,
  //   u8 arming-disable flag count, u32 arming-disable flags, ...
  if (length < 15) {
    noteError("MSP_STATUS payload too short");
    return;
  }

  // `packFlightModeFlags` indexes by *active* box, not by box id, and BOXARM
  // is unconditionally active and comes first, so bit 0 is ARM.
  const uint32_t mode_flags = readUint32(payload + 6);
  const bool armed = (mode_flags & 0x1u) != 0;

  // The extension byte count is variable, so the arming-disable word cannot be
  // read from a fixed offset.
  uint32_t arming_disable_flags = 0;
  const size_t extra_bytes = payload[15];
  const size_t disable_flags_at = 16 + extra_bytes + 1;
  if (disable_flags_at + 4 <= length) {
    arming_disable_flags = readUint32(payload + disable_flags_at);
  }

  const std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.feedback.armed = armed;
  // The bridge commands body rates, so that is the mode Betaflight is in
  // whenever it is armed at all.
  snapshot_.feedback.control_mode = armed ? Feedback::CTRLMODE::BODY_RATE
                                          : Feedback::CTRLMODE::OFF;
  snapshot_.arming_disable_flags = arming_disable_flags;
}

void BetaflightMspClient::decodeAttitude(const uint8_t* const payload,
                                         const size_t length) {
  if (length < 6) {
    noteError("MSP_ATTITUDE payload too short");
    return;
  }

  // Betaflight: roll and pitch in decidegrees, yaw in whole degrees, all in
  // the NED / FRD aerospace convention with yaw increasing clockwise.  Roll
  // and pitch carry over to FLU / ENU unchanged, heading flips sign.
  const Scalar roll = DECIDEG_TO_RAD * readInt16(payload);
  const Scalar pitch = DECIDEG_TO_RAD * readInt16(payload + 2);
  const Scalar yaw = -DEG_TO_RAD * readInt16(payload + 4);

  // Yaw is relative to whatever heading Betaflight booted at: SITL has no
  // magnetometer, so its heading is a free-running gyro integration with no
  // datum.  Only the tilt this carries is comparable to anything, which is why
  // no yaw offset is measured or applied here.
  const Quaternion attitude =
    Quaternion(Eigen::AngleAxis<Scalar>(yaw, Vector<3>::UnitZ()) *
               Eigen::AngleAxis<Scalar>(pitch, Vector<3>::UnitY()) *
               Eigen::AngleAxis<Scalar>(roll, Vector<3>::UnitX()))
      .normalized();

  const std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.feedback.attitude = attitude;
}

void BetaflightMspClient::decodeAnalog(const uint8_t* const payload,
                                       const size_t length) {
  // u8 legacy voltage (0.1 V), u16 mAh drawn, u16 RSSI,
  // i16 amperage (0.01 A), u16 voltage (0.01 V).
  if (length < 9) {
    noteError("MSP_ANALOG payload too short");
    return;
  }

  const Scalar current = 0.01 * readInt16(payload + 5);
  const Scalar voltage = 0.01 * readUint16(payload + 7);

  const std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.feedback.current = current;
  snapshot_.feedback.voltage = voltage;
}

}  // namespace agi
