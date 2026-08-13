#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "agilib/math/types.hpp"
#include "agilib/types/feedback.hpp"

namespace agi {

/// Minimal MSP v1 monitor for the Betaflight SITL serial-over-TCP port.
///
/// This is a *monitoring* channel, not a state source.  It reports what the
/// flight controller thinks of itself -- armed or not, why it refuses to arm,
/// battery, and its own attitude estimate -- and nothing it publishes reaches
/// the estimator or the controller.
///
/// That split is deliberate and was measured.  Betaflight stops correcting
/// attitude with the accelerometer outside a 0.9-1.1 g window
/// (`imuIsAccelerometerHealthy()` in its `flight/imu.c`), so on an aggressive
/// trajectory -- where specific force sits near 2 g -- its attitude free-runs
/// on the gyro.  Regressed against ground truth over a full flight, roll and
/// pitch come back with a slope of 0.90 and heading with r^2 = 0.22.  Feeding
/// that to an outer loop crashed the vehicle at 20 s, against 1.2 cm RMS
/// tracking on the same trajectory with a companion-computer state estimate.
/// Attitude and body rates therefore come from the companion IMU, and MSP is
/// kept for what it is uniquely good at: telling you what the flight
/// controller is doing.
///
/// Betaflight SITL exposes UART1 on TCP 5760 + 1.  That port carries the CLI
/// once a '#' has been sent (which is what betaflight_sitl/run.py does against
/// a separate, short-lived configuration process) and MSP otherwise.  This
/// client only ever speaks MSP, so it must not be pointed at a connection that
/// has already been switched into CLI mode.
class BetaflightMspClient {
 public:
  /// One monitoring snapshot, in agilib's own bridge vocabulary.
  ///
  /// `Feedback::imu` is deliberately left invalid: `Pilot` forwards a bridge's
  /// feedback IMU straight into the estimator and both controllers, and the
  /// companion sensor already fills that role.
  struct Snapshot {
    Feedback feedback;
    /// Betaflight's `armingDisableFlags` bitmask; non-zero explains a refusal
    /// to arm far better than the absence of motion does.
    uint32_t arming_disable_flags{0};
    /// Bumped once per completed poll cycle.
    uint64_t sequence{0};
  };

  /// \param host   IPv4 address of the Betaflight SITL serial port.
  /// \param port   TCP port of that serial port (5761 for UART1).
  /// \param rate_hz Polling rate for one STATUS + ATTITUDE + ANALOG sweep.
  ///        Betaflight serves MSP from a ~100 Hz task, and nothing here is in
  ///        a control loop, so a monitor wants tens of hertz, not hundreds.
  /// \param time_function Clock used to stamp snapshots.  Pass the same
  ///        simulated clock the rest of the loop runs on.
  BetaflightMspClient(std::string host, int port, Scalar rate_hz,
                      std::function<Scalar()> time_function);
  ~BetaflightMspClient();

  BetaflightMspClient(const BetaflightMspClient&) = delete;
  BetaflightMspClient& operator=(const BetaflightMspClient&) = delete;

  /// Connect and start polling.  Throws on connection failure.
  void start();
  /// Stop polling and close the socket.  Idempotent.
  void stop();

  /// Latest snapshot.  `sequence` is 0 until the first full sweep decoded.
  Snapshot snapshot() const;

  /// Human-readable names of the set bits in an `arming_disable_flags` word,
  /// e.g. "BOOTGRACE|THROTTLE".  Empty when nothing blocks arming.
  static std::string armingDisableFlagNames(uint32_t flags);

  /// True once a complete sweep has been decoded.
  bool healthy() const;
  /// Number of malformed or unanswered frames since start().
  uint64_t errorCount() const;
  /// Description of the most recent decode / transport error.
  std::string lastError() const;

 private:
  static constexpr int MSP_STATUS = 101;
  static constexpr int MSP_ATTITUDE = 108;
  static constexpr int MSP_ANALOG = 110;
  /// Longest MSP v1 frame: 6 header/trailer bytes plus a 255 byte payload.
  static constexpr size_t MAX_FRAME = 261;

  void pollLoop();
  bool request(int code);
  /// Read frames until one with `code` arrives or the deadline passes.
  bool readFrame(int code, std::array<uint8_t, MAX_FRAME>* payload,
                 size_t* length, Scalar deadline_seconds);
  void decodeStatus(const uint8_t* payload, size_t length);
  void decodeAttitude(const uint8_t* payload, size_t length);
  void decodeAnalog(const uint8_t* payload, size_t length);
  void noteError(const std::string& message);

  const std::string host_;
  const int port_;
  const Scalar period_;
  const std::function<Scalar()> time_function_;

  int socket_fd_{-1};
  std::thread thread_;
  std::atomic<bool> running_{false};

  mutable std::mutex mutex_;
  Snapshot snapshot_;
  uint64_t error_count_{0};
  std::string last_error_;

  /// Receive buffer for the incremental frame parser.
  std::vector<uint8_t> rx_;
};

}  // namespace agi
