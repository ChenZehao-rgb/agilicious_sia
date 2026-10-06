#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "agilib/bridge/betaflight/hardware_safety.hpp"

namespace agi::hardware {
double monotonicSeconds();
struct MspFrame {
	uint16_t code{0};
	bool error{false};
	std::vector<uint8_t> payload;
};
enum class MspWriteOutcome { Failed, Deferred, Complete, CompleteLate };
struct MspWriteDiagnostic {
	MspWriteOutcome outcome{MspWriteOutcome::Failed};
	size_t bytes_written{0}, frame_bytes{0};
	int system_error{0};
	double elapsed_seconds{0}, deadline_overrun_seconds{0};
};
// Incremental, bounded MSP v1 / native v2 reply decoder. Encapsulated v2
// ($M, code 255) is not supported and is never requested.
class MspDecoder {
public:
	void append(const uint8_t* data, size_t size);
	bool next(MspFrame* frame);
	uint64_t errors() const { return errors_; }

private:
	std::vector<uint8_t> bytes_;
	uint64_t errors_{0};
};

// Synchronous transport: construction, all I/O and destruction belong to ONE
// thread. A future control worker should own this object, with a latest-command
// mailbox. Neither Pilot nor a monitor thread may access its fd directly.
// No constructor/destructor/watchdog ever sends disarm or AUX channels.
class BetaflightMspBridge {
public:
	explicit BetaflightMspBridge(const std::string& device, int baud = 921600, NavigationPolicy policy = NavigationPolicy::Rtk);
	~BetaflightMspBridge();
	BetaflightMspBridge(const BetaflightMspBridge&) = delete;
	BetaflightMspBridge& operator=(const BetaflightMspBridge&) = delete;
	bool request(uint8_t code, MspFrame* reply, double deadline);
	// Split-phase diagnostic I/O. Caller owns scheduling and reply timeouts;
	// do not mix with synchronous request() while requests are outstanding.
	// Read-only zero-byte deadline misses defer; complete late writes await a
	// reply. Partial writes and I/O errors latch failed. Inspect the diagnostic.
	bool sendRequest(uint8_t code, double deadline);
	bool readOverrideSetting(const std::string& name, double deadline);
	bool receive(MspFrame* reply);  // bounded, nonblocking; includes RC ACKs
	// Explicit bench-only injection, independent of flight authorization.
	// Four AETR channels only; never use this API in a flight controller.
	bool sendBenchRc(const std::array<uint16_t, 4>& channels, double deadline);
	// Channels are AETR ONLY. False evidence stops the stream, without sending
	// a replacement throttle value. A send fault latches until process restart.
	bool sendOverride(const std::array<uint16_t, 4>& channels, const Evidence& evidence, double deadline);
	uint64_t errors() const { return errors_ + decoder_.errors(); }
	double lastSendTime() const { return last_send_time_; }
	double lastWriteSeconds() const { return last_write_seconds_; }
	const MspWriteDiagnostic& lastWriteDiagnostic() const { return _last_write_diagnostic; }

private:
	void checkOwner() const;
	bool writeFrame(uint16_t code, const std::vector<uint8_t>& payload, double deadline, bool read_only = false);
	MspWriteDiagnostic _last_write_diagnostic;
	int fd_{-1};
	const std::thread::id owner_;
	MspDecoder decoder_;
	SafetyGate gate_;
	uint64_t errors_{0};
	bool failed_{false};
	double last_send_time_{NAN}, last_write_seconds_{NAN};
};
}  // namespace agi::hardware
