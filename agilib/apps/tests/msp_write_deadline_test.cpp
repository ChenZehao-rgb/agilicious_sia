#include <poll.h>
#include <pty.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <iostream>
#include <stdexcept>

#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"

namespace {
enum class Injection { None, CompleteLate, PartialLate, Deferred, IoError };
Injection injection = Injection::None;
long clock_offset_ns = 0;

void require(bool condition, const char* message) {
	if (!condition) throw std::runtime_error(message);
}

class PseudoUart {
public:
	PseudoUart() {
		int slave;
		require(openpty(&_master, &slave, _path, nullptr, nullptr) == 0, "openpty failed");
		close(slave);
	}
	~PseudoUart() { close(_master); }
	const char* path() const { return _path; }
	void expectFrame(size_t size, uint8_t code) {
		pollfd ready{_master, POLLIN, 0};
		require(poll(&ready, 1, 50) == 1, "missing frame");
		uint8_t bytes[128];
		const auto count = read(_master, bytes, sizeof(bytes));
		require(count == static_cast<ssize_t>(size) && bytes[4] == code, "unexpected frame");
	}
	void expectSilent() {
		pollfd ready{_master, POLLIN, 0};
		require(poll(&ready, 1, 5) == 0, "unexpected serial write");
	}

private:
	int _master{-1};
	char _path[128]{};
};

void deferredQueryTests() {
	PseudoUart uart;
	agi::hardware::BetaflightMspBridge bridge(uart.path());
	using agi::hardware::monotonicSeconds;
	using agi::hardware::MspWriteOutcome;
	require(!bridge.sendRequest(150, monotonicSeconds() - .001), "expired query was sent");
	require(bridge.lastWriteDiagnostic().outcome == MspWriteOutcome::Deferred, "expired query was not deferred");
	uart.expectSilent();
	injection = Injection::Deferred;
	require(!bridge.sendRequest(150, monotonicSeconds() + .002), "blocked query was sent");
	require(bridge.lastWriteDiagnostic().outcome == MspWriteOutcome::Deferred, "zero-byte timeout was not deferred");
	require(bridge.errors() == 0, "deferred query counted as transport failure");
	uart.expectSilent();
	require(bridge.sendRequest(150, monotonicSeconds() + .05), "deferred query poisoned transport");
	uart.expectFrame(6, 150);
}

void completeLateQueryTests() {
	PseudoUart uart;
	agi::hardware::BetaflightMspBridge bridge(uart.path());
	injection = Injection::CompleteLate;
	require(bridge.sendRequest(238, agi::hardware::monotonicSeconds() + .002), "complete late query rejected");
	require(bridge.lastWriteDiagnostic().outcome == agi::hardware::MspWriteOutcome::CompleteLate, "missing late diagnostic");
	require(bridge.lastWriteDiagnostic().bytes_written == 6 && bridge.lastWriteDiagnostic().deadline_overrun_seconds > 0,
	        "missing write diagnostic");
	uart.expectFrame(6, 238);
	require(bridge.errors() == 0, "complete query poisoned transport");
	require(bridge.sendRequest(150, agi::hardware::monotonicSeconds() + .05), "transport did not remain usable");
	uart.expectFrame(6, 150);
}

void failedWriteTests(Injection failure, bool control) {
	PseudoUart uart;
	agi::hardware::BetaflightMspBridge bridge(uart.path());
	injection = failure;
	const double deadline = agi::hardware::monotonicSeconds() + .002;
	const bool sent = control ? bridge.sendBenchRc({1500, 1500, 1200, 1500}, deadline) : bridge.sendRequest(150, deadline);
	require(!sent, "unsafe write was accepted");
	require(bridge.errors() == 1, "failure was not counted");
	require(bridge.lastWriteDiagnostic().outcome == agi::hardware::MspWriteOutcome::Failed, "missing failure diagnostic");
	if (failure == Injection::IoError) require(bridge.lastWriteDiagnostic().system_error == EIO, "errno was lost");
	// A failed write flushes the kernel queue; delivery of any prefix is unknown.
	const auto diagnostic = bridge.lastWriteDiagnostic();
	require(diagnostic.code == (control ? 200 : 150) && diagnostic.write_calls == 1, "wrong failing operation identity");
	require(diagnostic.finished_steady_time >= diagnostic.started_steady_time && diagnostic.thread_cpu_seconds >= 0,
	        "invalid write timing");
	require(!bridge.sendRequest(105, agi::hardware::monotonicSeconds() + .05), "failed transport automatically recovered");
	const auto& blocked = bridge.lastWriteDiagnostic();
	require(blocked.outcome == agi::hardware::MspWriteOutcome::TransportLatched && blocked.code == 105 && blocked.bytes_written == 0 &&
	                blocked.frame_bytes == 6 && blocked.system_error == 0 && blocked.write_calls == 0 && blocked.elapsed_seconds == 0 &&
	                blocked.attempt_id > diagnostic.attempt_id,
	        "blocked request reused previous write diagnostic");
	const auto& cause = bridge.failureWriteDiagnostic();
	require(cause.attempt_id == diagnostic.attempt_id && cause.code == diagnostic.code &&
	                cause.bytes_written == diagnostic.bytes_written && cause.system_error == diagnostic.system_error,
	        "original failure diagnostic was lost");
	require(bridge.errors() == 1, "blocked request counted as a new serial failure");
}
}  // namespace

// Linker wrappers inject deterministic scheduling delays into this test's
// pseudo-UART only. No physical serial device or real host clock is changed.
extern "C" ssize_t __real_write(int fd, const void* buffer, size_t count);
extern "C" int __real_clock_gettime(clockid_t clock, timespec* value);
extern "C" int __wrap_clock_gettime(clockid_t clock, timespec* value) {
	const int result = __real_clock_gettime(clock, value);
	if (result == 0 && clock == CLOCK_MONOTONIC) {
		value->tv_nsec += clock_offset_ns;
		value->tv_sec += value->tv_nsec / 1000000000;
		value->tv_nsec %= 1000000000;
	}
	return result;
}
extern "C" ssize_t __wrap_write(int fd, const void* buffer, size_t count) {
	const Injection selected = injection;
	injection = Injection::None;
	if (selected == Injection::IoError) {
		errno = EIO;
		return -1;
	}
	if (selected == Injection::Deferred) {
		clock_offset_ns += 10000000;
		errno = EAGAIN;
		return -1;
	}
	const ssize_t result = __real_write(fd, buffer, selected == Injection::PartialLate ? 1 : count);
	if (selected == Injection::CompleteLate || selected == Injection::PartialLate) clock_offset_ns += 10000000;
	return result;
}

int main() {
	try {
		deferredQueryTests();
		completeLateQueryTests();
		failedWriteTests(Injection::PartialLate, false);
		failedWriteTests(Injection::IoError, false);
		failedWriteTests(Injection::CompleteLate, true);
		std::cout << "PASS: deferred/late queries, partial writes, errno and strict control deadlines\n";
		return 0;
	} catch (const std::exception& error) {
		std::cerr << error.what() << '\n';
		return 1;
	}
}
