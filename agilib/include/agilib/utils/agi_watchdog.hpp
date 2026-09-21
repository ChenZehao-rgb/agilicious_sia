#pragma once
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

#include "agilib/math/types.hpp"

namespace agi {
class AgiWatchdog {
public:
	AgiWatchdog(std::function<void()> timeout_callback, Scalar timeout_s, bool enabled = true);
	~AgiWatchdog();
	bool watchdogTimedOut();
	void refresh();
	void enable();
	void disable();

private:
	void run();
	std::thread _timeout_thread;
	std::mutex _mutex;
	std::condition_variable _changed;
	const std::function<void()> _timeout_callback;
	const Scalar _timeout_s;
	bool _enabled{false};
	bool _shutdown{false};
	bool _timed_out{false};
	uint64_t _generation{0};
};
}  // namespace agi
