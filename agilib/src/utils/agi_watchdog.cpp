#include "agilib/utils/agi_watchdog.hpp"

namespace agi {
AgiWatchdog::AgiWatchdog(std::function<void()> timeout_callback, Scalar timeout_s, bool enabled)
        : _timeout_callback(std::move(timeout_callback)), _timeout_s(timeout_s) {
	if (enabled) enable();
}

AgiWatchdog::~AgiWatchdog() {
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_shutdown = true;
		_enabled = false;
	}
	_changed.notify_all();
	if (_timeout_thread.joinable()) _timeout_thread.join();
}

void AgiWatchdog::enable() {
	std::lock_guard<std::mutex> lock(_mutex);
	if (_shutdown || !std::isfinite(_timeout_s) || _timeout_s <= 0) return;
	_enabled = true;
	_timed_out = false;
	++_generation;
	if (!_timeout_thread.joinable()) _timeout_thread = std::thread(&AgiWatchdog::run, this);
	_changed.notify_all();
}

void AgiWatchdog::disable() {
	std::lock_guard<std::mutex> lock(_mutex);
	_enabled = false;
	++_generation;
	_changed.notify_all();
}

void AgiWatchdog::refresh() {
	std::lock_guard<std::mutex> lock(_mutex);
	_timed_out = false;
	++_generation;
	_changed.notify_all();
}

bool AgiWatchdog::watchdogTimedOut() {
	std::lock_guard<std::mutex> lock(_mutex);
	return _timed_out;
}

void AgiWatchdog::run() {
	std::unique_lock<std::mutex> lock(_mutex);
	while (!_shutdown) {
		_changed.wait(lock, [this] { return _shutdown || _enabled; });
		if (_shutdown) break;
		const auto generation = _generation;
		if (_changed.wait_for(lock, std::chrono::duration<double>(_timeout_s),
		                      [this, generation] { return _shutdown || !_enabled || generation != _generation; }))
			continue;
		_timed_out = true;
		lock.unlock();
		_timeout_callback();
		lock.lock();
	}
}
}  // namespace agi
