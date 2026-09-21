#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "agilib/base/module.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/feedback.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/utils/agi_watchdog.hpp"
#include "agilib/utils/logger.hpp"
#include "agilib/utils/median_filter.hpp"
#include "agilib/utils/timer.hpp"

namespace agi {

using FeedbackCallbackFunction = std::function<void(const Feedback&)>;

class BridgeBase : public Module<BridgeBase> {
 public:
	 BridgeBase(const std::string& name, const TimeFunction time_function, const Scalar timeout = 0.10, const int n_max_timeouts = 10,
	            const bool start_timeout_guard = true, const bool start_voltage_watchdog = true);
	 virtual ~BridgeBase();

	 virtual bool send(const Command& command) final;
	 virtual bool activate() final;
	 virtual bool deactivate() final;
	 virtual void reset();

	 virtual bool active() const final;
	 virtual bool locked() const final;
	 virtual void setVoltage(const Scalar voltage) final;
	 virtual Scalar getVoltage() const final;

	 virtual bool getFeedback(Feedback* const feedback = nullptr);
	 virtual void registerFeedbackCallback(FeedbackCallbackFunction function);

 protected:
  virtual bool sendCommand(const Command& command, const bool active) = 0;
  virtual void guardTimeout();
  void startTimeoutGuard();
  void stopTimeoutGuard();
  void voltageTimeout() {
    logger_.warn("voltage value not updated for a long time");
  };

  const Scalar timeout_;
  const int n_max_timeouts_;
  const TimeFunction time_function_;

  std::atomic<bool> shutdown_{false};
  std::thread timeout_guard_thread_;
  std::mutex timeout_wait_mutex_;
  std::condition_variable timeout_reset_cv_;
  std::atomic<int> n_timeouts_{0};
  std::atomic<bool> active_{false};
  std::atomic<bool> got_command_{false};

  Median<Scalar, 15> voltage_{15.5};
  Scalar latest_raw_voltage{15.5};
  std::vector<FeedbackCallbackFunction> feedback_callbacks_;
  AgiWatchdog voltage_watchdog_;
};

}  // namespace agi
