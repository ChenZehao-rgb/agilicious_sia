#include "agilib/bridge/bridge_base.hpp"

namespace agi {
BridgeBase::BridgeBase(const std::string& name,
                       const TimeFunction time_function, const Scalar timeout,
                       const int n_max_timeouts,
                       const bool start_timeout_guard)
  : Module(name),
    timeout_(timeout),
    n_max_timeouts_(n_max_timeouts),
    time_function_(time_function),
    voltage_watchdog_(std::bind(&BridgeBase::voltageTimeout, this), 30.0) {
  if (start_timeout_guard) startTimeoutGuard();
}

BridgeBase::~BridgeBase() {
  stopTimeoutGuard();
  voltage_watchdog_.disable();
}

void BridgeBase::startTimeoutGuard() {
  if (timeout_guard_thread_.joinable()) return;
  if (timeout_ <= 0.0) {
    logger_.warn("Not starting guard!");
    return;
  }

  shutdown_.store(false);
  timeout_guard_thread_ = std::thread(&BridgeBase::guardTimeout, this);
}

void BridgeBase::stopTimeoutGuard() {
  shutdown_.store(true);
  timeout_reset_cv_.notify_all();
  if (timeout_guard_thread_.joinable()) timeout_guard_thread_.join();
}

bool BridgeBase::send(const Command& command) {
  // First check if timeout has locked out.
  if (locked()) return false;

  got_command_.store(true);
  const bool ret = sendCommand(command, active_.load());

  // If command is successfully sent, reset timeout.
  // Decrease timeout counter instead of resetting it to catch
  // critically low send rates.
  if (ret) {
    timeout_reset_cv_.notify_all();
    int n_timeouts = n_timeouts_.load();
    while (n_timeouts > 0 &&
           !n_timeouts_.compare_exchange_weak(n_timeouts, n_timeouts - 1)) {
    }
  }

  return ret;
}

void BridgeBase::guardTimeout() {
  const std::chrono::milliseconds timeout((int)(timeout_ * 1000));
  std::unique_lock<std::mutex> lock(timeout_wait_mutex_);

  while (!shutdown_.load()) {
    // Wait for timeout. If notified during waiting, nothing happens.
    const std::cv_status wait_status = timeout_reset_cv_.wait_for(lock, timeout);
    if (shutdown_.load()) break;
    if (wait_status == std::cv_status::timeout) {
      const bool active = active_.load();
      if (n_max_timeouts_ < 1 || !active || !got_command_.load()) {
        const Command zero_command(time_function_());
        sendCommand(zero_command, active);
        continue;  // not armed, or no command yet
      }

      // If we have not yet reached critical number of timeouts,
      // we can leave the platform armed and hope the user catches it.
      if (!locked()) {
        n_timeouts_.fetch_add(1);
        const Command zero_command(time_function_());
        sendCommand(zero_command, active);
      } else {  // Else we disarm.
        deactivate();
        const Command zero_command(time_function_());
        sendCommand(zero_command, active_.load());
      }
    }
  }
}

bool BridgeBase::activate() {
  if (locked()) {
    logger_.warn("Can't activate because locked!");
    return false;
  }

  active_.store(true);
  logger_.info("Activated!");

  return active_.load();
}

bool BridgeBase::deactivate() {
  active_.store(false);
  logger_.info("Deactivated!");
  return active_.load();
}

void BridgeBase::setVoltage(const Scalar voltage) {
  if (voltage != latest_raw_voltage) {
    voltage_watchdog_.refresh();
  }
  latest_raw_voltage = voltage;
  voltage_ += voltage;
}

Scalar BridgeBase::getVoltage() const { return voltage_.get(); }

void BridgeBase::reset() {
  this->n_timeouts_ = 0;
  got_command_ = false;
}

bool BridgeBase::active() const { return active_.load(); }

bool BridgeBase::locked() const {
  return n_max_timeouts_ > 0 && n_timeouts_.load() >= n_max_timeouts_;
}

bool BridgeBase::getFeedback(Feedback* const feedback) {
  if (feedback == nullptr) return false;
  feedback->t = time_function_();
  feedback->armed = active_.load();
  return true;
}

void BridgeBase::registerFeedbackCallback(FeedbackCallbackFunction function) {
  feedback_callbacks_.push_back(function);
}

}  // namespace agi
