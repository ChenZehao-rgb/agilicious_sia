#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include <csignal>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
}
int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: agilicious_betaflight_hw --monitor DEVICE [BAUD]\n"
      "Read-only MSP diagnostic. No ARM, RC override, motors, CLI writes,\n"
      "Gazebo or simulated sensors. Real RTK/IMU integration is pending.\n";
    return 0;
  }
  if ((argc != 3 && argc != 4) || std::string(argv[1]) != "--monitor") {
    std::cerr << "Only --monitor DEVICE [BAUD] is supported; see --help.\n";
    return 2;
  }
  try {
    int baud = 921600;
    if (argc == 4) {
      size_t used = 0;
      baud = std::stoi(argv[3], &used);
      if (used != std::string(argv[3]).size()) throw std::runtime_error("invalid baud");
    }
    agi::hardware::BetaflightMspBridge bridge(argv[2], baud);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    std::cout << "MONITOR ONLY; AUTO disabled: sensor drivers and hardware calibration pending.\n"
              << "monotonic_s,msp_code,payload_hex,error_count\n";
    // Identity/configuration snapshots are raw bytes: version-specific layouts
    // must be verified against the exact FC commit before enabling control.
    const auto query = [&](uint8_t code) {
      agi::hardware::MspFrame reply;
      if (!bridge.request(code, &reply, agi::hardware::monotonicSeconds() + 0.050))
        throw std::runtime_error("MSP query failed, code " + std::to_string(code));
      std::cout << std::fixed << std::setprecision(6)
                << agi::hardware::monotonicSeconds() << ',' << unsigned(code) << ',';
      for (uint8_t b : reply.payload)
        std::cout << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
      std::cout << std::dec << ',' << bridge.errors() << '\n';
    };
    for (uint8_t code : {1, 2, 3, 5, 64, 111}) query(code);
    auto next = std::chrono::steady_clock::now();
    while (running) {
      for (uint8_t code : {101, 105, 110}) {
        if (!running) break;
        query(code);
      }
      next += std::chrono::milliseconds(100);
      if (next < std::chrono::steady_clock::now()) next = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(next);
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
