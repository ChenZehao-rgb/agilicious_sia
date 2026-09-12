#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <chrono>
#include <thread>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
struct Poll {
  std::string name;
  uint8_t code;
  double hz, next{0}, sent{0};
  bool pending{false};
};
double number(const std::string& text) {
  size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value))
    throw std::runtime_error("invalid number: " + text);
  return value;
}
void advance(double& next, double period, double now) {
  next += (std::floor((now - next) / period) + 1) * period;
}
}
int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: agilicious_betaflight_hw --monitor|--bench DEVICE [BAUD] [options]\n"
      "  --rates attitude=10,rc=10,status=5,analog=2,battery=2,gps=2\n"
      "    Replaces defaults; omitted categories or rate 0 are disabled; max 100 Hz.\n"
      "  --duration SECONDS  (default 10; 0 runs until SIGINT)\n"
      "  --timeout-ms MS    (default 100; 1..5000)\n"
      "  --rc A,E,T,R       (bench only, default 1500,1500,1000,1500)\n"
      "  --props-removed    (required for bench; fixed RC at 100 Hz)\n"
      "Bench is a communication test, not AUTO flight. No AUX/ARM or CLI writes.\n"
      "CSV events: tx/rx/error/timeout/late, code, raw payload, cumulative errors.\n";
    return 0;
  }
  try {
    if (argc < 3) throw std::runtime_error("see --help");
    const bool bench = std::string(argv[1]) == "--bench";
    if (!bench && std::string(argv[1]) != "--monitor")
      throw std::runtime_error("expected --monitor or --bench");
    std::vector<Poll> polls{{"attitude",108,10}, {"rc",105,10},
      {"status",101,5}, {"analog",110,2}, {"battery",130,2}, {"gps",106,2}};
    int baud = 921600, arg = 3;
    if (arg < argc && std::string(argv[arg]).rfind("--", 0) != 0) {
      const double b = number(argv[arg++]);
      if (b != 115200 && b != 460800 && b != 921600)
        throw std::runtime_error("unsupported baud");
      baud = static_cast<int>(b);
    }
    double duration = 10, timeout = 0.1;
    bool props = false;
    std::array<uint16_t,4> channels{1500,1500,1000,1500};
    for (; arg < argc; ++arg) {
      const std::string option = argv[arg];
      if (option == "--props-removed") { props = true; continue; }
      if (++arg >= argc) throw std::runtime_error("missing option value");
      const std::string value = argv[arg];
      if (option == "--duration") {
        duration = number(value);
        if (duration < 0) throw std::runtime_error("negative duration");
      } else if (option == "--timeout-ms") {
        timeout = number(value) / 1000;
        if (timeout < 0.001 || timeout > 5) throw std::runtime_error("invalid timeout");
      } else if (option == "--rates") {
        for (auto& p : polls) p.hz = 0;
        std::istringstream input(value);
        std::string item;
        if (value.empty() || value.back() == ',') throw std::runtime_error("empty rate");
        while (std::getline(input, item, ',')) {
          const auto eq = item.find('=');
          bool found = false;
          for (auto& p : polls) if (p.name == item.substr(0, eq) && eq != std::string::npos) {
            p.hz = number(item.substr(eq+1));
            if (p.hz < 0 || p.hz > 100) throw std::runtime_error("rate outside 0..100 Hz");
            found = true;
          }
          if (!found) throw std::runtime_error("unknown rate: " + item);
        }
      } else if (option == "--rc" && bench) {
        std::istringstream input(value);
        std::string item;
        for (auto& channel : channels) {
          if (!std::getline(input, item, ',')) throw std::runtime_error("expected four AETR channels");
          const double n = number(item);
          if (n < 1000 || n > 2000 || std::floor(n) != n) throw std::runtime_error("invalid RC channel");
          channel = static_cast<uint16_t>(n);
        }
        if (std::getline(input, item, ',') || value.back() == ',') throw std::runtime_error("expected four channels");
      } else throw std::runtime_error("unknown option: " + option);
    }
    if (bench && !props) throw std::runtime_error("bench requires --props-removed; remove propellers before RC injection");
    agi::hardware::BetaflightMspBridge bridge(argv[2], baud);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    std::cout << (bench ? "BENCH RC 100 Hz; no AUTO flight\n" : "MONITOR ONLY\n")
              << "monotonic_s,event,msp_code,payload_hex,error_count\n";
    uint64_t timeouts = 0;
    const auto log = [&](const char* event, const agi::hardware::MspFrame& frame) {
      std::cout << std::fixed << std::setprecision(6) << agi::hardware::monotonicSeconds()
                << ',' << event << ',' << frame.code << ',';
      for (uint8_t b : frame.payload)
        std::cout << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
      std::cout << std::dec << ',' << bridge.errors() + timeouts << '\n';
    };
    // Identity before RC injection; API major and variant must be known.
    for (uint8_t code : {1, 2, 3, 5, 64, 111}) {
      agi::hardware::MspFrame reply;
      if (!bridge.request(code, &reply, agi::hardware::monotonicSeconds() + 0.050))
        throw std::runtime_error("MSP identity query failed: " + std::to_string(code));
      if (bench && ((code == 1 && (reply.payload.size() < 3 || reply.payload[1] != 1)) ||
          (code == 2 && reply.payload != std::vector<uint8_t>({'B','T','F','L'}))))
        throw std::runtime_error("bench requires Betaflight MSP API major 1");
      log("rx", reply);
    }
    const double start = agi::hardware::monotonicSeconds();
    double next_rc = start;
    // Phase telemetry across the first RC interval instead of one TX burst.
    for (size_t i = 0; i < polls.size(); ++i) polls[i].next = start + i * 0.001;
    while (running) {
      double now = agi::hardware::monotonicSeconds();
      if (duration > 0 && now - start >= duration) break;
      if (bench && now >= next_rc) {
        if (!bridge.sendBenchRc(channels, now + 0.002)) throw std::runtime_error("RC write failed");
        log("tx", {200,false,{}});
        advance(next_rc, 0.01, agi::hardware::monotonicSeconds());
      }
      // Bound receive work even if the peer floods the link.
      for (int i = 0; i < 32; ++i) {
        agi::hardware::MspFrame reply;
        if (!bridge.receive(&reply)) break;
        const char* event = reply.error ? "error" : "rx";
        for (auto& p : polls) if (p.code == reply.code) {
          if (!p.pending) event = "late";
          p.pending = false;
          if (reply.error) p.hz = 0;
        }
        log(event, reply);
      }
      for (auto& p : polls) {
        now = agi::hardware::monotonicSeconds();
        if (p.pending && now - p.sent >= timeout) {
          ++timeouts;
          p.pending = false;
          p.hz = 0; // No sequence ID: do not misattribute a late reply to a retry.
          log("timeout", {p.code,false,{}});
        }
        if (p.hz == 0 || p.pending || now < p.next) continue;
        if (bench && next_rc - now < 0.003) break;
        if (!bridge.sendRequest(p.code, now + 0.002)) throw std::runtime_error("telemetry write failed");
        p.sent = agi::hardware::monotonicSeconds();
        p.pending = true;
        advance(p.next, 1 / p.hz, p.sent);
        log("tx", {p.code,false,{}});
      }
      std::this_thread::sleep_for(std::chrono::microseconds(250));
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
