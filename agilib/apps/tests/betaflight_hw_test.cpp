#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include "agilib/bridge/betaflight/thrust_table.hpp"
#include <pty.h>
#include <poll.h>
#include <unistd.h>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace agi::hardware;
namespace {
void require(bool condition) {
  if (!condition) throw std::runtime_error("hardware test assertion failed");
}
Evidence healthy(double now = 10) {
  Evidence e;
  e.now = e.imu_time = e.rtk_time = e.rc_time = e.command_time = now;
  e.solve_seconds = 0.004;
  e.rtk_fixed = e.heading_valid = e.accuracy_ok = e.imu_calibrated = true;
  e.synchronized = e.converged = e.config_verified = e.thrust_calibrated = true;
  e.geofence_ok = e.msp_healthy = e.command_valid = e.controller_warm = true;
  e.armed = e.rc_link = true;
  e.kill = false;
  return e;
}
void safetyTests() {
  SafetyGate gate;
  auto e = healthy();
  e.auto_switch = true;
  require(!gate.update(e)); // reboot while switch high
  e.auto_switch = false;
  require(!gate.update(e));
  e.auto_switch = true;
  require(gate.update(e));
  require(gate.update(e));
  e.imu_time -= 0.011;
  require(!gate.update(e));
  e.imu_time = e.now;
  require(!gate.update(e)); // no automatic recovery
  e.auto_switch = false; require(!gate.update(e));
  e.auto_switch = true; require(gate.update(e));
  e.kill = true; require(!gate.update(e));
  for (int failure = 0; failure < 10; ++failure) {
    SafetyGate g;
    e = healthy(); g.update(e);
    e.auto_switch = true; require(g.update(e));
    switch (failure) {
      case 0: e.rtk_fixed = false; break;
      case 1: e.rc_link = false; break;
      case 2: e.command_time -= 0.030; break;
      case 3: e.solve_seconds = 0.009; break;
      case 4: e.rtk_time -= 0.301; break;
      case 5: e.imu_time += 0.001; break;
      case 6: e.now = NAN; break;
      case 7: e.thrust_calibrated = false; break;
      case 8: e.armed = false; break;
      case 9: e.geofence_ok = false; break;
    }
    require(!g.update(e));
    e = healthy(); e.auto_switch = true;
    require(!g.update(e));
  }
}
void parserTests() {
  MspDecoder decoder;
  const uint8_t v1[] = {'$', 'M', '>', 2, 101, 1, 2, 100};
  MspFrame frame;
  for (size_t i = 0; i < sizeof(v1); ++i) {
    decoder.append(v1 + i, 1);
    require(decoder.next(&frame) == (i == sizeof(v1) - 1));
  }
  require(frame.code == 101 && frame.payload == std::vector<uint8_t>({1,2}));
  auto bad = std::vector<uint8_t>(v1, v1 + sizeof(v1));
  bad.back() ^= 1;
  decoder.append(bad.data(), bad.size()); decoder.append(v1, sizeof(v1));
  require(decoder.next(&frame) && decoder.errors() > 0);
  std::vector<uint8_t> v2{'$', 'X', '!', 0, 0x34, 0x12, 1, 0, 42};
  uint8_t crc = 0;
  for (size_t i = 3; i < v2.size(); ++i) {
    crc ^= v2[i];
    for (int b = 0; b < 8; ++b) crc = crc & 128 ? (crc << 1) ^ 0xd5 : crc << 1;
  }
  v2.push_back(crc);
  decoder.append(v2.data(), v2.size());
  require(decoder.next(&frame) && frame.error && frame.code == 0x1234);
  std::vector<uint8_t> flood(5000, 0);
  decoder.append(flood.data(), flood.size());
  decoder.append(v1, sizeof(v1)); require(decoder.next(&frame));
}
void thrustTests() {
  ThrustTable table({12,16}, {1000,1500,2000}, {{0,10,20},{0,20,40}});
  require(table.collectiveThrustToRc(15, 1, 14) == 1500);
  require(table.collectiveThrustToRc(10, 2, 16) == 1500);
  require(table.collectiveThrustToRc(0, 1, 12) == 1000);
  for (double volts : {11.0,17.0,static_cast<double>(NAN)}) {
    bool caught = false;
    try { table.collectiveThrustToRc(10,1,volts); }
    catch (const std::out_of_range&) { caught = true; }
    require(caught);
  }
}
void transportTests() {
  int master, slave;
  char path[128];
  require(openpty(&master, &slave, path, nullptr, nullptr) == 0);
  close(slave);
  {
    BetaflightMspBridge bridge(path);
    auto e = healthy(monotonicSeconds());
    require(!bridge.sendOverride({1500,1500,1200,1500}, e, monotonicSeconds()+0.01));
    e.auto_switch = true;
    require(bridge.sendOverride({1500,1500,1200,1500}, e, monotonicSeconds()+0.01));
    uint8_t bytes[64];
    pollfd p{master, POLLIN, 0};
    require(poll(&p, 1, 50) == 1);
    const ssize_t count = read(master, bytes, sizeof(bytes));
    require(count == 14 && bytes[3] == 8 && bytes[4] == 200);
    require(bytes[9] == (1200 & 255) && bytes[10] == (1200 >> 8));
    e.kill = true;
    require(!bridge.sendOverride({1500,1500,1000,1500}, e, monotonicSeconds()+0.01));
    require(poll(&p, 1, 20) == 0); // no low-throttle/AUX shutdown frame
    bool wrong_thread = false;
    std::thread wrong([&] {
      MspFrame reply;
      try { bridge.request(101, &reply, monotonicSeconds()+0.01); }
      catch (const std::logic_error&) { wrong_thread = true; }
    });
    wrong.join(); require(wrong_thread);
    MspFrame reply;
    const double start = monotonicSeconds();
    require(!bridge.request(101, &reply, start + 0.005));
    require(monotonicSeconds() - start < 0.05);
    e = healthy(monotonicSeconds());
    bridge.sendOverride({1500,1500,1200,1500}, e, monotonicSeconds()+0.01);
    e.auto_switch = true;
    require(!bridge.sendOverride({1500,1500,1200,1500}, e, monotonicSeconds()+0.01));
  }
  close(master);
}
}
int main() {
  try {
    safetyTests(); parserTests(); thrustTests(); transportTests();
    std::cout << "PASS: safety, MSP v1/v2, calibrated thrust, pseudo-terminal transport\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
