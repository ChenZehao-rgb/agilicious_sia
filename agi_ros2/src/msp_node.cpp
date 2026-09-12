#include "agi_ros2/msp_telemetry.h"
#include <chrono>
#include <memory>
#include <stdexcept>

namespace agi_ros2 {
class MspNode : public rclcpp::Node {
 public:
  MspNode() : Node("betaflight_msp") {
    if (get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("MSP hardware requires wall time");
    const auto mode = declare_parameter<std::string>("mode", "monitor");
    if (mode != "monitor" && mode != "bench") throw std::invalid_argument("mode must be monitor or bench");
    bench_ = mode == "bench";
    const bool props = declare_parameter<bool>("props_removed", false);
    if (bench_ && !props) throw std::invalid_argument("Remove propellers and set props_removed=true for bench");
    const auto rc = declare_parameter<std::vector<int64_t>>("bench_aetr", {1500,1500,1000,1500});
    if (rc.size() != 4) throw std::invalid_argument("bench_aetr requires four channels");
    for (size_t i=0; i<4; ++i) {
      if (rc[i]<1000 || rc[i]>2000) throw std::invalid_argument("bench_aetr must be 1000..2000");
      channels_[i] = rc[i];
    }
    telemetry_ = std::make_unique<MspTelemetry>(*this);
    const auto device = declare_parameter<std::string>("device", "/dev/ttyAMA0");
    const auto baud = declare_parameter<int>("baud", 115200);
    bridge_ = std::make_unique<agi::hardware::BetaflightMspBridge>(device, baud);
    // Bench state writes require the expected protocol family.
    if (bench_) for (uint8_t code : {1,2}) {
      agi::hardware::MspFrame reply;
      if (!bridge_->request(code, &reply, agi::hardware::monotonicSeconds()+.05) ||
          (code == 1 && (reply.payload.size()<3 || reply.payload[1]!=1)) ||
          (code == 2 && reply.payload != std::vector<uint8_t>({'B','T','F','L'})))
        throw std::runtime_error("Bench requires Betaflight MSP API major 1");
    }
    next_ = agi::hardware::monotonicSeconds();
    timer_ = create_wall_timer(std::chrono::milliseconds(1), [this] {
      const double now = agi::hardware::monotonicSeconds();
      if (bench_ && now >= next_) {
        if (!bridge_->sendBenchRc(channels_, now+.002)) throw std::runtime_error("MSP RC write failed");
        telemetry_->sentRc(channels_, bridge_->errors());
        next_ += (std::floor((agi::hardware::monotonicSeconds()-next_)/.01)+1)*.01;
      }
      telemetry_->tick(*bridge_, bench_ ? next_-.001 : now+.003);
    });
    RCLCPP_INFO(get_logger(), "MSP %s: %s @ %d; telemetry on /msp/*", mode.c_str(), device.c_str(), static_cast<int>(baud));
  }
 private:
  bool bench_{false};
  double next_{0};
  std::array<uint16_t,4> channels_{};
  std::unique_ptr<agi::hardware::BetaflightMspBridge> bridge_;
  std::unique_ptr<MspTelemetry> telemetry_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<agi_ros2::MspNode>()); }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("betaflight_msp"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
