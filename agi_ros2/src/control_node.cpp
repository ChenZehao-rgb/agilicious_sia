#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <agi_ros2/msg/rtk.hpp>
#include <agi_ros2/msg/authority.hpp>
#include <agi_ros2/msg/health.hpp>
#include "agilib/pilot/hardware_pilot.hpp"
#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include "agilib/bridge/betaflight/betaflight_rc_mapper.hpp"
#include "agilib/bridge/betaflight/thrust_table.hpp"
#include "companion_ahrs.hpp"
#include "agilib/reference/trajectory_csv.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <deque>
#include <fstream>
#include <sstream>
#include <cstring>

using agi::hardware::monotonicSeconds;
using agi::hardware::SafetyGate;
using agi_ros2::msg::Rtk;
using agi_ros2::msg::Authority;
using agi_ros2::msg::Health;
static double stamp(const builtin_interfaces::msg::Time& t) { return t.sec + t.nanosec * 1e-9; }

class ControlNode : public rclcpp::Node {
 public:
  ControlNode() : Node("flight_control") {
    mode_ = declare_parameter<std::string>("mode", "sitl");
    if (mode_ != "sitl" && mode_ != "hardware") throw std::invalid_argument("mode must be sitl or hardware");
    params_dir_ = declare_parameter<std::string>("params_dir", "");
    pilot_file_ = declare_parameter<std::string>("pilot_config", "pilot_ros2.yaml");
    bridge_file_ = declare_parameter<std::string>("bridge_config", "betaflight_udp.yaml");
    device_ = declare_parameter<std::string>("device", "/dev/ttyAMA0");
    baud_ = declare_parameter<int>("baud", 921600);
    trajectory_ = declare_parameter<std::string>("trajectory", "");
    thrust_file_ = declare_parameter<std::string>("thrust_table", "");
    if (mode_ == "hardware" && get_parameter("use_sim_time").as_bool())
      throw std::invalid_argument("hardware transport requires use_sim_time=false");
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("sensors/imu", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr p) {
      std::lock_guard<std::mutex> l(mutex_);
      if (imus_.size() >= 256) { imus_.clear(); overflow_ = true; }
      imus_.push_back({*p, monotonicSeconds()});
    });
    rtk_sub_ = create_subscription<Rtk>("sensors/rtk", 10, [this](Rtk::ConstSharedPtr p) { std::lock_guard<std::mutex> l(mutex_); rtk_ = *p; rtk_rx_ = monotonicSeconds(); });
    rc_sub_ = create_subscription<Authority>("authority", 1, [this](Authority::ConstSharedPtr p) { std::lock_guard<std::mutex> l(mutex_); rc_ = *p; rc_rx_ = monotonicSeconds(); });
    health_sub_ = create_subscription<Health>("health", 1, [this](Health::ConstSharedPtr p) { std::lock_guard<std::mutex> l(mutex_); health_ = *p; health_rx_ = monotonicSeconds(); });
    state_pub_ = create_publisher<nav_msgs::msg::Odometry>("state", 1);
    status_pub_ = create_publisher<std_msgs::msg::String>("status", 1);
    // Fail configuration synchronously before starting either worker.
    params_ = std::make_unique<agi::PilotParams>(std::filesystem::path(params_dir_) / pilot_file_, params_dir_);
    if (!bridge_params_.load(std::filesystem::path(params_dir_) / bridge_file_)) throw std::runtime_error("invalid RC mapping");
    if (!thrust_file_.empty()) loadThrust();
    loadTrajectory();
    output_thread_ = std::thread([this] { outputLoop(); });
    control_thread_ = std::thread([this] { controlLoop(); });
    diagnostics_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      std_msgs::msg::String msg;
      { std::lock_guard<std::mutex> l(output_mutex_); msg.data = status_; }
      status_pub_->publish(msg);
    });
  }
  ~ControlNode() override {
    stop_ = true;
    if (control_thread_.joinable()) control_thread_.join();
    if (output_thread_.joinable()) output_thread_.join();
  }
 private:
  void loadThrust() {
    // CSV: first row 0,pwm1,pwm2,...; remaining rows voltage,total_N1,total_N2,...
    std::ifstream f(thrust_file_); if (!f) throw std::runtime_error("cannot open thrust table");
    std::vector<double> volts, pwm; std::vector<std::vector<double>> forces;
    std::string line; bool first = true;
    while (std::getline(f,line)) {
      std::replace(line.begin(),line.end(),',',' '); std::istringstream in(line);
      double x; std::vector<double> row; while(in>>x) row.push_back(x);
      if (row.size()<3) throw std::runtime_error("bad thrust table row");
      if (first) { pwm.assign(row.begin()+1,row.end()); first=false; }
      else { volts.push_back(row.front()); forces.emplace_back(row.begin()+1,row.end()); }
    }
    thrust_ = std::make_unique<agi::hardware::ThrustTable>(volts,pwm,forces);
  }
  void loadTrajectory() {
    if (trajectory_.empty()) return;
    const auto rows=agi::trajectory_csv::readTrajectoryRows(trajectory_);
    points_=agi::trajectory_csv::loadTrajectory(rows,0,agi::Vector<3>::Zero(),0,
      agi::trajectory_csv::estimateSourceMass(rows),-std::numeric_limits<double>::infinity());
  }

  void controlLoop() {
    try {
      agi::hardware::HardwarePilot pilot(*params_, [this]{return now().seconds();}, monotonicSeconds);
      if (!points_.empty() && !pilot.setTrajectory(points_)) throw std::runtime_error("invalid trajectory timestamps/states");
      auto ahrs = std::make_unique<agi::CompanionAhrs>(agi::CompanionAhrs::Params{});
      agi::QuadState state; state.setZero(); state.t=NAN;
      double last_clock=NAN, last_rtk=NAN, imu_live=NAN;
      auto next=std::chrono::steady_clock::now();
      while (!stop_ && rclcpp::ok()) {
        next += std::chrono::milliseconds(10);
        const double t=now().seconds(), wall=monotonicSeconds();
        std::deque<std::pair<sensor_msgs::msg::Imu,double>> samples;
        Rtk rtk; Authority rc; Health health; double rr, cr, hr; bool overflow;
        { std::lock_guard<std::mutex> l(mutex_); samples.swap(imus_); rtk=rtk_; rc=rc_; health=health_; rr=rtk_rx_; cr=rc_rx_; hr=health_rx_; overflow=overflow_; overflow_=false; }
        const bool jump=std::isfinite(last_clock) && (t<last_clock || t-last_clock>0.25);
        last_clock=t;
        if(jump || overflow) {
          pilot.reportOutputFault(); ahrs=std::make_unique<agi::CompanionAhrs>(agi::CompanionAhrs::Params{});
          state.t=NAN; last_rtk=NAN; imu_live=NAN; samples.clear();
          { std::lock_guard<std::mutex> l(mutex_); imus_.clear(); rtk_rx_=rc_rx_=health_rx_=NAN; }
          rr=cr=hr=NAN;
        }
        const double rt=stamp(rtk.header.stamp);
        if(SafetyGate::fresh(wall,rr,0.3) && SafetyGate::fresh(t,rt,0.3) && rtk.header.frame_id=="odom" && rtk.fixed && rtk.heading_valid && rtk.accuracy_ok && std::isfinite(rtk.heading) && (!std::isfinite(last_rtk)||rt>last_rtk)) {
          ahrs->setHeading(rtk.heading);
          agi::Vector<3> p(rtk.position.x,rtk.position.y,rtk.position.z), v(rtk.velocity.x,rtk.velocity.y,rtk.velocity.z);
          if(p.allFinite() && v.allFinite()) { state.p=p+v*std::max(0.0,(std::isfinite(state.t)?state.t:rt)-rt); state.v=v; ahrs->setVelocity(v,rt); last_rtk=rt; }
        }
        for(const auto& item:samples) {
          const auto& m=item.first; const double it=stamp(m.header.stamp);
          if(m.header.frame_id!="base_link" || !SafetyGate::fresh(t,it,0.1) || (std::isfinite(state.t)&&it<=state.t)) continue;
          agi::ImuSample imu(it,{m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z},{m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z});
          if(!imu.valid()) continue;
          ahrs->addImu(imu);
          if(!ahrs->initialized()) continue;
          const double dt=std::isfinite(state.t)?it-state.t:0;
          state.q(ahrs->attitude()); state.w=imu.omega-ahrs->gyroBias();
          state.a=state.q()*imu.acc+agi::Vector<3>(0,0,-9.8066);
          if(dt>0 && dt<=0.025 && std::isfinite(last_rtk)) { state.p+=state.v*dt+0.5*state.a*dt*dt; state.v+=state.a*dt; }
          state.t=it; imu_live=item.second;
        }
        agi::hardware::Evidence e;
        e.now=wall; e.imu_time=imu_live;
        // Preserve acquisition age as well as receipt watchdogs; never make a stale fix fresh.
        e.rtk_time=std::isfinite(last_rtk)?wall-(t-last_rtk):NAN;
        e.rc_time=wall-(t-stamp(rc.header.stamp));
        e.rc_link=rc.rc_link && SafetyGate::fresh(wall,cr,0.1);
        e.armed=rc.armed; e.auto_switch=rc.auto_switch; e.kill=rc.kill;
        e.rtk_fixed=rtk.fixed && SafetyGate::fresh(wall,rr,0.3); e.heading_valid=rtk.heading_valid; e.accuracy_ok=rtk.accuracy_ok; e.synchronized=rtk.synchronized;
        const bool h=SafetyGate::fresh(wall,hr,0.2)&&SafetyGate::fresh(t,stamp(health.header.stamp),0.2);
        e.imu_calibrated=h&&health.imu_calibrated; e.converged=h&&health.converged&&ahrs->initialized();
        e.config_verified=h&&health.config_verified; e.thrust_calibrated=h&&health.thrust_calibrated&&(mode_=="sitl"||bool(thrust_));
        e.geofence_ok=h&&health.geofence_ok; e.msp_healthy=h&&health.transport_healthy&&transport_ok_;
        if(output_fault_.exchange(false)) pilot.reportOutputFault();
        auto decision=pilot.tick(state,e);
        { std::lock_guard<std::mutex> l(output_mutex_); decision_=decision; output_rc_=rc; voltage_=health.battery_voltage; status_=std::string(SafetyGate::name(decision.mode))+": "+decision.reason; }
        if(state.valid()) {
          nav_msgs::msg::Odometry msg; msg.header.stamp=rclcpp::Time(static_cast<int64_t>(state.t*1e9)); msg.header.frame_id="odom"; msg.child_frame_id="base_link";
          msg.pose.pose.position.x=state.p.x(); msg.pose.pose.position.y=state.p.y(); msg.pose.pose.position.z=state.p.z();
          msg.pose.pose.orientation.w=state.q().w(); msg.pose.pose.orientation.x=state.q().x(); msg.pose.pose.orientation.y=state.q().y(); msg.pose.pose.orientation.z=state.q().z();
          const auto v=state.q().conjugate()*state.v;
          msg.twist.twist.linear.x=v.x(); msg.twist.twist.linear.y=v.y(); msg.twist.twist.linear.z=v.z();
          msg.twist.twist.angular.x=state.w.x(); msg.twist.twist.angular.y=state.w.y(); msg.twist.twist.angular.z=state.w.z();
          state_pub_->publish(msg);
        }
        std::this_thread::sleep_until(next);
        if(std::chrono::steady_clock::now()>next+std::chrono::milliseconds(10)) next=std::chrono::steady_clock::now();
      }
    } catch(const std::exception& e) { RCLCPP_ERROR(get_logger(),"control stopped: %s",e.what()); stop_=true; rclcpp::shutdown(); }
  }
  void outputLoop() {
    int fd=-1; sockaddr_in destination{};
    try {
      std::unique_ptr<agi::hardware::BetaflightMspBridge> msp;
      if(mode_=="hardware") msp=std::make_unique<agi::hardware::BetaflightMspBridge>(device_,baud_);
      else {
        fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0); destination.sin_family=AF_INET; destination.sin_port=htons(bridge_params_.port);
        if(fd<0 || inet_pton(AF_INET,bridge_params_.host.c_str(),&destination.sin_addr)!=1) throw std::runtime_error("UDP setup failed");
      }
      transport_ok_=true;
      agi::BetaflightRcMapper mapper(bridge_params_);
      auto next=std::chrono::steady_clock::now();
      while(!stop_ && rclcpp::ok()) {
        next+=std::chrono::milliseconds(10);
        agi::hardware::ControlDecision d; Authority rc; double voltage;
        { std::lock_guard<std::mutex> l(output_mutex_); d=decision_; rc=output_rc_; voltage=voltage_; }
        d.evidence.now=monotonicSeconds();
        bool active=d.permit_override && SafetyGate::inputsHealthy(d.evidence) && SafetyGate::fresh(d.evidence.now,d.evidence.command_time,0.025);
        std::array<uint16_t,4> channels{1500,1500,1000,1500};
        if(active) {
          try {
            const auto rates=d.command.omega*(180/M_PI);
            channels[0]=mapper.rateToPwm(mapper.inverseActualRate(rates.x(),0),bridge_params_.deadband);
            // Real Betaflight body frame is FRD; SITL model already flips sensor axes.
            const double sign=mode_=="hardware"?-1:1;
            channels[1]=mapper.rateToPwm(mapper.inverseActualRate(sign*rates.y(),1),bridge_params_.deadband);
            channels[3]=mapper.rateToPwm(mapper.inverseActualRate(sign*rates.z(),2),bridge_params_.yaw_deadband);
            if(msp) channels[2]=thrust_->collectiveThrustToRc(d.command.collective_thrust,params_->quad_.m_,voltage);
            else {
              const double motor=(bridge_params_.motor_idle+(1-bridge_params_.motor_idle)*bridge_params_.hover_throttle)*std::sqrt(d.command.collective_thrust/9.8066);
              channels[2]=static_cast<uint16_t>(std::lround(bridge_params_.min_check+(2000-bridge_params_.min_check)*std::clamp((motor-bridge_params_.motor_idle)/(1-bridge_params_.motor_idle),0.0,1.0)));
            }
          } catch(const std::exception&) { active=false; d.evidence.command_valid=false; output_fault_=true; }
        }
        if(msp) {
          // Feed manual/low decisions too: the transport's second gate must observe AUTO low.
          if(!msp->sendOverride(channels,d.evidence,monotonicSeconds()+0.003) && active) { output_fault_=true; transport_ok_=false; }
        } else {
          const bool rc_ok=SafetyGate::fresh(d.evidence.now,d.evidence.rc_time,0.1)&&d.evidence.rc_link&&!rc.kill;
          if(!active && rc_ok && !rc.auto_switch) channels=rc.manual_aetr;
          const bool arm=rc_ok&&rc.armed&&(!rc.auto_switch||active);
          sendUdp(fd,destination,channels,arm);
        }
        std::this_thread::sleep_until(next);
        if(std::chrono::steady_clock::now()>next+std::chrono::milliseconds(10)) next=std::chrono::steady_clock::now();
      }
    } catch(const std::exception& e) { transport_ok_=false; RCLCPP_ERROR(get_logger(),"output stopped: %s",e.what()); stop_=true; rclcpp::shutdown(); }
    if(fd>=0) { sendUdp(fd,destination,{1500,1500,1000,1500},false); close(fd); }
  }
  void sendUdp(int fd,const sockaddr_in& destination,const std::array<uint16_t,4>& channels,bool arm) {
    std::array<uint8_t,40> bytes{}; double t=monotonicSeconds(); uint64_t bits; std::memcpy(&bits,&t,8);
    for(int i=0;i<8;++i) bytes[i]=(bits>>(8*i))&255;
    for(int i=0;i<16;++i) { uint16_t v=i<4?channels[i]:1000; if(i==4&&arm)v=2000; if(v<1000||v>2000)v=1000; bytes[8+2*i]=v&255; bytes[9+2*i]=v>>8; }
    if(sendto(fd,bytes.data(),bytes.size(),0,reinterpret_cast<const sockaddr*>(&destination),sizeof(destination))!=40) { output_fault_=true; transport_ok_=false; }
  }
  std::string mode_,params_dir_,pilot_file_,bridge_file_,device_,trajectory_,thrust_file_; int baud_;
  std::unique_ptr<agi::PilotParams> params_; agi::BetaflightUdpBridgeParams bridge_params_;
  std::unique_ptr<agi::hardware::ThrustTable> thrust_; agi::SetpointVector points_;
  std::mutex mutex_,output_mutex_; std::deque<std::pair<sensor_msgs::msg::Imu,double>> imus_;
  Rtk rtk_; Authority rc_,output_rc_; Health health_;
  double rtk_rx_{NAN},rc_rx_{NAN},health_rx_{NAN},voltage_{NAN}; bool overflow_{false};
  agi::hardware::ControlDecision decision_; std::string status_{"BOOT"};
  std::atomic<bool> stop_{false},transport_ok_{false},output_fault_{false};
  std::thread control_thread_,output_thread_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Rtk>::SharedPtr rtk_sub_; rclcpp::Subscription<Authority>::SharedPtr rc_sub_; rclcpp::Subscription<Health>::SharedPtr health_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_; rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<ControlNode>()); }
  catch(const std::exception& e) { fprintf(stderr,"agi_ros2: %s\n",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
