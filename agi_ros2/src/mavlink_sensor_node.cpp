#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <common/mavlink.h>
#include "agi_ros2/msg/heading.hpp"

#include <sys/file.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
double steady() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
constexpr double pi = 3.14159265358979323846;
constexpr double nan = std::numeric_limits<double>::quiet_NaN();

class MavlinkSensorNode : public rclcpp::Node {
 public:
  MavlinkSensorNode() : Node("mavlink_sensor") {
    if (get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("MAVLink hardware requires use_sim_time=false");
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = true;
    device_ = declare_parameter<std::string>("device", "", desc);
    if (device_.empty()) throw std::invalid_argument("device must name a dedicated MAVLink serial port");
    baud_ = declare_parameter<int>("baud", 921600, desc);
    if (baud_ != 115200 && baud_ != 230400 && baud_ != 460800 && baud_ != 921600) throw std::invalid_argument("Unsupported baud");
    sys_ = declare_parameter<int>("system_id", 1, desc);
    comp_ = declare_parameter<int>("component_id", 1, desc);
    if (sys_ < 1 || sys_ > 255 || comp_ < 1 || comp_ > 255) throw std::invalid_argument("Invalid source IDs");
    gps_mode_ = declare_parameter<std::string>("gps_mode", "gnss", desc);
    altitude_source_ = declare_parameter<std::string>("altitude_source", "unknown", desc);
    if (gps_mode_ != "gnss" && gps_mode_ != "rtk") throw std::invalid_argument("gps_mode must be gnss or rtk");
    if (altitude_source_ != "unknown" && altitude_source_ != "ellipsoid") throw std::invalid_argument("altitude_source must be unknown or ellipsoid");
    const auto rate = [&](const char *name, int value, int max) {
      int v = declare_parameter<int>(name, value, desc);
      if (v < 0 || v > max) throw std::invalid_argument(name);
      return v;
    };
    imu_hz_ = rate("imu_rate_hz", 500, 500);
    gps_hz_ = rate("gps_rate_hz", 10, 50);
    attitude_hz_ = rate("attitude_rate_hz", 0, 100);
    acc_variance_ = declare_parameter<std::vector<double>>("acceleration_variance", {0.,0.,0.}, desc);
    gyro_variance_ = declare_parameter<std::vector<double>>("angular_velocity_variance", {0.,0.,0.}, desc);
    for (const auto &v : {acc_variance_, gyro_variance_}) {
      if (v.size() != 3 || std::any_of(v.begin(), v.end(), [](double x){ return !std::isfinite(x) || x < 0; }))
        throw std::invalid_argument("Variance must have three finite nonnegative entries (all zero means unknown)");
    }
    auto qos = rclcpp::SensorDataQoS();
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("sensors/imu", qos);
    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("sensors/gps/fix", qos);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("sensors/gps/velocity", qos);
    attitude_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>("sensors/fc_attitude", qos);
    heading_pub_ = create_publisher<agi_ros2::msg::Heading>("sensors/fc_heading", qos);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("sensors/mavlink/status", rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(std::chrono::milliseconds(1), [this]{ tick(); });
    diagnostic_timer_ = create_wall_timer(std::chrono::seconds(1), [this]{ diagnose(); });
    diagnostic_time_ = steady();
  }
  ~MavlinkSensorNode() override { disconnect(); }

 private:
  struct Tx { std::vector<uint8_t> bytes; size_t offset{0}; };
  struct Fix { mavlink_gps_raw_int_t value; double arrival; };
  struct Position { mavlink_global_position_int_t value; double arrival; };
  void resetEpoch() {
    sync_count_ = 0;
    offset_ = nan;
    last_remote_ns_ = 0;
    requests_.clear();
    last_stamp_.clear();
    fixes_.clear(); positions_.clear();
    gps_fix_type_ = 0;
    heading_valid_ = false;
    last_heading_receive_ = 0;
    last_fix_receive_ = 0;
    velocity_valid_ = false;
    command_index_ = 0; command_attempts_ = 0; command_waiting_ = false;
    commands_failed_ = false;
    next_sync_ = 0;
  }
  void disconnect() {
    if (fd_ >= 0) { ioctl(fd_, TIOCNXCL); close(fd_); }
    fd_ = -1;
    tx_.clear();
    resetEpoch();
  }
  void connectPort(double t) {
    next_connect_ = t + 1;
    int fd = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) { reason_ = "Cannot open serial port"; return; }
    if (flock(fd, LOCK_EX | LOCK_NB) || ioctl(fd, TIOCEXCL)) { close(fd); reason_ = "Serial port is not exclusive"; return; }
    termios cfg{};
    if (tcgetattr(fd, &cfg)) { ioctl(fd, TIOCNXCL); close(fd); return; }
    cfmakeraw(&cfg);
    cfg.c_cflag |= CLOCAL | CREAD;
    cfg.c_cflag &= ~CRTSCTS;
    speed_t baud = baud_ == 921600 ? B921600 : baud_ == 460800 ? B460800 : baud_ == 230400 ? B230400 : B115200;
    cfsetispeed(&cfg, baud); cfsetospeed(&cfg, baud);
    if (tcsetattr(fd, TCSANOW, &cfg)) { ioctl(fd, TIOCNXCL); close(fd); return; }
    tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    parser_ = {}; parser_status_ = {};
    resetEpoch();
    last_receive_ = t;
    reason_ = "Waiting for clock synchronization";
  }
  bool enqueue(const mavlink_message_t &m) {
    if (tx_.size() >= 16) { ++tx_errors_; return false; }
    Tx tx; tx.bytes.resize(MAVLINK_MAX_PACKET_LEN);
    tx.bytes.resize(mavlink_msg_to_send_buffer(tx.bytes.data(), &m));
    tx_.push_back(std::move(tx));
    return true;
  }
  void flush() {
    size_t budget = 2048;
    while (!tx_.empty() && budget) {
      auto &tx = tx_.front();
      ssize_t n = write(fd_, tx.bytes.data() + tx.offset, std::min(budget, tx.bytes.size() - tx.offset));
      if (n < 0 && (errno == EAGAIN || errno == EINTR)) return;
      if (n <= 0) { reason_ = "Serial write failed"; ++tx_errors_; disconnect(); return; }
      tx.offset += n; budget -= n;
      if (tx.offset == tx.bytes.size()) tx_.pop_front();
    }
  }
  void tick() {
    double t = steady();
    double ros_offset = now().seconds() - t;
    if (std::isfinite(ros_offset_) && std::abs(ros_offset - ros_offset_) > .05) {
      resetEpoch(); tx_.clear(); reason_ = "ROS clock jumped; resynchronizing";
    }
    ros_offset_ = ros_offset;
    if (fd_ < 0) { if (t >= next_connect_) connectPort(t); return; }
    std::array<uint8_t, 4096> buffer{};
    size_t budget = 16384;
    while (budget) {
      ssize_t n = read(fd_, buffer.data(), std::min(buffer.size(), budget));
      if (n < 0 && (errno == EAGAIN || errno == EINTR)) break;
      if (n < 0) { reason_ = "Serial read failed"; disconnect(); return; }
      if (n == 0) break;
      budget -= n;
      for (ssize_t i = 0; i < n; ++i) {
        mavlink_message_t msg{}; mavlink_status_t status{};
        auto framing = mavlink_frame_char_buffer(&parser_, &parser_status_, buffer[i], &msg, &status);
        if (framing == MAVLINK_FRAMING_BAD_CRC || framing == MAVLINK_FRAMING_BAD_SIGNATURE) { ++bad_frames_; continue; }
        if (framing != MAVLINK_FRAMING_OK) continue;
        if (msg.sysid != sys_ || msg.compid != comp_) { ++wrong_source_; continue; }
        last_receive_ = steady();
        handle(msg);
      }
    }
    t = steady();
    if (t - last_receive_ > 2) { reason_ = "Telemetry timeout"; disconnect(); return; }
    if (sync_count_ && t - last_sync_ > 2) { resetEpoch(); reason_ = "Time synchronization expired"; }
    if (t >= next_sync_ && tx_.empty()) {
      next_sync_ = t + .1;
      int64_t token = static_cast<int64_t>(t * 1e9);
      requests_[token] = t;
      while (requests_.size() > 20) requests_.erase(requests_.begin());
      mavlink_message_t msg{};
      mavlink_msg_timesync_pack(245, 191, &msg, 0, token, sys_, comp_);
      enqueue(msg);
    }
    configureRates(t);
    flush();
    pairGps();
  }
  void configureRates(double t) {
    if (command_index_ == 4) return;
    if (command_waiting_ && t - command_sent_ < .5) return;
    if (command_attempts_ >= 3) {
      commands_failed_ = true;
      ++command_index_; command_attempts_ = 0; command_waiting_ = false;
      return;
    }
    const std::array<int,4> ids{105,24,33,30};
    const std::array<int,4> hz{imu_hz_,gps_hz_,gps_hz_,attitude_hz_};
    mavlink_message_t msg{};
    float interval = hz[command_index_] ? 1e6f / hz[command_index_] : -1;
    mavlink_msg_command_long_pack(245,191,&msg,sys_,comp_,MAV_CMD_SET_MESSAGE_INTERVAL,0,
      ids[command_index_],interval,0,0,0,0,0);
    if (enqueue(msg)) { command_sent_ = t; command_waiting_ = true; ++command_attempts_; }
  }
  void timesync(const mavlink_message_t &m) {
    mavlink_timesync_t ts{}; mavlink_msg_timesync_decode(&m,&ts);
    if (ts.tc1 <= 0 || (ts.target_system && ts.target_system != 245) || (ts.target_component && ts.target_component != 191)) return;
    auto it = requests_.find(ts.ts1);
    if (it == requests_.end()) return;
    double sent = it->second, received = steady(); requests_.erase(it);
    double rtt = received - sent;
    if (rtt <= 0 || rtt > .020) { ++rejected_sync_; return; }
    if (last_remote_ns_ && ts.tc1 < last_remote_ns_) { resetEpoch(); tx_.clear(); reason_ = "Flight controller restarted"; return; }
    last_remote_ns_ = ts.tc1;
    double candidate = (sent + received) * .5 - ts.tc1 * 1e-9;
    if (std::isfinite(offset_) && std::abs(candidate - offset_) > .05) { resetEpoch(); reason_ = "Device clock discontinuity"; return; }
    offset_ = std::isfinite(offset_) ? .9 * offset_ + .1 * candidate : candidate;
    rtt_ms_ = rtt * 1000; last_sync_ = received; ++sync_count_;
  }
  bool stamp(uint32_t id, double remote, builtin_interfaces::msg::Time &out) {
    if (sync_count_ < 5 || !std::isfinite(remote)) return false;
    double mapped = remote + offset_ + ros_offset_;
    double age = now().seconds() - mapped;
    if (age < -.02 || age > (id == 105 ? .05 : .5)) { ++stale_; return false; }
    auto it = last_stamp_.find(id);
    if (it != last_stamp_.end() && remote <= it->second) { ++duplicates_; return false; }
    last_stamp_[id] = remote;
    out = rclcpp::Time(static_cast<int64_t>(mapped * 1e9));
    return true;
  }
  double bootMs(uint32_t ms) const {
    // Lift the 32-bit millisecond timestamp to the current synchronized epoch.
    constexpr double wrap = 4294967.296;
    double now_remote = last_remote_ns_ * 1e-9;
    double value = ms * .001;
    return value + std::round((now_remote - value) / wrap) * wrap;
  }
  void handle(const mavlink_message_t &m) {
    if (m.msgid == MAVLINK_MSG_ID_TIMESYNC) { timesync(m); return; }
    if (m.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
      mavlink_command_ack_t ack{}; mavlink_msg_command_ack_decode(&m,&ack);
      if (ack.command == MAV_CMD_SET_MESSAGE_INTERVAL && command_waiting_ &&
          (!ack.target_system || ack.target_system == 245) && (!ack.target_component || ack.target_component == 191)) {
        if (ack.result == MAV_RESULT_IN_PROGRESS) return;
        if (ack.result != MAV_RESULT_ACCEPTED) commands_failed_ = true;
        ++command_index_; command_attempts_ = 0; command_waiting_ = false;
      }
      return;
    }
    if (sync_count_ < 5 || publisher_conflict_) return;
    if (m.msgid == MAVLINK_MSG_ID_HIGHRES_IMU && imu_hz_) {
      mavlink_highres_imu_t v{}; mavlink_msg_highres_imu_decode(&m,&v);
      if ((v.fields_updated & 63) != 63) return;
      for (float x : {v.xacc,v.yacc,v.zacc,v.xgyro,v.ygyro,v.zgyro}) if (!std::isfinite(x)) return;
      sensor_msgs::msg::Imu out;
      if (!stamp(105,v.time_usec*1e-6,out.header.stamp)) return;
      out.header.frame_id = "base_link";
      out.linear_acceleration.x=v.xacc; out.linear_acceleration.y=-v.yacc; out.linear_acceleration.z=-v.zacc;
      out.angular_velocity.x=v.xgyro; out.angular_velocity.y=-v.ygyro; out.angular_velocity.z=-v.zgyro;
      out.orientation_covariance[0]=-1;
      for (int i=0;i<3;++i) { out.linear_acceleration_covariance[i*4]=acc_variance_[i]; out.angular_velocity_covariance[i*4]=gyro_variance_[i]; }
      imu_pub_->publish(out); ++counts_[0]; last_imu_receive_=steady();
    } else if (m.msgid == MAVLINK_MSG_ID_GPS_RAW_INT && gps_hz_) {
      mavlink_gps_raw_int_t v{}; mavlink_msg_gps_raw_int_decode(&m,&v);
      sensor_msgs::msg::NavSatFix out;
      if (!stamp(24,v.time_usec*1e-6,out.header.stamp)) return;
      out.header.frame_id="gps_link";
      const bool valid = v.fix_type >= 3 && v.fix_type <= 6 && std::abs(static_cast<int64_t>(v.lat)) <= 900000000 && std::abs(static_cast<int64_t>(v.lon)) <= 1800000000;
      out.status.status = valid ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      out.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      out.latitude=valid ? v.lat*1e-7 : nan; out.longitude=valid ? v.lon*1e-7 : nan;
      out.altitude = valid && altitude_source_ == "ellipsoid" && v.alt_ellipsoid != INT32_MIN ? v.alt_ellipsoid*.001 : nan;
      if (valid && v.h_acc && v.v_acc) {
        out.position_covariance[0]=out.position_covariance[4]=std::pow(v.h_acc*.001,2);
        out.position_covariance[8]=std::pow(v.v_acc*.001,2);
        out.position_covariance_type=sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
      }
      gps_fix_type_=valid ? v.fix_type : 1; msl_=v.alt*.001;
      ellipsoid_valid_=std::isfinite(out.altitude); accuracy_valid_=valid && v.h_acc && v.v_acc;
      last_fix_receive_=steady(); velocity_valid_=false;
      fix_pub_->publish(out); ++counts_[1];
      fixes_.push_back({v,steady()}); if (fixes_.size()>16) fixes_.pop_front();
    } else if (m.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT && gps_hz_) {
      mavlink_global_position_int_t v{}; mavlink_msg_global_position_int_decode(&m,&v);
      agi_ros2::msg::Heading heading;
      // Keep a separate dedup key from velocity; both originate in message 33.
      if (stamp(0x10000 + 33, bootMs(v.time_boot_ms), heading.header.stamp)) {
        heading.header.frame_id = "gps_enu";
        heading.valid = v.hdg < 36000; // includes rejection of UINT16_MAX
        heading.heading = heading.valid ? std::remainder(pi/2 - v.hdg*pi/18000, 2*pi) : nan;
        heading_pub_->publish(heading);
        heading_valid_ = heading.valid;
        last_heading_receive_ = steady();
        ++counts_[4];
      }
      positions_.push_back({v,steady()}); if(positions_.size()>16) positions_.pop_front();
    } else if (m.msgid == MAVLINK_MSG_ID_ATTITUDE && attitude_hz_) {
      mavlink_attitude_t v{}; mavlink_msg_attitude_decode(&m,&v);
      if (!std::isfinite(v.roll)||!std::isfinite(v.pitch)||!std::isfinite(v.yaw)) return;
      geometry_msgs::msg::QuaternionStamped out;
      if (!stamp(30,bootMs(v.time_boot_ms),out.header.stamp)) return;
      out.header.frame_id="gps_enu";
      // R_ENU_FLU = R_ENU_NED * R_NED_FRD * diag(1,-1,-1).
      const double cr=cos(v.roll*.5),sr=sin(v.roll*.5),cp=cos(v.pitch*.5),sp=sin(v.pitch*.5),cy=cos(v.yaw*.5),sy=sin(v.yaw*.5);
      const double w=cr*cp*cy+sr*sp*sy,x=sr*cp*cy-cr*sp*sy,y=cr*sp*cy+sr*cp*sy,z=cr*cp*sy-sr*sp*cy;
      const double k=std::sqrt(.5);
      out.quaternion.w=k*(w+z); out.quaternion.x=k*(x+y); out.quaternion.y=k*(x-y); out.quaternion.z=k*(w-z);
      attitude_pub_->publish(out); ++counts_[3];
    }
  }
  void pairGps() {
    double t=steady();
    while(!fixes_.empty() && t-fixes_.front().arrival>.5) fixes_.pop_front();
    while(!positions_.empty() && t-positions_.front().arrival>.5) positions_.pop_front();
    for(auto it=positions_.begin();it!=positions_.end();) {
      auto fix=std::find_if(fixes_.begin(),fixes_.end(),[&](const Fix &f){return static_cast<uint32_t>(f.value.time_usec/1000)==it->value.time_boot_ms;});
      if(fix==fixes_.end()) { ++it; continue; }
      const auto &v=it->value;
      geometry_msgs::msg::TwistStamped out;
      if(!publisher_conflict_ && fix->value.fix_type>=3 && fix->value.fix_type<=6 &&
         std::abs(static_cast<int64_t>(fix->value.lat)) <= 900000000 &&
         std::abs(static_cast<int64_t>(fix->value.lon)) <= 1800000000 && v.vx!=INT16_MAX && v.vy!=INT16_MAX && v.vz!=INT16_MAX && stamp(33,fix->value.time_usec*1e-6,out.header.stamp)) {
        out.header.frame_id="gps_enu";
        out.twist.linear.x=v.vy*.01; out.twist.linear.y=v.vx*.01; out.twist.linear.z=-v.vz*.01;
        velocity_pub_->publish(out); ++counts_[2]; velocity_valid_=true;
      }
      it=positions_.erase(it);
    }
  }
  void diagnose() {
    publisher_conflict_ = count_publishers(imu_pub_->get_topic_name()) > 1;
    diagnostic_msgs::msg::DiagnosticArray out; out.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus s; s.name="mavlink_sensor"; s.hardware_id=device_;
    bool synchronized=sync_count_>=5 && steady()-last_sync_<2;
    s.level=fd_<0||publisher_conflict_ ? s.ERROR : !synchronized||commands_failed_||(imu_hz_ && steady()-last_imu_receive_>.1) ? s.WARN : s.OK;
    s.message=fd_>=0 && synchronized ? "Receiving; timestamps are FC update times, not hardware PPS" : reason_;
    auto add=[&](const std::string &key,const std::string &value){diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=value;s.values.push_back(kv);};
    add("imu_publisher_conflict",publisher_conflict_?"true":"false");
    add("imu_fresh",steady()-last_imu_receive_<.1?"true":"false");
    add("connected",fd_>=0?"true":"false"); add("synchronized",synchronized?"true":"false");
    add("rate_commands_ok",command_index_==4&&!commands_failed_?"true":"false");
    add("heading_source", "GLOBAL_POSITION_INT.hdg / FC attitude yaw");
    add("heading_valid", heading_valid_ && steady()-last_heading_receive_<.5 ? "true" : "false");
    add("gps_mode",gps_mode_); add("gps_fix_type",std::to_string(gps_fix_type_));
    bool gps_fresh=steady()-last_fix_receive_<.5;
    add("gps_ready",gps_fresh && (gps_mode_=="rtk"?gps_fix_type_==6:gps_fix_type_>=3)?"true":"false");
    add("velocity_valid",gps_fresh&&velocity_valid_?"true":"false");
    add("ellipsoid_valid",gps_fresh&&ellipsoid_valid_?"true":"false");add("accuracy_valid",gps_fresh&&accuracy_valid_?"true":"false");
    add("altitude_msl_m",std::to_string(msl_)); add("sync_rtt_ms",std::to_string(rtt_ms_));
    add("bad_frames",std::to_string(bad_frames_)); add("wrong_source",std::to_string(wrong_source_));
    add("duplicates",std::to_string(duplicates_)); add("stale",std::to_string(stale_));
    add("rejected_sync",std::to_string(rejected_sync_)); add("tx_errors",std::to_string(tx_errors_));
    double t=steady(), elapsed=t-diagnostic_time_; diagnostic_time_=t;
    const std::array<std::string,5> names{"imu_hz","gps_fix_hz","gps_velocity_hz","attitude_hz","heading_hz"};
    for(size_t i=0;i<names.size();++i) { add(names[i],std::to_string(counts_[i]/elapsed));counts_[i]=0; }
    out.status.push_back(s);status_pub_->publish(out);
  }
  bool heading_valid_{false};
  double last_heading_receive_{0};
  rclcpp::Publisher<agi_ros2::msg::Heading>::SharedPtr heading_pub_;
  int fd_{-1},baud_,sys_,comp_,imu_hz_,gps_hz_,attitude_hz_;
  std::string device_,gps_mode_,altitude_source_,reason_{"Disconnected"};
  std::vector<double> acc_variance_,gyro_variance_;
  mavlink_message_t parser_{}; mavlink_status_t parser_status_{};
  std::deque<Tx> tx_; std::map<int64_t,double> requests_; std::map<uint32_t,double> last_stamp_;
  std::deque<Fix> fixes_; std::deque<Position> positions_;
  double last_imu_receive_{0};
  bool publisher_conflict_{false};
  double next_connect_{0},last_receive_{0},next_sync_{0},offset_{nan},ros_offset_{nan},last_sync_{0},rtt_ms_{0},command_sent_{0},diagnostic_time_{0},last_fix_receive_{0},msl_{nan};
  int64_t last_remote_ns_{0}; unsigned sync_count_{0},command_index_{0},command_attempts_{0};
  bool command_waiting_{false},commands_failed_{false},velocity_valid_{false},ellipsoid_valid_{false},accuracy_valid_{false};
  unsigned gps_fix_type_{0}; uint64_t bad_frames_{0},wrong_source_{0},duplicates_{0},stale_{0},rejected_sync_{0},tx_errors_{0};
  std::array<unsigned,5> counts_{};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_,diagnostic_timer_;
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<MavlinkSensorNode>()); }
  catch(const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("mavlink_sensor"),"%s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
