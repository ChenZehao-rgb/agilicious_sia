#include <common/mavlink.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <limits>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "agi_ros2/msg/heading.hpp"
#include "agi_ros2/msg/navigation.hpp"

namespace {
double steady() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
constexpr double pi = 3.14159265358979323846;
constexpr double nan = std::numeric_limits<double>::quiet_NaN();

class MavlinkSensorNode : public rclcpp::Node {
public:
	MavlinkSensorNode() : Node("mavlink_sensor") {
		if (get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("MAVLink hardware requires use_sim_time=false");
		rcl_interfaces::msg::ParameterDescriptor desc;
		desc.read_only = true;
		_device = declare_parameter<std::string>("device", "", desc);
		if (_device.empty()) throw std::invalid_argument("device must name a dedicated MAVLink serial port");
		_baud = declare_parameter<int>("baud", 921600, desc);
		if (_baud != 115200 && _baud != 230400 && _baud != 460800 && _baud != 921600)
			throw std::invalid_argument("Unsupported baud");
		_sys = declare_parameter<int>("system_id", 1, desc);
		_comp = declare_parameter<int>("component_id", 1, desc);
		if (_sys < 1 || _sys > 255 || _comp < 1 || _comp > 255) throw std::invalid_argument("Invalid source IDs");
		_gps_mode = declare_parameter<std::string>("gps_mode", "gnss", desc);
		_altitude_source = declare_parameter<std::string>("altitude_source", "unknown", desc);
		if (_gps_mode != "gnss" && _gps_mode != "rtk") throw std::invalid_argument("gps_mode must be gnss or rtk");
		if (_altitude_source != "unknown" && _altitude_source != "ellipsoid" && _altitude_source != "msl")
			throw std::invalid_argument("altitude_source must be unknown, ellipsoid or msl");
		const auto rate = [&](const char *name, int value, int max) {
			int v = declare_parameter<int>(name, value, desc);
			if (v < 0 || v > max) throw std::invalid_argument(name);
			return v;
		};
		_imu_hz = rate("imu_rate_hz", 500, 500);
		_gps_hz = rate("gps_rate_hz", 10, 50);
		_attitude_hz = rate("attitude_rate_hz", 0, 100);
		_acc_variance = declare_parameter<std::vector<double>>("acceleration_variance", {0., 0., 0.}, desc);
		_gyro_variance = declare_parameter<std::vector<double>>("angular_velocity_variance", {0., 0., 0.}, desc);
		for (const auto &v : {_acc_variance, _gyro_variance}) {
			if (v.size() != 3 || std::any_of(v.begin(), v.end(), [](double x) { return !std::isfinite(x) || x < 0; }))
				throw std::invalid_argument("Variance must have three finite nonnegative entries (all zero means unknown)");
		}
		auto qos = rclcpp::SensorDataQoS();
		_imu_pub = create_publisher<sensor_msgs::msg::Imu>("sensors/imu", qos);
		_fix_pub = create_publisher<sensor_msgs::msg::NavSatFix>("sensors/gps/fix", qos);
		_velocity_pub = create_publisher<geometry_msgs::msg::TwistStamped>("sensors/gps/velocity", qos);
		_attitude_pub = create_publisher<geometry_msgs::msg::QuaternionStamped>("sensors/fc_attitude", qos);
		_navigation_pub = create_publisher<agi_ros2::msg::Navigation>("sensors/navigation", qos);
		_heading_pub = create_publisher<agi_ros2::msg::Heading>("sensors/fc_heading", qos);
		_status_pub = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("sensors/mavlink/status", rclcpp::QoS(10).reliable());
		_timer = create_wall_timer(std::chrono::milliseconds(1), [this] { tick(); });
		_diagnostic_timer = create_wall_timer(std::chrono::seconds(1), [this] { diagnose(); });
		_diagnostic_time = steady();
	}
	~MavlinkSensorNode() override { disconnect(); }

private:
	struct Tx {
		std::vector<uint8_t> bytes;
		size_t offset{0};
	};
	struct Fix {
		mavlink_gps_raw_int_t value;
		double arrival;
	};
	struct Position {
		mavlink_global_position_int_t value;
		double arrival;
	};
	void invalidateNavigation(bool clock_aligned = false) {
		if (!_navigation_pub || !_gps_hz) return;
		agi_ros2::msg::Navigation out;
		// Detection-time failure event only; it is never a position observation.
		out.header.stamp = now();
		out.header.frame_id = "gps_enu";
		out.source_session = _source_session;
		out.fix_type = _gps_fix_type;
		out.altitude_reference = _altitude_source;
		out.latitude = out.longitude = out.altitude = nan;
		out.velocity.x = out.velocity.y = out.velocity.z = nan;
		out.heading = nan;
		out.heading_valid = false;
		out.horizontal_accuracy = out.vertical_accuracy = out.velocity_accuracy = nan;
		out.clock_aligned = clock_aligned;
		_navigation_pub->publish(out);
	}
	void resetEpoch() {
		_source_session = std::to_string(steady());
		_sync_count = 0;
		_offset = nan;
		_last_remote_ns = 0;
		_requests.clear();
		_last_stamp.clear();
		_fixes.clear();
		_positions.clear();
		_gps_fix_type = 0;
		_heading_valid = false;
		_last_heading_receive = 0;
		_last_fix_receive = 0;
		_velocity_valid = false;
		_command_index = 0;
		_command_attempts = 0;
		_command_waiting = false;
		_commands_failed = false;
		_next_sync = 0;
		invalidateNavigation();
	}
	void disconnect() {
		if (_fd >= 0) {
			ioctl(_fd, TIOCNXCL);
			close(_fd);
		}
		_fd = -1;
		_tx.clear();
		resetEpoch();
	}
	void connectPort(double t) {
		_next_connect = t + 1;
		int fd = open(_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
		if (fd < 0) {
			_reason = "Cannot open serial port";
			return;
		}
		if (flock(fd, LOCK_EX | LOCK_NB) || ioctl(fd, TIOCEXCL)) {
			close(fd);
			_reason = "Serial port is not exclusive";
			return;
		}
		termios cfg{};
		if (tcgetattr(fd, &cfg)) {
			ioctl(fd, TIOCNXCL);
			close(fd);
			return;
		}
		cfmakeraw(&cfg);
		cfg.c_cflag |= CLOCAL | CREAD;
		cfg.c_cflag &= ~CRTSCTS;
		speed_t baud = _baud == 921600 ? B921600 : _baud == 460800 ? B460800 : _baud == 230400 ? B230400 : B115200;
		cfsetispeed(&cfg, baud);
		cfsetospeed(&cfg, baud);
		if (tcsetattr(fd, TCSANOW, &cfg)) {
			ioctl(fd, TIOCNXCL);
			close(fd);
			return;
		}
		tcflush(fd, TCIOFLUSH);
		_fd = fd;
		_parser = {};
		_parser_status = {};
		resetEpoch();
		_last_receive = t;
		_reason = "Waiting for clock synchronization";
	}
	bool enqueue(const mavlink_message_t &m) {
		if (_tx.size() >= 16) {
			++_tx_errors;
			return false;
		}
		Tx tx;
		tx.bytes.resize(MAVLINK_MAX_PACKET_LEN);
		tx.bytes.resize(mavlink_msg_to_send_buffer(tx.bytes.data(), &m));
		_tx.push_back(std::move(tx));
		return true;
	}
	void flush() {
		size_t budget = 2048;
		while (!_tx.empty() && budget) {
			auto &tx = _tx.front();
			ssize_t n = write(_fd, tx.bytes.data() + tx.offset, std::min(budget, tx.bytes.size() - tx.offset));
			if (n < 0 && (errno == EAGAIN || errno == EINTR)) return;
			if (n <= 0) {
				_reason = "Serial write failed";
				++_tx_errors;
				disconnect();
				return;
			}
			tx.offset += n;
			budget -= n;
			if (tx.offset == tx.bytes.size()) _tx.pop_front();
		}
	}
	void tick() {
		double t = steady();
		double ros_offset = now().seconds() - t;
		if (std::isfinite(_ros_offset) && std::abs(ros_offset - _ros_offset) > .05) {
			resetEpoch();
			_tx.clear();
			_reason = "ROS clock jumped; resynchronizing";
		}
		_ros_offset = ros_offset;
		if (_fd < 0) {
			if (t >= _next_connect) connectPort(t);
			return;
		}
		std::array<uint8_t, 4096> buffer{};
		size_t budget = 16384;
		while (budget) {
			ssize_t n = read(_fd, buffer.data(), std::min(buffer.size(), budget));
			if (n < 0 && (errno == EAGAIN || errno == EINTR)) break;
			if (n < 0) {
				_reason = "Serial read failed";
				disconnect();
				return;
			}
			if (n == 0) break;
			budget -= n;
			for (ssize_t i = 0; i < n; ++i) {
				mavlink_message_t msg{};
				mavlink_status_t status{};
				auto framing = mavlink_frame_char_buffer(&_parser, &_parser_status, buffer[i], &msg, &status);
				if (framing == MAVLINK_FRAMING_BAD_CRC || framing == MAVLINK_FRAMING_BAD_SIGNATURE) {
					++_bad_frames;
					continue;
				}
				if (framing != MAVLINK_FRAMING_OK) continue;
				if (msg.sysid != _sys || msg.compid != _comp) {
					++_wrong_source;
					continue;
				}
				_last_receive = steady();
				handle(msg);
			}
		}
		t = steady();
		if (t - _last_receive > 2) {
			_reason = "Telemetry timeout";
			disconnect();
			return;
		}
		if (_sync_count && t - _last_sync > 2) {
			resetEpoch();
			_reason = "Time synchronization expired";
		}
		if (t >= _next_sync && _tx.empty()) {
			_next_sync = t + .1;
			int64_t token = static_cast<int64_t>(t * 1e9);
			_requests[token] = t;
			while (_requests.size() > 20) _requests.erase(_requests.begin());
			mavlink_message_t msg{};
			mavlink_msg_timesync_pack(245, 191, &msg, 0, token, _sys, _comp);
			enqueue(msg);
		}
		configureRates(t);
		flush();
		pairGps();
	}
	void configureRates(double t) {
		if (_command_index == 4) return;
		if (_command_waiting && t - _command_sent < .5) return;
		if (_command_attempts >= 3) {
			_commands_failed = true;
			++_command_index;
			_command_attempts = 0;
			_command_waiting = false;
			return;
		}
		const std::array<int, 4> ids{105, 24, 33, 30};
		const std::array<int, 4> hz{_imu_hz, _gps_hz, _gps_hz, _attitude_hz};
		mavlink_message_t msg{};
		float interval = hz[_command_index] ? 1e6f / hz[_command_index] : -1;
		mavlink_msg_command_long_pack(245, 191, &msg, _sys, _comp, MAV_CMD_SET_MESSAGE_INTERVAL, 0, ids[_command_index], interval,
		                              0, 0, 0, 0, 0);
		if (enqueue(msg)) {
			_command_sent = t;
			_command_waiting = true;
			++_command_attempts;
		}
	}
	void timesync(const mavlink_message_t &m) {
		mavlink_timesync_t ts{};
		mavlink_msg_timesync_decode(&m, &ts);
		if (ts.tc1 <= 0 || (ts.target_system && ts.target_system != 245) || (ts.target_component && ts.target_component != 191))
			return;
		auto it = _requests.find(ts.ts1);
		if (it == _requests.end()) return;
		double sent = it->second, received = steady();
		_requests.erase(it);
		double rtt = received - sent;
		if (rtt <= 0 || rtt > .020) {
			++_rejected_sync;
			return;
		}
		if (_last_remote_ns && ts.tc1 < _last_remote_ns) {
			resetEpoch();
			_tx.clear();
			_reason = "Flight controller restarted";
			return;
		}
		_last_remote_ns = ts.tc1;
		double candidate = (sent + received) * .5 - ts.tc1 * 1e-9;
		if (std::isfinite(_offset) && std::abs(candidate - _offset) > .05) {
			resetEpoch();
			_reason = "Device clock discontinuity";
			return;
		}
		_offset = std::isfinite(_offset) ? .9 * _offset + .1 * candidate : candidate;
		_rtt_ms = rtt * 1000;
		_last_sync = received;
		++_sync_count;
	}
	bool stamp(uint32_t id, double remote, builtin_interfaces::msg::Time &out) {
		if (_sync_count < 5 || !std::isfinite(remote)) return false;
		double mapped = remote + _offset + _ros_offset;
		double age = now().seconds() - mapped;
		if (age < -.02 || age > (id == 105 ? .05 : .5)) {
			++_stale;
			return false;
		}
		auto it = _last_stamp.find(id);
		if (it != _last_stamp.end() && remote <= it->second) {
			++_duplicates;
			return false;
		}
		_last_stamp[id] = remote;
		out = rclcpp::Time(static_cast<int64_t>(mapped * 1e9));
		return true;
	}
	double bootMs(uint32_t ms) const {
		// Lift the 32-bit millisecond timestamp to the current synchronized epoch.
		constexpr double wrap = 4294967.296;
		double now_remote = _last_remote_ns * 1e-9;
		double value = ms * .001;
		return value + std::round((now_remote - value) / wrap) * wrap;
	}
	void handle(const mavlink_message_t &m) {
		if (m.msgid == MAVLINK_MSG_ID_TIMESYNC) {
			timesync(m);
			return;
		}
		if (m.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
			mavlink_command_ack_t ack{};
			mavlink_msg_command_ack_decode(&m, &ack);
			if (ack.command == MAV_CMD_SET_MESSAGE_INTERVAL && _command_waiting &&
			    (!ack.target_system || ack.target_system == 245) && (!ack.target_component || ack.target_component == 191)) {
				if (ack.result == MAV_RESULT_IN_PROGRESS) return;
				if (ack.result != MAV_RESULT_ACCEPTED) _commands_failed = true;
				++_command_index;
				_command_attempts = 0;
				_command_waiting = false;
			}
			return;
		}
		if (_sync_count < 5 || _publisher_conflict) return;
		if (m.msgid == MAVLINK_MSG_ID_HIGHRES_IMU && _imu_hz) {
			mavlink_highres_imu_t v{};
			mavlink_msg_highres_imu_decode(&m, &v);
			if ((v.fields_updated & 63) != 63) return;
			for (float x : {v.xacc, v.yacc, v.zacc, v.xgyro, v.ygyro, v.zgyro})
				if (!std::isfinite(x)) return;
			sensor_msgs::msg::Imu out;
			if (!stamp(105, v.time_usec * 1e-6, out.header.stamp)) return;
			out.header.frame_id = "base_link";
			out.linear_acceleration.x = v.xacc;
			out.linear_acceleration.y = -v.yacc;
			out.linear_acceleration.z = -v.zacc;
			out.angular_velocity.x = v.xgyro;
			out.angular_velocity.y = -v.ygyro;
			out.angular_velocity.z = -v.zgyro;
			out.orientation_covariance[0] = -1;
			for (int i = 0; i < 3; ++i) {
				out.linear_acceleration_covariance[i * 4] = _acc_variance[i];
				out.angular_velocity_covariance[i * 4] = _gyro_variance[i];
			}
			_imu_pub->publish(out);
			++_counts[0];
			_last_imu_receive = steady();
		} else if (m.msgid == MAVLINK_MSG_ID_GPS_RAW_INT && _gps_hz) {
			mavlink_gps_raw_int_t v{};
			mavlink_msg_gps_raw_int_decode(&m, &v);
			sensor_msgs::msg::NavSatFix out;
			if (!stamp(24, v.time_usec * 1e-6, out.header.stamp)) return;
			out.header.frame_id = "gps_link";
			const bool valid = v.fix_type >= 3 && v.fix_type <= 6 && std::abs(static_cast<int64_t>(v.lat)) <= 900000000 &&
			                   std::abs(static_cast<int64_t>(v.lon)) <= 1800000000;
			out.status.status =
			        valid ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
			out.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
			out.latitude = valid ? v.lat * 1e-7 : nan;
			out.longitude = valid ? v.lon * 1e-7 : nan;
			out.altitude =
			        valid && _altitude_source == "ellipsoid" && v.alt_ellipsoid != INT32_MIN ? v.alt_ellipsoid * .001 : nan;
			if (valid && v.h_acc && v.v_acc) {
				out.position_covariance[0] = out.position_covariance[4] = std::pow(v.h_acc * .001, 2);
				out.position_covariance[8] = std::pow(v.v_acc * .001, 2);
				out.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
			}
			_gps_fix_type = valid ? v.fix_type : 1;
			if (!valid) invalidateNavigation(true);
			_msl = v.alt * .001;
			_ellipsoid_valid = std::isfinite(out.altitude);
			_accuracy_valid = valid && v.h_acc && v.v_acc;
			_last_fix_receive = steady();
			_velocity_valid = false;
			_fix_pub->publish(out);
			++_counts[1];
			_fixes.push_back({v, steady()});
			if (_fixes.size() > 16) _fixes.pop_front();
		} else if (m.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT && _gps_hz) {
			mavlink_global_position_int_t v{};
			mavlink_msg_global_position_int_decode(&m, &v);
			agi_ros2::msg::Heading heading;
			// Keep a separate dedup key from velocity; both originate in message 33.
			if (stamp(0x10000 + 33, bootMs(v.time_boot_ms), heading.header.stamp)) {
				heading.header.frame_id = "gps_enu";
				heading.valid = v.hdg < 36000;  // includes rejection of UINT16_MAX
				heading.heading = heading.valid ? std::remainder(pi / 2 - v.hdg * pi / 18000, 2 * pi) : nan;
				_heading_pub->publish(heading);
				_heading_valid = heading.valid;
				if (!heading.valid) invalidateNavigation(true);
				_last_heading_receive = steady();
				++_counts[4];
			}
			_positions.push_back({v, steady()});
			if (_positions.size() > 16) _positions.pop_front();
		} else if (m.msgid == MAVLINK_MSG_ID_ATTITUDE && _attitude_hz) {
			mavlink_attitude_t v{};
			mavlink_msg_attitude_decode(&m, &v);
			if (!std::isfinite(v.roll) || !std::isfinite(v.pitch) || !std::isfinite(v.yaw)) return;
			geometry_msgs::msg::QuaternionStamped out;
			if (!stamp(30, bootMs(v.time_boot_ms), out.header.stamp)) return;
			out.header.frame_id = "gps_enu";
			// R_ENU_FLU = R_ENU_NED * R_NED_FRD * diag(1,-1,-1).
			const double cr = cos(v.roll * .5), sr = sin(v.roll * .5), cp = cos(v.pitch * .5), sp = sin(v.pitch * .5),
			             cy = cos(v.yaw * .5), sy = sin(v.yaw * .5);
			const double w = cr * cp * cy + sr * sp * sy, x = sr * cp * cy - cr * sp * sy, y = cr * sp * cy + sr * cp * sy,
			             z = cr * cp * sy - sr * sp * cy;
			const double k = std::sqrt(.5);
			out.quaternion.w = k * (w + z);
			out.quaternion.x = k * (x + y);
			out.quaternion.y = k * (x - y);
			out.quaternion.z = k * (w - z);
			_attitude_pub->publish(out);
			++_counts[3];
		}
	}
	void pairGps() {
		double t = steady();
		while (!_fixes.empty() && t - _fixes.front().arrival > .5) _fixes.pop_front();
		while (!_positions.empty() && t - _positions.front().arrival > .5) _positions.pop_front();
		for (auto it = _positions.begin(); it != _positions.end();) {
			auto fix = std::find_if(_fixes.begin(), _fixes.end(), [&](const Fix &f) {
				return static_cast<uint32_t>(f.value.time_usec / 1000) == it->value.time_boot_ms;
			});
			if (fix == _fixes.end()) {
				++it;
				continue;
			}
			const auto& v = it->value;
			geometry_msgs::msg::TwistStamped out;
			if (v.vx == INT16_MAX || v.vy == INT16_MAX || v.vz == INT16_MAX) invalidateNavigation(true);
			if (!_publisher_conflict && fix->value.fix_type >= 3 && fix->value.fix_type <= 6 &&
			    std::abs(static_cast<int64_t>(fix->value.lat)) <= 900000000 &&
			    std::abs(static_cast<int64_t>(fix->value.lon)) <= 1800000000 && v.vx != INT16_MAX && v.vy != INT16_MAX &&
			    v.vz != INT16_MAX && stamp(33, fix->value.time_usec * 1e-6, out.header.stamp)) {
				out.header.frame_id = "gps_enu";
				out.twist.linear.x = v.vy * .01;
				out.twist.linear.y = v.vx * .01;
				out.twist.linear.z = -v.vz * .01;
				_velocity_pub->publish(out);
				agi_ros2::msg::Navigation navigation;
				navigation.header = out.header;
				navigation.source_session = _source_session;
				navigation.device_time_usec = fix->value.time_usec;
				navigation.fix_type = fix->value.fix_type;
				navigation.latitude = fix->value.lat * 1e-7;
				navigation.longitude = fix->value.lon * 1e-7;
				navigation.altitude_reference = _altitude_source;
				navigation.altitude = nan;
				if (_altitude_source == "ellipsoid" && fix->value.alt_ellipsoid != INT32_MIN)
					navigation.altitude = fix->value.alt_ellipsoid * .001;
				if (_altitude_source == "msl" && fix->value.alt != INT32_MIN) navigation.altitude = fix->value.alt * .001;
				navigation.velocity = out.twist.linear;
				navigation.heading_valid = v.hdg < 36000;
				navigation.heading = navigation.heading_valid ? std::remainder(pi / 2 - v.hdg * pi / 18000, 2 * pi) : nan;
				navigation.horizontal_accuracy = fix->value.h_acc ? fix->value.h_acc * .001 : nan;
				navigation.vertical_accuracy = fix->value.v_acc ? fix->value.v_acc * .001 : nan;
				navigation.velocity_accuracy = fix->value.vel_acc ? fix->value.vel_acc * .001 : nan;
				navigation.clock_aligned = _sync_count >= 5 && steady() - _last_sync < 2;
				_navigation_pub->publish(navigation);
				++_counts[2];
				_velocity_valid = true;
			}
			it = _positions.erase(it);
		}
	}
	void diagnose() {
		const bool publisher_conflict = count_publishers(_imu_pub->get_topic_name()) > 1;
		if (publisher_conflict && !_publisher_conflict) invalidateNavigation();
		_publisher_conflict = publisher_conflict;
		diagnostic_msgs::msg::DiagnosticArray out;
		out.header.stamp = now();
		diagnostic_msgs::msg::DiagnosticStatus s;
		s.name = "mavlink_sensor";
		s.hardware_id = _device;
		bool synchronized = _sync_count >= 5 && steady() - _last_sync < 2;
		s.level = _fd < 0 || _publisher_conflict                                                        ? s.ERROR
		          : !synchronized || _commands_failed || (_imu_hz && steady() - _last_imu_receive > .1) ? s.WARN
		                                                                                                : s.OK;
		s.message = _fd >= 0 && synchronized ? "Receiving; timestamps are FC update times, not hardware PPS" : _reason;
		auto add = [&](const std::string &key, const std::string &value) {
			diagnostic_msgs::msg::KeyValue kv;
			kv.key = key;
			kv.value = value;
			s.values.push_back(kv);
		};
		add("imu_publisher_conflict", _publisher_conflict ? "true" : "false");
		add("imu_fresh", steady() - _last_imu_receive < .1 ? "true" : "false");
		add("connected", _fd >= 0 ? "true" : "false");
		add("synchronized", synchronized ? "true" : "false");
		add("rate_commands_ok", _command_index == 4 && !_commands_failed ? "true" : "false");
		add("heading_source", "GLOBAL_POSITION_INT.hdg / FC attitude yaw");
		add("heading_valid", _heading_valid && steady() - _last_heading_receive < .5 ? "true" : "false");
		add("gps_mode", _gps_mode);
		add("gps_fix_type", std::to_string(_gps_fix_type));
		bool gps_fresh = steady() - _last_fix_receive < .5;
		add("gps_ready", gps_fresh && (_gps_mode == "rtk" ? _gps_fix_type == 6 : _gps_fix_type >= 3) ? "true" : "false");
		add("velocity_valid", gps_fresh && _velocity_valid ? "true" : "false");
		add("ellipsoid_valid", gps_fresh && _ellipsoid_valid ? "true" : "false");
		add("accuracy_valid", gps_fresh && _accuracy_valid ? "true" : "false");
		add("altitude_msl_m", std::to_string(_msl));
		add("sync_rtt_ms", std::to_string(_rtt_ms));
		add("bad_frames", std::to_string(_bad_frames));
		add("wrong_source", std::to_string(_wrong_source));
		add("duplicates", std::to_string(_duplicates));
		add("stale", std::to_string(_stale));
		add("rejected_sync", std::to_string(_rejected_sync));
		add("tx_errors", std::to_string(_tx_errors));
		double t = steady(), elapsed = t - _diagnostic_time;
		_diagnostic_time = t;
		const std::array<std::string, 5> names{"imu_hz", "gps_fix_hz", "gps_velocity_hz", "attitude_hz", "heading_hz"};
		for (size_t i = 0; i < names.size(); ++i) {
			add(names[i], std::to_string(_counts[i] / elapsed));
			_counts[i] = 0;
		}
		out.status.push_back(s);
		_status_pub->publish(out);
	}
	bool _heading_valid{false};
	double _last_heading_receive{0};
	rclcpp::Publisher<agi_ros2::msg::Heading>::SharedPtr _heading_pub;
	rclcpp::Publisher<agi_ros2::msg::Navigation>::SharedPtr _navigation_pub;
	std::string _source_session;
	int _fd{-1}, _baud, _sys, _comp, _imu_hz, _gps_hz, _attitude_hz;
	std::string _device, _gps_mode, _altitude_source, _reason{"Disconnected"};
	std::vector<double> _acc_variance, _gyro_variance;
	mavlink_message_t _parser{};
	mavlink_status_t _parser_status{};
	std::deque<Tx> _tx;
	std::map<int64_t, double> _requests;
	std::map<uint32_t, double> _last_stamp;
	std::deque<Fix> _fixes;
	std::deque<Position> _positions;
	double _last_imu_receive{0};
	bool _publisher_conflict{false};
	double _next_connect{0}, _last_receive{0}, _next_sync{0}, _offset{nan}, _ros_offset{nan}, _last_sync{0}, _rtt_ms{0},
	        _command_sent{0}, _diagnostic_time{0}, _last_fix_receive{0}, _msl{nan};
	int64_t _last_remote_ns{0};
	unsigned _sync_count{0}, _command_index{0}, _command_attempts{0};
	bool _command_waiting{false}, _commands_failed{false}, _velocity_valid{false}, _ellipsoid_valid{false}, _accuracy_valid{false};
	unsigned _gps_fix_type{0};
	uint64_t _bad_frames{0}, _wrong_source{0}, _duplicates{0}, _stale{0}, _rejected_sync{0}, _tx_errors{0};
	std::array<unsigned, 5> _counts{};
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr _fix_pub;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _velocity_pub;
	rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr _attitude_pub;
	rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr _status_pub;
	rclcpp::TimerBase::SharedPtr _timer, _diagnostic_timer;
};
}  // namespace
int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	try {
		rclcpp::spin(std::make_shared<MavlinkSensorNode>());
	} catch (const std::exception &e) {
		RCLCPP_FATAL(rclcpp::get_logger("mavlink_sensor"), "%s", e.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
