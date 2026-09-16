#include <gz/msgs/clock.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/odometry.pb.h>

#include <agi_ros2/msg/health.hpp>
#include <agi_ros2/msg/rtk.hpp>
#include <cmath>
#include <deque>
#include <gz/transport/Node.hh>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>

class GazeboSensors : public rclcpp::Node {
public:
	GazeboSensors() : Node("gazebo_sensors") {
		declare_parameter<double>("rtk_rate", 10.0);
		declare_parameter<double>("rtk_delay", 0.08);
		declare_parameter<double>("position_noise", 0.02);
		declare_parameter<double>("velocity_noise", 0.03);
		declare_parameter<double>("heading_noise", 0.00698);
		declare_parameter<bool>("rtk_fixed", true);
		declare_parameter<bool>("heading_valid", true);
		declare_parameter<bool>("drop_imu", false);
		declare_parameter<bool>("drop_rtk", false);
		declare_parameter<bool>("config_verified", false);
		_imu = create_publisher<sensor_msgs::msg::Imu>("sensors/imu", rclcpp::SensorDataQoS());
		_rtk = create_publisher<agi_ros2::msg::Rtk>("sensors/rtk", 10);
		_health = create_publisher<agi_ros2::msg::Health>("health", 1);
		_truth = create_publisher<nav_msgs::msg::Odometry>("ground_truth", 1);
		_clock = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
		if (!_gz.Subscribe("/clock", &GazeboSensors::onClock, this) ||
		    !_gz.Subscribe("/model/iris/companion_imu", &GazeboSensors::onImu, this) ||
		    !_gz.Subscribe("/model/iris/odometry", &GazeboSensors::onOdom, this)) {
			throw std::runtime_error("Gazebo subscription failed");
		}
	}
	~GazeboSensors() override {
		_gz.Unsubscribe("/clock");
		_gz.Unsubscribe("/model/iris/companion_imu");
		_gz.Unsubscribe("/model/iris/odometry");
	}

private:
	static builtin_interfaces::msg::Time time(const gz::msgs::Time& s) {
		builtin_interfaces::msg::Time t;
		t.sec = s.sec();
		t.nanosec = s.nsec();
		return t;
	}
	void onClock(const gz::msgs::Clock& m) {
		rosgraph_msgs::msg::Clock c;
		c.clock = time(m.sim());
		_clock->publish(c);
	}
	void onImu(const gz::msgs::IMU& m) {
		if (get_parameter("drop_imu").as_bool()) return;
		sensor_msgs::msg::Imu out;
		out.header.stamp = time(m.header().stamp());
		out.header.frame_id = "base_link";
		out.orientation_covariance[0] = -1;
		out.linear_acceleration.x = m.linear_acceleration().x();
		out.linear_acceleration.y = m.linear_acceleration().y();
		out.linear_acceleration.z = m.linear_acceleration().z();
		out.angular_velocity.x = m.angular_velocity().x();
		out.angular_velocity.y = m.angular_velocity().y();
		out.angular_velocity.z = m.angular_velocity().z();
		_imu->publish(out);
	}
	void onOdom(const gz::msgs::Odometry& m) {
		std::lock_guard<std::mutex> l(_mutex);
		const double t = m.header().stamp().sec() + m.header().stamp().nsec() * 1e-9;
		if (t < _previous) {
			_pending.clear();
			_last_fix = -1;
		}
		_previous = t;
		const auto& p = m.pose().position();
		const auto& q = m.pose().orientation();
		const auto& v = m.twist().linear();
		nav_msgs::msg::Odometry truth;
		truth.header.stamp = time(m.header().stamp());
		truth.header.frame_id = "odom";
		truth.child_frame_id = "base_link";
		truth.pose.pose.position.x = p.x();
		truth.pose.pose.position.y = p.y();
		truth.pose.pose.position.z = p.z();
		truth.pose.pose.orientation.w = q.w();
		truth.pose.pose.orientation.x = q.x();
		truth.pose.pose.orientation.y = q.y();
		truth.pose.pose.orientation.z = q.z();
		truth.twist.twist.linear.x = v.x();
		truth.twist.twist.linear.y = v.y();
		truth.twist.twist.linear.z = v.z();
		_truth->publish(truth);
		const double rate = get_parameter("rtk_rate").as_double(), delay = get_parameter("rtk_delay").as_double();
		if (!std::isfinite(rate) || rate <= 0 || !std::isfinite(delay) || delay < 0 || delay > 1) return;
		if (t - _last_fix >= 1 / rate) {
			_last_fix = t;
			agi_ros2::msg::Rtk out;
			out.header = truth.header;
			const double pn = get_parameter("position_noise").as_double(), vn = get_parameter("velocity_noise").as_double();
			out.position.x = p.x() + pn * _noise(_random);
			out.position.y = p.y() + pn * _noise(_random);
			out.position.z = p.z() + pn * _noise(_random);
			// Gazebo odometry twist is body FLU. Rotate into ENU before RTK emulation.
			const double tx = 2 * (q.y() * v.z() - q.z() * v.y());
			const double ty = 2 * (q.z() * v.x() - q.x() * v.z());
			const double tz = 2 * (q.x() * v.y() - q.y() * v.x());
			out.velocity.x = v.x() + q.w() * tx + q.y() * tz - q.z() * ty + vn * _noise(_random);
			out.velocity.y = v.y() + q.w() * ty + q.z() * tx - q.x() * tz + vn * _noise(_random);
			out.velocity.z = v.z() + q.w() * tz + q.x() * ty - q.y() * tx + vn * _noise(_random);
			out.heading = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z())) +
			              get_parameter("heading_noise").as_double() * _noise(_random);
			out.fixed = get_parameter("rtk_fixed").as_bool();
			out.heading_valid = get_parameter("heading_valid").as_bool();
			out.accuracy_ok = true;
			out.synchronized = true;
			if (_pending.size() < 32) _pending.push_back({t + delay, out});
		}
		while (!_pending.empty() && _pending.front().first <= t) {
			if (!get_parameter("drop_rtk").as_bool()) _rtk->publish(_pending.front().second);
			_pending.pop_front();
		}
		agi_ros2::msg::Health h;
		h.header = truth.header;
		h.imu_calibrated = true;
		h.converged = t > 2;
		h.config_verified = get_parameter("config_verified").as_bool();
		h.thrust_calibrated = true;
		h.geofence_ok = true;
		h.transport_healthy = true;
		h.battery_voltage = 16;
		_health->publish(h);
	}
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu;
	rclcpp::Publisher<agi_ros2::msg::Rtk>::SharedPtr _rtk;
	rclcpp::Publisher<agi_ros2::msg::Health>::SharedPtr _health;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _truth;
	rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr _clock;
	std::mutex _mutex;
	std::deque<std::pair<double, agi_ros2::msg::Rtk>> _pending;
	double _previous{-1}, _last_fix{-1};
	std::mt19937 _random{42};
	std::normal_distribution<double> _noise{0, 1};
	gz::transport::Node _gz;
};
int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GazeboSensors>());
	rclcpp::shutdown();
}
