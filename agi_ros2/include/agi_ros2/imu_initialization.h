#ifndef AGI_ROS2_IMU_INITIALIZATION_H_
#define AGI_ROS2_IMU_INITIALIZATION_H_

#include <algorithm>
#include <cmath>
#include <deque>

#include "agilib/math/gravity.hpp"
#include "agilib/types/imu_sample.hpp"

namespace agi_ros2 {

// A stationary bias/tilt initialization check, not a full accelerometer calibration.
class ImuInitialization {
public:
	struct Params {
		double duration = 3.0;
		int minimum_samples = 1000;
		double max_gyro_bias = 0.15;
		double max_gyro_stddev = 0.02;
		double max_acceleration_stddev = 0.2;
		double gravity_tolerance = 0.5;
	};

	explicit ImuInitialization(const Params& params) : _params(params) {}

	void reset() {
		_samples.clear();
		_acceleration_sum.setZero();
		_acceleration_squared_sum.setZero();
		_gyro_sum.setZero();
		_gyro_squared_sum.setZero();
		_acceleration.setZero();
		_gyro_bias.setZero();
		_gyro_variance.setZero();
		_acceleration_variance.setZero();
		_ready = false;
	}

	bool addSample(const agi::ImuSample& sample) {
		if (!sample.valid()) {
			reset();
			return false;
		}
		if (!_samples.empty() && (sample.t <= _samples.back().t || sample.t - _samples.back().t > 0.025)) reset();
		_samples.push_back(sample);
		_acceleration_sum += sample.acc;
		_acceleration_squared_sum += sample.acc.cwiseProduct(sample.acc);
		_gyro_sum += sample.omega;
		_gyro_squared_sum += sample.omega.cwiseProduct(sample.omega);
		while (_samples.size() > 1 && (_samples[1].t <= sample.t - _params.duration || _samples.size() > 10000)) {
			const auto& oldest = _samples.front();
			_acceleration_sum -= oldest.acc;
			_acceleration_squared_sum -= oldest.acc.cwiseProduct(oldest.acc);
			_gyro_sum -= oldest.omega;
			_gyro_squared_sum -= oldest.omega.cwiseProduct(oldest.omega);
			_samples.pop_front();
		}
		const double count = static_cast<double>(_samples.size());
		_acceleration = _acceleration_sum / count;
		_gyro_bias = _gyro_sum / count;
		_gyro_variance = (_gyro_squared_sum / count - _gyro_bias.cwiseProduct(_gyro_bias)).cwiseMax(0.0);
		_acceleration_variance = (_acceleration_squared_sum / count - _acceleration.cwiseProduct(_acceleration)).cwiseMax(0.0);
		_ready = _samples.size() >= static_cast<size_t>(_params.minimum_samples) &&
		         sample.t - _samples.front().t >= _params.duration && _gyro_bias.norm() <= _params.max_gyro_bias &&
		         _gyro_variance.maxCoeff() <= _params.max_gyro_stddev * _params.max_gyro_stddev &&
		         _acceleration_variance.maxCoeff() <= _params.max_acceleration_stddev * _params.max_acceleration_stddev &&
		         std::abs(_acceleration.norm() - agi::G) <= _params.gravity_tolerance;
		return _ready;
	}

	bool ready() const { return _ready; }
	const agi::Vector<3>& acceleration() const { return _acceleration; }
	const agi::Vector<3>& gyroBias() const { return _gyro_bias; }
	size_t sampleCount() const { return _samples.size(); }
	double windowDuration() const { return _samples.empty() ? 0.0 : _samples.back().t - _samples.front().t; }
	double gyroStddev() const { return _samples.empty() ? NAN : std::sqrt(_gyro_variance.maxCoeff()); }
	double accelerationStddev() const { return _samples.empty() ? NAN : std::sqrt(_acceleration_variance.maxCoeff()); }
	double gravityError() const { return _samples.empty() ? NAN : std::abs(_acceleration.norm() - agi::G); }
	const char* blockingReason() const {
		if (_samples.size() < static_cast<size_t>(_params.minimum_samples)) return "Collecting stationary IMU samples";
		if (windowDuration() < _params.duration) return "Waiting for full stationary IMU window";
		if (_gyro_bias.norm() > _params.max_gyro_bias) return "Stationary IMU gyro bias exceeds limit";
		if (_gyro_variance.maxCoeff() > _params.max_gyro_stddev * _params.max_gyro_stddev)
			return "Stationary IMU gyro noise exceeds limit";
		if (_acceleration_variance.maxCoeff() > _params.max_acceleration_stddev * _params.max_acceleration_stddev)
			return "Stationary IMU acceleration noise exceeds limit";
		if (gravityError() > _params.gravity_tolerance) return "Stationary IMU gravity residual exceeds limit";
		return _ready ? "Stationary IMU ready" : "Invalid stationary IMU statistics";
	}

private:
	Params _params;
	std::deque<agi::ImuSample> _samples;
	agi::Vector<3> _acceleration_sum = agi::Vector<3>::Zero();
	agi::Vector<3> _acceleration_squared_sum = agi::Vector<3>::Zero();
	agi::Vector<3> _gyro_sum = agi::Vector<3>::Zero();
	agi::Vector<3> _gyro_squared_sum = agi::Vector<3>::Zero();
	agi::Vector<3> _acceleration = agi::Vector<3>::Zero();
	agi::Vector<3> _gyro_bias = agi::Vector<3>::Zero();
	agi::Vector<3> _gyro_variance = agi::Vector<3>::Zero();
	agi::Vector<3> _acceleration_variance = agi::Vector<3>::Zero();
	bool _ready = false;
};

}  // namespace agi_ros2

#endif  // AGI_ROS2_IMU_INITIALIZATION_H_
