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
		const agi::Vector<3> gyro_variance = (_gyro_squared_sum / count - _gyro_bias.cwiseProduct(_gyro_bias)).cwiseMax(0.0);
		const agi::Vector<3> acceleration_variance =
		        (_acceleration_squared_sum / count - _acceleration.cwiseProduct(_acceleration)).cwiseMax(0.0);
		_ready = _samples.size() >= static_cast<size_t>(_params.minimum_samples) &&
		         sample.t - _samples.front().t >= _params.duration && _gyro_bias.norm() <= _params.max_gyro_bias &&
		         gyro_variance.maxCoeff() <= _params.max_gyro_stddev * _params.max_gyro_stddev &&
		         acceleration_variance.maxCoeff() <= _params.max_acceleration_stddev * _params.max_acceleration_stddev &&
		         std::abs(_acceleration.norm() - agi::G) <= _params.gravity_tolerance;
		return _ready;
	}

	bool ready() const { return _ready; }
	const agi::Vector<3>& acceleration() const { return _acceleration; }
	const agi::Vector<3>& gyroBias() const { return _gyro_bias; }

private:
	Params _params;
	std::deque<agi::ImuSample> _samples;
	agi::Vector<3> _acceleration_sum = agi::Vector<3>::Zero();
	agi::Vector<3> _acceleration_squared_sum = agi::Vector<3>::Zero();
	agi::Vector<3> _gyro_sum = agi::Vector<3>::Zero();
	agi::Vector<3> _gyro_squared_sum = agi::Vector<3>::Zero();
	agi::Vector<3> _acceleration = agi::Vector<3>::Zero();
	agi::Vector<3> _gyro_bias = agi::Vector<3>::Zero();
	bool _ready = false;
};

}  // namespace agi_ros2

#endif  // AGI_ROS2_IMU_INITIALIZATION_H_
