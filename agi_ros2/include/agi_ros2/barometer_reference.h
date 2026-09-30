#ifndef AGI_ROS2_BAROMETER_REFERENCE_H_
#define AGI_ROS2_BAROMETER_REFERENCE_H_

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace agi_ros2 {

// The reference error is common to all subsequent observations. Its variance
// initializes the EKF bias once; it is never reused as independent sample noise.
class BarometerReference {
public:
	struct Params {
		double duration = 2.0;
		size_t minimum_samples = 40;
		double maximum_gap = 0.25;
		double maximum_pressure_stddev = 15.0;
		double maximum_height_stddev = 0.3;
		double alignment_variance = 4.0;
	};

	explicit BarometerReference(const Params& params) : _params(params) {}

	void reset() {
		_count = 0;
		_first_time = _last_time = NAN;
		_pressure = _height = _pressure_m2 = _height_m2 = _pressure_variance_sum = 0.0;
		_independent_variance = NAN;
		_ready = false;
	}

	bool addSample(double time, double pressure, double pressure_variance, double height, bool stationary) {
		if (_ready) return true;
		if (!stationary || !std::isfinite(time) || !std::isfinite(pressure) || pressure <= 0 || !std::isfinite(pressure_variance) ||
		    pressure_variance <= 0 || !std::isfinite(height)) {
			reset();
			return false;
		}
		if (_count && time <= _last_time) return false;
		if (_count && time - _last_time > _params.maximum_gap) reset();
		if (!_count) _first_time = time;
		_last_time = time;
		++_count;
		const double pressure_delta = pressure - _pressure;
		_pressure += pressure_delta / _count;
		_pressure_m2 += pressure_delta * (pressure - _pressure);
		const double height_delta = height - _height;
		_height += height_delta / _count;
		_height_m2 += height_delta * (height - _height);
		_pressure_variance_sum += pressure_variance;
		if (time - _first_time < _params.duration || _count < _params.minimum_samples) return false;
		if (_pressure_m2 / (_count - 1) > std::pow(_params.maximum_pressure_stddev, 2) ||
		    _height_m2 / (_count - 1) > std::pow(_params.maximum_height_stddev, 2)) {
			reset();
			return false;
		}
		const double derivative = -kAtmosphereHeight * kExponent / _pressure;
		_independent_variance = _params.alignment_variance + derivative * derivative * _pressure_variance_sum / (_count * _count);
		_ready = true;
		return true;
	}

	bool heightObservation(double pressure, double pressure_variance, double model_variance, double* height, double* variance) const {
		if (!_ready || !height || !variance || !std::isfinite(pressure) || pressure <= 0 || !std::isfinite(pressure_variance) ||
		    pressure_variance <= 0 || !std::isfinite(model_variance) || model_variance <= 0)
			return false;
		const double ratio = pressure / _pressure;
		*height = _height + kAtmosphereHeight * (1.0 - std::pow(ratio, kExponent));
		const double derivative = -kAtmosphereHeight * kExponent / _pressure * std::pow(ratio, kExponent - 1.0);
		*variance = derivative * derivative * pressure_variance + model_variance;
		return std::isfinite(*height) && std::isfinite(*variance) && *variance > 0;
	}

	bool ready() const { return _ready; }
	// Align to the EKF state at the end of the stationary reference window.
	// The EKF separately preserves the corresponding state/bias cross covariance.
	void alignHeight(double height) { _height = height; }
	size_t sampleCount() const { return _count; }
	double pressure() const { return _pressure; }
	double height() const { return _height; }
	double independentVariance() const { return _independent_variance; }

private:
	static constexpr double kAtmosphereHeight = 44330.0;
	static constexpr double kExponent = 0.190295;
	Params _params;
	size_t _count = 0;
	double _first_time = NAN;
	double _last_time = NAN;
	double _pressure = 0.0;
	double _height = 0.0;
	double _pressure_m2 = 0.0;
	double _height_m2 = 0.0;
	double _pressure_variance_sum = 0.0;
	double _independent_variance = NAN;
	bool _ready = false;
};

}  // namespace agi_ros2

#endif  // AGI_ROS2_BAROMETER_REFERENCE_H_
