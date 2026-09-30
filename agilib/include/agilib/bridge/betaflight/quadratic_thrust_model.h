#ifndef AGILIB_BRIDGE_BETAFLIGHT_QUADRATIC_THRUST_MODEL_H_
#define AGILIB_BRIDGE_BETAFLIGHT_QUADRATIC_THRUST_MODEL_H_

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace agi::hardware {

// Total vehicle thrust is Fmax * ((1 - k) * u + k * u^2), with u in [0, 1].
// This approximation has no voltage compensation or measured FC/ESC correction.
class QuadraticThrustModel {
public:
	QuadraticThrustModel(double thrust_factor, double max_total_thrust_n, int min_check)
	        : _thrust_factor(thrust_factor), _max_total_thrust_n(max_total_thrust_n), _min_check(min_check) {
		if (!std::isfinite(thrust_factor) || thrust_factor < 0.0 || thrust_factor > 1.0 || !std::isfinite(max_total_thrust_n) ||
		    max_total_thrust_n <= 0.0 || min_check < 1000 || min_check >= 2000) {
			throw std::invalid_argument(
			        "Invalid quadratic thrust model: require factor in [0, 1], positive Fmax and min_check in [1000, 2000)");
		}
	}

	double totalThrustToNormalized(double total_thrust_n) const {
		if (!std::isfinite(total_thrust_n) || total_thrust_n < 0.0 || total_thrust_n > _max_total_thrust_n) {
			throw std::out_of_range("Requested thrust outside quadratic model envelope");
		}
		const double normalized_thrust = total_thrust_n / _max_total_thrust_n;
		if (normalized_thrust == 0.0) return 0.0;
		const double linear_factor = 1.0 - _thrust_factor;
		// Rationalizing the positive root avoids cancellation near zero thrust.
		return 2.0 * normalized_thrust /
		       (linear_factor + std::sqrt(linear_factor * linear_factor + 4.0 * _thrust_factor * normalized_thrust));
	}

	uint16_t totalThrustToRc(double total_thrust_n) const {
		const double normalized = totalThrustToNormalized(total_thrust_n);
		return static_cast<uint16_t>(std::lround(_min_check + (2000 - _min_check) * normalized));
	}

private:
	const double _thrust_factor;
	const double _max_total_thrust_n;
	const int _min_check;
};

}  // namespace agi::hardware

#endif  // AGILIB_BRIDGE_BETAFLIGHT_QUADRATIC_THRUST_MODEL_H_
