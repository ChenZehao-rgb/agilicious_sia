#ifndef AGILIB_BRIDGE_BETAFLIGHT_QUADRATIC_THRUST_MODEL_H_
#define AGILIB_BRIDGE_BETAFLIGHT_QUADRATIC_THRUST_MODEL_H_

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace agi::hardware {

// Quadratic is a historical name retained to minimize integration changes; the formula is now cubic.
// Total vehicle thrust at the reference voltage is u * (a1 + u * (a2 + u * a3)), u in [0, 1].
class QuadraticThrustModel {
public:
	QuadraticThrustModel(double a1_n, double a2_n, double a3_n, double reference_voltage_v, double voltage_exponent_p, int min_check)
	        : _a1_n(a1_n),
	          _a2_n(a2_n),
	          _a3_n(a3_n),
	          _reference_voltage_v(reference_voltage_v),
	          _voltage_exponent_p(voltage_exponent_p),
	          _min_check(min_check) {
		if (!std::isfinite(a1_n) || !std::isfinite(a2_n) || !std::isfinite(a3_n) || !std::isfinite(reference_voltage_v) ||
		    reference_voltage_v <= 0.0 || !std::isfinite(voltage_exponent_p) || min_check < 1000 || min_check >= 2000) {
			throw std::invalid_argument(
			        "Invalid measured cubic thrust model: require finite coefficients, positive reference voltage and "
			        "min_check in [1000, 2000)");
		}
	}

	double totalThrustToNormalized(double total_thrust_n, double voltage_v) const {
		if (!std::isfinite(total_thrust_n) || total_thrust_n < 0.0 || !std::isfinite(voltage_v) || voltage_v <= 0.0) {
			throw std::out_of_range("Thrust must be finite and nonnegative; battery voltage must be finite and positive");
		}
		if (total_thrust_n == 0.0) return 0.0;
		const double voltage_ratio = _reference_voltage_v / voltage_v;
		const double scale = std::pow(voltage_ratio, _voltage_exponent_p);
		const double lookup_thrust_n = total_thrust_n * scale;
		const double endpoint_thrust_n = referenceThrust(1.0);
		if (!std::isfinite(voltage_ratio) || !std::isfinite(scale) || !std::isfinite(lookup_thrust_n) ||
		    !std::isfinite(endpoint_thrust_n)) {
			throw std::overflow_error("Nonfinite measured cubic thrust calculation");
		}
		if (lookup_thrust_n >= endpoint_thrust_n) return 1.0;
		double lower = 0.0;
		double upper = 1.0;
		// The configured measured cubic is monotonic throughout [0, 1].
		for (int iteration = 0; iteration < 32; ++iteration) {
			const double middle = (lower + upper) * 0.5;
			const double thrust_n = referenceThrust(middle);
			if (!std::isfinite(thrust_n)) throw std::overflow_error("Nonfinite measured cubic thrust calculation");
			if (thrust_n < lookup_thrust_n) {
				lower = middle;
			} else {
				upper = middle;
			}
		}
		return (lower + upper) * 0.5;
	}

	uint16_t totalThrustToRc(double total_thrust_n, double voltage_v) const {
		const double normalized = totalThrustToNormalized(total_thrust_n, voltage_v);
		return static_cast<uint16_t>(std::lround(_min_check + (2000 - _min_check) * normalized));
	}

private:
	double referenceThrust(double normalized) const { return normalized * (_a1_n + normalized * (_a2_n + normalized * _a3_n)); }

	const double _a1_n;
	const double _a2_n;
	const double _a3_n;
	const double _reference_voltage_v;
	const double _voltage_exponent_p;
	const int _min_check;
};

}  // namespace agi::hardware

#endif  // AGILIB_BRIDGE_BETAFLIGHT_QUADRATIC_THRUST_MODEL_H_
