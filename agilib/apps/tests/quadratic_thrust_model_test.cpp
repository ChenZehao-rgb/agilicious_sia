#include "agilib/bridge/betaflight/quadratic_thrust_model.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using agi::hardware::QuadraticThrustModel;
constexpr double kGramsForceToNewtons = 0.00980665;
constexpr double kThrustFactor = 898.0 / 2231.0;
constexpr double kMaxTotalThrust = 4.0 * 2231.0 * kGramsForceToNewtons;

void require(bool condition, const char* message) {
	if (!condition) throw std::runtime_error(message);
}

template <typename Exception, typename Function>
void requireThrows(Function function, const char* message) {
	try {
		function();
	} catch (const Exception&) {
		return;
	}
	throw std::runtime_error(message);
}

void testManufacturerAnchors() {
	const QuadraticThrustModel model(kThrustFactor, kMaxTotalThrust, 1050);
	const double half_input_thrust = 4.0 * 891.0 * kGramsForceToNewtons;
	require(std::abs(model.totalThrustToNormalized(half_input_thrust) - 0.5) < 1e-14,
	        "Manufacturer half-input thrust must invert to 0.5");
	require(model.totalThrustToRc(half_input_thrust) == 1525, "Half input must map to RC 1525 with min_check 1050");
	require(model.totalThrustToRc(kMaxTotalThrust) == 2000, "Maximum total thrust must map to RC 2000");
	require(model.totalThrustToNormalized(0.0) == 0.0, "Zero thrust must invert to zero input");
	require(model.totalThrustToRc(0.0) == 1050, "Zero model thrust must map to min_check");
}

void testLinearAndSquareLimits() {
	const QuadraticThrustModel linear(0.0, 40.0, 1000);
	require(linear.totalThrustToNormalized(10.0) == 0.25, "Zero factor must give a linear thrust model");
	require(linear.totalThrustToRc(10.0) == 1250, "Linear quarter thrust must map to RC 1250");
	const QuadraticThrustModel square(1.0, 40.0, 1000);
	require(square.totalThrustToNormalized(10.0) == 0.5, "Unit factor must give a square thrust model");
	require(square.totalThrustToRc(10.0) == 1500, "Square model quarter thrust must map to RC 1500");
	require(square.totalThrustToNormalized(0.0) == 0.0, "Square model must handle its zero denominator at zero thrust");
	const QuadraticThrustModel narrow(0.0, 40.0, 1999);
	require(narrow.totalThrustToRc(0.0) == 1999, "Largest valid min_check must remain usable");
	require(narrow.totalThrustToRc(20.0) == 2000, "Half-unit RC values must round to the nearest integer");
}

void testRoundTripsAndMonotonicity() {
	for (const double factor : {0.0, 1e-12, kThrustFactor, 1.0 - 1e-12, 1.0}) {
		const QuadraticThrustModel model(factor, kMaxTotalThrust, 1050);
		double previous = -1.0;
		for (const double input : {0.0, 1e-12, 1e-8, 0.01, 0.1, 0.25, 0.5, 0.75, 1.0}) {
			const double thrust = kMaxTotalThrust * ((1.0 - factor) * input + factor * input * input);
			const double recovered = model.totalThrustToNormalized(thrust);
			require(std::isfinite(recovered), "Valid thrust must produce finite normalized input");
			require(std::abs(recovered - input) <= 1e-14 * input,
			        "Thrust inversion must remain accurate near zero and factor limits");
			require(recovered > previous, "Increasing thrust must produce increasing normalized input");
			previous = recovered;
		}
	}
}

void testInvalidConfiguration() {
	const double nan = std::numeric_limits<double>::quiet_NaN();
	const double infinity = std::numeric_limits<double>::infinity();
	for (const double factor : {-0.01, 1.01, nan, infinity, -infinity}) {
		requireThrows<std::invalid_argument>([&] { QuadraticThrustModel model(factor, kMaxTotalThrust, 1050); },
		                                     "Invalid thrust factor must be rejected");
	}
	for (const double maximum : {0.0, -1.0, nan, infinity, -infinity}) {
		requireThrows<std::invalid_argument>([&] { QuadraticThrustModel model(kThrustFactor, maximum, 1050); },
		                                     "Invalid maximum total thrust must be rejected");
	}
	for (const int min_check : {999, 2000, 2001}) {
		requireThrows<std::invalid_argument>([&] { QuadraticThrustModel model(kThrustFactor, kMaxTotalThrust, min_check); },
		                                     "Invalid min_check must be rejected");
	}
}

void testOutOfRangeThrust() {
	const QuadraticThrustModel model(kThrustFactor, kMaxTotalThrust, 1050);
	const double infinity = std::numeric_limits<double>::infinity();
	for (const double thrust : {-1.0, std::nextafter(kMaxTotalThrust, infinity), kMaxTotalThrust * 2.0,
	                            std::numeric_limits<double>::quiet_NaN(), infinity, -infinity}) {
		requireThrows<std::out_of_range>([&] { model.totalThrustToNormalized(thrust); },
		                                 "Invalid thrust must be rejected before inversion");
		requireThrows<std::out_of_range>([&] { model.totalThrustToRc(thrust); },
		                                 "Invalid thrust must not be silently clipped to an RC value");
	}
}
}  // namespace

int main() {
	try {
		testManufacturerAnchors();
		testLinearAndSquareLimits();
		testRoundTripsAndMonotonicity();
		testInvalidConfiguration();
		testOutOfRangeThrust();
		std::cout << "PASS: quadratic thrust anchors, stable inverse, RC rounding and invalid-input rejection\n";
		return 0;
	} catch (const std::exception& error) {
		std::cerr << error.what() << '\n';
		return 1;
	}
}
