#include "agilib/bridge/betaflight/quadratic_thrust_model.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {
using agi::hardware::QuadraticThrustModel;
constexpr double kA1 = 3.0411331993565565;
constexpr double kA2 = 138.51944503645788;
constexpr double kA3 = -65.93137222790723;
constexpr double kReferenceVoltage = 24.0;
constexpr double kVoltageExponent = 1.253471255346232;
constexpr double kEndpointThrust = kA1 + (kA2 + kA3);

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

void testMeasuredAnchorsAndVoltage() {
	const QuadraticThrustModel model(kA1, kA2, kA3, kReferenceVoltage, kVoltageExponent, 1050);
	require(model.totalThrustToRc(0.0, 24.0) == 1050, "Zero thrust must map to min_check");
	require(model.totalThrustToRc(7.7472535, 24.0) == 1277, "Reference-voltage hover thrust must map to RC 1277");
	require(model.totalThrustToRc(7.7472535, 20.0) == 1308, "20 V must be accepted and map hover thrust to RC 1308");
	require(model.totalThrustToRc(7.7472535, 25.2) == 1270, "25.2 V must be accepted and map hover thrust to RC 1270");
	for (const int min_check : {1000, 1100, 1999}) {
		const QuadraticThrustModel alternate(kA1, kA2, kA3, kReferenceVoltage, kVoltageExponent, min_check);
		require(alternate.totalThrustToRc(0.0, 24.0) == min_check, "Existing min_check range must remain usable");
		require(alternate.totalThrustToRc(kEndpointThrust, 24.0) == 2000, "Endpoint must map to RC 2000 for any min_check");
	}
}

void testFullRangeAndSaturation() {
	const QuadraticThrustModel model(kA1, kA2, kA3, kReferenceVoltage, kVoltageExponent, 1050);
	for (const double voltage : {20.0, 24.0, 25.2}) {
		double previous = -1.0;
		for (int step = 0; step <= 1000; ++step) {
			const double input = step / 1000.0;
			const double thrust = (kA1 * input + kA2 * input * input + kA3 * input * input * input) *
			                      std::pow(voltage / kReferenceVoltage, kVoltageExponent);
			const double recovered = model.totalThrustToNormalized(thrust, voltage);
			require(std::abs(recovered - input) <= 1.0 / 4294967296.0, "Full-range inverse must agree with the cubic");
			require(recovered > previous, "Increasing thrust must produce increasing normalized input");
			previous = recovered;
		}
		require(model.totalThrustToRc(2.0 * kEndpointThrust, voltage) == 2000, "Excess thrust must saturate at RC 2000");
	}
	require(model.totalThrustToRc(50.0, 24.0) > 1702, "High thrust must use the throttle range above RC 1702");
	require(model.totalThrustToNormalized(kEndpointThrust, 24.0) == 1.0, "Model endpoint must saturate to normalized one");
	require(model.totalThrustToRc(kEndpointThrust, 24.0) == 2000, "Model endpoint must map to RC 2000");
	require(model.totalThrustToRc(std::numeric_limits<double>::max(), 24.0) == 2000, "Finite excessive thrust must saturate");
}

void testExplicitSmallMatchesDefault() {
	const QuadraticThrustModel default_model(kA1, kA2, kA3, kReferenceVoltage, kVoltageExponent, 1050);
	const QuadraticThrustModel small_model(kA1, kA2, kA3, kReferenceVoltage, kVoltageExponent, 1050, "small");
	for (const double voltage : {20.0, 24.0, 25.2}) {
		for (int thrust = 0; thrust <= 200; ++thrust) {
			require(default_model.totalThrustToNormalized(thrust, voltage) ==
			                small_model.totalThrustToNormalized(thrust, voltage),
			        "Explicit small airframe must preserve the default inverse exactly");
			require(default_model.totalThrustToRc(thrust, voltage) == small_model.totalThrustToRc(thrust, voltage),
			        "Explicit small airframe must preserve default RC output exactly");
		}
	}
}

void testManufacturerFitAndSaturation() {
	// Big airframe ignores cubic coefficients, including their validity.
	const double unused = std::numeric_limits<double>::quiet_NaN();
	const QuadraticThrustModel model(unused, unused, unused, kReferenceVoltage, 0.0, 1050, "big");
	constexpr double endpoint = 160.1959166931985;
	const std::pair<double, uint16_t> anchors[] = {
	        {0.0, 1050}, {10.0, 1292}, {20.0, 1355}, {40.0, 1441}, {80.0, 1567}, {120.0, 1694}, {endpoint, 2000}, {200.0, 2000}};
	for (const auto& anchor : anchors) {
		for (const double voltage : {20.0, 24.0, 25.2}) {
			require(model.totalThrustToRc(anchor.first, voltage) == anchor.second,
			        "Manufacturer fit RC anchors must match at all positive test voltages when p=0");
		}
	}
	double previous_thrust = -1.0;
	double previous_input = -1.0;
	for (int step = 0; step <= 1000; ++step) {
		const double input = step / 1000.0;
		const double thrust = 161.946600121777 * (-std::expm1(-std::pow(input / 0.6159720191140124, 3.1165108548961253)));
		require(std::isfinite(thrust) && thrust >= 0.0 && thrust > previous_thrust,
		        "Manufacturer reference formula must be finite, nonnegative and monotonic throughout [0, 1]");
		const double recovered = model.totalThrustToNormalized(thrust, 24.0);
		require(std::abs(recovered - input) <= 1.0 / 4294967296.0, "Full-range inverse must agree with the manufacturer fit");
		require(recovered > previous_input, "Manufacturer inverse must be monotonic throughout [0, 1]");
		previous_thrust = thrust;
		previous_input = recovered;
	}
	require(std::abs(previous_thrust - endpoint) < 1e-12, "Manufacturer endpoint must be 160.1959166931985 N");
	require(model.totalThrustToNormalized(endpoint, 24.0) == 1.0, "Manufacturer endpoint must saturate to normalized one");
	require(model.totalThrustToNormalized(endpoint - 0.01, 24.0) < 1.0, "Thrust below the endpoint must not saturate early");
	require(model.totalThrustToNormalized(200.0, 24.0) == 1.0, "Excess manufacturer thrust must saturate");

	// Test-only exponent verifies the retained interface; it is not a big-airframe calibration value.
	constexpr double test_exponent = 0.5;
	const QuadraticThrustModel compensated(unused, unused, unused, kReferenceVoltage, test_exponent, 1050, "big");
	const double expected_lookup = 40.0 * std::pow(kReferenceVoltage / 20.0, test_exponent);
	require(compensated.totalThrustToNormalized(40.0, 20.0) == model.totalThrustToNormalized(expected_lookup, 24.0),
	        "Big airframe must retain the existing voltage-compensation calculation");
}

void testBigInvalidConfigurationAndInputs() {
	const double nan = std::numeric_limits<double>::quiet_NaN();
	const double infinity = std::numeric_limits<double>::infinity();
	for (const char* airframe : {"", "large", "Small", "BIG"}) {
		requireThrows<std::invalid_argument>(
		        [&] { QuadraticThrustModel model(kA1, kA2, kA3, kReferenceVoltage, 0.0, 1050, airframe); },
		        "Only small and big airframe names must be accepted");
	}
	for (const double invalid : {0.0, -1.0, nan, infinity, -infinity}) {
		requireThrows<std::invalid_argument>([&] { QuadraticThrustModel model(0.0, 0.0, 0.0, invalid, 0.0, 1050, "big"); },
		                                     "Big airframe must reject invalid reference voltages");
	}
	for (const double invalid : {nan, infinity, -infinity}) {
		requireThrows<std::invalid_argument>(
		        [&] { QuadraticThrustModel model(0.0, 0.0, 0.0, kReferenceVoltage, invalid, 1050, "big"); },
		        "Big airframe must reject nonfinite voltage exponents");
	}
	for (const int min_check : {999, 2000, 2001}) {
		requireThrows<std::invalid_argument>(
		        [&] { QuadraticThrustModel model(0.0, 0.0, 0.0, kReferenceVoltage, 0.0, min_check, "big"); },
		        "Big airframe must reject invalid min_check");
	}
	const QuadraticThrustModel model(0.0, 0.0, 0.0, kReferenceVoltage, 0.0, 1050, "big");
	for (const double thrust : {-1.0, nan, infinity, -infinity}) {
		requireThrows<std::out_of_range>([&] { model.totalThrustToRc(thrust, 24.0); }, "Big airframe must reject invalid thrust");
	}
	for (const double voltage : {0.0, -1.0, nan, infinity, -infinity}) {
		for (const double thrust : {0.0, 40.0}) {
			requireThrows<std::out_of_range>([&] { model.totalThrustToRc(thrust, voltage); },
			                                 "Big airframe must reject invalid voltages even with p=0 and zero thrust");
		}
	}
}

void testInvalidConfiguration() {
	const double nan = std::numeric_limits<double>::quiet_NaN();
	const double infinity = std::numeric_limits<double>::infinity();
	for (const double invalid : {nan, infinity, -infinity}) {
		for (int index = 0; index < 5; ++index) {
			double parameters[] = {kA1, kA2, kA3, kReferenceVoltage, kVoltageExponent};
			parameters[index] = invalid;
			requireThrows<std::invalid_argument>(
			        [&] {
				        QuadraticThrustModel model(parameters[0], parameters[1], parameters[2], parameters[3],
				                                   parameters[4], 1050);
			        },
			        "Nonfinite model parameters must be rejected");
		}
	}
	for (const double voltage : {0.0, -1.0}) {
		requireThrows<std::invalid_argument>([&] { QuadraticThrustModel model(kA1, kA2, kA3, voltage, kVoltageExponent, 1050); },
		                                     "Nonpositive reference voltage must be rejected");
	}
	for (const int min_check : {999, 2000, 2001}) {
		requireThrows<std::invalid_argument>(
		        [&] { QuadraticThrustModel model(kA1, kA2, kA3, kReferenceVoltage, kVoltageExponent, min_check); },
		        "Invalid min_check must be rejected");
	}
}

void testInvalidInputsAndNumericalOverflow() {
	const QuadraticThrustModel model(kA1, kA2, kA3, kReferenceVoltage, kVoltageExponent, 1050);
	const double nan = std::numeric_limits<double>::quiet_NaN();
	const double infinity = std::numeric_limits<double>::infinity();
	for (const double thrust : {-1.0, nan, infinity, -infinity}) {
		requireThrows<std::out_of_range>([&] { model.totalThrustToRc(thrust, 24.0); }, "Invalid thrust must be rejected");
	}
	for (const double voltage : {0.0, -1.0, nan, infinity, -infinity}) {
		for (const double thrust : {0.0, 7.7472535}) {
			requireThrows<std::out_of_range>([&] { model.totalThrustToRc(thrust, voltage); },
			                                 "Invalid voltage must be rejected even for zero thrust");
		}
	}
	for (const double voltage : {std::numeric_limits<double>::min(), 1e-250}) {
		requireThrows<std::overflow_error>([&] { model.totalThrustToRc(1.0, voltage); },
		                                   "Nonfinite voltage ratio or power must be a numerical error");
	}
	requireThrows<std::overflow_error>([&] { model.totalThrustToRc(std::numeric_limits<double>::max(), 20.0); },
	                                   "Nonfinite lookup thrust must be rejected before saturation");
}
}  // namespace

int main() {
	try {
		testMeasuredAnchorsAndVoltage();
		testFullRangeAndSaturation();
		testExplicitSmallMatchesDefault();
		testManufacturerFitAndSaturation();
		testBigInvalidConfigurationAndInputs();
		testInvalidConfiguration();
		testInvalidInputsAndNumericalOverflow();
		std::cout << "PASS: small measured cubic and big manufacturer fit anchors, voltage compensation, full-range inverse, "
		             "saturation and invalid-input rejection\n";
		return 0;
	} catch (const std::exception& error) {
		std::cerr << error.what() << '\n';
		return 1;
	}
}
