#include "agilib/bridge/betaflight/quadratic_thrust_model.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

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
		testInvalidConfiguration();
		testInvalidInputsAndNumericalOverflow();
		std::cout << "PASS: measured cubic anchors, voltage compensation, full-range inverse, saturation and invalid-input "
		             "rejection\n";
		return 0;
	} catch (const std::exception& error) {
		std::cerr << error.what() << '\n';
		return 1;
	}
}
