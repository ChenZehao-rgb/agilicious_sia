#include <gtest/gtest.h>

#include "agi_ros2/barometer_reference.h"

namespace agi_ros2 {
namespace {

BarometerReference::Params referenceParams() {
	BarometerReference::Params params;
	params.duration = 0.2;
	params.minimum_samples = 3;
	return params;
}

TEST(BarometerReference, KeepsLocalHeightAndPropagatesPressureVariance) {
	BarometerReference reference(referenceParams());
	EXPECT_FALSE(reference.addSample(0.0, 101325.0, 4.0, 17.0, true));
	EXPECT_FALSE(reference.addSample(0.1, 101325.0, 4.0, 17.0, true));
	ASSERT_TRUE(reference.addSample(0.2, 101325.0, 4.0, 17.0, true));
	double height = NAN, variance = NAN;
	ASSERT_TRUE(reference.heightObservation(101325.0, 4.0, 0.25, &height, &variance));
	EXPECT_DOUBLE_EQ(height, 17.0);
	const double sensitivity = 44330.0 * 0.190295 / 101325.0;
	EXPECT_NEAR(variance, sensitivity * sensitivity * 4.0 + 0.25, 1e-12);
	// Only independent alignment error is returned; the EKF adds correlated height uncertainty.
	EXPECT_NEAR(reference.independentVariance(), 4.0 + sensitivity * sensitivity * 4.0 / 3.0, 1e-12);
	ASSERT_TRUE(reference.heightObservation(101324.0, 4.0, 0.25, &height, &variance));
	EXPECT_NEAR(height - 17.0, 0.08325, 0.0001);
	ASSERT_TRUE(reference.heightObservation(101326.0, 4.0, 0.25, &height, &variance));
	EXPECT_LT(height, 17.0);
}

TEST(BarometerReference, RequiresStationarityDistinctSamplesAndStablePressure) {
	BarometerReference reference(referenceParams());
	EXPECT_FALSE(reference.addSample(0.0, 101325.0, 4.0, 17.0, true));
	EXPECT_FALSE(reference.addSample(0.0, 101325.0, 4.0, 17.0, true));
	EXPECT_EQ(reference.sampleCount(), 1u);
	EXPECT_FALSE(reference.addSample(0.1, 101325.0, 4.0, 17.0, false));
	EXPECT_EQ(reference.sampleCount(), 0u);
	EXPECT_FALSE(reference.addSample(0.2, 101325.0, 4.0, 17.0, true));
	EXPECT_FALSE(reference.addSample(0.3, 101425.0, 4.0, 17.0, true));
	EXPECT_FALSE(reference.addSample(0.5, 101325.0, 4.0, 17.0, true));
	EXPECT_EQ(reference.sampleCount(), 0u);
	EXPECT_FALSE(reference.ready());
}

TEST(BarometerReference, GapResetAndInvalidNoiseCannotReuseReference) {
	BarometerReference reference(referenceParams());
	reference.addSample(0.0, 101325.0, 4.0, 17.0, true);
	reference.addSample(0.1, 101325.0, 4.0, 17.0, true);
	EXPECT_FALSE(reference.addSample(0.5, 101325.0, 4.0, 17.0, true));
	EXPECT_EQ(reference.sampleCount(), 1u);
	reference.addSample(0.6, 101325.0, 4.0, 17.0, true);
	ASSERT_TRUE(reference.addSample(0.75, 101325.0, 4.0, 17.0, true));
	double height, variance;
	EXPECT_FALSE(reference.heightObservation(101325.0, 0.0, 0.25, &height, &variance));
	EXPECT_FALSE(reference.heightObservation(NAN, 4.0, 0.25, &height, &variance));
	reference.reset();
	EXPECT_FALSE(reference.ready());
	EXPECT_FALSE(reference.heightObservation(101325.0, 4.0, 0.25, &height, &variance));
	EXPECT_FALSE(reference.addSample(1.0, 101325.0, 0.0, 17.0, true));
}

}  // namespace
}  // namespace agi_ros2
