#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include "agilib/estimator/ekf_imu/ekf_imu.hpp"
#include "agilib/math/gravity.hpp"

namespace agi {
class EkfImuTestPeer {
public:
	static Matrix<17, 17> covariance(const EkfImu& ekf) { return ekf.P_; }
	static Vector<17> posterior(const EkfImu& ekf) { return ekf.posterior_; }
	static void setCovariance(EkfImu* ekf, const Matrix<17, 17>& covariance) { ekf->P_ = covariance; }
};
}  // namespace agi

using namespace agi;
namespace {
QuadState initial() {
  QuadState state;
  state.setZero();
  state.t = 1.0;
  return state;
}
void samples(EkfImu& ekf, bool query) {
  for (int i = 0; i <= 20; ++i) {
    ASSERT_TRUE(ekf.addImu({1.0 + i * 0.01, -GVEC, Vector<3>::Zero()}));
    if (query) {
      QuadState out;
      out.setZero();
      ASSERT_TRUE(ekf.getAt(1.0 + i * 0.01, &out));
    }
  }
}
bool fix(EkfImu& ekf, Scalar t, Scalar yaw = 0.0, bool heading = true) {
  return ekf.addRtk(t, Vector<3>(0.1,0,0), Vector<3>(1,0,0), yaw, heading,
                    Vector<3>::Constant(1e-4), Vector<3>::Constant(1e-4), 1e-4);
}
}
TEST(EkfImuRtk, DelayedUpdateIndependentOfQueries) {
  EkfImu a, b;
  ASSERT_TRUE(a.initialize(initial()));
  ASSERT_TRUE(b.initialize(initial()));
  samples(a, false);
  samples(b, true);
  ASSERT_TRUE(fix(a, 1.1));
  ASSERT_TRUE(fix(b, 1.1));
  QuadState x = initial(), y = initial();
  ASSERT_TRUE(a.getAt(1.2, &x));
  ASSERT_TRUE(b.getAt(1.2, &y));
  EXPECT_TRUE(x.x.isApprox(y.x, 1e-10));
  EXPECT_GT(x.v.x(), 0.8);
  EXPECT_GT(x.p.x(), 0.08);
  EXPECT_FALSE(fix(a, 1.1));
  EXPECT_FALSE(fix(a, 1.3));
  EXPECT_FALSE(a.addImu({1.19, -GVEC, Vector<3>::Zero()}));
}
TEST(EkfImuRtk, BiasCorrectedOutputAndUninitialized) {
  EkfImu ekf;
  QuadState state = initial();
  EXPECT_FALSE(ekf.getAt(1.0, &state));
  state.bw = Vector<3>(0.01,0.02,0.03);
  state.ba = Vector<3>(0.1,0.2,0.3);
  ASSERT_TRUE(ekf.initialize(state));
  ASSERT_TRUE(ekf.addImu({1.0, -GVEC + state.ba, state.bw}));
  ASSERT_TRUE(ekf.getAt(1.0, &state));
  EXPECT_LT(state.w.norm(), 1e-10);
  EXPECT_LT(state.a.norm(), 1e-10);
}
TEST(EkfImuRtk, HeadingWrapAndOptionalHeading) {
  auto params = std::make_shared<EkfImuParameters>();
  params->Q_init_att.setConstant(0.1);
  EkfImu ekf(params);
  QuadState state = initial();
  state.q(Quaternion(Eigen::AngleAxis<Scalar>(M_PI - 0.01, Vector<3>::UnitZ())));
  ASSERT_TRUE(ekf.initialize(state));
  samples(ekf, true);
  ASSERT_TRUE(fix(ekf, 1.1, -M_PI + 0.01));
  ASSERT_TRUE(ekf.getAt(1.1, &state));
  EXPECT_NEAR(std::remainder(state.getYaw() - (-M_PI + 0.01), 2*M_PI), 0, 0.002);
  ASSERT_TRUE(fix(ekf, 1.2, NAN, false));
  EXPECT_FALSE(ekf.addRtk(1.21, Vector<3>::Zero(), Vector<3>::Zero(), 0, true,
                        Vector<3>::Constant(-1), Vector<3>::Ones(), 1));
}
TEST(EkfImuRtk, ReinitializeResetsCovariance) {
  EkfImu a, b;
  ASSERT_TRUE(a.initialize(initial()));
  samples(a, true);
  ASSERT_TRUE(fix(a, 1.1));
  QuadState reset = initial(); reset.t = 1.2;
  ASSERT_TRUE(a.initialize(reset));
  ASSERT_TRUE(b.initialize(reset));
  ASSERT_TRUE(a.addImu({1.3, -GVEC, Vector<3>::Zero()}));
  ASSERT_TRUE(b.addImu({1.3, -GVEC, Vector<3>::Zero()}));
  ASSERT_TRUE(fix(a, 1.3)); ASSERT_TRUE(fix(b, 1.3));
  QuadState x = initial(), y = initial();
  ASSERT_TRUE(a.getAt(1.3, &x)); ASSERT_TRUE(b.getAt(1.3, &y));
  EXPECT_TRUE(x.x.isApprox(y.x, 1e-9));
}
TEST(EkfImuRtk, ExplicitResetAllowsClockRewind) {
  EkfImu ekf;
  ASSERT_TRUE(ekf.initialize(initial()));
  samples(ekf, true);
  ASSERT_TRUE(ekf.initialize(initial()));
  EXPECT_TRUE(ekf.addImu({1.0, -GVEC, Vector<3>::Zero()}));
}
TEST(EkfImuRtk, PoseUpdatesIndependentOfQueries) {
  EkfImu a, b;
  ASSERT_TRUE(a.initialize(initial())); ASSERT_TRUE(b.initialize(initial()));
  samples(a, false); samples(b, true);
  const Pose pose{1.1, Vector<3>(0.1, 0, 0), Quaternion::Identity()};
  ASSERT_TRUE(a.addPose(pose)); ASSERT_TRUE(b.addPose(pose));
  QuadState x = initial(), y = initial();
  ASSERT_TRUE(a.getAt(1.2, &x)); ASSERT_TRUE(b.getAt(1.2, &y));
  EXPECT_TRUE(x.x.isApprox(y.x, 1e-10));
  EXPECT_GT(x.p.x(), 0.05);
}
TEST(EkfImuRtk, DelayedFixReplaysChangingImu) {
  EkfImu immediate, delayed;
  ASSERT_TRUE(immediate.initialize(initial()));
  ASSERT_TRUE(delayed.initialize(initial()));
  for (int i = 0; i <= 300; ++i) {
    const Scalar t = 1.0 + 0.001 * i;
    const ImuSample imu(t, -GVEC + Vector<3>(0.2 * std::sin(i * 0.1), 0, 0),
                        Vector<3>(0, 0, 0.1));
    ASSERT_TRUE(immediate.addImu(imu)); ASSERT_TRUE(delayed.addImu(imu));
    if (i == 100) {
      ASSERT_TRUE(fix(immediate, t, 0.01));
    }
    QuadState out = initial();
    ASSERT_TRUE(immediate.getAt(t, &out));
    ASSERT_TRUE(delayed.getAt(t, &out));
  }
  ASSERT_TRUE(fix(delayed, 1.1, 0.01));
  QuadState x = initial(), y = initial();
  ASSERT_TRUE(immediate.getAt(1.3, &x));
  ASSERT_TRUE(delayed.getAt(1.3, &y));
  EXPECT_TRUE(x.x.isApprox(y.x, 1e-9));
}
TEST(EkfImuRtk, RejectsDiscardedHistory) {
  EkfImu ekf;
  ASSERT_TRUE(ekf.initialize(initial()));
  for (int i = 0; i < 4100; ++i)
    ASSERT_TRUE(ekf.addImu({1.0 + i * 0.001, -GVEC, Vector<3>::Zero()}));
  QuadState out = initial();
  EXPECT_FALSE(ekf.getAt(5.099, &out));
  EXPECT_FALSE(fix(ekf, 5.0));
}

TEST(EkfImuRtk, InnovationRejectionPreservesPosteriorAndRecovery) {
	EkfImu rejected, reference;
	ASSERT_TRUE(rejected.initialize(initial()));
	ASSERT_TRUE(reference.initialize(initial()));
	samples(rejected, true);
	samples(reference, false);
	const auto before = rejected.navigationQuality();
	EXPECT_FALSE(
	        rejected.addRtk(1.1, Vector<3>(100, 0, 0), Vector<3>::Zero(), 0, true, Vector<3>::Ones(), Vector<3>::Ones(), 0.03, 24.322));
	const auto after = rejected.navigationQuality();
	EXPECT_EQ(after.stamp, before.stamp);
	EXPECT_TRUE(after.position_variance.isApprox(before.position_variance));
	EXPECT_TRUE(after.velocity_variance.isApprox(before.velocity_variance));
	EXPECT_EQ(after.rejected_updates, 1u);
	EXPECT_GT(after.innovation_squared, 24.322);
	ASSERT_TRUE(rejected.addRtk(1.2, Vector<3>(0.01, 0, 0), Vector<3>::Zero(), 0, true, Vector<3>::Ones(), Vector<3>::Ones(), 0.03,
	                            24.322));
	ASSERT_TRUE(reference.addRtk(1.2, Vector<3>(0.01, 0, 0), Vector<3>::Zero(), 0, true, Vector<3>::Ones(), Vector<3>::Ones(), 0.03,
	                             24.322));
	QuadState a = initial(), b = initial();
	ASSERT_TRUE(rejected.getAt(1.2, &a));
	ASSERT_TRUE(reference.getAt(1.2, &b));
	EXPECT_TRUE(a.x.isApprox(b.x, 1e-10));
	EXPECT_EQ(rejected.navigationQuality().accepted_updates, 1u);
	EXPECT_TRUE(rejected.navigationQuality().valid);
}

TEST(EkfImuRtk, PredictionCacheHandlesRewindAndNewImuAfterExtrapolation) {
	EkfImu cached, reference;
	ASSERT_TRUE(cached.initialize(initial()));
	ASSERT_TRUE(reference.initialize(initial()));
	samples(cached, false);
	samples(reference, false);
	QuadState a = initial(), b = initial();
	for (Scalar time : {1.1, 1.2, 1.05, 1.2, 1.4}) ASSERT_TRUE(cached.getAt(time, &a));
	const ImuSample changed{1.3, -GVEC + Vector<3>(0.2, 0, 0), Vector<3>(0, 0, 0.1)};
	ASSERT_TRUE(cached.addImu(changed));
	ASSERT_TRUE(reference.addImu(changed));
	ASSERT_TRUE(cached.getAt(1.3, &a));
	ASSERT_TRUE(reference.getAt(1.3, &b));
	EXPECT_TRUE(a.x.isApprox(b.x, 1e-10));
	EXPECT_EQ(cached.navigationQuality().stamp, 1.0);
	EXPECT_TRUE(cached.navigationQuality().position_variance.isApprox(reference.navigationQuality().position_variance));
}

TEST(EkfImuRtk, NavigationQualityResetAndHeadingCovariance) {
	auto params = std::make_shared<EkfImuParameters>();
	params->Q_init_att.setConstant(0.01);
	EkfImu ekf(params);
	EXPECT_FALSE(ekf.navigationQuality().valid);
	ASSERT_TRUE(ekf.initialize(initial()));
	const auto quality = ekf.navigationQuality();
	EXPECT_TRUE(quality.valid);
	EXPECT_NEAR(quality.heading_variance, 0.04, 1e-12);
	samples(ekf, false);
	EXPECT_FALSE(ekf.addRtk(1.1, Vector<3>(100, 0, 0), Vector<3>::Zero(), 0, true, Vector<3>::Ones(), Vector<3>::Ones(), 0.03, 24.322));
	ASSERT_TRUE(ekf.initialize(initial()));
	EXPECT_EQ(ekf.navigationQuality().rejected_updates, 0u);
	EXPECT_EQ(ekf.navigationQuality().accepted_updates, 0u);
	EXPECT_TRUE(std::isnan(ekf.navigationQuality().innovation_squared));
}

TEST(EkfImuBaro, ScalarObservationHasUpwardHeightAndIndependentQuality) {
	auto params = std::make_shared<EkfImuParameters>();
	params->Q_init_pos.z() = 9;
	params->Q_init_baro_bias = 1;
	EkfImu ekf(params);
	EXPECT_FALSE(ekf.barometerQuality().valid);
	ASSERT_TRUE(ekf.initialize(initial()));
	ASSERT_TRUE(ekf.addImu({1.0, -GVEC, Vector<3>::Zero()}));
	ASSERT_TRUE(ekf.addBaro(1.0, 2.0, 1.0, 9.0));
	QuadState out = initial();
	ASSERT_TRUE(ekf.getAt(1.0, &out));
	EXPECT_NEAR(out.p.z(), 18.0 / 11.0, 1e-12);
	const auto quality = ekf.barometerQuality();
	EXPECT_TRUE(quality.valid);
	EXPECT_DOUBLE_EQ(quality.stamp, 1.0);
	EXPECT_NEAR(quality.bias, 2.0 / 11.0, 1e-12);
	EXPECT_NEAR(quality.bias_variance, 10.0 / 11.0, 1e-12);
	EXPECT_NEAR(quality.innovation, -2.0, 1e-12);
	EXPECT_NEAR(quality.innovation_squared, 4.0 / 11.0, 1e-12);
	EXPECT_EQ(quality.accepted_updates, 1u);
	EXPECT_EQ(ekf.navigationQuality().accepted_updates, 0u);
	EXPECT_EQ(ekf.navigationQuality().rejected_updates, 0u);
	EXPECT_FALSE(ekf.addBaro(1.0, 2.0, 1.0, 9.0));
}

TEST(EkfImuBaro, RejectsInvalidInputAndRequiresImuCoverage) {
	EkfImu ekf;
	EXPECT_FALSE(ekf.addBaro(1.0, 0.0, 1.0, 9.0));
	EXPECT_FALSE(ekf.resetBaroBias(1.0));
	ASSERT_TRUE(ekf.initialize(initial()));
	EXPECT_FALSE(ekf.addBaro(1.0, 0.0, 1.0, 9.0));
	samples(ekf, false);
	EXPECT_FALSE(ekf.addBaro(NAN, 0.0, 1.0, 9.0));
	EXPECT_FALSE(ekf.addBaro(1.1, NAN, 1.0, 9.0));
	EXPECT_FALSE(ekf.addBaro(1.1, 0.0, NAN, 9.0));
	EXPECT_FALSE(ekf.addBaro(1.1, 0.0, 0.0, 9.0));
	EXPECT_FALSE(ekf.addBaro(1.1, 0.0, -1.0, 9.0));
	EXPECT_FALSE(ekf.addBaro(1.1, 0.0, 1.0, NAN));
	EXPECT_FALSE(ekf.addBaro(1.1, 0.0, 1.0, 0.0));
	EXPECT_FALSE(ekf.addBaro(1.3, 0.0, 1.0, 9.0));
	EXPECT_FALSE(ekf.addBaro(0.9, 0.0, 1.0, 9.0));
	EXPECT_FALSE(ekf.resetBaroBias(NAN));
	EXPECT_FALSE(ekf.resetBaroBias(-1.0));
	EXPECT_FALSE(ekf.resetBaroBias(0.0));
	EXPECT_DOUBLE_EQ(ekf.navigationQuality().stamp, 1.0);
	EXPECT_EQ(ekf.barometerQuality().accepted_updates, 0u);
}

TEST(EkfImuBaro, InnovationRejectionRestoresStateCovarianceAndPrediction) {
	EkfImu rejected, reference;
	ASSERT_TRUE(rejected.initialize(initial()));
	ASSERT_TRUE(reference.initialize(initial()));
	samples(rejected, true);
	samples(reference, false);
	const auto covariance_before = EkfImuTestPeer::covariance(rejected);
	QuadState before = initial(), after = initial();
	ASSERT_TRUE(rejected.getAt(1.2, &before));
	EXPECT_FALSE(rejected.addBaro(1.1, 100.0, 1.0, 9.0));
	EXPECT_TRUE(EkfImuTestPeer::covariance(rejected).isApprox(covariance_before, 1e-14));
	ASSERT_TRUE(rejected.getAt(1.2, &after));
	EXPECT_TRUE(after.x.isApprox(before.x, 1e-14));
	EXPECT_DOUBLE_EQ(rejected.navigationQuality().stamp, 1.0);
	EXPECT_EQ(rejected.barometerQuality().rejected_updates, 1u);
	EXPECT_GT(rejected.barometerQuality().innovation_squared, 9.0);
	EXPECT_FALSE(rejected.barometerQuality().valid);
	ASSERT_TRUE(rejected.addBaro(1.2, 0.1, 1.0, 9.0));
	ASSERT_TRUE(reference.addBaro(1.2, 0.1, 1.0, 9.0));
	ASSERT_TRUE(rejected.getAt(1.2, &after));
	ASSERT_TRUE(reference.getAt(1.2, &before));
	EXPECT_TRUE(after.x.isApprox(before.x, 1e-12));
	EXPECT_TRUE(EkfImuTestPeer::covariance(rejected).isApprox(EkfImuTestPeer::covariance(reference), 1e-12));
	EXPECT_EQ(rejected.navigationQuality().rejected_updates, 0u);
}

TEST(EkfImuBaro, SameTimestampGnssAndBarometerBothOrders) {
	auto params = std::make_shared<EkfImuParameters>();
	params->Q_init_pos.setConstant(4);
	params->Q_init_baro_bias = 2;
	EkfImu baro_first(params), gnss_first(params);
	ASSERT_TRUE(baro_first.initialize(initial()));
	ASSERT_TRUE(gnss_first.initialize(initial()));
	ASSERT_TRUE(baro_first.addImu({1.0, -GVEC, Vector<3>::Zero()}));
	ASSERT_TRUE(gnss_first.addImu({1.0, -GVEC, Vector<3>::Zero()}));
	const auto navigation = [](EkfImu& ekf) {
		return ekf.addRtk(1.0, Vector<3>(0, 0, 1), Vector<3>::Zero(), NAN, false, Vector<3>::Ones(), Vector<3>::Ones(), NAN);
	};
	ASSERT_TRUE(baro_first.addBaro(1.0, 1.5, 0.5, 9.0));
	ASSERT_TRUE(navigation(baro_first));
	ASSERT_TRUE(navigation(gnss_first));
	ASSERT_TRUE(gnss_first.addBaro(1.0, 1.5, 0.5, 9.0));
	QuadState a = initial(), b = initial();
	ASSERT_TRUE(baro_first.getAt(1.0, &a));
	ASSERT_TRUE(gnss_first.getAt(1.0, &b));
	EXPECT_TRUE(a.x.isApprox(b.x, 1e-12));
	EXPECT_NEAR(baro_first.barometerQuality().bias, gnss_first.barometerQuality().bias, 1e-12);
	EXPECT_TRUE(EkfImuTestPeer::covariance(baro_first).isApprox(EkfImuTestPeer::covariance(gnss_first), 1e-12));
	EXPECT_FALSE(navigation(baro_first));
	EXPECT_FALSE(navigation(gnss_first));
	EXPECT_FALSE(baro_first.addBaro(1.0, 1.5, 0.5, 9.0));
	EXPECT_FALSE(gnss_first.addBaro(1.0, 1.5, 0.5, 9.0));
	EXPECT_EQ(baro_first.navigationQuality().accepted_updates, 1u);
	EXPECT_EQ(baro_first.barometerQuality().accepted_updates, 1u);
}

TEST(EkfImuBaro, DelayedSortedObservationsMatchImmediateFusionWithInterleavedQueries) {
	EkfImu immediate, delayed;
	ASSERT_TRUE(immediate.initialize(initial()));
	ASSERT_TRUE(delayed.initialize(initial()));
	const auto update = [](EkfImu& ekf, int sample) {
		const Scalar time = 1.0 + sample * 0.001;
		if (sample == 100 || sample == 200) {
			ASSERT_TRUE(fix(ekf, time, 0.1 * (time - 1.0)));
		}
		ASSERT_TRUE(ekf.addBaro(time, 0.05 * (time - 1.0), 0.1, 9.0));
	};
	for (int i = 0; i <= 300; ++i) {
		const Scalar time = 1.0 + 0.001 * i;
		const ImuSample imu(time, -GVEC + Vector<3>(0.2 * std::sin(i * 0.1), 0, 0.03), Vector<3>(0, 0, 0.1));
		ASSERT_TRUE(immediate.addImu(imu));
		ASSERT_TRUE(delayed.addImu(imu));
		if (i == 100 || i == 150 || i == 200) update(immediate, i);
		QuadState out = initial();
		ASSERT_TRUE(immediate.getAt(time, &out));
		ASSERT_TRUE(delayed.getAt(time, &out));
	}
	for (int sample : {100, 150, 200}) {
		update(delayed, sample);
		QuadState out = initial();
		ASSERT_TRUE(delayed.getAt(1.3, &out));
	}
	QuadState a = initial(), b = initial();
	ASSERT_TRUE(immediate.getAt(1.3, &a));
	ASSERT_TRUE(delayed.getAt(1.3, &b));
	EXPECT_TRUE(a.x.isApprox(b.x, 1e-9));
	EXPECT_NEAR(immediate.barometerQuality().bias, delayed.barometerQuality().bias, 1e-10);
	EXPECT_TRUE(EkfImuTestPeer::covariance(immediate).isApprox(EkfImuTestPeer::covariance(delayed), 1e-9));
}

TEST(EkfImuBaro, ContinuousBiasRandomWalkScalesWithElapsedTime) {
	auto params = std::make_shared<EkfImuParameters>();
	params->baro_bias_random_walk = 0.4;
	params->Q_init_baro_bias = 3;
	EkfImu single(params), split(params);
	ASSERT_TRUE(single.initialize(initial()));
	ASSERT_TRUE(split.initialize(initial()));
	samples(single, false);
	samples(split, false);
	ASSERT_TRUE(fix(single, 1.2));
	ASSERT_TRUE(fix(split, 1.1));
	ASSERT_TRUE(fix(split, 1.2));
	EXPECT_NEAR(single.barometerQuality().bias_variance, 3.0 + 0.4 * 0.2, 1e-10);
	EXPECT_NEAR(split.barometerQuality().bias_variance, 3.0 + 0.4 * 0.2, 1e-10);
	EXPECT_EQ(single.barometerQuality().accepted_updates, 0u);
	EXPECT_FALSE(single.barometerQuality().valid);
}

TEST(EkfImuBaro, SlowPressureDriftIsConstrainedByGnssHeight) {
	auto params = std::make_shared<EkfImuParameters>();
	params->Q_init_pos.z() = 1.0;
	params->Q_init_baro_bias = 1.0;
	params->baro_bias_random_walk = 0.2;
	EkfImu ekf(params);
	ASSERT_TRUE(ekf.initialize(initial()));
	for (int i = 0; i <= 200; ++i) {
		const Scalar time = 1.0 + 0.01 * i;
		ASSERT_TRUE(ekf.addImu({time, -GVEC, Vector<3>::Zero()}));
		if (i % 10 != 0) continue;
		ASSERT_TRUE(ekf.addRtk(time, Vector<3>::Zero(), Vector<3>::Zero(), 0, true, Vector<3>::Constant(0.001),
		                       Vector<3>::Constant(0.001), 0.001));
		if (i == 0) {
			Scalar anchor = NAN;
			ASSERT_TRUE(ekf.alignBarometerReference(time, 1.0, &anchor));
			EXPECT_DOUBLE_EQ(anchor, 0.0);
		}
		ASSERT_TRUE(ekf.addBaro(time, 0.2 * (time - 1.0), 0.002, 9.0));
	}
	QuadState out = initial();
	ASSERT_TRUE(ekf.getAt(3.0, &out));
	EXPECT_LT(std::abs(out.p.z()), 0.02);
	EXPECT_NEAR(ekf.barometerQuality().bias, 0.4, 0.03);
	EXPECT_EQ(ekf.barometerQuality().rejected_updates, 0u);
	const auto quality = ekf.barometerQuality();
	ASSERT_TRUE(ekf.addImu({3.1, -GVEC, Vector<3>::Zero()}));
	ASSERT_TRUE(fix(ekf, 3.1));
	EXPECT_EQ(ekf.barometerQuality().accepted_updates, quality.accepted_updates);
	EXPECT_EQ(ekf.barometerQuality().stamp, quality.stamp);
}

TEST(EkfImuBaro, ReferenceResetPreservesNavigationAndClearsBiasCorrelation) {
	EkfImu ekf;
	ASSERT_TRUE(ekf.initialize(initial()));
	samples(ekf, true);
	ASSERT_TRUE(fix(ekf, 1.1));
	ASSERT_TRUE(ekf.addBaro(1.1, 2.0, 0.1, 9.0));
	const auto covariance_before = EkfImuTestPeer::covariance(ekf);
	const auto navigation_before = ekf.navigationQuality();
	QuadState before = initial(), after = initial();
	ASSERT_TRUE(ekf.getAt(1.2, &before));
	ASSERT_TRUE(ekf.resetBaroBias(9.0));
	ASSERT_TRUE(ekf.getAt(1.2, &after));
	EXPECT_TRUE(after.x.isApprox(before.x, 1e-12));
	const auto covariance_after = EkfImuTestPeer::covariance(ekf);
	EXPECT_TRUE((covariance_before.topLeftCorner<16, 16>().isApprox(covariance_after.topLeftCorner<16, 16>(), 1e-14)));
	EXPECT_TRUE(covariance_after.row(16).head<16>().isZero());
	EXPECT_TRUE(covariance_after.col(16).head<16>().isZero());
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias, 0.0);
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias_variance, 9.0);
	EXPECT_FALSE(ekf.barometerQuality().valid);
	EXPECT_EQ(ekf.barometerQuality().accepted_updates, 1u);
	EXPECT_EQ(ekf.navigationQuality().accepted_updates, navigation_before.accepted_updates);
	EXPECT_EQ(ekf.navigationQuality().stamp, navigation_before.stamp);
	EXPECT_FALSE(fix(ekf, 1.1));
	ASSERT_TRUE(ekf.addBaro(1.1, 0.1, 0.1, 9.0));
	EXPECT_TRUE(ekf.barometerQuality().valid);
	ASSERT_TRUE(ekf.initialize(initial()));
	EXPECT_EQ(ekf.barometerQuality().accepted_updates, 0u);
	EXPECT_FALSE(ekf.barometerQuality().valid);
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias, 0.0);
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias_variance, 25.0);
}

TEST(EkfImuBaro, JointCovarianceRemainsFiniteSymmetricAndPositiveSemidefinite) {
	auto params = std::make_shared<EkfImuParameters>();
	params->Q_init_att.setConstant(0.01);
	params->Q_init_pos.setConstant(1.0);
	EkfImu ekf(params);
	ASSERT_TRUE(ekf.initialize(initial()));
	for (int i = 0; i <= 100; ++i) {
		const Scalar time = 1.0 + i * 0.005;
		ASSERT_TRUE(ekf.addImu({time, -GVEC + Vector<3>(0.01, 0.02, 0.03), Vector<3>(0.01, 0.02, 0.03)}));
		if (i % 5 != 0) continue;
		ASSERT_TRUE(ekf.addBaro(time, 0.05 * std::sin(time), 0.02, 9.0));
		ASSERT_TRUE(ekf.addRtk(time, Vector<3>::Zero(), Vector<3>::Zero(), 0, true, Vector<3>::Constant(0.1),
		                       Vector<3>::Constant(0.1), 0.1));
		const auto covariance = EkfImuTestPeer::covariance(ekf);
		EXPECT_TRUE(covariance.allFinite());
		EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-12));
		const Eigen::SelfAdjointEigenSolver<Matrix<17, 17>> eigenvalues(covariance);
		ASSERT_EQ(eigenvalues.info(), Eigen::Success);
		EXPECT_GE(eigenvalues.eigenvalues().minCoeff(), -1e-12);
	}
}

TEST(EkfImuBaro, BiasNoiseParametersRequireFinitePhysicalValues) {
	EkfImuParameters params;
	EXPECT_TRUE(params.valid());
	params.baro_bias_random_walk = -0.01;
	EXPECT_FALSE(params.valid());
	params.baro_bias_random_walk = NAN;
	EXPECT_FALSE(params.valid());
	params.baro_bias_random_walk = 0;
	EXPECT_TRUE(params.valid());
	params.Q_init_baro_bias = 0;
	EXPECT_FALSE(params.valid());
	params.Q_init_baro_bias = NAN;
	EXPECT_FALSE(params.valid());
}

TEST(EkfImuBaro, AlignedPressureReferenceDoesNotInventAbsoluteNavigationInformation) {
	auto params = std::make_shared<EkfImuParameters>();
	params->Q_init_pos.setConstant(9.0);
	params->Q_init_vel.setConstant(4.0);
	EkfImu ekf(params);
	QuadState state = initial();
	state.p.z() = 12.0;
	ASSERT_TRUE(ekf.initialize(state));
	samples(ekf, true);
	// A navigation update yields a normalized attitude covariance and nonzero position/velocity correlations.
	ASSERT_TRUE(ekf.addRtk(1.1, state.p, Vector<3>::Zero(), 0, true, Vector<3>::Constant(1000.0),
	                       Vector<3>::Constant(1000.0), 0.1));
	const auto before = EkfImuTestPeer::covariance(ekf);
	const auto navigation_before = ekf.navigationQuality();
	ASSERT_TRUE(ekf.getAt(1.1, &state));
	Scalar anchor = NAN;
	ASSERT_TRUE(ekf.alignBarometerReference(1.1, 2.0, &anchor));
	EXPECT_DOUBLE_EQ(anchor, state.p.z());
	const auto aligned = EkfImuTestPeer::covariance(ekf);
	EXPECT_TRUE((aligned.topLeftCorner<16, 16>().isApprox(before.topLeftCorner<16, 16>(), 1e-14)));
	EXPECT_TRUE((aligned.col(16).head<16>().isApprox(-before.col(2).head<16>(), 1e-14)));
	EXPECT_NEAR(aligned(16, 16), before(2, 2) + 2.0, 1e-12);
	EXPECT_EQ(ekf.navigationQuality().accepted_updates, navigation_before.accepted_updates);
	ASSERT_TRUE(ekf.addBaro(1.1, anchor, 0.1, 9.0));
	const auto after = EkfImuTestPeer::covariance(ekf);
	EXPECT_TRUE((after.topLeftCorner<16, 16>().isApprox(before.topLeftCorner<16, 16>(), 1e-12)));
	QuadState updated = initial();
	ASSERT_TRUE(ekf.getAt(1.1, &updated));
	EXPECT_TRUE(updated.x.isApprox(state.x, 1e-12));
	EXPECT_LT(after(16, 16), aligned(16, 16));
	const Eigen::SelfAdjointEigenSolver<Matrix<17, 17>> eigenvalues(after);
	ASSERT_EQ(eigenvalues.info(), Eigen::Success);
	EXPECT_GE(eigenvalues.eigenvalues().minCoeff(), -1e-12);
}

TEST(EkfImuBaro, AlignedReferenceConstrainsRelativeHeightAndVelocityOverTime) {
	auto params = std::make_shared<EkfImuParameters>();
	params->Q_init_pos.z() = 9.0;
	params->Q_init_vel.z() = 4.0;
	params->baro_bias_random_walk = 0.0;
	EkfImu constrained(params), reference(params);
	for (EkfImu* ekf : {&constrained, &reference}) {
		ASSERT_TRUE(ekf->initialize(initial()));
		ASSERT_TRUE(ekf->addImu({1.0, -GVEC, Vector<3>::Zero()}));
		Scalar anchor = NAN;
		ASSERT_TRUE(ekf->alignBarometerReference(1.0, 0.2, &anchor));
		ASSERT_TRUE(ekf->addBaro(1.0, anchor, 0.01, 9.0));
		ASSERT_TRUE(ekf->addImu({1.2, -GVEC, Vector<3>::Zero()}));
	}
	ASSERT_TRUE(constrained.addBaro(1.2, 0.5, 0.01, 9.0));
	// Almost uninformative observation advances the reference covariance to the identical comparison time.
	ASSERT_TRUE(reference.addBaro(1.2, 0.0, 1e12, 9.0));
	QuadState state = initial();
	ASSERT_TRUE(constrained.getAt(1.2, &state));
	EXPECT_GT(state.p.z(), 0.3);
	EXPECT_GT(state.v.z(), 1.0);
	const auto covariance = EkfImuTestPeer::covariance(constrained);
	const auto reference_covariance = EkfImuTestPeer::covariance(reference);
	EXPECT_LT(covariance(2, 2), reference_covariance(2, 2));
	EXPECT_LT(covariance(9, 9), reference_covariance(9, 9));
	// Relative pressure alone cannot reduce the common unknown absolute height below its initial uncertainty.
	EXPECT_GE(covariance(2, 2), 9.0);
}

TEST(EkfImuBaro, ReferenceAlignmentRequiresCoverageAndRollsBackNumericalFailure) {
	EkfImu ekf;
	Scalar height = 123.0;
	EXPECT_FALSE(ekf.alignBarometerReference(1.0, 1.0, &height));
	ASSERT_TRUE(ekf.initialize(initial()));
	EXPECT_FALSE(ekf.alignBarometerReference(1.0, 1.0, &height));
	samples(ekf, true);
	EXPECT_FALSE(ekf.alignBarometerReference(1.1, 1.0, nullptr));
	EXPECT_FALSE(ekf.alignBarometerReference(NAN, 1.0, &height));
	EXPECT_FALSE(ekf.alignBarometerReference(1.1, NAN, &height));
	EXPECT_FALSE(ekf.alignBarometerReference(1.1, 0.0, &height));
	EXPECT_FALSE(ekf.alignBarometerReference(1.3, 1.0, &height));
	EXPECT_DOUBLE_EQ(height, 123.0);
	ASSERT_TRUE(ekf.alignBarometerReference(1.1, 1.0, &height));
	EXPECT_DOUBLE_EQ(height, 0.0);
	EXPECT_DOUBLE_EQ(ekf.navigationQuality().stamp, 1.1);
	EXPECT_EQ(ekf.navigationQuality().accepted_updates, 0u);
	EXPECT_FALSE(ekf.alignBarometerReference(1.0, 1.0, &height));
	EXPECT_FALSE(ekf.barometerQuality().valid);
	ASSERT_TRUE(fix(ekf, 1.1));
	ASSERT_TRUE(ekf.addBaro(1.1, height, 0.1, 9.0));
	ASSERT_TRUE(ekf.alignBarometerReference(1.2, 1.0, &height));
	EXPECT_EQ(ekf.barometerQuality().accepted_updates, 1u);
	EXPECT_FALSE(ekf.barometerQuality().valid);

	auto params = std::make_shared<EkfImuParameters>();
	params->Q_init_pos.z() = 1e308;
	EkfImu failed(params);
	ASSERT_TRUE(failed.initialize(initial()));
	samples(failed, true);
	const auto before = EkfImuTestPeer::covariance(failed);
	QuadState state_before = initial(), state_after = initial();
	ASSERT_TRUE(failed.getAt(1.2, &state_before));
	height = 456.0;
	EXPECT_FALSE(failed.alignBarometerReference(1.1, 1e308, &height));
	EXPECT_DOUBLE_EQ(height, 456.0);
	EXPECT_DOUBLE_EQ(failed.navigationQuality().stamp, 1.0);
	EXPECT_TRUE((EkfImuTestPeer::covariance(failed).array() == before.array()).all());
	ASSERT_TRUE(failed.getAt(1.2, &state_after));
	EXPECT_TRUE(state_after.x.isApprox(state_before.x, 1e-14));
}

TEST(EkfImuRtk, HorizontalModeOmitsVerticalInnovationAndPreservesHorizontalUpdates) {
	EkfImu reference, corrupted, full;
	for (EkfImu* ekf : {&reference, &corrupted, &full}) {
		ASSERT_TRUE(ekf->initialize(initial()));
		samples(*ekf, false);
	}
	const Vector<3> position(0.01, -0.02, 0.0);
	const Vector<3> velocity(0.1, -0.1, 0.0);
	const auto horizontal = EkfImu::NavigationMeasurementMode::kHorizontal;
	ASSERT_TRUE(reference.addRtk(1.1, position, velocity, 0.01, true, Vector<3>::Ones(), Vector<3>::Ones(), 0.1, 20.515, horizontal));
	ASSERT_TRUE(corrupted.addRtk(1.1, position + Vector<3>(0, 0, 1000), velocity + Vector<3>(0, 0, 100), 0.01, true, Vector<3>::Ones(),
	                             Vector<3>::Ones(), 0.1, 20.515, horizontal));
	QuadState expected = initial(), actual = initial();
	ASSERT_TRUE(reference.getAt(1.2, &expected));
	ASSERT_TRUE(corrupted.getAt(1.2, &actual));
	EXPECT_TRUE(expected.x.isApprox(actual.x, 1e-12));
	EXPECT_TRUE(EkfImuTestPeer::covariance(reference).isApprox(EkfImuTestPeer::covariance(corrupted), 1e-12));
	EXPECT_GT(actual.p.x(), 0);
	EXPECT_LT(actual.p.y(), 0);
	EXPECT_EQ(corrupted.navigationQuality().observation_dimensions, 5u);
	EXPECT_FALSE(full.addRtk(1.1, Vector<3>(0, 0, 1000), velocity, 0.01, true, Vector<3>::Ones(), Vector<3>::Ones(), 0.1, 24.322));
	EXPECT_EQ(full.navigationQuality().observation_dimensions, 7u);
	ASSERT_TRUE(corrupted.addRtk(1.2, position, velocity, NAN, false, Vector<3>::Ones(), Vector<3>::Ones(), NAN, 18.467, horizontal));
	EXPECT_EQ(corrupted.navigationQuality().observation_dimensions, 4u);
}

TEST(EkfImuBaro, RelativeHeightRejectsGnssHeightDriftWithoutInventingAnAbsoluteDatum) {
	auto params = std::make_shared<EkfImuParameters>();
	params->Q_init_pos.setConstant(4.0);
	params->Q_init_vel.setConstant(0.1);
	params->Q_init_bacc.setConstant(0.01);
	params->baro_bias_random_walk = 0;
	EkfImu ekf(params);
	QuadState state = initial();
	state.p.z() = 3.0;
	ASSERT_TRUE(ekf.initialize(state));
	ASSERT_TRUE(ekf.addImu({state.t, -GVEC, Vector<3>::Zero()}));
	Scalar anchor = NAN;
	ASSERT_TRUE(ekf.alignBarometerReference(state.t, 0.25, &anchor));
	for (int i = 1; i <= 3000; ++i) {
		const Scalar time = 1.0 + i * 0.01;
		ASSERT_TRUE(ekf.addImu({time, -GVEC, Vector<3>::Zero()}));
		if (i % 10 == 0) {
			ASSERT_TRUE(ekf.addRtk(time, Vector<3>(0, 0, 3.0 + 0.2 * (time - 1.0)), Vector<3>(0, 0, 0.2), 0, true,
			                       Vector<3>::Constant(4.0), Vector<3>::Constant(0.09), 0.01, 20.515,
			                       EkfImu::NavigationMeasurementMode::kHorizontal));
		}
		if (i % 2 == 0) {
			ASSERT_TRUE(ekf.addBaro(time, anchor, 0.278, 10.828));
		}
	}
	ASSERT_TRUE(ekf.getAt(31.0, &state));
	EXPECT_NEAR(state.p.z(), 3.0, 0.01);
	EXPECT_NEAR(state.v.z(), 0.0, 0.01);
	EXPECT_NEAR(ekf.barometerQuality().bias, 0.0, 0.01);
	// Pressure is relative: the common unknown GNSS datum must remain uncertain.
	EXPECT_GE(ekf.navigationQuality().position_variance.z(), 4.0 - 1e-6);
	EXPECT_EQ(ekf.navigationQuality().accepted_updates, 300u);
	EXPECT_EQ(ekf.barometerQuality().accepted_updates, 1500u);
	const Eigen::SelfAdjointEigenSolver<Matrix<17, 17>> eigenvalues(EkfImuTestPeer::covariance(ekf));
	ASSERT_EQ(eigenvalues.info(), Eigen::Success);
	EXPECT_GE(eigenvalues.eigenvalues().minCoeff(), -1e-10);
}

TEST(EkfImuBaro, RelativeHeightTracksRealClimbAndDescentWithMisleadingGnssHeight) {
	auto params = std::make_shared<EkfImuParameters>();
	params->Q_init_pos.setConstant(4.0);
	params->Q_init_vel.setConstant(0.1);
	params->Q_init_bacc.setConstant(0.01);
	params->baro_bias_random_walk = 0;
	EkfImu ekf(params);
	QuadState state = initial();
	state.p.z() = 3.0;
	ASSERT_TRUE(ekf.initialize(state));
	const Scalar frequency = 2.0 * M_PI / 12.0;
	ASSERT_TRUE(ekf.addImu({state.t, -GVEC + Vector<3>(0, 0, 1.5 * frequency * frequency), Vector<3>::Zero()}));
	Scalar anchor = NAN;
	ASSERT_TRUE(ekf.alignBarometerReference(state.t, 0.25, &anchor));
	Scalar max_height_error = 0, max_velocity_error = 0;
	for (int i = 1; i <= 2400; ++i) {
		const Scalar elapsed = i * 0.01, time = 1.0 + elapsed;
		const Scalar height = 3.0 + 1.5 * (1.0 - std::cos(frequency * elapsed));
		const Scalar velocity = 1.5 * frequency * std::sin(frequency * elapsed);
		const Scalar acceleration = 1.5 * frequency * frequency * std::cos(frequency * elapsed);
		ASSERT_TRUE(ekf.addImu({time, -GVEC + Vector<3>(0, 0, acceleration), Vector<3>::Zero()}));
		if (i % 10 == 0) {
			ASSERT_TRUE(ekf.addRtk(time, Vector<3>(0, 0, 3.0), Vector<3>::Zero(), 0, true, Vector<3>::Constant(4.0),
			                       Vector<3>::Constant(0.09), 0.01, 20.515, EkfImu::NavigationMeasurementMode::kHorizontal));
		}
		if (i % 2 == 0) {
			ASSERT_TRUE(ekf.addBaro(time, height, 0.278, 10.828));
		}
		ASSERT_TRUE(ekf.getAt(time, &state));
		max_height_error = std::max(max_height_error, std::abs(state.p.z() - height));
		max_velocity_error = std::max(max_velocity_error, std::abs(state.v.z() - velocity));
	}
	EXPECT_LT(max_height_error, 0.10);
	EXPECT_LT(max_velocity_error, 0.10);
	EXPECT_EQ(ekf.barometerQuality().rejected_updates, 0u);
	EXPECT_EQ(ekf.navigationQuality().rejected_updates, 0u);
}

namespace {
std::shared_ptr<EkfImuParameters> relativeReferenceParameters() {
	auto params = std::make_shared<EkfImuParameters>();
	params->baro_relative_reference = true;
	params->baro_bias_random_walk = 0;
	params->Q_init_pos.setConstant(4.0);
	params->Q_init_vel.setConstant(0.1);
	params->Q_init_bacc.setConstant(0.01);
	return params;
}

EkfImu::NavigationUpdateResult navigation(EkfImu& ekf, Scalar time, const Vector<3>& position, const Vector<3>& velocity,
                                          Scalar heading = 0) {
	return ekf.addNavigation(time, position, velocity, heading, true, Vector<3>(4, 4, 18), Vector<3>(0.04, 0.04, 0.09), 0.01, 20.515,
	                         10.828, 10.828);
}

void expectCovarianceValid(const EkfImu& ekf) {
	const auto covariance = EkfImuTestPeer::covariance(ekf);
	EXPECT_TRUE(covariance.allFinite());
	EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-12));
	const Eigen::SelfAdjointEigenSolver<Matrix<17, 17>> eigenvalues(covariance);
	ASSERT_EQ(eigenvalues.info(), Eigen::Success);
	EXPECT_GE(eigenvalues.eigenvalues().minCoeff(), -1e-10);
}
}  // namespace

TEST(EkfImuNavigation, ReferencePolicyRequiresZeroRandomWalkAndExplicitAlignment) {
	EkfImuParameters invalid;
	invalid.baro_relative_reference = true;
	EXPECT_FALSE(invalid.valid());
	invalid.baro_bias_random_walk = 0;
	EXPECT_TRUE(invalid.valid());
	EkfImu ekf(relativeReferenceParameters());
	ASSERT_TRUE(ekf.initialize(initial()));
	samples(ekf, false);
	EXPECT_FALSE(ekf.addBaro(1.0, 0, 0.278, 10.828));
	EXPECT_FALSE(ekf.barometerQuality().relative_reference_active);
	Scalar anchor = NAN;
	ASSERT_TRUE(ekf.alignBarometerReference(1.0, 4.0, &anchor));
	ASSERT_TRUE(ekf.barometerQuality().reference_valid);
	ASSERT_TRUE(ekf.barometerQuality().relative_reference_active);
	EXPECT_FALSE(ekf.updateParameters(std::make_shared<EkfImuParameters>()));
	EXPECT_FALSE(ekf.resetBaroBias(4.0));
	ASSERT_TRUE(ekf.initialize(initial()));
	EXPECT_FALSE(ekf.barometerQuality().reference_valid);
	EXPECT_FALSE(ekf.barometerQuality().relative_reference_active);
}

TEST(EkfImuNavigation, JointCorrectionMatchesSchmidtInTransformedCoordinates) {
	EkfImu ekf(relativeReferenceParameters());
	ASSERT_TRUE(ekf.initialize(initial()));
	ASSERT_TRUE(ekf.addImu({1.0, -GVEC, Vector<3>::Zero()}));
	Matrix<17, 17> square_root;
	for (int row = 0; row < 17; ++row)
		for (int column = 0; column < 17; ++column)
			square_root(row, column) = 0.1 * std::sin(1.7 * row + 0.6 * column) + (row == column ? 0.3 : 0.0);
	const Matrix<17, 17> dense_covariance = square_root * square_root.transpose();
	EkfImuTestPeer::setCovariance(&ekf, dense_covariance);
	Scalar anchor = NAN;
	ASSERT_TRUE(ekf.alignBarometerReference(1.0, 0.25, &anchor));
	const Matrix<17, 17> covariance = EkfImuTestPeer::covariance(ekf);
	const Vector<17> prior = EkfImuTestPeer::posterior(ekf);
	Matrix<17, 17> transform = Matrix<17, 17>::Identity(), inverse = transform;
	transform(2, 16) = 1;
	inverse(2, 16) = -1;
	const Matrix<17, 17> transformed_covariance = transform * covariance * transform.transpose();
	Matrix<7, 17> observation = Matrix<7, 17>::Zero();
	observation.block<3, 3>(0, 0).setIdentity();
	observation.block<3, 3>(3, 7).setIdentity();
	observation(6, 6) = 2;  // Yaw Jacobian at the identity quaternion.
	const Matrix<7, 17> transformed_observation = observation * inverse;
	Vector<7> variances;
	variances << 4, 4, 18, 0.04, 0.04, 0.09, 0.01;
	const Matrix<7, 7> noise = variances.asDiagonal();
	const Matrix<7, 7> innovation = transformed_observation * transformed_covariance * transformed_observation.transpose() + noise;
	Matrix<17, 7> transformed_gain = transformed_covariance * transformed_observation.transpose() * innovation.inverse();
	transformed_gain.row(16).setZero();
	Vector<7> residual;
	residual << -0.2, 0.1, -0.7, -0.04, 0.03, -0.08, 0;
	Vector<17> expected = inverse * (transform * prior - transformed_gain * residual);
	const Matrix<17, 17> correction = Matrix<17, 17>::Identity() - transformed_gain * transformed_observation;
	Matrix<17, 17> expected_covariance =
	        inverse *
	        (correction * transformed_covariance * correction.transpose() + transformed_gain * noise * transformed_gain.transpose()) *
	        inverse.transpose();
	const Scalar quaternion_norm = expected.segment<4>(3).norm();
	const Vector<4> quaternion = expected.segment<4>(3) / quaternion_norm;
	Matrix<17, 17> normalization = Matrix<17, 17>::Identity();
	normalization.block<4, 4>(3, 3) = (Matrix<4, 4>::Identity() - quaternion * quaternion.transpose()) / quaternion_norm;
	expected.segment<4>(3) = quaternion;
	expected_covariance = (normalization * expected_covariance * normalization.transpose()).eval();
	const auto result = navigation(ekf, 1.0, Vector<3>(0.2, -0.1, 0.7), Vector<3>(0.04, -0.03, 0.08));
	ASSERT_TRUE(result.committed);
	EXPECT_EQ(result.accepted_groups, 31u);
	EXPECT_TRUE(EkfImuTestPeer::posterior(ekf).isApprox(expected, 1e-12));
	EXPECT_TRUE(EkfImuTestPeer::covariance(ekf).isApprox(expected_covariance, 1e-12));
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias, 0);
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias_variance, covariance(16, 16));
	expectCovarianceValid(ekf);
}

TEST(EkfImuNavigation, IndependentHeightAndVelocityGatesCommitOneEpoch) {
	EkfImu ekf(relativeReferenceParameters());
	ASSERT_TRUE(ekf.initialize(initial()));
	samples(ekf, false);
	Scalar anchor = NAN;
	ASSERT_TRUE(ekf.alignBarometerReference(1.0, 4.0, &anchor));
	const auto height_rejected = navigation(ekf, 1.1, Vector<3>(0.1, 0, 1000), Vector<3>(0.1, 0, 0.1));
	ASSERT_TRUE(height_rejected.committed);
	EXPECT_EQ(height_rejected.accepted_groups, EkfImu::kHorizontalGroup | EkfImu::kVerticalVelocityGroup | EkfImu::kHeadingGroup);
	EXPECT_EQ(height_rejected.rejected_groups, EkfImu::kHeightGroup);
	auto quality = ekf.navigationQuality();
	EXPECT_EQ(quality.horizontal.stamp, 1.1);
	EXPECT_TRUE(std::isnan(quality.height.stamp));
	EXPECT_EQ(quality.height.reason, EkfImu::NavigationUpdateReason::kInnovation);
	EXPECT_EQ(quality.height.measurement_variance(0), 18);
	EXPECT_EQ(quality.vertical_velocity.measurement_variance(0), 0.09);
	EXPECT_EQ(quality.accepted_updates, 1u);
	EXPECT_EQ(quality.observation_dimensions, 4u);
	EXPECT_EQ(quality.innovation_squared, quality.horizontal.innovation_squared);
	const auto posterior = EkfImuTestPeer::posterior(ekf);
	const auto duplicate = navigation(ekf, 1.1, Vector<3>::Zero(), Vector<3>::Zero());
	EXPECT_FALSE(duplicate.committed);
	EXPECT_EQ(duplicate.reason, EkfImu::NavigationUpdateReason::kTimestamp);
	EXPECT_TRUE(EkfImuTestPeer::posterior(ekf).isApprox(posterior, 1e-15));
	EXPECT_FALSE(ekf.navigationQuality().horizontal.accepted);
	EXPECT_EQ(ekf.navigationQuality().horizontal.reason, EkfImu::NavigationUpdateReason::kTimestamp);
	EXPECT_EQ(ekf.navigationQuality().horizontal.stamp, 1.1);
	EXPECT_EQ(ekf.navigationQuality().horizontal.accepted_updates, 1u);
	EXPECT_EQ(ekf.navigationQuality().horizontal.rejected_updates, 0u);
	EXPECT_TRUE(std::isnan(ekf.navigationQuality().innovation_squared));
	EXPECT_EQ(ekf.navigationQuality().observation_dimensions, ekf.navigationQuality().horizontal.observation_dimensions);
	ASSERT_TRUE(ekf.addBaro(1.1, anchor, 0.278, 10.828));
	const auto velocity_rejected = navigation(ekf, 1.2, Vector<3>(0.1, 0, 0), Vector<3>(0.1, 0, 1000));
	ASSERT_TRUE(velocity_rejected.committed);
	EXPECT_EQ(velocity_rejected.accepted_groups, EkfImu::kHorizontalGroup | EkfImu::kHeightGroup | EkfImu::kHeadingGroup);
	EXPECT_EQ(velocity_rejected.rejected_groups, EkfImu::kVerticalVelocityGroup);
	quality = ekf.navigationQuality();
	EXPECT_EQ(quality.stamp, 1.2);
	EXPECT_EQ(quality.height.stamp, 1.2);
	EXPECT_EQ(quality.vertical_velocity.stamp, 1.1);
	EXPECT_EQ(quality.vertical_velocity.reason, EkfImu::NavigationUpdateReason::kInnovation);
	EXPECT_EQ(quality.accepted_updates, 2u);
	EXPECT_EQ(quality.rejected_updates, 0u);
	expectCovarianceValid(ekf);
}

TEST(EkfImuNavigation, VerticalOnlyCommitDoesNotRenewHorizontalAcceptance) {
	EkfImu ekf(relativeReferenceParameters());
	ASSERT_TRUE(ekf.initialize(initial()));
	samples(ekf, false);
	const auto result = navigation(ekf, 1.1, Vector<3>(1000, 1000, 0), Vector<3>(1000, 1000, 0));
	ASSERT_TRUE(result.committed);
	EXPECT_EQ(result.accepted_groups, EkfImu::kHeightGroup | EkfImu::kVerticalVelocityGroup | EkfImu::kHeadingGroup);
	const auto quality = ekf.navigationQuality();
	EXPECT_EQ(quality.stamp, 1.1);
	EXPECT_TRUE(std::isnan(quality.horizontal.stamp));
	EXPECT_EQ(quality.accepted_updates, 0u);
	EXPECT_EQ(quality.rejected_updates, 1u);
	EXPECT_EQ(quality.height.stamp, 1.1);
}

TEST(EkfImuNavigation, AllRejectedAndNumericalFailureRestorePosteriorAndCovariance) {
	EkfImu rejected, reference;
	for (EkfImu* ekf : {&rejected, &reference}) {
		ASSERT_TRUE(ekf->initialize(initial()));
		samples(*ekf, true);
	}
	const auto prior = EkfImuTestPeer::posterior(rejected);
	const auto covariance = EkfImuTestPeer::covariance(rejected);
	const auto result = navigation(rejected, 1.1, Vector<3>::Constant(1000), Vector<3>::Constant(1000), M_PI);
	EXPECT_FALSE(result.committed);
	EXPECT_FALSE(result.numerical_failure);
	EXPECT_EQ(result.rejected_groups, 31u);
	EXPECT_TRUE(EkfImuTestPeer::posterior(rejected).isApprox(prior, 1e-15));
	EXPECT_TRUE(EkfImuTestPeer::covariance(rejected).isApprox(covariance, 1e-15));
	EXPECT_EQ(rejected.navigationQuality().stamp, 1.0);
	const auto duplicate = navigation(rejected, 1.1, Vector<3>::Zero(), Vector<3>::Zero());
	EXPECT_FALSE(duplicate.committed);
	EXPECT_EQ(duplicate.reason, EkfImu::NavigationUpdateReason::kTimestamp);
	EXPECT_EQ(rejected.navigationQuality().horizontal.rejected_updates, 1u);
	EXPECT_EQ(rejected.navigationQuality().height.rejected_updates, 1u);
	EXPECT_EQ(rejected.navigationQuality().vertical_velocity.rejected_updates, 1u);
	ASSERT_TRUE(navigation(rejected, 1.2, Vector<3>::Zero(), Vector<3>::Zero()).committed);
	ASSERT_TRUE(navigation(reference, 1.2, Vector<3>::Zero(), Vector<3>::Zero()).committed);
	EXPECT_TRUE(EkfImuTestPeer::posterior(rejected).isApprox(EkfImuTestPeer::posterior(reference), 1e-12));
	EXPECT_TRUE(EkfImuTestPeer::covariance(rejected).isApprox(EkfImuTestPeer::covariance(reference), 1e-12));

	EkfImu numerical;
	ASSERT_TRUE(numerical.initialize(initial()));
	ASSERT_TRUE(numerical.addImu({1.0, -GVEC, Vector<3>::Zero()}));
	Matrix<17, 17> invalid = Matrix<17, 17>::Identity();
	invalid(0, 0) = -100;
	EkfImuTestPeer::setCovariance(&numerical, invalid);
	const auto failed = navigation(numerical, 1.0, Vector<3>::Zero(), Vector<3>::Zero());
	EXPECT_FALSE(failed.committed);
	EXPECT_TRUE(failed.numerical_failure);
	EXPECT_EQ(failed.accepted_groups, 0u);
	EXPECT_EQ(failed.rejected_groups, 31u);
	EXPECT_TRUE(EkfImuTestPeer::covariance(numerical).isApprox(invalid, 1e-15));
	EXPECT_EQ(numerical.navigationQuality().height.reason, EkfImu::NavigationUpdateReason::kNumericalFailure);
}

TEST(EkfImuNavigation, FrozenReferenceSurvivesEveryMeasurementPathAndExplicitInvalidation) {
	EkfImu ekf(relativeReferenceParameters());
	ASSERT_TRUE(ekf.initialize(initial()));
	samples(ekf, false);
	Scalar anchor = NAN;
	ASSERT_TRUE(ekf.alignBarometerReference(1.0, 4.0, &anchor));
	const Scalar bias_variance = ekf.barometerQuality().bias_variance;
	ASSERT_TRUE(ekf.addRtk(1.05, Vector<3>(0, 0, 1), Vector<3>::Zero(), 0, true, Vector<3>::Constant(18), Vector<3>::Constant(0.09),
	                       0.01, 24.322));
	ASSERT_TRUE(ekf.addBaro(1.05, 0.2, 0.278, 10.828));
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias, 0);
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias_variance, bias_variance);
	ASSERT_TRUE(ekf.addPose({1.1, Vector<3>(0, 0, 0.25), Quaternion::Identity()}));
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias, 0);
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias_variance, bias_variance);
	expectCovarianceValid(ekf);
	const auto posterior = EkfImuTestPeer::posterior(ekf);
	const auto covariance = EkfImuTestPeer::covariance(ekf);
	ekf.clearBarometerReference();
	EXPECT_FALSE(ekf.barometerQuality().reference_valid);
	EXPECT_FALSE(ekf.barometerQuality().valid);
	EXPECT_TRUE(ekf.barometerQuality().relative_reference_active);
	EXPECT_TRUE(EkfImuTestPeer::posterior(ekf).isApprox(posterior, 1e-15));
	EXPECT_TRUE(EkfImuTestPeer::covariance(ekf).isApprox(covariance, 1e-15));
	EXPECT_FALSE(ekf.addBaro(1.2, 0.25, 0.278, 10.828));
	ASSERT_TRUE(navigation(ekf, 1.2, Vector<3>(0, 0, 6), Vector<3>::Zero()).committed);
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias, 0);
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias_variance, bias_variance);
	expectCovarianceValid(ekf);
}

TEST(EkfImuNavigation, RelativeSchmidtSuppressesSlowGnssDriftAndRetainsDatumUncertainty) {
	EkfImu ekf(relativeReferenceParameters());
	ASSERT_TRUE(ekf.initialize(initial()));
	ASSERT_TRUE(ekf.addImu({1.0, -GVEC, Vector<3>::Zero()}));
	Scalar anchor = NAN;
	ASSERT_TRUE(ekf.alignBarometerReference(1.0, 4.0, &anchor));
	const Scalar bias_variance = ekf.barometerQuality().bias_variance;
	for (int step = 1; step <= 3000; ++step) {
		const Scalar elapsed = step * 0.01, time = 1.0 + elapsed;
		ASSERT_TRUE(ekf.addImu({time, -GVEC, Vector<3>::Zero()}));
		if (step % 10 == 0) {
			const auto result = navigation(ekf, time, Vector<3>(0, 0, elapsed * 0.2), Vector<3>::Zero());
			ASSERT_TRUE(result.committed);
			ASSERT_EQ(result.accepted_groups, 31u);
		}
		if (step % 2 == 0) {
			ASSERT_TRUE(ekf.addBaro(time, -elapsed / 300.0, 0.278, 10.828));
		}
	}
	QuadState state = initial();
	ASSERT_TRUE(ekf.getAt(31.0, &state));
	EXPECT_NEAR(state.p.z(), -0.1, 0.05);
	EXPECT_NEAR(state.v.z(), 0, 0.03);
	const auto quality = ekf.barometerQuality();
	EXPECT_DOUBLE_EQ(quality.bias, 0);
	EXPECT_DOUBLE_EQ(quality.bias_variance, bias_variance);
	EXPECT_LT(quality.relative_height_variance, 0.1);
	EXPECT_GT(ekf.navigationQuality().position_variance.z(), 4.0);
	EXPECT_EQ(ekf.navigationQuality().height.accepted_updates, 300u);
	EXPECT_EQ(ekf.navigationQuality().vertical_velocity.accepted_updates, 300u);
	expectCovarianceValid(ekf);
}

TEST(EkfImuNavigation, RelativeSchmidtTracksClimbAndDescentWithIndependentVerticalVelocity) {
	EkfImu ekf(relativeReferenceParameters());
	ASSERT_TRUE(ekf.initialize(initial()));
	const Scalar frequency = 2 * M_PI / 12;
	ASSERT_TRUE(ekf.addImu({1.0, -GVEC + Vector<3>(0, 0, 1.5 * frequency * frequency), Vector<3>::Zero()}));
	Scalar anchor = NAN;
	ASSERT_TRUE(ekf.alignBarometerReference(1.0, 4.0, &anchor));
	Scalar maximum_height_error = 0, maximum_velocity_error = 0;
	for (int step = 1; step <= 1200; ++step) {
		const Scalar elapsed = step * 0.01, time = 1.0 + elapsed;
		const Scalar height = 1.5 * (1 - std::cos(frequency * elapsed));
		const Scalar velocity = 1.5 * frequency * std::sin(frequency * elapsed);
		const Scalar acceleration = 1.5 * frequency * frequency * std::cos(frequency * elapsed);
		ASSERT_TRUE(ekf.addImu({time, -GVEC + Vector<3>(0, 0, acceleration), Vector<3>::Zero()}));
		if (step % 10 == 0) {
			ASSERT_TRUE(navigation(ekf, time, Vector<3>(0, 0, height + 0.2 * elapsed), Vector<3>(0, 0, velocity)).committed);
		}
		if (step % 2 == 0) {
			ASSERT_TRUE(ekf.addBaro(time, height, 0.278, 10.828));
		}
		QuadState state = initial();
		ASSERT_TRUE(ekf.getAt(time, &state));
		maximum_height_error = std::max(maximum_height_error, std::abs(state.p.z() - height));
		maximum_velocity_error = std::max(maximum_velocity_error, std::abs(state.v.z() - velocity));
	}
	EXPECT_LT(maximum_height_error, 0.1);
	EXPECT_LT(maximum_velocity_error, 0.1);
	EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias, 0);
	EXPECT_EQ(ekf.navigationQuality().vertical_velocity.rejected_updates, 0u);
	expectCovarianceValid(ekf);
}

TEST(EkfImuNavigation, DelayedOrderedFusionMatchesImmediateFusionWithQueries) {
	EkfImu immediate(relativeReferenceParameters()), delayed(relativeReferenceParameters());
	for (EkfImu* ekf : {&immediate, &delayed}) {
		ASSERT_TRUE(ekf->initialize(initial()));
		ASSERT_TRUE(ekf->addImu({1.0, -GVEC, Vector<3>::Zero()}));
		Scalar anchor = NAN;
		ASSERT_TRUE(ekf->alignBarometerReference(1.0, 4.0, &anchor));
	}
	const auto observe = [](EkfImu& ekf, const Scalar time) {
		ASSERT_TRUE(navigation(ekf, time, Vector<3>(0.1, -0.1, 0.5), Vector<3>(0.02, 0, 0.1)).committed);
		ASSERT_TRUE(ekf.addBaro(time, 0.1 * (time - 1.0), 0.278, 10.828));
	};
	for (int step = 1; step <= 300; ++step) {
		const Scalar time = 1.0 + step * 0.001;
		const ImuSample imu{time, -GVEC + Vector<3>(0.02, 0, 0.1 * std::sin(time)), Vector<3>(0, 0, 0.03)};
		ASSERT_TRUE(immediate.addImu(imu));
		ASSERT_TRUE(delayed.addImu(imu));
		if (step == 100 || step == 200) observe(immediate, time);
		QuadState state = initial();
		ASSERT_TRUE(immediate.getAt(time, &state));
		ASSERT_TRUE(delayed.getAt(time, &state));
	}
	observe(delayed, 1.1);
	observe(delayed, 1.2);
	QuadState expected = initial(), actual = initial();
	ASSERT_TRUE(immediate.getAt(1.3, &expected));
	ASSERT_TRUE(delayed.getAt(1.3, &actual));
	EXPECT_TRUE(actual.x.isApprox(expected.x, 1e-10));
	EXPECT_TRUE(EkfImuTestPeer::covariance(delayed).isApprox(EkfImuTestPeer::covariance(immediate), 1e-10));
	EXPECT_DOUBLE_EQ(delayed.barometerQuality().bias, immediate.barometerQuality().bias);
	EXPECT_EQ(delayed.navigationQuality().horizontal.accepted_updates, 2u);
}

TEST(EkfImuNavigation, SameEpochPressureAndNavigationAreAcceptedInEitherOrder) {
	for (const bool pressure_first : {false, true}) {
		EkfImu ekf(relativeReferenceParameters());
		ASSERT_TRUE(ekf.initialize(initial()));
		ASSERT_TRUE(ekf.addImu({1.0, -GVEC, Vector<3>::Zero()}));
		Scalar anchor = NAN;
		ASSERT_TRUE(ekf.alignBarometerReference(1.0, 4.0, &anchor));
		if (pressure_first) {
			ASSERT_TRUE(ekf.addBaro(1.0, 0.1, 0.278, 10.828));
		}
		ASSERT_TRUE(navigation(ekf, 1.0, Vector<3>(0.1, 0, 0.5), Vector<3>::Zero()).committed);
		if (!pressure_first) {
			ASSERT_TRUE(ekf.addBaro(1.0, 0.1, 0.278, 10.828));
		}
		EXPECT_EQ(ekf.navigationQuality().accepted_updates, 1u);
		EXPECT_EQ(ekf.barometerQuality().accepted_updates, 1u);
		EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias, 0);
		EXPECT_DOUBLE_EQ(ekf.barometerQuality().bias_variance, 8);
		EXPECT_FALSE(navigation(ekf, 1.0, Vector<3>::Zero(), Vector<3>::Zero()).committed);
		EXPECT_FALSE(ekf.addBaro(1.0, 0.1, 0.278, 10.828));
		expectCovarianceValid(ekf);
	}
	// Schmidt corrections need not commute; ROS establishes navigation-before-pressure ordering.
}

TEST(EkfImuNavigation, BadOrMissingHeadingDoesNotDiscardPositionAndVelocity) {
	for (const bool heading_valid : {true, false}) {
		EkfImu ekf;
		ASSERT_TRUE(ekf.initialize(initial()));
		samples(ekf, true);
		const auto result = ekf.addNavigation(1.1, Vector<3>(0.1, 0, 0), Vector<3>(0.1, 0, 0), heading_valid ? M_PI : NAN,
		                                      heading_valid, Vector<3>::Ones(), Vector<3>::Ones(), 0.01, 18.467, 10.828, 10.828);
		ASSERT_TRUE(result.committed);
		EXPECT_EQ(result.accepted_groups, 15u);
		EXPECT_EQ(result.rejected_groups, EkfImu::kHeadingGroup);
		const auto quality = ekf.navigationQuality();
		EXPECT_TRUE(quality.horizontal.accepted);
		EXPECT_TRUE(quality.position.accepted);
		EXPECT_TRUE(quality.velocity.accepted);
		EXPECT_FALSE(quality.heading.accepted);
		EXPECT_TRUE(std::isnan(quality.heading.stamp));
		EXPECT_EQ(quality.heading.reason,
		          heading_valid ? EkfImu::NavigationUpdateReason::kInnovation : EkfImu::NavigationUpdateReason::kInvalidInput);
		expectCovarianceValid(ekf);
	}
}

TEST(EkfImuNavigation, InvalidPositionDoesNotDiscardVelocityOrRefreshHorizontalAcceptance) {
	EkfImu ekf;
	ASSERT_TRUE(ekf.initialize(initial()));
	samples(ekf, false);
	const auto result = ekf.addNavigation(1.1, Vector<3>(NAN, 0, 0), Vector<3>::Zero(), 0, true, Vector<3>::Ones(), Vector<3>::Ones(),
	                                      0.01, 18.467, 10.828, 10.828);
	ASSERT_TRUE(result.committed);
	EXPECT_EQ(result.rejected_groups, EkfImu::kPositionGroup);
	const auto quality = ekf.navigationQuality();
	EXPECT_EQ(quality.velocity.stamp, 1.1);
	EXPECT_TRUE(std::isnan(quality.position.stamp));
	EXPECT_TRUE(std::isnan(quality.horizontal.stamp));
	EXPECT_FALSE(quality.horizontal.accepted);
	expectCovarianceValid(ekf);
}

TEST(EkfImuNavigation, DiagnosticsContainCommonPriorAndCorrelatedInnovation) {
	EkfImu ekf;
	ASSERT_TRUE(ekf.initialize(initial()));
	ASSERT_TRUE(ekf.addImu({1.0, -GVEC, Vector<3>::Zero()}));
	Matrix<17, 17> covariance = Matrix<17, 17>::Identity() * 0.01;
	covariance(0, 0) = covariance(1, 1) = 2;
	covariance(0, 1) = covariance(1, 0) = 1.8;
	covariance(0, 7) = covariance(7, 0) = 0.02;
	EkfImuTestPeer::setCovariance(&ekf, covariance);
	ekf.setNavigationDiagnosticsEnabled(true);
	const auto result = ekf.addNavigation(1.0, Vector<3>(0.5, -0.5, 0.1), Vector<3>(0.1, 0, 0), 0.02, true, Vector<3>::Ones() * 0.1,
	                                      Vector<3>::Ones() * 0.1, 0.1, 18.467, 10.828, 10.828);
	ASSERT_TRUE(result.committed);
	const auto diagnostics = ekf.navigationDiagnostics();
	ASSERT_TRUE(diagnostics.valid);
	EXPECT_TRUE(diagnostics.prior_covariance.isApprox(covariance, 1e-14));
	EXPECT_TRUE(diagnostics.posterior_covariance.isApprox(EkfImuTestPeer::covariance(ekf), 1e-14));
	EXPECT_TRUE(diagnostics.posterior_state.isApprox(EkfImuTestPeer::posterior(ekf), 1e-14));
	const Vector<2> residual = diagnostics.residual.head<2>();
	const Matrix<2, 2> innovation = diagnostics.innovation_covariance.topLeftCorner<2, 2>();
	const Scalar expected_nis = residual.dot(innovation.inverse() * residual);
	const Scalar diagonal_approximation = (residual.array().square() / innovation.diagonal().array()).sum();
	EXPECT_NEAR(diagnostics.groups[0].innovation_squared, expected_nis, 1e-12);
	EXPECT_NEAR(diagnostics.whitened_innovation.col(0).squaredNorm(), expected_nis, 1e-12);
	EXPECT_GT(std::abs(expected_nis - diagonal_approximation), 1.0);
	EXPECT_EQ(diagnostics.groups[0].observation_dimensions, 2u);
	EXPECT_EQ(diagnostics.groups[1].observation_dimensions, 2u);
	EXPECT_EQ(diagnostics.groups[2].observation_dimensions, 1u);
	Vector<17> expected_state = diagnostics.prior_state + diagnostics.correction_contributions.rowwise().sum();
	expected_state.segment<4>(3).normalize();
	EXPECT_TRUE(expected_state.isApprox(diagnostics.posterior_state, 1e-12));
	expectCovarianceValid(ekf);
}

TEST(EkfImuNavigation, PredictedCovarianceSharesStateEpochWithoutChangingDelayedPosterior) {
	auto parameters = std::make_shared<EkfImuParameters>();
	parameters->Q_init_vel.setConstant(1);
	EkfImu queried(parameters), reference(parameters);
	for (EkfImu* ekf : {&queried, &reference}) {
		ASSERT_TRUE(ekf->initialize(initial()));
		samples(*ekf, false);
	}
	const Matrix<17, 17> before = EkfImuTestPeer::covariance(queried);
	EkfImu::NavigationQuality quality, repeated;
	QuadState state = initial();
	ASSERT_TRUE(queried.getAt(1.1, &state));
	ASSERT_TRUE(queried.getAt(1.2, &state, &quality));
	EXPECT_DOUBLE_EQ(quality.stamp, state.t);
	EXPECT_DOUBLE_EQ(quality.stamp, 1.2);
	EXPECT_DOUBLE_EQ(queried.navigationQuality().stamp, 1.0);
	EXPECT_GT(quality.position_variance.x(), before(0, 0) + 0.039);
	ASSERT_TRUE(queried.getAt(1.15, &state));
	ASSERT_TRUE(queried.getAt(1.2, &state, &repeated));
	EXPECT_TRUE(quality.position_variance.isApprox(repeated.position_variance, 1e-14));
	EXPECT_TRUE(EkfImuTestPeer::covariance(queried).isApprox(before, 1e-14));
	ASSERT_TRUE(navigation(queried, 1.1, Vector<3>::Zero(), Vector<3>::Zero()).committed);
	ASSERT_TRUE(navigation(reference, 1.1, Vector<3>::Zero(), Vector<3>::Zero()).committed);
	ASSERT_TRUE(queried.getAt(1.2, &state, &quality));
	ASSERT_TRUE(reference.getAt(1.2, &state, &repeated));
	EXPECT_TRUE(quality.position_variance.isApprox(repeated.position_variance, 1e-12));
	EXPECT_TRUE(EkfImuTestPeer::covariance(queried).isApprox(EkfImuTestPeer::covariance(reference), 1e-12));
}

namespace {
std::shared_ptr<EkfImuParameters> continuousNoiseParameters(const Scalar integration_step) {
	auto parameters = std::make_shared<EkfImuParameters>();
	parameters->process_noise_model = "continuous_psd";
	parameters->integration_step = integration_step;
	parameters->R_acc.setConstant(0.04);
	parameters->R_omega.setZero();
	parameters->Q_pos.setZero();
	parameters->Q_att.setZero();
	parameters->Q_vel.setZero();
	parameters->Q_bome.setZero();
	parameters->Q_bacc.setConstant(0.003);
	parameters->Q_init_pos.setZero();
	parameters->Q_init_att.setZero();
	parameters->Q_init_vel.setZero();
	parameters->Q_init_bome.setZero();
	parameters->Q_init_bacc.setZero();
	parameters->baro_bias_random_walk = 0;
	return parameters;
}
}  // namespace

TEST(EkfImuNavigation, ContinuousNoiseIsIndependentOfSubstepsAndInputFrequency) {
	Matrix<17, 17> reference = Matrix<17, 17>::Zero();
	for (const int sample_rate : {250, 500, 1000}) {
		for (const Scalar integration_step : {1e-4, 5e-4, 1e-3, 2e-3}) {
			EkfImu ekf(continuousNoiseParameters(integration_step));
			ASSERT_TRUE(ekf.initialize(initial()));
			for (int sample = 0; sample <= sample_rate; ++sample)
				ASSERT_TRUE(ekf.addImu({1.0 + Scalar(sample) / sample_rate, -GVEC, Vector<3>::Zero()}));
			ekf.setNavigationDiagnosticsEnabled(true);
			ASSERT_TRUE(navigation(ekf, 2.0, Vector<3>::Zero(), Vector<3>::Zero()).committed);
			const Matrix<17, 17> covariance = ekf.navigationDiagnostics().prior_covariance;
			// Closed-form integrated white acceleration noise plus acceleration-bias random walk at dt=1 s.
			EXPECT_NEAR(covariance(0, 0), 0.04 / 3 + 0.003 / 20, 2e-8);
			EXPECT_NEAR(covariance(7, 7), 0.04 + 0.003 / 3, 2e-8);
			EXPECT_NEAR(covariance(13, 13), 0.003, 1e-12);
			EXPECT_NEAR(covariance(7, 13), -0.003 / 2, 1e-10);
			if (reference.isZero()) reference = covariance;
			EXPECT_LT((covariance - reference).cwiseAbs().maxCoeff(), 2e-8);
			expectCovarianceValid(ekf);
		}
	}
}

TEST(EkfImuNavigation, NoiseUnitsRequireExplicitModelAndRejectNegativeVariance) {
	EkfImuParameters parameters;
	EXPECT_EQ(parameters.process_noise_model, "legacy_dt2");
	parameters.process_noise_model = "sample_variance";
	EXPECT_FALSE(parameters.valid());
	parameters.process_noise_model = "continuous_psd";
	EXPECT_TRUE(parameters.valid());
	parameters.R_acc.x() = -1;
	EXPECT_FALSE(parameters.valid());
	parameters.R_acc.x() = 1;
	parameters.Q_bacc.x() = -1;
	EXPECT_FALSE(parameters.valid());
	parameters.Q_bacc.x() = 0;
	parameters.integration_step = 0;
	EXPECT_FALSE(parameters.valid());
}

TEST(EkfImuNavigation, DynamicPropagationConvergesAcrossIntegrationSubsteps) {
	QuadState reference = initial();
	EkfImu::NavigationQuality reference_quality;
	for (const Scalar integration_step : {1e-4, 5e-4, 1e-3, 2e-3}) {
		EkfImu ekf(continuousNoiseParameters(integration_step));
		ASSERT_TRUE(ekf.initialize(initial()));
		for (int sample = 0; sample <= 500; ++sample)
			ASSERT_TRUE(ekf.addImu({1.0 + 0.002 * sample, -GVEC + Vector<3>(1, -0.3, 0.2), Vector<3>(0.3, -0.2, 0.1)}));
		QuadState state = initial();
		EkfImu::NavigationQuality quality;
		ASSERT_TRUE(ekf.getAt(2.0, &state, &quality));
		if (integration_step == 1e-4) {
			reference = state;
			reference_quality = quality;
		}
		EXPECT_LT((state.p - reference.p).norm(), 0.005);
		EXPECT_LT((state.v - reference.v).norm(), 0.005);
		EXPECT_LT((state.qx - reference.qx).norm(), 1e-6);
		EXPECT_LT((quality.position_variance - reference_quality.position_variance).norm(), 1e-5);
		EXPECT_LT((quality.velocity_variance - reference_quality.velocity_variance).norm(), 1e-5);
	}
}

TEST(EkfImuBaro, DiagnosticsExposeCrossCovarianceCorrectionAndRejectedPrior) {
	EkfImu ekf;
	ASSERT_TRUE(ekf.initialize(initial()));
	samples(ekf, false);
	Matrix<17, 17> covariance = Matrix<17, 17>::Identity();
	covariance(2, 8) = covariance(8, 2) = 0.2;
	covariance(2, 14) = covariance(14, 2) = 0.1;
	EkfImuTestPeer::setCovariance(&ekf, covariance);
	ekf.setNavigationDiagnosticsEnabled(true);
	ASSERT_TRUE(ekf.addBaro(1.0, 1, 0.1, 10.828));
	const auto diagnostics = ekf.barometerDiagnostics();
	ASSERT_TRUE(diagnostics.valid);
	EXPECT_TRUE(diagnostics.result.committed);
	EXPECT_EQ(diagnostics.groups[3].reason, EkfImu::NavigationUpdateReason::kAccepted);
	EXPECT_TRUE(diagnostics.prior_covariance.isApprox(covariance, 1e-14));
	EXPECT_DOUBLE_EQ(diagnostics.observation(2, 2), 1);
	EXPECT_DOUBLE_EQ(diagnostics.observation(2, 16), 1);
	EXPECT_NEAR(diagnostics.innovation_covariance(2, 2), 2.1, 1e-12);
	EXPECT_NEAR(diagnostics.groups[3].innovation_squared, 1.0 / 2.1, 1e-12);
	EXPECT_NEAR(diagnostics.whitened_innovation.col(3).squaredNorm(), 1.0 / 2.1, 1e-12);
	EXPECT_NEAR(diagnostics.correction_contributions(8, 2), 0.2 / 2.1, 1e-12);
	EXPECT_NEAR(diagnostics.correction_contributions(14, 2), 0.1 / 2.1, 1e-12);
	EXPECT_TRUE(diagnostics.posterior_state.isApprox(EkfImuTestPeer::posterior(ekf), 1e-14));
	EXPECT_TRUE(diagnostics.posterior_covariance.isApprox(EkfImuTestPeer::covariance(ekf), 1e-14));
	const auto posterior = EkfImuTestPeer::posterior(ekf);
	EXPECT_FALSE(ekf.addBaro(1.1, 10000, 0.1, 10.828));
	const auto rejected = ekf.barometerDiagnostics();
	EXPECT_EQ(rejected.groups[3].reason, EkfImu::NavigationUpdateReason::kInnovation);
	EXPECT_TRUE(rejected.posterior_state.isApprox(rejected.prior_state, 1e-14));
	EXPECT_TRUE(rejected.posterior_covariance.isApprox(rejected.prior_covariance, 1e-14));
	EXPECT_TRUE(EkfImuTestPeer::posterior(ekf).isApprox(posterior, 1e-14));
	expectCovarianceValid(ekf);
}

TEST(EkfImuNavigation, AbsoluteTimestampMagnitudeDoesNotChangeIntegratedDuration) {
	for (const Scalar epoch : {1.0, 1792398261.0}) {
		EkfImu ekf(continuousNoiseParameters(1e-4));
		QuadState state = initial();
		state.t = epoch;
		ASSERT_TRUE(ekf.initialize(state));
		for (int sample = 0; sample <= 500; ++sample)
			ASSERT_TRUE(ekf.addImu({epoch + 0.002 * sample, -GVEC + Vector<3>(0.7, 0, 0), Vector<3>::Zero()}));
		EkfImu::NavigationQuality quality;
		ASSERT_TRUE(ekf.getAt(epoch + 1.0, &state, &quality));
		EXPECT_DOUBLE_EQ(state.t, quality.stamp);
		EXPECT_NEAR(state.v.x(), 0.7, 1e-10);
		EXPECT_NEAR(state.p.x(), 0.35, 1e-10);
		EXPECT_NEAR(quality.velocity_variance.x(), 0.04 + 0.003 / 3, 2e-8);
	}
}
