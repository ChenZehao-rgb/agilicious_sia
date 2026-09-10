#include <gtest/gtest.h>
#include "agilib/estimator/ekf_imu/ekf_imu.hpp"
#include "agilib/math/gravity.hpp"

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
