#include <gtest/gtest.h>

#include "companion_ahrs.hpp"
#include "agilib/math/gravity.hpp"

using namespace agi;

namespace {
ImuSample sample(Scalar t, const Vector<3>& acc,
                 const Vector<3>& omega = Vector<3>::Zero()) {
  ImuSample imu;
  imu.t = t;
  imu.acc = acc;
  imu.omega = omega;
  return imu;
}
Scalar attitudeError(const Quaternion& a, const Quaternion& b) {
  return 2.0 * std::acos(std::min(1.0, std::abs(a.coeffs().dot(b.coeffs()))));
}
}  // namespace

TEST(CompanionAhrs, RejectsDuplicateAndOutOfOrderSamples) {
  CompanionAhrs::Params params;
  params.acc_gain = 0.0;
  params.heading_gain = 0.0;
  params.bias_gain = 0.0;
  CompanionAhrs clean(params), reordered(params);
  const auto init = sample(0.0, Vector<3>(0, 0, G));
  clean.addImu(init);
  reordered.addImu(init);
  for (int i = 1; i <= 100; ++i) {
    const auto imu = sample(i * .001, Vector<3>(0, 0, G), Vector<3>(1, 0, 0));
    clean.addImu(imu);
    reordered.addImu(imu);
    reordered.addImu(imu);
    reordered.addImu(sample(imu.t - .0005, imu.acc, imu.omega));
  }
  EXPECT_LT(attitudeError(clean.attitude(), reordered.attitude()), 1e-7);
}

TEST(CompanionAhrs, DynamicGateRejectsLaggingAccelerationDirection) {
  CompanionAhrs::Params gated_params;
  CompanionAhrs::Params legacy_params = gated_params;
  legacy_params.acc_dynamic_tolerance = 0.0;
  CompanionAhrs gated(gated_params), legacy(legacy_params);
  for (auto* filter : {&gated, &legacy}) {
    filter->setHeading(0.0);
    filter->setVelocity(Vector<3>(0, 0, -10), 0.0);
    filter->addImu(sample(0.0, Vector<3>(0, 0, G)));
  }
  Scalar gated_max = 0, legacy_max = 0;
  for (int i = 1; i <= 3000; ++i) {
    const Scalar t = i * .001;
    const Quaternion truth(Eigen::AngleAxis<Scalar>(3*t, Vector<3>::UnitX()));
    if (i % 10 == 0) {
      const Scalar tv = t - .01;
      const Vector<3> velocity(0, 10*std::sin(3*tv), -10*std::cos(3*tv));
      gated.setVelocity(velocity, tv);
      legacy.setVelocity(velocity, tv);
    }
    const Vector<3> acceleration(0, 30*std::cos(3*t), 30*std::sin(3*t));
    const auto imu = sample(t, truth.inverse()*(acceleration+Vector<3>(0,0,G)),
                           Vector<3>(3,0,0));
    gated.addImu(imu);
    legacy.addImu(imu);
    gated_max = std::max(gated_max, attitudeError(gated.attitude(), truth));
    legacy_max = std::max(legacy_max, attitudeError(legacy.attitude(), truth));
  }
  EXPECT_LT(gated_max, 2.0*M_PI/180.0);
  EXPECT_LT(gated_max, legacy_max*.2);
  EXPECT_LT(gated.lastAccWeight(), .02);
}

TEST(CompanionAhrs, RetainsStationaryTiltCorrection) {
  CompanionAhrs::Params params;
  CompanionAhrs filter(params);
  filter.setHeading(0.0);
  // Start with a 5 degree leveling error, then supply correct stationary IMU.
  const Quaternion tilt(Eigen::AngleAxis<Scalar>(5*M_PI/180, Vector<3>::UnitX()));
  filter.addImu(sample(0, tilt.inverse()*Vector<3>(0,0,G)));
  for (int i=1; i<=5000; ++i) {
    if (i%10==0) filter.setVelocity(Vector<3>::Zero(), i*.001);
    filter.addImu(sample(i*.001, Vector<3>(0,0,G)));
  }
  EXPECT_LT(attitudeError(filter.attitude(), Quaternion::Identity()), M_PI/180);
  EXPECT_GT(filter.lastAccWeight(), .99);
}

TEST(CompanionAhrs, DuplicateVelocitiesDoNotChangeDifferentiationBaseline) {
  CompanionAhrs::Params params;
  CompanionAhrs clean(params), duplicates(params);
  for (auto* filter : {&clean, &duplicates}) {
    filter->addImu(sample(0, Vector<3>(0,0,G)));
    filter->setVelocity(Vector<3>::Zero(), 0);
    filter->setVelocity(Vector<3>(1,0,0), .01);
  }
  duplicates.setVelocity(Vector<3>(100,0,0), .01);
  duplicates.setVelocity(Vector<3>(100,0,0), .009);
  for (auto* filter : {&clean, &duplicates}) {
    filter->setVelocity(Vector<3>(2,0,0), .02);
    filter->addImu(sample(.02, Vector<3>(10,0,G)));
  }
  EXPECT_NEAR(clean.lastAccWeight(), duplicates.lastAccWeight(), 1e-12);
  EXPECT_LT(attitudeError(clean.attitude(), duplicates.attitude()), 1e-7);
}
