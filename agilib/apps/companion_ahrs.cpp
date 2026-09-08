#include "companion_ahrs.hpp"

#include <cmath>

#include "agilib/math/gravity.hpp"
#include "agilib/utils/yaml.hpp"

namespace agi {

bool CompanionAhrs::Params::load(const std::filesystem::path& file) {
  const Yaml node(file);
  if (node.isNull()) return false;

  node["acc_gain"].getIfDefined(acc_gain);
  node["acc_tolerance"].getIfDefined(acc_tolerance);
  node["acc_dynamic_tolerance"].getIfDefined(acc_dynamic_tolerance);
  node["heading_gain"].getIfDefined(heading_gain);
  node["bias_gain"].getIfDefined(bias_gain);
  node["max_bias"].getIfDefined(max_bias);
  node["acc_tau"].getIfDefined(acc_tau);
  node["heading_noise"].getIfDefined(heading_noise_deg);
  node["heading_rate"].getIfDefined(heading_rate_hz);

  return valid();
}

bool CompanionAhrs::Params::valid() const {
  return acc_gain >= 0.0 && acc_tolerance > 0.0 &&
         std::isfinite(acc_dynamic_tolerance) && acc_dynamic_tolerance >= 0.0 &&
         heading_gain >= 0.0 &&
         bias_gain >= 0.0 && max_bias >= 0.0 && acc_tau >= 0.0 &&
         heading_noise_deg >= 0.0 &&
         heading_rate_hz > 0.0;
}

CompanionAhrs::CompanionAhrs(const Params& params) : params_(params) {}

void CompanionAhrs::setHeading(const Scalar heading) {
  if (std::isfinite(heading)) heading_ = heading;
}

void CompanionAhrs::setVelocity(const Vector<3>& velocity_world,
                                const Scalar t) {
  if (!velocity_world.allFinite() || !std::isfinite(t)) return;

  const Scalar dt = t - previous_velocity_time_;
  // Do not move the differentiation baseline backwards, or discard a
  // sub-millisecond increment that should belong to the next difference.
  if (std::isfinite(previous_velocity_time_) && dt <= 1e-3) return;
  if (std::isfinite(previous_velocity_time_) && dt > 1e-3) {
    const Vector<3> raw = (velocity_world - previous_velocity_) / dt;
    // First-order low pass; see Params::acc_tau for the trade this makes.
    kinematic_acc_ += (dt / (params_.acc_tau + dt)) * (raw - kinematic_acc_);
  }
  previous_velocity_ = velocity_world;
  previous_velocity_time_ = t;
}

void CompanionAhrs::initialize(const ImuSample& imu) {
  // Level off one accelerometer sample.  The vehicle is sitting still during
  // the disarmed hold, so specific force is gravity and nothing else.
  const Vector<3> acc = imu.acc;
  if (acc.norm() < 1e-3) return;

  const Scalar roll = std::atan2(acc.y(), acc.z());
  const Scalar pitch =
    std::atan2(-acc.x(), std::sqrt(acc.y() * acc.y() + acc.z() * acc.z()));
  // Heading has no inertial observability at all, so it starts at whatever the
  // RTK baseline last reported and stays wrong until the first fix arrives.
  const Scalar yaw = std::isfinite(heading_) ? heading_ : 0.0;

  q_ = Quaternion(Eigen::AngleAxis<Scalar>(yaw, Vector<3>::UnitZ()) *
                  Eigen::AngleAxis<Scalar>(pitch, Vector<3>::UnitY()) *
                  Eigen::AngleAxis<Scalar>(roll, Vector<3>::UnitX()));
  q_.normalize();
  bias_.setZero();
  t_last_ = imu.t;
  initialized_ = true;
}

void CompanionAhrs::addImu(const ImuSample& imu) {
  if (!imu.valid()) return;

  if (!initialized_) {
    initialize(imu);
    return;
  }

  const Scalar dt = imu.t - t_last_;
  // Out of order or absurdly stale: propagating over it would be worse than
  // skipping it.  0.1 s is one RTK period, far beyond any 1 kHz sample gap.
  if (!(dt > 0.0)) return;
  t_last_ = imu.t;
  if (dt > 0.1) return;

  // Correction expressed as a body-frame rotation rate, accumulated from every
  // available observation before it is applied.
  Vector<3> correction = Vector<3>::Zero();

  // ---- Accelerometer: tilt ----
  //
  // The specific force a real accelerometer reports is the kinematic
  // acceleration minus gravity, in the body frame.  With `kinematic_acc_`
  // known from the RTK-aided velocity, the direction it *should* report is
  // computable at any acceleration; left at zero this degenerates to "the
  // accelerometer measures gravity", which is only true in steady flight.
  const Vector<3> expected_world = kinematic_acc_ + Vector<3>(0.0, 0.0, G);
  const Scalar acc_norm = imu.acc.norm();
  const Scalar expected_norm = expected_world.norm();
  if (acc_norm > 1e-3 && expected_norm > 1e-3) {
    // Weight by how far the measured magnitude is from the predicted one.
    // Compensated, this stays near one through a manoeuvre; uncompensated it
    // is Betaflight's |a| against g, with a soft edge instead of a cliff.
    const Scalar mismatch =
      std::abs(acc_norm - expected_norm) / params_.acc_tolerance;
    last_acc_weight_ = 1.0 / (1.0 + mismatch * mismatch);
    // The velocity derivative is filtered and contains RTK reset increments.
    // In a fast turn its magnitude may be right while its direction is late.
    // Trust the gyro during those manoeuvres, and restore tilt correction as
    // motion subsides. The measured-force term also catches manoeuvre onset
    // before the filtered kinematic acceleration has caught up.
    if (params_.acc_dynamic_tolerance > 0.0) {
      const Scalar motion = std::max(kinematic_acc_.norm(),
                                     std::abs(acc_norm - G));
      const Scalar ratio = motion / params_.acc_dynamic_tolerance;
      last_acc_weight_ /= 1.0 + ratio * ratio;
    }

    // Measured direction against where the estimate puts it.  The cross
    // product of the two unit vectors is the rotation error, to first order.
    const Vector<3> measured = imu.acc / acc_norm;
    const Vector<3> estimated = q_.inverse() * (expected_world / expected_norm);
    correction +=
      params_.acc_gain * last_acc_weight_ * measured.cross(estimated);
  } else {
    last_acc_weight_ = 0.0;
  }

  // ---- RTK baseline: heading ----
  if (std::isfinite(heading_)) {
    const Matrix<3, 3> rotation = q_.toRotationMatrix();
    const Scalar estimated_heading =
      std::atan2(rotation(1, 0), rotation(0, 0));
    const Scalar error = std::remainder(heading_ - estimated_heading, 2.0 * M_PI);
    // Rotate about the world z axis, expressed in the body frame, so the
    // correction moves heading without disturbing the tilt the accelerometer
    // just fixed.
    correction +=
      params_.heading_gain * error * (q_.inverse() * Vector<3>::UnitZ());
  }

  // ---- Gyro bias ----
  // The integral half of the complementary filter.  A constant tilt or heading
  // error can only come from a rate offset, so integrating the same error that
  // drives the proportional term recovers it.
  if (params_.bias_gain > 0.0) {
    bias_ -= params_.bias_gain * dt * correction;
    bias_ = bias_.cwiseMax(-params_.max_bias).cwiseMin(params_.max_bias);
  }

  // ---- Propagate ----
  const Vector<3> rate = imu.omega - bias_ + correction;
  const Vector<3> delta = 0.5 * dt * rate;
  // Small-angle quaternion increment; at 1 kHz `delta` stays far below the
  // range where the exact exponential map differs measurably.
  q_ = q_ * Quaternion(1.0, delta.x(), delta.y(), delta.z());
  q_.normalize();
}

RtkHeadingSensor::RtkHeadingSensor(const Scalar noise_rad, const Scalar rate_hz)
  : noise_(noise_rad), period_(1.0 / rate_hz), generator_(0x5eed) {}

Scalar RtkHeadingSensor::get(const Scalar time, const Scalar truth_yaw) {
  if (!std::isfinite(last_fix_) || time - last_fix_ >= period_) {
    last_fix_ = time;
    last_heading_ =
      truth_yaw + (noise_ > 0.0
                     ? std::normal_distribution<Scalar>(0.0, noise_)(generator_)
                     : 0.0);
  }
  return last_heading_;
}

}  // namespace agi