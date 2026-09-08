#pragma once

#include <filesystem>
#include <random>

#include "agilib/math/types.hpp"
#include "agilib/types/imu_sample.hpp"

namespace agi {

/// Attitude and heading reference system for the companion computer.
///
/// A Mahony-style complementary filter: the gyro integrates attitude, the
/// accelerometer pulls tilt back towards gravity, an RTK dual-antenna baseline
/// pulls heading back towards true north, and the integral term of both
/// corrections is fed back as a gyro bias estimate.
///
/// This exists because Betaflight's own AHRS is not usable as an outer-loop
/// state source.  `imuIsAccelerometerHealthy()` in its `flight/imu.c` gates the
/// accelerometer correction on a hard 0.9-1.1 g window, so on an aggressive
/// trajectory -- where specific force sits near 2 g for seconds at a time --
/// its attitude free-runs on the gyro and its tilt error was measured past
/// 140 deg.  The gate is the right idea (an accelerometer under acceleration
/// does not measure gravity) implemented in the way that fails worst: a cliff.
///
/// This filter fixes that in two steps, and the second matters far more than
/// the first:
///
///  1. The accelerometer weight degrades smoothly instead of switching off, so
///     a manoeuvre weakens the correction rather than removing it.
///
///  2. It does not assume the accelerometer measures gravity.  The vehicle
///     carries an RTK velocity measurement, so its own acceleration is
///     observable: feed the velocity in with setVelocity() and the filter
///     compares the reading against the specific force it *should* see,
///     `R^T (a_world + g z)`, rather than against gravity alone.
///
/// Acceleration compensation has finite bandwidth. During aggressive motion
/// the expected force can have the correct magnitude but a lagging direction.
/// A separate dynamic gate therefore reduces tilt correction and its bias
/// contribution until the motion subsides; gyro propagation and RTK heading
/// remain active throughout.
///
/// The velocity is differentiated rather than using the `omega x v_body`
/// centripetal term that fixed-wing AHRS implementations use, and the reason
/// is that a multirotor is not in coordinated flight.  `omega x v_body` is the
/// centripetal acceleration only when the velocity is body-fixed in direction;
/// a quadrotor translating around a circle at constant heading has a velocity
/// that rotates at 2 rad/s in the world while its body rate is near zero, so
/// that term is ~0 while the true acceleration is 20 m/s^2.  Measured, it was
/// worse than making no correction at all: 9.9 deg mean tilt error against
/// 6.7 deg.  Differentiating is the formulation that holds for a vehicle whose
/// thrust axis and velocity are decoupled.
class CompanionAhrs {
 public:
  struct Params {
    /// Proportional gain on the accelerometer tilt correction [1/s].
    Scalar acc_gain{1.0};
    /// Mismatch between the measured and predicted specific-force magnitude
    /// at which the accelerometer correction is weighted by one half [m/s^2].
    /// The weight is 1 / (1 + (mismatch / tolerance)^2) -- Betaflight's hard
    /// 0.9-1.1 g gate with a soft edge, and against the *predicted* magnitude
    /// rather than against g, so a compensated manoeuvre does not trip it.
    Scalar acc_tolerance{3.0};
    /// Motion acceleration at which tilt correction gets half weight [m/s^2].
    /// Compensated force magnitude alone cannot detect a lagging direction.
    /// Zero disables this gate for simulation ablation.
    Scalar acc_dynamic_tolerance{3.0};
    /// Proportional gain on the RTK heading correction [1/s].
    Scalar heading_gain{1.0};
    /// Integral gain that drives the gyro bias estimate [1/s].
    Scalar bias_gain{0.02};
    /// Clamp on each gyro bias component [rad/s].
    Scalar max_bias{0.1};
    /// Time constant of the low pass on the differentiated velocity [s].
    ///
    /// This is the one real cost of the acceleration compensation.  Too short
    /// and RTK velocity noise swamps the reference direction; too long and the
    /// reference lags a turning acceleration vector, which shows up directly
    /// as a tilt bias -- a 10 m/s turn on a 5 m radius swings it at 2 rad/s,
    /// so every 0.1 s of lag is about 11 deg of reference error.
    Scalar acc_tau{0.05};

    // ---- Heading sensor model (simulation only) ----
    /// Dual-antenna RTK heading noise [deg]; 0.4 suits a ~0.5 m baseline.
    Scalar heading_noise_deg{0.4};
    /// Heading fix rate [Hz]; matches the RTK position rate.
    Scalar heading_rate_hz{10.0};

    bool load(const std::filesystem::path& file);
    bool valid() const;
  };

  explicit CompanionAhrs(const Params& params);

  /// Integrate one inertial sample.  `imu.acc` is specific force in the FLU
  /// body frame, `imu.omega` the body rate, both as the sensor reports them.
  void addImu(const ImuSample& imu);

  /// Latest heading fix [rad], held between fixes as a real receiver does.
  void setHeading(Scalar heading);

  /// Latest world-frame velocity [m/s] and its timestamp, from the RTK-aided
  /// estimator.  The filter differentiates and low-passes it internally into
  /// the kinematic acceleration its accelerometer correction needs.
  ///
  /// Leave it unset and the filter falls back to assuming the accelerometer
  /// measures gravity -- the classic complementary-filter assumption, and the
  /// one that costs Betaflight its attitude above 1.1 g.
  void setVelocity(const Vector<3>& velocity_world, Scalar t);

  /// True once initialised from the first accelerometer sample.
  bool initialized() const { return initialized_; }

  /// Attitude in the FLU-ENU convention QuadState::q() expects.
  const Quaternion& attitude() const { return q_; }
  /// Estimated gyro bias [rad/s]; subtract it from the raw rate.
  const Vector<3>& gyroBias() const { return bias_; }
  /// Accelerometer weight applied on the last sample, in [0, 1].  Watching
  /// this is how you see the filter coast through an aggressive segment.
  Scalar lastAccWeight() const { return last_acc_weight_; }

 private:
  /// Level the filter off one accelerometer sample plus the latest heading.
  void initialize(const ImuSample& imu);

  const Params params_;

  Quaternion q_{1.0, 0.0, 0.0, 0.0};
  Vector<3> bias_{Vector<3>::Zero()};
  /// Zero means "assume unaccelerated"; see setVelocity().
  Vector<3> kinematic_acc_{Vector<3>::Zero()};
  Vector<3> previous_velocity_{Vector<3>::Zero()};
  Scalar previous_velocity_time_{NAN};
  Scalar heading_{NAN};
  Scalar t_last_{NAN};
  Scalar last_acc_weight_{0.0};
  bool initialized_{false};
};

/// Emulates a dual-antenna RTK heading: the true yaw, sampled at the fix rate
/// and corrupted by gaussian noise.
///
/// A single GNSS antenna cannot supply this and neither can the flight
/// controller -- SITL has no magnetometer, so Betaflight's heading is an
/// unbounded gyro integration.  The second antenna is what closes it, and it
/// is the only heading observation the AHRS above ever gets.
class RtkHeadingSensor {
 public:
  RtkHeadingSensor(Scalar noise_rad, Scalar rate_hz);

  /// Heading at `time`, held between fixes.
  Scalar get(Scalar time, Scalar truth_yaw);

 private:
  const Scalar noise_;
  const Scalar period_;
  std::mt19937 generator_;
  Scalar last_fix_{NAN};
  Scalar last_heading_{0.0};
};

}  // namespace agi