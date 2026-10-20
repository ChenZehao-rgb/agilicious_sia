#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>

#include "agilib/estimator/ekf_imu/ekf_imu_params.hpp"
#include "agilib/estimator/estimator_base.hpp"
#include "agilib/math/integrator_rk4.hpp"
#include "agilib/math/math.hpp"
#include "agilib/math/types.hpp"
#include "agilib/types/imu_sample.hpp"
#include "agilib/types/pose.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/logger.hpp"
#include "agilib/utils/timer.hpp"

namespace agi {

/// EKF filter for pose measurements.
class EkfImu : public EstimatorBase {
public:
	enum class NavigationMeasurementMode { kFull3d, kHorizontal };
	enum class NavigationUpdateReason { kNotAttempted, kAccepted, kInvalidInput, kTimestamp, kInnovation, kNumericalFailure };
	static constexpr uint8_t kPositionGroup = 1;
	static constexpr uint8_t kVelocityGroup = 8;
	static constexpr uint8_t kHeadingGroup = 16;
	static constexpr uint8_t kHorizontalGroup = kPositionGroup | kVelocityGroup;
	static constexpr uint8_t kHeightGroup = 2;
	static constexpr uint8_t kVerticalVelocityGroup = 4;

	struct NavigationGroupQuality {
		Scalar stamp{NAN};  // Last accepted observation.
		Scalar observation_stamp{NAN};
		Scalar innovation_squared{NAN};
		Scalar innovation_limit{NAN};
		// Horizontal aggregate order: px, py, vx, vy (four DOF). Scalar groups use element zero.
		Vector<5> measurement_variance = Vector<5>::Constant(NAN);
		uint64_t accepted_updates{0};
		uint64_t rejected_updates{0};
		uint32_t observation_dimensions{0};
		bool accepted{false};
		NavigationUpdateReason reason{NavigationUpdateReason::kNotAttempted};
	};
	struct NavigationUpdateResult {
		bool committed{false};
		bool numerical_failure{false};
		uint8_t accepted_groups{0};
		uint8_t rejected_groups{0};
		NavigationUpdateReason reason{NavigationUpdateReason::kNotAttempted};
	};

	struct NavigationQuality {
		Scalar stamp{NAN};
		Vector<3> position_variance = Vector<3>::Constant(NAN);
		Vector<3> velocity_variance = Vector<3>::Constant(NAN);
		Scalar heading_variance{NAN};
		Scalar innovation_squared{NAN};
		uint64_t accepted_updates{0};
		uint64_t rejected_updates{0};
		uint32_t observation_dimensions{0};
		bool valid{false};
		NavigationGroupQuality horizontal;
		NavigationGroupQuality position;
		NavigationGroupQuality velocity;
		NavigationGroupQuality heading;
		NavigationGroupQuality height;
		NavigationGroupQuality vertical_velocity;
	};
	// Observation order: px, py, pz, vx, vy, vz, yaw. State order: p, quaternion(wxyz), v, bw, ba, baro bias.
	// residual = h(x_prior) - z, with wrapped yaw. Group order: position XY, velocity XY, heading, height, vertical velocity.
	struct NavigationDiagnostics {
		Scalar stamp{NAN};
		bool valid{false};
		Vector<17> prior_state = Vector<17>::Constant(NAN);
		Vector<17> posterior_state = Vector<17>::Constant(NAN);
		Matrix<17, 17> prior_covariance = Matrix<17, 17>::Constant(NAN);
		Matrix<17, 17> posterior_covariance = Matrix<17, 17>::Constant(NAN);
		Vector<7> measurement = Vector<7>::Constant(NAN);
		Vector<7> prediction = Vector<7>::Constant(NAN);
		Vector<7> residual = Vector<7>::Constant(NAN);
		Matrix<7, 17> observation = Matrix<7, 17>::Constant(NAN);
		Matrix<7, 7> measurement_covariance = Matrix<7, 7>::Constant(NAN);
		Matrix<7, 7> innovation_covariance = Matrix<7, 7>::Constant(NAN);
		// Each column is L^-1 residual for a group embedded in seven rows; inactive rows are zero.
		Matrix<7, 5> whitened_innovation = Matrix<7, 5>::Constant(NAN);
		// Column i is -K.col(i) * residual(i), before quaternion normalization.
		Matrix<17, 7> correction_contributions = Matrix<17, 7>::Zero();
		std::array<NavigationGroupQuality, 5> groups;
		NavigationUpdateResult result;
	};
	struct BarometerQuality {
		// Last accepted barometer time; bias and variance describe the current posterior.
		Scalar stamp{NAN};
		Scalar innovation_squared{NAN};
		Scalar innovation{NAN};
		Scalar bias{NAN};
		Scalar bias_variance{NAN};
		// Variance of z + bias, not the absolute navigation height variance.
		Scalar relative_height_variance{NAN};
		uint64_t accepted_updates{0};
		uint64_t rejected_updates{0};
		bool valid{false};
		bool reference_valid{false};
		bool relative_reference_active{false};
	};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EkfImu(const std::shared_ptr<EkfImuParameters>& params =
           std::shared_ptr<EkfImuParameters>());
  ~EkfImu();

	bool getAt(const Scalar t, QuadState* const state) override;
	// State and covariance share one predicted epoch; querying never advances the delayed-update posterior.
	bool getAt(Scalar time, QuadState* state, NavigationQuality* quality);
	void setNavigationDiagnosticsEnabled(bool enabled);
	NavigationDiagnostics navigationDiagnostics();
	// Same diagnostic envelope; pressure uses observation row 2 and group 3 (height).
	NavigationDiagnostics barometerDiagnostics();

  bool initialize(const QuadState& state) override;

  bool addPose(const Pose& pose) override;
  bool addState(const QuadState& pose) override;
  bool addImu(const ImuSample& imu) override;
  // ENU position/velocity and yaw CCW from East. Variances are in SI units.
  // Requires initialized state and IMU coverage through t; false means rejected.
	// Full 3D NIS has seven observations with heading, six without; horizontal mode has five or four.
	// Horizontal mode omits both vertical position and velocity. The caller must choose a matching NIS limit.
	bool addRtk(Scalar t, const Vector<3>& position, const Vector<3>& velocity, Scalar heading, bool heading_valid,
	            const Vector<3>& position_variance, const Vector<3>& velocity_variance, Scalar heading_variance,
	            Scalar max_innovation_squared = std::numeric_limits<Scalar>::infinity(),
	            NavigationMeasurementMode mode = NavigationMeasurementMode::kFull3d);
	// Gate position XY, velocity XY, heading, height and vertical velocity independently from one prior.
	// horizontal_nis_limit is retained for aggregate diagnostics only; it does not veto the independent groups.
	// Aggregate navigation acceptance requires position and velocity, while heading readiness is a caller policy.
	NavigationUpdateResult addNavigation(Scalar time, const Vector<3>& position, const Vector<3>& velocity, Scalar heading,
	                                     bool heading_valid, const Vector<3>& position_variance, const Vector<3>& velocity_variance,
	                                     Scalar heading_variance, Scalar horizontal_nis_limit, Scalar height_nis_limit,
	                                     Scalar vertical_velocity_nis_limit, Scalar position_nis_limit = 13.815510557964274,
	                                     Scalar velocity_nis_limit = 13.815510557964274, Scalar heading_nis_limit = 10.827566170662733);

	// Posterior covariance and innovation at stamp; not a claim of convergence.
	NavigationQuality navigationQuality();
	// ENU height in metres, independent white measurement variance in m^2 and a one-dimensional NIS limit.
	// Model: height = position.z + barometer bias + noise. GNSS may update at the same timestamp.
	bool addBaro(Scalar time, Scalar height, Scalar variance, Scalar nis_limit);
	// Reset an independently known bias; this does not align a pressure reference derived from this EKF's height.
	bool resetBaroBias(Scalar variance);
	// Anchor a relative pressure reference to the estimated height while preserving their correlation.
	// independent_variance is pressure-reference/model uncertainty only, in m^2; height receives the anchor.
	bool alignBarometerReference(Scalar time, Scalar independent_variance, Scalar* height);
	// Invalidate pressure observations without changing the state or releasing an already frozen reference.
	void clearBarometerReference();
	BarometerQuality barometerQuality();

  bool addMotorSpeeds(const Vector<4>& speeds) override;

  bool updateParameters(const std::shared_ptr<EkfImuParameters>& params_in);

  bool healthy() const override;

  inline void printTiming() { printTimings(false); }
  void printTimings(const bool all = true) const;
  void logTiming() const override;

private:
  enum IDX : int {
    POS = 0,
    POSX = 0,
    POSY = 1,
    POSZ = 2,
    NPOS = 3,
    ATT = 3,
    ATTW = 3,
    ATTX = 4,
    ATTY = 5,
    ATTZ = 6,
    NATT = 4,
    VEL = 7,
    VELX = 7,
    VELY = 8,
    VELZ = 9,
    NVEL = 3,
    BOME = 10,
    BOMEX = 10,
    BOMEY = 11,
    BOMEZ = 12,
    NBOME = 3,
    BACC = 13,
    BACCX = 13,
    BACCY = 14,
    BACCZ = 15,
    NBACC = 3,
		BARO_BIAS = 16,
		SIZE = 17,
  };

  using StateVector = Vector<IDX::SIZE>;
  using StateMatrix = Matrix<IDX::SIZE, IDX::SIZE>;
  static constexpr int MAX_QUEUE_SIZE = 4096;
  static constexpr int SRPOSE = 6;  // Size of pose measurement.
  static constexpr int SRIMU = 6;   // Size of IMU measurement.


	bool init(const QuadState& state);
	bool process();
	bool updatePose(const Pose& pose);
	bool propagatePrior(const Scalar time);
	bool propagatePriorAndCovariance(const Scalar time);
	NavigationQuality qualityAt(Scalar time, const StateVector& state, const StateMatrix& covariance) const;
	bool normalizeCorrection(StateVector* state, StateMatrix* covariance) const;
	template <int Columns>
	void constrainReferenceGain(Matrix<IDX::SIZE, Columns>* gain) const {
		if (!_relative_reference_active) return;
		// Schmidt update in coordinates (z + bias, bias), mapped back to (z, bias).
		gain->row(POSZ) += gain->row(BARO_BIAS);
		gain->row(BARO_BIAS).setZero();
	}
	bool vectorToState(const Scalar t, const StateVector& x, QuadState* const state) const;
	bool stateToVector(const QuadState& state, Scalar* const t, StateVector* const x) const;

  // Parameters
  std::shared_ptr<EkfImuParameters> params_;
  StateMatrix Q_ = StateMatrix::Zero();
  StateMatrix Q_init_ = StateMatrix::Zero();
  Matrix<SRPOSE, SRPOSE> R_pose_;
  Matrix<SRIMU, SRIMU> R_imu_;
  ImuSample imu_last_;

  // Working variables
  Scalar t_prior_{NAN}, t_posterior_{NAN};
  StateVector prior_ = StateVector::Zero();
  StateVector posterior_ = StateVector::Zero();
  StateMatrix P_ = StateMatrix::Zero();
	// Prediction state and covariance are independent of the delayed posterior workspace.
	Scalar _prediction_time{NAN};
	StateVector _prediction = StateVector::Zero();
	StateMatrix _prediction_covariance = StateMatrix::Zero();
	bool _prediction_has_covariance{false};
	std::unique_ptr<NavigationDiagnostics> _navigation_diagnostics;
	std::unique_ptr<NavigationDiagnostics> _barometer_diagnostics;
	NavigationQuality _navigation_quality;
	BarometerQuality _barometer_quality;
	Scalar _last_rtk_time{NAN};
	Scalar _last_navigation_attempt_time{NAN};
	bool _barometer_reference_valid{false};
	bool _relative_reference_active{false};
	friend class EkfImuTestPeer;
  Vector<4> motor_speeds_{0, 0, 0, 0};

  // Thread Safety
  std::mutex mutex_;

  std::deque<Pose> poses_;
  std::deque<ImuSample> imus_;

  // Logger and timers
  Timer timer_process_{"Process"};
  Timer timer_update_pose_{"Update Pose"};
  Timer timer_compute_gain_{"Kalman gain computation"};
  Timer timer_propagation_{"Propagation Prior"};
  Timer timer_jacobian_{"Propagation Jacobian"};
};

}  // namespace agi
