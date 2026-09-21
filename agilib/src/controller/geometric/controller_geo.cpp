#include "agilib/controller/geometric/controller_geo.hpp"

#include <cmath>

#include "agilib/math/math.hpp"

namespace agi {
namespace {

constexpr Scalar kDirectionEpsilon = 1e-6;
constexpr Scalar kQuaternionTolerance = 1e-3;

bool validQuaternion(const Quaternion& quaternion) {
	return quaternion.coeffs().allFinite() && std::abs(quaternion.squaredNorm() - 1.0) < kQuaternionTolerance;
}

}  // namespace

GeometricController::GeometricController(const Quadrotor& quad, const std::shared_ptr<GeometricControllerParams>& params,
                                         const Scalar exec_dt)
        : ControllerBase("GEO", exec_dt, 1), _quad(quad), _params(params), _has_full_model(quad.valid()) {
	pred_dt_ = exec_dt_;
	if (_params && _params->valid() && (_has_full_model || _params->drag_compensation_)) {
		_filter_mot =
		        std::make_unique<LowPassFilter<4>>(_params->filter_cutoff_frequency_, _params->filter_sampling_frequency_, 0.0);
		if (_params->drag_compensation_) {
			_filter_acc = std::make_unique<LowPassFilter<3>>(_params->filter_cutoff_frequency_,
			                                                 _params->filter_sampling_frequency_, 0.0);
		}
	}
}

GeometricController::~GeometricController() = default;

bool GeometricController::getCommand(const QuadState& state, const SetpointVector& references, SetpointVector* const setpoints) {
	if (setpoints == nullptr) return false;
	setpoints->clear();
	if (!_params || !_params->valid() || !_quad.validRatesThrust() || !std::isfinite(pred_dt_) || pred_dt_ <= 0.0 || !state.valid() ||
	    !validQuaternion(state.q()) || references.empty()) {
		return false;
	}
	const Setpoint& reference = references.front();
	if (!reference.state.valid() || !validQuaternion(reference.state.q()) || !reference.input.valid()) return false;
	const Scalar yaw = reference.state.getYaw();
	if (!std::isfinite(yaw)) return false;
	const Quaternion attitude = state.q().normalized();

	const Vector<3> pos_error = clip(reference.state.p - state.p, _params->p_err_max_);
	const Vector<3> vel_error = clip(reference.state.v - state.v, _params->v_err_max_);
	Vector<3> acc_cmd = _params->kp_acc_.cwiseProduct(pos_error) + _params->kd_acc_.cwiseProduct(vel_error) + reference.state.a - GVEC;

	Scalar measured_thrust = 0.0;
	if (_filter_mot) {
		if (!_quad.thrust_map_.allFinite()) return false;
		const Vector<4> motor_speeds = _filter_mot->add(state.mot);
		measured_thrust = _quad.thrust_map_(0) * motor_speeds.squaredNorm();
		if (!std::isfinite(measured_thrust)) return false;
	}
	if (_params->drag_compensation_) {
		// This observer needs measured motor speeds, not default zero values from a rates-only state source.
		if (!_filter_acc || !_filter_mot || !(state.mot.array() > 0.0).all() || !_quad.thrust_map_.allFinite() ||
		    _quad.thrust_map_(0) <= 0.0) {
			return false;
		}
		ImuSample imu;
		{
			std::lock_guard<std::mutex> lock(_imu_mutex);
			imu = _imu;
		}
		const Vector<3> acc_body = imu.valid() ? imu.acc + state.ba : attitude.inverse() * (state.a - GVEC);
		const Vector<3> measured_acc = _filter_acc->add(acc_body);
		if (!measured_acc.allFinite()) return false;
		if (state.v.norm() > 3.0) {
			acc_cmd += attitude * (measured_thrust * Vector<3>::UnitZ() / _quad.m_ - measured_acc);
		}
	}

	// The hardware hover controller cannot request inverted thrust or a direction with zero upward component.
	if (!acc_cmd.allFinite() || acc_cmd.z() <= kDirectionEpsilon) return false;
	const Scalar horizontal_acc = acc_cmd.head<2>().norm();
	const Scalar horizontal_limit = acc_cmd.z() * std::tan(_params->max_tilt_rad_);
	if (!std::isfinite(horizontal_acc) || !std::isfinite(horizontal_limit)) return false;
	if (horizontal_acc > horizontal_limit) acc_cmd.head<2>() *= horizontal_limit / horizontal_acc;
	const Scalar collective_thrust = acc_cmd.norm();
	if (!std::isfinite(collective_thrust) || collective_thrust <= kDirectionEpsilon) return false;

	const Quaternion heading(Eigen::AngleAxis<Scalar>(yaw, Vector<3>::UnitZ()));
	const Vector<3> desired_z = acc_cmd / collective_thrust;
	const Vector<3> desired_x = ((heading * Vector<3>::UnitY()).cross(desired_z)).normalized();
	const Vector<3> desired_y = desired_z.cross(desired_x);
	Matrix<3, 3> rotation;
	rotation.col(0) = desired_x;
	rotation.col(1) = desired_y;
	rotation.col(2) = desired_z;
	const Quaternion desired_attitude(rotation);
	if (!validQuaternion(desired_attitude)) return false;
	const Vector<3> bodyrates = tiltPrioritizedControl(attitude, desired_attitude);
	if (!bodyrates.allFinite()) return false;

	Command command;
	command.t = state.t;
	command.omega = _quad.clampBodyrates(bodyrates);
	command.collective_thrust = _quad.clampCollectiveThrust(collective_thrust);
	if (!command.isRatesThrust()) return false;
	QuadState state_cmd = state;
	// The ROS rates/thrust model has no motor dynamics and emits no angular-acceleration command.
	state_cmd.tau.setZero();
	if (_has_full_model) {
		// Preserve the historical angular-acceleration reference consumed by a downstream INDI controller.
		const Vector<3> body_x = attitude * Vector<3>::UnitX();
		const Vector<3> body_y = attitude * Vector<3>::UnitY();
		const Vector<3> body_z = attitude * Vector<3>::UnitZ();
		Vector<3> jerk_rates = _quad.m_ * (reference.state.j - body_z.dot(reference.state.j) * body_z);
		if (measured_thrust >= 0.01) jerk_rates /= measured_thrust;
		const Vector<3> reference_rates(-jerk_rates.dot(body_y), jerk_rates.dot(body_x), reference.state.w.z());
		state_cmd.tau = bodyrates + _params->kp_rate_.cwiseProduct(reference_rates - state.w);
		if (!state_cmd.tau.allFinite()) return false;
	}
	setpoints->push_back({state_cmd, command});
	return true;
}

Vector<3> GeometricController::tiltPrioritizedControl(const Quaternion& q, const Quaternion& q_des) {
	const Quaternion error = q.inverse() * q_des;
	const Scalar denominator = std::hypot(error.w(), error.z());
	// The tilt-prioritized expression is undefined at a 180-degree tilt error; never emit an arbitrary finite command.
	if (denominator <= kDirectionEpsilon) return Vector<3>::Constant(NAN);
	Vector<3> rates(error.w() * error.x() - error.y() * error.z(), error.w() * error.y() + error.x() * error.z(), error.z());
	if (error.w() <= 0.0) rates.z() *= -1.0;
	rates = (2.0 / denominator) * Vector<3>(_params->kp_att_xy_, _params->kp_att_xy_, _params->kp_att_z_).cwiseProduct(rates);
	return rates;
}

void GeometricController::addImuSample(const ImuSample& imu) { addImu(imu); }

bool GeometricController::addImu(const ImuSample& imu) {
	if (!imu.valid()) return false;
	std::lock_guard<std::mutex> lock(_imu_mutex);
	_imu = imu;
	return true;
}

}  // namespace agi
