#include "agilib/controller/mpc/mpc_params.hpp"

#include <cmath>

namespace agi {

MpcParameters::MpcParameters()
        : timing_(true),
          Q_pos_(200, 200, 500),
          Q_att_(5, 5, 200),
          Q_vel_(Vector<3>::Constant(1)),
          R_((Vector<4>() << 0.1, 50.0, 50.0, 50.0).finished()),
          exp_decay_(1.0) {}

bool MpcParameters::load(const Yaml& node) {
	if (!node.isNode()) return false;
	if (node["R"].isDefined()) {
		throw ParameterException(
		        "MPC input units changed: replace rotor-force R with R_collective_thrust (m/s^2) and R_body_rates (rad/s)");
	}
	for (const auto* name :
	     {"Q_omega_xy", "Q_omega_z", "cog_abs_velocity_limit", "cog_abs_omega_limit", "cog_rel_hover_thrust_limit", "cog_height_limit",
	      "Q_cog_omega", "Q_cog_legnths", "Q_cog_lengths", "Q_cog_omega_init", "Q_cog_legnths_init", "Q_cog_lengths_init", "R_cog",
	      "max_wait_time_before_preparation", "max_wait_time_after_preparation"}) {
		if (node[name].isDefined()) {
			throw ParameterException(std::string("Obsolete rotor-force MPC parameter: ") + name);
		}
	}
	for (const auto* name : {"threaded_preparation", "cog_enable"}) {
		if (node[name].isDefined() && node[name].as<bool>()) {
			throw ParameterException(std::string("Rate/thrust MPC does not support ") + name);
		}
	}
	node["timing"].getIfDefined(timing_);
	Q_pos_(0) = node["Q_pos_x"].as<Scalar>();
	Q_pos_(1) = node["Q_pos_y"].as<Scalar>();
	Q_pos_(2) = node["Q_pos_z"].as<Scalar>();
	Q_att_(0) = node["Q_att_x"].as<Scalar>();
	Q_att_(1) = node["Q_att_y"].as<Scalar>();
	Q_att_(2) = node["Q_att_z"].as<Scalar>();
	node["Q_vel"] >> Q_vel_;
	R_(0) = node["R_collective_thrust"].as<Scalar>();
	Vector<3> body_rate_weights;
	if (node["R_body_rates"].size() != 3 || !node["R_body_rates"].getIfDefined(body_rate_weights)) {
		throw ParameterException("MPC R_body_rates must contain three weights for roll, pitch and yaw rates");
	}
	R_.tail<3>() = body_rate_weights;
	exp_decay_ = node["exp_decay"].as<Scalar>();
	return valid();
}

bool MpcParameters::valid() const {
	return Q_pos_.allFinite() && (Q_pos_.array() >= 0.0).all() && Q_att_.allFinite() && (Q_att_.array() >= 0.0).all() &&
	       Q_vel_.allFinite() && (Q_vel_.array() >= 0.0).all() && R_.allFinite() && (R_.array() >= 0.0).all() &&
	       std::isfinite(exp_decay_) && exp_decay_ > 0.0;
}

std::ostream& operator<<(std::ostream& os, const MpcParameters& params) {
	return os << "Rate/thrust MPC Parameters\n"
	          << "timing: " << params.timing_ << '\n'
	          << "Q_pos: " << params.Q_pos_.transpose() << '\n'
	          << "Q_att: " << params.Q_att_.transpose() << '\n'
	          << "Q_vel: " << params.Q_vel_.transpose() << '\n'
	          << "R_collective_thrust: " << params.R_(0) << '\n'
	          << "R_body_rates: " << params.R_.tail<3>().transpose() << '\n'
	          << "exp_decay: " << params.exp_decay_ << '\n';
}

}  // namespace agi
