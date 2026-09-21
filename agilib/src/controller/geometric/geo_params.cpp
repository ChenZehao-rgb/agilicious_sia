#include "agilib/controller/geometric/geo_params.hpp"

#include <cmath>

namespace agi {

GeometricControllerParams::GeometricControllerParams()
        : kp_acc_(18.0, 18.0, 18.0),
          kd_acc_(8.0, 8.0, 8.0),
          kp_rate_(20.0, 20.0, 2.0),
          kp_att_xy_(150.0),
          kp_att_z_(2.0),
          filter_sampling_frequency_(100),
          filter_cutoff_frequency_(10),
          drag_compensation_(false) {}

bool GeometricControllerParams::load(const Yaml& node) {
	node["kpacc"] >> kp_acc_;
	node["kdacc"] >> kd_acc_;
	// The full-model GEO->INDI path uses kprate; the rates/thrust command sent to Betaflight does not.
	node["kprate"] >> kp_rate_;
	kp_att_z_ = node["kpatt_z"].as<Scalar>();
	kp_att_xy_ = node["kpatt_xy"].as<Scalar>();
	filter_sampling_frequency_ = node["filter_sampling_frequency"].as<Scalar>();
	filter_cutoff_frequency_ = node["filter_cutoff_frequency"].as<Scalar>();
	drag_compensation_ = node["drag_compensation"].as<bool>();
	node["p_err_max"].getIfDefined(p_err_max_);
	node["v_err_max"].getIfDefined(v_err_max_);
	node["max_tilt_rad"].getIfDefined(max_tilt_rad_);
	return valid();
}

bool GeometricControllerParams::valid() const {
	return kp_acc_.allFinite() && (kp_acc_.array() >= 0.0).all() && kd_acc_.allFinite() && (kd_acc_.array() >= 0.0).all() &&
	       kp_rate_.allFinite() && (kp_rate_.array() >= 0.0).all() && std::isfinite(kp_att_z_) && kp_att_z_ > 0.0 &&
	       std::isfinite(kp_att_xy_) && kp_att_xy_ >= kp_att_z_ && std::isfinite(filter_sampling_frequency_) &&
	       std::isfinite(filter_cutoff_frequency_) && filter_sampling_frequency_ > 2.0 * filter_cutoff_frequency_ &&
	       filter_cutoff_frequency_ > 0.0 && p_err_max_.allFinite() && (p_err_max_.array() >= 0.0).all() && v_err_max_.allFinite() &&
	       (v_err_max_.array() >= 0.0).all() && std::isfinite(max_tilt_rad_) && max_tilt_rad_ > 0.0 &&
	       max_tilt_rad_ < std::acos(-1.0) / 2.0;
}

}  // namespace agi
