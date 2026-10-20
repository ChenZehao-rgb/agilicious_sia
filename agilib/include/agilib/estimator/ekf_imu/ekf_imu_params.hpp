
#pragma once

#include <string>

#include "agilib/base/parameter_base.hpp"
#include "agilib/math/types.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

struct EkfImuParameters : public ParameterBase {
  EkfImuParameters();

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  bool valid() const override;

  Scalar jump_pos_threshold;
  Scalar jump_att_threshold;

	// legacy_dt2 preserves historic per-substep scaling. continuous_psd uses R_acc/R_omega as
	// continuous white-noise PSD ((m/s^2)^2*s and (rad/s)^2*s), and Q_* as state variance rates /s.
	// Q_bacc/Q_bome then describe bias random walks. No implicit unit conversion is performed.
	std::string process_noise_model = "legacy_dt2";
	Scalar integration_step = 1e-4;

  Vector<3> R_pos;
  Vector<3> R_att;
  Vector<3> R_acc;
  Vector<3> R_omega;

  Vector<3> Q_pos;
  Vector<4> Q_att;
  Vector<3> Q_vel;
  Vector<3> Q_bome;
  Vector<3> Q_bacc;
	// Continuous barometer bias random walk variance rate, m^2/s (discretized with dt).
	Scalar baro_bias_random_walk = 0.01;
	// Freeze the pressure reference in relative-height coordinates after alignment. Requires zero random walk.
	bool baro_relative_reference = false;

  Vector<3> Q_init_pos;
  Vector<4> Q_init_att;
  Vector<3> Q_init_vel;
  Vector<3> Q_init_bome;
  Vector<3> Q_init_bacc;
	// Initial pressure reference/bias uncertainty, m^2.
	Scalar Q_init_baro_bias = 25.0;

  bool update_on_get;

  friend std::ostream& operator<<(std::ostream& os,
                                  const EkfImuParameters& params);
};

}  // namespace agi
