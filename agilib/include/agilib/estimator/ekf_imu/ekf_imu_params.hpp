
#pragma once

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
