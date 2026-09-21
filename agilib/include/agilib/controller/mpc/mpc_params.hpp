#pragma once

#include "agilib/base/parameter_base.hpp"

namespace agi {

struct MpcParameters : public ParameterBase {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	MpcParameters();
	MpcParameters(const MpcParameters& rhs) = default;

	using ParameterBase::load;
	bool load(const Yaml& node) override;
	bool valid() const override;

	bool timing_;
	Vector<3> Q_pos_;
	Vector<3> Q_att_;
	Vector<3> Q_vel_;
	// Weights for [collective acceleration (m/s^2), body rates x/y/z (rad/s)].
	Vector<4> R_;
	Scalar exp_decay_;

	friend std::ostream& operator<<(std::ostream& os, const MpcParameters& params);
};

}  // namespace agi
