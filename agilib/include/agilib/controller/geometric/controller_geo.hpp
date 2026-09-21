#pragma once

#include <memory>
#include <mutex>

#include "agilib/controller/controller_base.hpp"
#include "agilib/controller/geometric/geo_params.hpp"
#include "agilib/math/gravity.hpp"
#include "agilib/math/types.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/imu_sample.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/low_pass_filter.hpp"

namespace agi {

class GeometricController : public ControllerBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	GeometricController(const Quadrotor& quad, const std::shared_ptr<GeometricControllerParams>& params, const Scalar exec_dt = 0.01);
	~GeometricController();

	bool getCommand(const QuadState& state, const SetpointVector& references, SetpointVector* const setpoints) override;

	bool updateParameters(const Quadrotor& quad, const std::shared_ptr<GeometricControllerParams> params);
	bool updateParameters(const Quadrotor& params);
	bool updateParameters(const std::shared_ptr<GeometricControllerParams> params);
	std::shared_ptr<GeometricControllerParams> getParameters();

	void addImuSample(const ImuSample& imu) override;
	bool addImu(const ImuSample& imu);

private:
	Vector<3> tiltPrioritizedControl(const Quaternion& q, const Quaternion& q_des);

	Quadrotor _quad;
	std::shared_ptr<GeometricControllerParams> _params;
	bool _has_full_model;
	std::unique_ptr<LowPassFilter<3>> _filter_acc;
	std::unique_ptr<LowPassFilter<4>> _filter_mot;
	std::mutex _imu_mutex;
	ImuSample _imu;
};

}  // namespace agi
