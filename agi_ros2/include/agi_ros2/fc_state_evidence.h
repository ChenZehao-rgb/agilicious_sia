#ifndef AGI_ROS2_FC_STATE_EVIDENCE_H_
#define AGI_ROS2_FC_STATE_EVIDENCE_H_
#include "agi_ros2/msg/fused_state.hpp"
#include "agilib/bridge/betaflight/hardware_safety.hpp"
namespace agi_ros2 {
inline void setFcEvidence(const msg::FusedState& s, agi::hardware::Evidence& e) {
	e.fc_valid = s.state_source == "fc_fused" && s.fc_state_valid;
	e.fc_sample_time = s.fc_state_sample_steady_time;
	e.fc_receive_time = s.fc_state_receive_steady_time;
	e.fc_publish_time = s.published_steady_time;
	e.fc_health_sample_time = s.fc_health_sample_steady_time;
	e.fc_health_receive_time = s.fc_health_receive_steady_time;
	e.fc_xy_observation_time = s.fc_xy_observation_time;
	e.fc_z_observation_time = s.fc_z_observation_time;
	e.fc_flags = s.fc_flags;
	e.fc_generation = s.reset_counter;
	e.fc_reset_counter = s.fc_reset_counter;
	e.fc_session = s.navigation_session;
	e.fc_position_variance = s.position_variance;
	e.fc_velocity_variance = s.velocity_variance;
	e.clock_aligned = s.clock_aligned;
}
}  // namespace agi_ros2
#endif
