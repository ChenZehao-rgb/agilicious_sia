#ifndef AGI_ROS2_FC_CONTROL_SCHEDULE_H_
#define AGI_ROS2_FC_CONTROL_SCHEDULE_H_

#include <cmath>
#include <cstdint>
#include <string>

namespace agi_ros2 {

// Single-owner admission/coalescing only. SafetyGate still checks original epochs.
// A health notification or a validity-generation change is not a new sample.
class FcControlSchedule {
public:
	enum class Update { Ignore, Sample, Fault };
	static constexpr double kMinStartInterval = .008;

	Update observe(const std::string& session, uint64_t generation, uint32_t reset, uint64_t fc_time, double sample, double published,
	               double now, bool valid) {
		if (!std::isfinite(published) || published > now || session.empty()) {
			_pending = false;
			return Update::Fault;
		}
		if (std::isfinite(_last_publication) && published <= _last_publication) {
			++ignored;
			return Update::Ignore;
		}
		_last_publication = published;
		if (session == _session && generation < _generation) {
			++ignored;
			return Update::Ignore;
		}
		const bool epoch_changed = session != _session || reset != _reset;
		const bool generation_changed = epoch_changed || generation != _generation;
		_session = session;
		_generation = generation;
		_reset = reset;
		if (epoch_changed) {
			_last_fc_time = 0;
			_last_sample = NAN;
		}
		if (!valid || !std::isfinite(sample) || sample > now || fc_time == 0 || fc_time < _last_fc_time ||
		    (std::isfinite(_last_sample) && sample < _last_sample)) {
			_pending = false;
			return Update::Fault;
		}
		if (fc_time == _last_fc_time || sample == _last_sample) {
			++ignored;
			if (generation_changed) {
				_pending = false;
				return Update::Fault;
			}
			return Update::Ignore;
		}
		_last_fc_time = fc_time;
		_last_sample = sample;
		if (_pending) ++coalesced;
		_pending = true;
		return Update::Sample;
	}

	bool take(double now) {
		if (!_pending || !std::isfinite(now)) return false;
		if (std::isfinite(_last_start) && now - _last_start < kMinStartInterval) return false;
		_pending = false;
		_last_start = now;
		return true;
	}

	uint64_t ignored{0};
	uint64_t coalesced{0};

private:
	std::string _session;
	uint64_t _generation{0};
	uint32_t _reset{0};
	uint64_t _last_fc_time{0};
	double _last_sample{NAN};
	double _last_publication{NAN};
	double _last_start{NAN};
	bool _pending{false};
};

}  // namespace agi_ros2
#endif  // AGI_ROS2_FC_CONTROL_SCHEDULE_H_
