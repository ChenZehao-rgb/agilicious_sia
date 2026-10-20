#ifndef AGI_ROS2_NAVIGATION_READINESS_H_
#define AGI_ROS2_NAVIGATION_READINESS_H_

#include <algorithm>

namespace agi_ros2 {

// Readiness does not reset or modify the estimator. Recovery uses ordinary
// innovation-gated observations; fresh physical AUTO authorization is separate.
class NavigationReadiness {
public:
	enum class Phase { kInitializing, kTracking, kCoasting, kRecovering };

	NavigationReadiness(int initialization_updates = 30, int recovery_updates = 5)
	        : _initialization_updates(initialization_updates), _recovery_updates(recovery_updates) {}

	void observe(bool navigation_accepted, bool heading_accepted) {
		if (navigation_accepted && heading_accepted) {
			_rejections = 0;
			_consecutive = std::min(_consecutive + 1, _initialization_updates);
			if (_phase == Phase::kCoasting) _phase = Phase::kTracking;
			return;
		}
		_consecutive = 0;
		++_rejections;
		// Permit one isolated non-heading rejection only while all external
		// freshness, covariance and observability checks continue to hold.
		if (_phase == Phase::kTracking && heading_accepted && _rejections == 1)
			_phase = Phase::kCoasting;
		else
			revoke();
	}

	bool evaluate(bool evidence_valid) {
		if (!evidence_valid) {
			if (_acquired) revoke();
			return false;
		}
		const int required = _acquired ? _recovery_updates : _initialization_updates;
		if ((_phase == Phase::kInitializing || _phase == Phase::kRecovering) && _consecutive >= required) {
			_acquired = true;
			_phase = Phase::kTracking;
		}
		return _phase == Phase::kTracking || _phase == Phase::kCoasting;
	}

	void revoke() {
		_phase = _acquired ? Phase::kRecovering : Phase::kInitializing;
		_consecutive = 0;
	}

	int consecutive() const { return _consecutive; }
	Phase phase() const { return _phase; }

private:
	int _initialization_updates;
	int _recovery_updates;
	int _consecutive = 0;
	int _rejections = 0;
	bool _acquired = false;
	Phase _phase = Phase::kInitializing;
};

}  // namespace agi_ros2

#endif  // AGI_ROS2_NAVIGATION_READINESS_H_
