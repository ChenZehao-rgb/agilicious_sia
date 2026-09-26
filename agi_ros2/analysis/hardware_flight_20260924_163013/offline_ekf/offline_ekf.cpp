// Offline sensitivity replay of the production agi::EkfImu. No ROS publishers or control outputs.
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "agilib/estimator/ekf_imu/ekf_imu.hpp"
#include "agilib/utils/yaml.hpp"

namespace {
using agi::Scalar;
using agi::Vector;

struct Event {
	char kind;
	std::vector<double> values;
};

Vector<3> vectorAt(const Event& event, size_t offset) {
	return {event.values.at(offset), event.values.at(offset + 1), event.values.at(offset + 2)};
}

void writeVector(std::ostream& output, const Vector<3>& value) { output << ',' << value.x() << ',' << value.y() << ',' << value.z(); }

std::shared_ptr<agi::EkfImuParameters> readParameters(const std::string& filename) {
	const agi::Yaml document{fs::path(filename)};
	const auto config = document["fusion"];
	auto parameters = std::make_shared<agi::EkfImuParameters>();
	parameters->R_acc.setConstant(config["imu_acceleration_variance"].as<double>());
	parameters->R_omega.setConstant(config["imu_angular_velocity_variance"].as<double>());
	config["ekf_process_position_variance"] >> parameters->Q_pos;
	config["ekf_process_attitude_variance"] >> parameters->Q_att;
	config["ekf_process_velocity_variance"] >> parameters->Q_vel;
	config["ekf_process_gyro_bias_variance"] >> parameters->Q_bome;
	config["ekf_process_acceleration_bias_variance"] >> parameters->Q_bacc;
	config["ekf_initial_attitude_variance"] >> parameters->Q_init_att;
	config["ekf_initial_gyro_bias_variance"] >> parameters->Q_init_bome;
	config["ekf_initial_acceleration_bias_variance"] >> parameters->Q_init_bacc;
	if (!parameters->valid()) throw std::runtime_error("Invalid fusion parameters");
	return parameters;
}

class Replay {
public:
	Replay(const std::string& prefix, std::shared_ptr<agi::EkfImuParameters> parameters, double max_nis, double max_imu_gap)
	        : _parameters(std::move(parameters)),
	          _states(prefix + "_states.csv"),
	          _updates(prefix + "_updates.csv"),
	          _segments(prefix + "_segments.csv"),
	          _max_nis(max_nis),
	          _max_imu_gap(max_imu_gap) {
		if (!_states || !_updates || !_segments) throw std::runtime_error("Cannot create output CSVs");
		_states << std::setprecision(15);
		_updates << std::setprecision(15);
		_segments << std::setprecision(15);
		_states << "segment,t,source,px,py,pz,vx,vy,vz,qw,qx,qy,qz,yaw,bgx,bgy,bgz,bax,bay,baz,"
		           "covariance_t,pvarx,pvary,pvarz,vvarx,vvary,vvarz,yawvar,accepted_count,rejected_count\n";
		_updates << "segment,t,status,nis,innovation_px,innovation_py,innovation_pz,innovation_vx,innovation_vy,innovation_vz,"
		            "innovation_yaw,gps_px,gps_py,gps_pz,gps_vx,gps_vy,gps_vz,gps_yaw,"
		            "post_px,post_py,post_pz,post_vx,post_vy,post_vz,post_yaw,"
		            "covariance_t,pvarx,pvary,pvarz,vvarx,vvary,vvarz,yawvar,accepted_count,rejected_count\n";
		_segments << "segment,t,action,detail\n";
	}

	void addEvent(const Event& event) {
		const size_t expected = event.kind == 'S' ? 24 : event.kind == 'I' ? 7 : event.kind == 'G' ? 15 : 0;
		if (!expected || event.values.size() != expected) throw std::runtime_error("Wrong CSV kind or field count");
		for (const auto value : event.values) {
			if (!std::isfinite(value)) throw std::runtime_error("Nonfinite event field");
		}
		if (event.kind == 'S') {
			startSegment(event);
			return;
		}
		if (!_ekf) {
			if (event.kind == 'G') logUncovered(event, "segment_inactive");
			return;
		}
		if (event.kind == 'I') {
			const double time = event.values[0];
			const double reference = std::isfinite(_last_imu) ? _last_imu : _segment_start;
			if ((std::isfinite(_last_imu) && time <= _last_imu) || time < reference || time - reference > _max_imu_gap) {
				_segments << _segment << ',' << time << ",stopped_imu_gap," << time - reference << '\n';
				flushPending("imu_gap");
				_ekf.reset();
				return;
			}
			if (!_ekf->addImu({time, vectorAt(event, 1), vectorAt(event, 4)})) {
				throw std::runtime_error("addImu rejected an input; input must start at S.t and increase strictly");
			}
			_last_imu = time;
			while (!_pending.empty() && _pending.front().values[0] <= time) {
				update(_pending.front());
				_pending.pop_front();
			}
			writeState(time, "imu");
			return;
		}
		if (!_pending.empty() && event.values[0] <= _pending.back().values[0]) {
			throw std::runtime_error("GPS samples must have strictly increasing timestamps");
		}
		if (event.values[0] <= _last_imu) {
			update(event);
		} else {
			_pending.push_back(event);
		}
	}

	void finish() { flushPending("end_without_imu_coverage"); }

private:
	void startSegment(const Event& event) {
		flushPending("segment_reset");
		_segment = event.values[1];
		agi::QuadState state;
		state.setZero();
		state.t = event.values[0];
		state.p = vectorAt(event, 2);
		state.v = vectorAt(event, 5);
		agi::Quaternion rotation(event.values[8], event.values[9], event.values[10], event.values[11]);
		if (std::abs(rotation.norm() - 1.0) > 1e-5) throw std::runtime_error("Initial quaternion must be normalized");
		state.q(rotation);
		state.bw = vectorAt(event, 12);
		state.ba = vectorAt(event, 15);
		auto parameters = std::make_shared<agi::EkfImuParameters>(*_parameters);
		parameters->Q_init_pos = vectorAt(event, 18);
		parameters->Q_init_vel = vectorAt(event, 21);
		if ((parameters->Q_init_pos.array() <= 0).any() || (parameters->Q_init_vel.array() <= 0).any()) {
			throw std::runtime_error("Initial position/velocity variances must be positive");
		}
		_ekf = std::make_unique<agi::EkfImu>(parameters);
		if (!_ekf->initialize(state)) throw std::runtime_error("EKF initialization failed");
		_last_imu = std::numeric_limits<double>::quiet_NaN();
		_segment_start = state.t;
		_segments << _segment << ',' << state.t << ",initialized,explicit_state\n";
		writeState(state.t, "initial");
	}

	void writeQuality(std::ostream& output) {
		const auto quality = _ekf->navigationQuality();
		output << ',' << quality.stamp;
		writeVector(output, quality.position_variance);
		writeVector(output, quality.velocity_variance);
		output << ',' << quality.heading_variance << ',' << quality.accepted_updates << ',' << quality.rejected_updates << '\n';
	}

	void writeState(double time, const char* source) {
		agi::QuadState state;
		state.setZero();
		if (!_ekf->getAt(time, &state) || !state.valid()) throw std::runtime_error("EKF prediction failed");
		_states << _segment << ',' << time << ',' << source;
		writeVector(_states, state.p);
		writeVector(_states, state.v);
		const auto q = state.q();
		_states << ',' << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << state.getYaw();
		writeVector(_states, state.bw);
		writeVector(_states, state.ba);
		writeQuality(_states);
	}

	void update(const Event& event) {
		const double time = event.values[0];
		agi::QuadState before;
		before.setZero();
		if (!_ekf->getAt(time, &before) || before.t != time) {
			logUncovered(event, "nonincreasing_or_uncovered_gps");
			return;
		}
		const auto position = vectorAt(event, 1);
		const auto velocity = vectorAt(event, 4);
		const double heading = event.values[7];
		const auto quality_before = _ekf->navigationQuality();
		const bool accepted = _ekf->addRtk(time, position, velocity, heading, true, vectorAt(event, 8), vectorAt(event, 11),
		                                   event.values[14], _max_nis);
		const auto quality = _ekf->navigationQuality();
		const bool evaluated = accepted || quality.rejected_updates > quality_before.rejected_updates;
		const char* status = accepted ? "accepted" : evaluated ? "rejected" : "not_evaluated";
		_updates << _segment << ',' << time << ',' << status << ',' << (evaluated ? quality.innovation_squared : NAN);
		writeVector(_updates, position - before.p);
		writeVector(_updates, velocity - before.v);
		_updates << ',' << std::remainder(heading - before.getYaw(), 2.0 * M_PI);
		writeVector(_updates, position);
		writeVector(_updates, velocity);
		_updates << ',' << heading;
		agi::QuadState after;
		after.setZero();
		if (!_ekf->getAt(time, &after)) throw std::runtime_error("Post-update prediction failed");
		writeVector(_updates, after.p);
		writeVector(_updates, after.v);
		_updates << ',' << after.getYaw();
		writeQuality(_updates);
		writeState(time, accepted ? "gps_accepted" : "gps_rejected");
	}

	void logUncovered(const Event& event, const char* reason) {
		_updates << _segment << ',' << event.values[0] << ',' << reason;
		for (int i = 0; i < 32; ++i) _updates << ",nan";
		_updates << '\n';
	}

	void flushPending(const char* reason) {
		for (const auto& event : _pending) logUncovered(event, reason);
		_pending.clear();
	}

	std::shared_ptr<agi::EkfImuParameters> _parameters;
	std::unique_ptr<agi::EkfImu> _ekf;
	std::ofstream _states;
	std::ofstream _updates;
	std::ofstream _segments;
	std::deque<Event> _pending;
	double _max_nis;
	double _max_imu_gap;
	double _last_imu = NAN;
	double _segment_start = NAN;
	double _segment = -1;
};
}  // namespace

int main(int argc, char** argv) {
	if (argc != 6) {
		std::cerr << "Usage: offline_ekf events.csv output_prefix max_nis max_imu_gap hardware.yaml\n";
		return 2;
	}
	size_t line_number = 0;
	try {
		const double max_nis = std::stod(argv[3]);
		const double max_imu_gap = std::stod(argv[4]);
		if (!(max_nis > 0) || !(max_imu_gap > 0)) throw std::runtime_error("NIS and IMU gap limits must be positive");
		Replay replay(argv[2], readParameters(argv[5]), max_nis, max_imu_gap);
		std::ifstream input(argv[1]);
		if (!input) throw std::runtime_error("Cannot read event CSV");
		std::string line;
		while (std::getline(input, line)) {
			++line_number;
			if (line.empty() || line[0] == '#') continue;
			std::istringstream fields(line);
			std::string field;
			std::getline(fields, field, ',');
			if (field.size() != 1) throw std::runtime_error("Event kind must be S, I or G");
			Event event{field[0], {}};
			while (std::getline(fields, field, ',')) event.values.push_back(std::stod(field));
			replay.addEvent(event);
		}
		replay.finish();
		std::cerr << "Processed " << line_number << " CSV lines using agi::EkfImu\n";
		return 0;
	} catch (const std::exception& error) {
		std::cerr << "Replay failed at input line " << line_number << ": " << error.what() << '\n';
		return 1;
	}
}
