// Native height-fusion replay. CSV inputs preserve recorded sampling times; no hardware access.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "agilib/estimator/ekf_imu/ekf_imu.hpp"

namespace {

struct Csv {
	std::map<std::string, size_t> columns;
	std::vector<std::vector<double>> rows;
	explicit Csv(const std::string& path) {
		std::ifstream input(path);
		if (!input) throw std::runtime_error("Cannot open " + path);
		std::string line, field;
		std::getline(input, line);
		std::istringstream header(line);
		while (std::getline(header, field, ',')) columns.emplace(field, columns.size());
		while (std::getline(input, line)) {
			std::istringstream stream(line);
			std::vector<double> row;
			while (std::getline(stream, field, ',')) row.push_back(std::stod(field));
			if (row.size() != columns.size()) throw std::runtime_error("CSV width mismatch");
			rows.push_back(row);
		}
	}
	double get(const std::vector<double>& row, const std::string& name) const { return row.at(columns.at(name)); }
	agi::Vector<3> xyz(const std::vector<double>& row, const std::string& prefix) const {
		return {get(row, prefix + "x"), get(row, prefix + "y"), get(row, prefix + "z")};
	}
};

struct Event {
	double time;
	int kind;
	size_t index;
	size_t reference;
};

int run(int argc, char** argv) {
	std::map<std::string, std::string> options;
	for (int i = 1; i < argc; i += 2) {
		if (i + 1 >= argc) throw std::runtime_error("Options require values");
		options[argv[i]] = argv[i + 1];
	}
	const auto number = [&options](const std::string& key, double fallback) {
		return options.count(key) ? std::stod(options.at(key)) : fallback;
	};
	const std::string directory = options.at("--input");
	const Csv imus(directory + "/imu.csv"), nav(directory + "/nav.csv"), baro(directory + "/baro.csv");
	const Csv init(directory + "/initial.csv"), refs(directory + "/references.csv");
	const auto& initial = init.rows.at(0);
	agi::QuadState state;
	state.setZero();
	state.t = init.get(initial, "t");
	state.p = init.xyz(initial, "p");
	state.v = init.xyz(initial, "v");
	state.bw = init.xyz(initial, "bw");
	state.ba = init.xyz(initial, "ba");
	state.q(agi::Quaternion(init.get(initial, "qw"), init.get(initial, "qx"), init.get(initial, "qy"), init.get(initial, "qz")));
	const double start = state.t;
	const bool weighted = options.count("--nav-mode") && options.at("--nav-mode") == "weighted";
#ifndef REPLAY_WEIGHTED_MODE
	if (weighted) throw std::runtime_error("This source does not support grouped navigation fusion");
#endif
	auto params = std::make_shared<agi::ReplayEkfImuParameters>();
	params->R_acc.setConstant(0.1);
	params->R_omega.setConstant(0.0001);
	params->Q_pos.setConstant(1e-6);
	params->Q_att.setConstant(1e-6);
	params->Q_vel.setConstant(0.001);
	params->Q_bome.setConstant(0.0001);
	params->Q_bacc.setConstant(0.1);
	params->Q_init_pos = init.xyz(initial, "pvar");
	params->Q_init_vel = init.xyz(initial, "vvar");
	params->Q_init_att.setConstant(0.01);
	params->Q_init_bome.setConstant(0.001);
	params->Q_init_bacc.setConstant(0.01);
	params->baro_bias_random_walk = number("--baro-q", 0.01);
#ifdef REPLAY_WEIGHTED_MODE
	params->baro_relative_reference = weighted;
#endif
	agi::ReplayEkfImu ekf(params);
	if (!ekf.initialize(state)) throw std::runtime_error("Initialization failed");
	std::vector<Event> events;
	const bool continuous_reference = options.count("--reference-mode") && options.at("--reference-mode") == "continuous";
	const bool baro_primary = options.count("--nav-mode") && options.at("--nav-mode") == "baro-primary";
#ifndef REPLAY_HORIZONTAL_MODE
	if (baro_primary) throw std::runtime_error("This source does not support horizontal navigation mode");
#endif
	for (size_t i = 0; i < nav.rows.size(); ++i) events.push_back({nav.get(nav.rows[i], "t"), 0, i, 0});
	for (size_t r = 0; r < refs.rows.size(); ++r) {
		if (continuous_reference && r > 0) break;
		const double begin = refs.get(refs.rows[r], "t");
		const double end = continuous_reference ? imus.get(imus.rows.back(), "t") : refs.get(refs.rows[r], "stop");
		events.push_back({begin, 1, r, r});
		for (size_t b = 0; b < baro.rows.size(); ++b) {
			const double time = baro.get(baro.rows[b], "t");
			if (time > begin && time <= end) events.push_back({time, 2, b, r});
		}
	}
	std::sort(events.begin(), events.end(), [](const Event& left, const Event& right) {
		return left.time < right.time || (left.time == right.time && left.kind < right.kind);
	});
	std::vector<double> anchors(refs.rows.size(), NAN);
	std::ofstream output(options.at("--output"));
	output << std::setprecision(17);
	output << "t,t_rel,z,vz,speed,pvarz,nav_accepted,nav_rejected,baro_accepted,baro_rejected,bias,bias_variance,baro_nis,"
	          "nav_nis,reference,baro_height,imu_ok,get_ok,nav_horizontal_attempts,nav_full_attempts,x,y,vx,vy,baro_healthy,"
	          "height_accepted,height_rejected,vz_accepted,vz_rejected,relative_height_variance\n";
	size_t next = 0, reference = 0;
	size_t nav_horizontal_attempts = 0, nav_full_attempts = 0;
	double last_height = NAN;
	for (const auto& row : imus.rows) {
		const double time = imus.get(row, "t");
		if (time - start > number("--duration", 1e9)) break;
		const bool imu_ok = ekf.addImu(agi::ImuSample(time, imus.xyz(row, "a"), imus.xyz(row, "g")));
		while (next < events.size() && events[next].time <= time - 0.2) {
			const Event event = events[next++];
			if (event.kind == 0) {
				const auto& measurement = nav.rows[event.index];
				auto pv = nav.xyz(measurement, "pvar"), vv = nav.xyz(measurement, "vvar");
				pv.z() *= number("--nav-z-scale", 1.0);
				vv.z() *= number("--nav-vz-scale", 1.0);
				const auto baro_quality = ekf.barometerQuality();
				const bool reference_open = continuous_reference || event.time <= refs.get(refs.rows[reference], "stop");
				const bool use_baro = (baro_primary || weighted) && reference_open && baro_quality.valid &&
				                      std::isfinite(anchors[reference]) && event.time - baro_quality.stamp >= -0.010 &&
				                      event.time - baro_quality.stamp <= 0.25;
				if (use_baro && !weighted)
					++nav_horizontal_attempts;
				else
					++nav_full_attempts;
#ifdef REPLAY_HORIZONTAL_MODE
				if (weighted) {
#ifdef REPLAY_WEIGHTED_MODE
					if (use_baro) pv.z() = std::max(18.0, 4.5 * pv.z());
					vv.z() = std::max(0.09, 2.25 * vv.z());
					ekf.addNavigation(event.time, nav.xyz(measurement, "p"), nav.xyz(measurement, "v"),
					                  nav.get(measurement, "heading"), true, pv, vv, nav.get(measurement, "hvar"),
					                  20.515, 10.828, 10.828);
#endif
				} else {
					const auto mode = use_baro ? agi::ReplayEkfImu::NavigationMeasurementMode::kHorizontal
					                           : agi::ReplayEkfImu::NavigationMeasurementMode::kFull3d;
					ekf.addRtk(event.time, nav.xyz(measurement, "p"), nav.xyz(measurement, "v"),
					           nav.get(measurement, "heading"), true, pv, vv, nav.get(measurement, "hvar"),
					           use_baro ? 20.515 : 24.322, mode);
				}
#else
				ekf.addRtk(event.time, nav.xyz(measurement, "p"), nav.xyz(measurement, "v"),
				           nav.get(measurement, "heading"), true, pv, vv, nav.get(measurement, "hvar"), 24.322);
#endif
			} else if (event.kind == 1) {
				reference = event.reference;
				if (!ekf.alignBarometerReference(event.time, refs.get(refs.rows[reference], "independent_variance"),
				                                 &anchors[reference]))
					throw std::runtime_error("Reference alignment failed");
			} else {
				const double pressure = baro.get(baro.rows[event.index], "pressure");
				const double p0 = refs.get(refs.rows[event.reference], "pressure");
				const double ratio = pressure / p0;
				last_height = anchors[event.reference] + 44330.0 * (1.0 - std::pow(ratio, 0.190295));
				const double derivative = -44330.0 * 0.190295 / p0 * std::pow(ratio, -0.809705);
				const double variance = derivative * derivative * baro.get(baro.rows[event.index], "variance") + 0.25;
				ekf.addBaro(event.time, last_height, variance, 10.828);
			}
		}
		const bool get_ok = ekf.getAt(time, &state);
		const auto nq = ekf.navigationQuality();
		const auto bq = ekf.barometerQuality();
		const bool reference_available = continuous_reference || time - 0.2 <= refs.get(refs.rows[reference], "stop");
		const bool baro_healthy = reference_available && bq.valid && time - bq.stamp >= 0 && time - bq.stamp <= 0.45;
		output << time << ',' << time - start << ',' << state.p.z() << ',' << state.v.z() << ',' << state.v.norm() << ','
		       << nq.position_variance.z() << ',' << nq.accepted_updates << ',' << nq.rejected_updates << ',' << bq.accepted_updates
		       << ',' << bq.rejected_updates << ',' << bq.bias << ',' << bq.bias_variance << ',' << bq.innovation_squared << ','
		       << nq.innovation_squared << ',' << reference << ',' << last_height << ',' << imu_ok << ',' << get_ok << ','
		       << nav_horizontal_attempts << ',' << nav_full_attempts << ',' << state.p.x() << ',' << state.p.y() << ','
		       << state.v.x() << ',' << state.v.y() << ',' << baro_healthy;
#ifdef REPLAY_WEIGHTED_MODE
		output << ',' << nq.height.accepted_updates << ',' << nq.height.rejected_updates << ','
		       << nq.vertical_velocity.accepted_updates << ',' << nq.vertical_velocity.rejected_updates << ','
		       << bq.relative_height_variance;
#else
		output << ",0,0,0,0,nan";
#endif
		output << '\n';
	}
	return 0;
}

}  // namespace

int main(int argc, char** argv) {
	try {
		return run(argc, argv);
	} catch (const std::exception& error) {
		std::cerr << error.what() << '\n';
		return 1;
	}
}
