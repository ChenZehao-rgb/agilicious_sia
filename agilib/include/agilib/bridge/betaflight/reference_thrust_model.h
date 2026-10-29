#ifndef AGILIB_BRIDGE_BETAFLIGHT_REFERENCE_THRUST_MODEL_H_
#define AGILIB_BRIDGE_BETAFLIGHT_REFERENCE_THRUST_MODEL_H_

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace agi::hardware {

// File-only model. All forces are vehicle-total newtons; no mass/motor scaling.
class ReferenceThrustModel {
public:
	struct Mapping {
		double scale;
		double lookup_n;
		double rc_continuous;
		uint16_t rc;
		double minimum_n;
		double maximum_n;
		bool lower_saturated = false;
		bool idle_requested = false;
	};

	explicit ReferenceThrustModel(const std::string& filename, bool shadow = false) {
		const auto yaml = YAML::LoadFile(filename);
		if (!yaml.IsMap()) throw std::invalid_argument("Calibration must be a mapping");
		std::set<std::string> keys;
		for (const auto& item : yaml) {
			if (!keys.insert(item.first.as<std::string>()).second) throw std::invalid_argument("Duplicate calibration key");
		}
		const auto exact = [&](const char* key, const char* value) {
			if (!yaml[key].IsScalar() || yaml[key].Scalar() != value)
				throw std::invalid_argument(std::string("Invalid calibration ") + key);
		};
		for (const char* key : {"schema_version", "no_ir_correction", "tested_motor_count", "motor_count"}) {
			if (yaml[key].Tag() == "!") throw std::invalid_argument("Quoted boolean/integer calibration field");
		}
		exact("schema_version", "1");
		exact("model", "reference_table");
		exact("command_axis", "msp_raw_rc_throttle");
		exact("thrust_unit", "N");
		exact("thrust_scope", "total_vehicle");
		exact("calibration_voltage_source", "blackbox_vbatLatest");
		exact("runtime_voltage_source", "msp130_display_filtered");
		exact("voltage_compatibility", "steady_state");
		exact("no_ir_correction", "true");
		exact("tested_motor_count", "1");
		exact("motor_count", "4");
		exact("aggregation", "single_motor_times_four");
		for (const char* key : {"calibration_id", "axis_source", "axis_conversion", "axis_evidence", "table_file"}) {
			if (!yaml[key].IsScalar() || yaml[key].Scalar().find_first_not_of(" \t\r\n") == std::string::npos ||
			    yaml[key].Scalar() == "unknown")
				throw std::invalid_argument(std::string("Missing calibration ") + key);
		}
		_id = yaml["calibration_id"].Scalar();
		const std::string provenance = yaml["provenance"].as<std::string>();
		_provenance = provenance;
		const std::string status = yaml["calibration_status"].as<std::string>();
		_status = status;
		const bool nominal = provenance == "measured_motor_curve_with_nominal_rc_conversion";
		if (nominal) {
			if (yaml["axis_verified"]) throw std::invalid_argument("Nominal mapping must omit axis_verified");
			exact("calibration_status", "provisional");
			exact("axis_mapping_basis", "source_and_config_assumptions");
			exact("measured_motor_index", "0");
			if (yaml["measured_motor_index"].Tag() == "!") throw std::invalid_argument("Quoted motor index");
			const auto target = yaml["selected_target_profile"].as<std::string>();
			if (target != "static" && target != "dynamic_nominal") throw std::invalid_argument("Invalid target profile");
		} else {
			exact("axis_verified", "true");
			if (yaml["axis_verified"].Tag() == "!") throw std::invalid_argument("Quoted axis_verified");
			if ((provenance != "measured" && provenance != "synthetic") || (status != "validated" && status != "unvalidated"))
				throw std::invalid_argument("Invalid calibration provenance/status");
			_measured = provenance == "measured" && status == "validated";
			if (!shadow && !_measured) throw std::invalid_argument("Production requires measured validated calibration");
			if (!shadow) {
				const auto validation = yaml["heldout_metrics"];
				if (!validation.IsMap() || validation["passed"].Scalar() != "true" ||
				    validation["independent_final_test"].Scalar() != "true" || validation["passed"].Tag() == "!" ||
				    validation["independent_final_test"].Tag() == "!")
					throw std::invalid_argument("Production requires passed independent final-test evidence");
			}
		}
		const auto lower_policy = yaml["lower_policy"] ? yaml["lower_policy"].as<std::string>() : "reject";
		if (lower_policy != "reject" && lower_policy != "minimum_supported_rc") throw std::invalid_argument("Invalid lower_policy");
		_minimum_supported = lower_policy == "minimum_supported_rc";
		const auto number = [&](const char* key) {
			if (!yaml[key].IsScalar() || yaml[key].Tag() == "!")
				throw std::invalid_argument(std::string("Invalid number ") + key);
			return parseNumber(yaml[key].Scalar());
		};
		_vref = number("reference_voltage_v");
		_p = number("voltage_exponent_p");
		_vmin = number("valid_voltage_min_v");
		_vmax = number("valid_voltage_max_v");
		_tau = number("voltage_filter_time_constant_s");
		_max_age = number("max_voltage_sample_age_s");
		const double cmin = number("valid_rc_min"), cmax = number("valid_rc_max");
		if (!(_vmin > 0 && _vmin <= _vref && _vref <= _vmax && _vmin < _vmax && _tau >= 0 && _max_age > 0 && cmin >= 1000 &&
		      cmax <= 2000 && cmin < cmax && cmin == std::floor(cmin) && cmax == std::floor(cmax))) {
			throw std::invalid_argument("Invalid reference calibration domain/filter");
		}
		const auto table = std::filesystem::path(filename).parent_path() / yaml["table_file"].Scalar();
		std::ifstream input(table);
		std::string line;
		if (!std::getline(input, line)) throw std::invalid_argument("Cannot read reference table");
		if (!line.empty() && line.back() == '\r') line.pop_back();
		if (line != "rc_throttle,total_thrust_n_at_vref") throw std::invalid_argument("Wrong reference CSV schema");
		while (std::getline(input, line)) {
			if (!line.empty() && line.back() == '\r') line.pop_back();
			const auto comma = line.find(',');
			if (comma == std::string::npos || line.find(',', comma + 1) != std::string::npos)
				throw std::invalid_argument("Invalid CSV row");
			const double c = parseNumber(line.substr(0, comma)), t = parseNumber(line.substr(comma + 1));
			if (c < cmin || c > cmax || c != std::floor(c) || t < 0 ||
			    (!_rc.empty() && (c <= _rc.back() || t < _force.back()))) {
				throw std::invalid_argument("Invalid/nonmonotonic reference table");
			}
			_rc.push_back(c);
			_force.push_back(t);
		}
		if (_rc.size() < 2 || _rc.front() != cmin || _rc.back() != cmax || _force.front() >= _force.back()) {
			throw std::invalid_argument("Reference table must span exactly the declared domain and have distinct forces");
		}
	}

	Mapping map(double total_n, double voltage) const {
		checkVoltage(voltage);
		if (!std::isfinite(total_n) || total_n < 0) throw std::out_of_range("Invalid total thrust N");
		const double scale = std::pow(_vref / voltage, _p), lookup = total_n * scale;
		if (!std::isfinite(scale) || scale <= 0 || !std::isfinite(lookup)) throw std::out_of_range("Compensation overflow");
		// Zero is an idle request; the output layer applies its existing idle channel.
		if (total_n == 0) return {scale, lookup, 1000, 1000, _force.front() / scale, _force.back() / scale, false, true};
		if (lookup < _force.front() && _minimum_supported)
			return {scale,
			        lookup,
			        _rc.front(),
			        static_cast<uint16_t>(_rc.front()),
			        _force.front() / scale,
			        _force.back() / scale,
			        true,
			        false};
		if (lookup < _force.front() || lookup > _force.back()) throw std::out_of_range("Thrust outside reference domain");
		const auto it = std::lower_bound(_force.begin(), _force.end(), lookup);
		const size_t i = static_cast<size_t>(it - _force.begin());
		double c = _rc[i];
		if (*it != lookup) {
			// lower_bound uses the plateau's right end for a target above it.
			c = _rc[i - 1] + (lookup - _force[i - 1]) * (_rc[i] - _rc[i - 1]) / (_force[i] - _force[i - 1]);
		}
		return {scale, lookup, c, static_cast<uint16_t>(std::lround(c)), _force.front() / scale, _force.back() / scale};
	}

	void checkVoltage(double voltage) const {
		if (!std::isfinite(voltage) || voltage <= 0 || voltage < _vmin || voltage > _vmax) {
			throw std::out_of_range("Voltage outside reference calibration domain");
		}
	}
	const std::string& status() const { return _status; }
	bool measured() const { return _measured; }
	const std::string& id() const { return _id; }
	double referenceVoltage() const { return _vref; }
	double exponent() const { return _p; }
	double filterTimeConstant() const { return _tau; }
	double maxSampleAge() const { return _max_age; }
	const std::string& provenance() const { return _provenance; }
	double minimumVoltage() const { return _vmin; }
	double maximumVoltage() const { return _vmax; }
	double minimumRc() const { return _rc.front(); }
	double maximumRc() const { return _rc.back(); }

private:
	static double parseNumber(const std::string& text) {
		static const std::regex decimal(R"(^[ \t]*[+-]?([0-9]+(\.[0-9]*)?|\.[0-9]+)([eE][+-]?[0-9]+)?[ \t]*$)");
		if (!std::regex_match(text, decimal)) throw std::invalid_argument("Expected finite decimal value");
		size_t used = 0;
		const double value = std::stod(text, &used);
		while (used < text.size() && (text[used] == ' ' || text[used] == '\t')) ++used;
		if (used != text.size() || !std::isfinite(value)) throw std::invalid_argument("Invalid finite numeric value");
		return value;
	}
	std::vector<double> _rc, _force;
	std::string _id, _provenance, _status;
	bool _measured = false;
	bool _minimum_supported = false;
	double _vref, _p, _vmin, _vmax, _tau, _max_age;
};

// Request time is the conservative age origin; receive time drives the extra LPF.
// Identity/age survive repeated Health publications. No fallback to Vref.
class ReferenceVoltageFilter {
public:
	void invalidate() { _filtered = std::numeric_limits<double>::quiet_NaN(); }
	void update(const ReferenceThrustModel& model, double now, const std::string& session, uint64_t sequence, double request_time,
	            double receive_time, double raw) {
		if (!std::isfinite(now) || (std::isfinite(_last_now) && now < _last_now)) {
			invalidate();
			_last_now = now;
			throw std::out_of_range("Voltage clock reset");
		}
		_last_now = now;
		if (session.empty() || sequence == 0 || !std::isfinite(request_time) || !std::isfinite(receive_time) || request_time <= 0 ||
		    receive_time < request_time || receive_time > now || now - request_time > model.maxSampleAge()) {
			invalidate();
			throw std::out_of_range("Voltage report unavailable/stale/future");
		}
		try {
			model.checkVoltage(raw);
		} catch (...) {
			invalidate();
			throw;
		}
		if (session != _session) {
			if (_retired.count(session)) {
				invalidate();
				throw std::out_of_range("Retired voltage session");
			}
			if (!_session.empty()) _retired.insert(_session);
			_session = session;
			_sequence = 0;
			invalidate();
		}
		if (sequence == _sequence) {
			if (request_time != _request || receive_time != _receive || raw != _raw || !std::isfinite(_filtered)) {
				invalidate();
				throw std::out_of_range("Changed/invalid cached voltage report");
			}
			return;
		}
		if (sequence < _sequence || (_sequence && (receive_time <= _receive || request_time <= _request))) {
			invalidate();
			throw std::out_of_range("Reordered voltage report");
		}
		const double tau = model.filterTimeConstant();
		_filtered = !std::isfinite(_filtered) || tau == 0
		                    ? raw
		                    : _filtered + (-std::expm1(-(receive_time - _receive) / tau)) * (raw - _filtered);
		_sequence = sequence;
		_request = request_time;
		_receive = receive_time;
		_raw = raw;
		model.checkVoltage(_filtered);
	}
	double value(const ReferenceThrustModel& model, double now) const {
		if (!std::isfinite(now) || !std::isfinite(_request) || now < _receive || now - _request > model.maxSampleAge()) {
			throw std::out_of_range("Voltage report stale/future");
		}
		model.checkVoltage(_filtered);
		return _filtered;
	}
	double filtered() const { return _filtered; }
	double raw() const { return _raw; }
	double age(double now) const { return now - _request; }

private:
	std::string _session;
	std::set<std::string> _retired;
	uint64_t _sequence = 0;
	double _filtered = NAN, _raw = NAN, _request = NAN, _receive = NAN, _last_now = NAN;
};
}  // namespace agi::hardware
#endif  // AGILIB_BRIDGE_BETAFLIGHT_REFERENCE_THRUST_MODEL_H_
