#ifndef AGI_ROS2_RUNTIME_CONFIG_H_
#define AGI_ROS2_RUNTIME_CONFIG_H_

#include <Eigen/LU>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "agilib/pilot/pilot_params.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/yaml.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agi_ros2 {

// A profile is read once at startup. Relative data paths belong to its directory.
class RuntimeConfig {
public:
	explicit RuntimeConfig(const std::string& filename)
	        : _filename(std::filesystem::absolute(filename).lexically_normal()), _document(_filename) {
		if (!_document.isNode()) throw std::invalid_argument("Runtime configuration must be a mapping: " + _filename.string());
	}

	const agi::Yaml& document() const { return _document; }
	agi::Yaml section(const std::string& name) const { return _document[name]; }
	std::string filename() const { return _filename.string(); }

	std::string resolvePath(const std::string& path) const {
		if (path.empty()) return "";
		const std::filesystem::path value(path);
		return (value.is_absolute() ? value : _filename.parent_path() / value).lexically_normal().string();
	}

	agi::Quadrotor loadQuadrotor() const {
		const auto model = _document["pilot"]["quadrotor"];
		if (!model.isNode()) throw std::invalid_argument("Runtime profile requires an inline pilot.quadrotor mapping");
		std::vector<std::string> invalid;
		const auto positive = [&](const std::string& name) {
			try {
				const double value = model[name].as<double>();
				if (!std::isfinite(value) || value <= 0) invalid.push_back(name);
			} catch (const std::exception&) {
				invalid.push_back(name);
			}
		};
		for (const auto* name : {"mass", "motor_omega_max", "motor_tau", "kappa", "thrust_max"}) positive(name);
		const auto vector = [&](const std::string& name, bool positive_elements) {
			try {
				agi::Vector<3> value;
				if (model[name].size() != 3 || !model[name].getIfDefined(value) || !value.allFinite() ||
				    (positive_elements ? (value.array() <= 0).any() : value.squaredNorm() == 0)) {
					invalid.push_back(name);
				}
			} catch (const std::exception&) {
				invalid.push_back(name);
			}
		};
		vector("inertia", true);
		vector("omega_max", true);
		vector("thrust_map", false);
		for (const auto* name : {"tbm_fr", "tbm_bl", "tbm_br", "tbm_fl"}) vector(name, false);
		if (!invalid.empty()) {
			std::ostringstream message;
			message << "Missing or invalid measured pilot.quadrotor fields in " << filename() << ": ";
			for (size_t i = 0; i < invalid.size(); ++i) message << (i ? ", " : "") << invalid[i];
			throw std::invalid_argument(message.str());
		}
		agi::Quadrotor quad;
		if (!quad.load(model) || !quad.valid())
			throw std::invalid_argument("Invalid or unconfigured pilot.quadrotor in " + filename());
		if (quad.getAllocationMatrix().fullPivLu().rank() != 4) {
			throw std::invalid_argument(
			        "pilot.quadrotor rotor geometry must provide independent collective and three-axis torque control");
		}
		return quad;
	}

	std::unique_ptr<agi::PilotParams> createPilotParams() const {
		// Validate the measured model before constructing the controller or output.
		loadQuadrotor();
		auto parameters = std::make_unique<agi::PilotParams>();
		parameters->directory_ = _filename.parent_path();
		if (!parameters->load(_document["pilot"])) throw std::invalid_argument("Invalid pilot section in " + filename());
		return parameters;
	}

private:
	std::filesystem::path _filename;
	agi::Yaml _document;
};

inline std::optional<RuntimeConfig> loadRuntimeConfig(rclcpp::Node& node, const std::string& mode) {
	rcl_interfaces::msg::ParameterDescriptor descriptor;
	descriptor.read_only = true;
	const auto filename = node.declare_parameter<std::string>("runtime_config", "", descriptor);
	if (filename.empty()) return std::nullopt;
	RuntimeConfig config(filename);
	if (config.document()["mode"].as<std::string>() != mode) {
		throw std::invalid_argument("runtime_config mode does not match node mode: " + config.filename());
	}
	for (const auto* legacy : {"params_dir", "pilot_config", "bridge_config"}) {
		const auto& overrides = node.get_node_parameters_interface()->get_parameter_overrides();
		const auto found = overrides.find(legacy);
		if (found != overrides.end() && !found->second.get<std::string>().empty()) {
			throw std::invalid_argument(std::string("runtime_config cannot be combined with ") + legacy);
		}
	}
	RCLCPP_INFO(node.get_logger(), "Runtime configuration: %s (mode=%s)", config.filename().c_str(), mode.c_str());
	return config;
}

}  // namespace agi_ros2

#endif  // AGI_ROS2_RUNTIME_CONFIG_H_
