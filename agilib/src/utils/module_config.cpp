#include "agilib/utils/module_config.hpp"

#include <stdexcept>

namespace agi {

std::ostream& operator<<(std::ostream& os, const ModuleConfig& config) {
  return os << "Type: " << config.type << "\nFile: " << config.file << '\n';
}

bool ModuleConfig::loadIfUndefined(const Yaml& yaml, const std::string& type_override) {
	const auto parameter_sets = yaml["parameter_sets"];
	if (parameter_sets.isDefined() && (yaml["parameters"].isDefined() || yaml["file"].isDefined())) {
		throw std::invalid_argument("Module parameter_sets cannot be combined with parameters or file");
	}
	if (!type_override.empty()) {
		if (!parameter_sets.isDefined() && yaml["type"].as<std::string>() != type_override) {
			throw std::invalid_argument("Changing controller requires parameter_sets for the selected type");
		}
		type = type_override;
	}
	if (type.empty()) yaml["type"].getIfDefined(type);
	if (file.empty()) yaml["file"].getIfDefined(file);
	if (parameter_sets.isDefined()) {
		if (!parameter_sets.isNode() || !parameter_sets[type].isNode()) {
			throw std::invalid_argument("Missing parameter_sets mapping for selected controller: " + type);
		}
		parameters = parameter_sets[type];
	} else if (!parameters.isDefined()) {
		parameters = yaml["parameters"];
	}
	if (!file.empty() && parameters.isDefined()) {
		throw std::invalid_argument("Module configuration cannot combine file and parameters");
	}
	if (parameters.isDefined() && !parameters.isNode()) {
		throw std::invalid_argument("Module parameters must be a mapping");
	}
	return !type.empty() && (!file.empty() || parameters.isDefined());
}

}  // namespace agi
