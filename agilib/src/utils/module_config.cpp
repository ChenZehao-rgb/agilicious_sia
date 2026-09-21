#include "agilib/utils/module_config.hpp"

#include <stdexcept>

namespace agi {

std::ostream& operator<<(std::ostream& os, const ModuleConfig& config) {
  return os << "Type: " << config.type << "\nFile: " << config.file << '\n';
}

bool ModuleConfig::loadIfUndefined(const Yaml& yaml) {
	if (type.empty()) yaml["type"].getIfDefined(type);
	if (file.empty()) yaml["file"].getIfDefined(file);
	if (!parameters.isDefined()) parameters = yaml["parameters"];
	if (!file.empty() && parameters.isDefined()) {
		throw std::invalid_argument("Module configuration cannot combine file and parameters");
	}
	if (parameters.isDefined() && !parameters.isNode()) {
		throw std::invalid_argument("Module parameters must be a mapping");
	}
	return !type.empty() && (!file.empty() || parameters.isDefined());
}

}  // namespace agi
