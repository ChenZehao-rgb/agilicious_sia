#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "agilib/bridge/betaflight/reference_thrust_model.h"

namespace {
void require(bool ok) {
	if (!ok) throw std::runtime_error("reference model assertion failed");
}
template <typename F>
void rejects(F fn) {
	bool rejected = false;
	try {
		fn();
	} catch (const std::exception&) {
		rejected = true;
	}
	require(rejected);
}
}  // namespace

int main(int argc, char** argv) {
	try {
		if (argc < 3) throw std::invalid_argument("probe YAML shadow|production [N V|filter]");
		const agi::hardware::ReferenceThrustModel m(argv[1], std::string(argv[2]) == "shadow");
		if (argc == 4 && std::string(argv[3]) == "filter") {
			agi::hardware::ReferenceVoltageFilter f;
			rejects([&] { f.value(m, 10); });
			f.update(m, 10.1, "session1", 1, 10, 10.05, 22);
			require(f.value(m, 10.2) == 22);
			f.update(m, 10.3, "session1", 1, 10, 10.05, 22);
			require(std::abs(f.age(10.3) - .3) < 1e-12);
			f.update(m, 10.6, "session1", 2, 10.5, 10.55, 24);
			const double expected = 22 + (1 - std::exp(-.5 / m.filterTimeConstant())) * 2;
			require(std::abs(f.value(m, 10.6) - expected) < 1e-12);
			f.update(m, 10.7, "session1", 2, 10.5, 10.55, 24);
			require(std::abs(f.value(m, 10.7) - expected) < 1e-12);
			rejects([&] { f.value(m, 10.6 + m.maxSampleAge()); });
			rejects([&] { f.update(m, 10.6 + m.maxSampleAge(), "session1", 2, 10.5, 10.55, 24); });
			f.update(m, 12.2, "session2", 1, 12.1, 12.15, 23);
			require(f.value(m, 12.3) == 23);
			rejects([&] { f.update(m, 12.4, "session1", 3, 12.3, 12.35, 24); });
			rejects([&] { f.value(m, 12.4); });
			f.update(m, 12.6, "session2", 2, 12.5, 12.55, 22);
			f.invalidate();
			rejects([&] { f.update(m, 12.7, "session2", 2, 12.5, 12.55, 22); });
			f.update(m, 12.9, "session2", 3, 12.8, 12.85, 23);
			rejects([&] { f.update(m, 1, "session2", 3, 12.8, 12.85, 23); });
			rejects([&] { f.value(m, 13); });
			std::cout << "PASS filter identity, sample age, clock/session reset, stale/replayed report rejection\n";
		} else if (argc == 5 || argc == 6) {
			const auto r = m.map(std::stod(argv[3]), std::stod(argv[4]));
			std::cout << std::setprecision(17) << r.rc_continuous << ' ' << r.rc << ' ' << r.scale << ' ' << r.lookup_n;
			if (argc == 6) std::cout << ' ' << r.lower_saturated << ' ' << r.idle_requested;
			std::cout << '\n';
		} else {
			std::cout << "PASS load\n";
		}
		return 0;
	} catch (const std::exception& e) {
		std::cerr << e.what() << '\n';
		return 2;
	}
}
