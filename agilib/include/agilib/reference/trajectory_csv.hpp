#pragma once
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include "agilib/types/setpoint.hpp"
#include "agilib/math/gravity.hpp"
namespace agi::trajectory_csv {
inline std::vector<double> parseCsvRow(const std::string& line,
                                const std::size_t line_number) {
  std::vector<double> values;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    std::size_t parsed = 0;
    try {
      const double value = std::stod(field, &parsed);
      if (parsed != field.size() || !std::isfinite(value)) {
        throw std::runtime_error("not finite");
      }
      values.push_back(value);
    } catch (const std::exception&) {
      throw std::runtime_error("invalid numeric field in trajectory line " +
                               std::to_string(line_number));
    }
  }
  if (values.size() != 30) {
    throw std::runtime_error("trajectory line " +
                             std::to_string(line_number) + " has " +
                             std::to_string(values.size()) +
                             " columns; expected 30");
  }
  return values;
}

inline std::vector<std::vector<double>> readTrajectoryRows(const std::filesystem::path& path) {
	std::ifstream file(path);
	if (!file) {
		throw std::runtime_error("could not open trajectory: " + path.string());
	}

	std::string line;
	const std::string short_header = "t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z";
	const bool have_header = static_cast<bool>(std::getline(file, line));
	if (!line.empty() && line.back() == '\r') line.pop_back();
	if (!have_header || line != short_header) {
		throw std::runtime_error("unsupported trajectory CSV header: " + path.string());
	}

	std::vector<std::vector<double>> rows;
	std::size_t line_number = 1;
	while (std::getline(file, line)) {
		++line_number;
		if (!line.empty() && line.back() == '\r') line.pop_back();
		if (line.empty()) continue;
		rows.push_back(parseCsvRow(line, line_number));
	}
	if (rows.size() < 2) {
		throw std::runtime_error("trajectory must contain at least two samples");
	}
	return rows;
}

/// Recover the mass of the vehicle the CSV was generated for.
///
/// A rigid quadrotor's rotor thrusts sum to m * |a - g|, so a least-squares fit
/// over the whole file pins the mass down without trusting a command-line
/// constant.  The bundled datasets were generated for two different vehicles
/// (0.700 kg and 0.857 kg), and using one number for both scales every
/// feed-forward thrust by up to 20%.
inline double estimateSourceMass(const std::vector<std::vector<double>>& rows) {
  double numerator = 0.0;
  double denominator = 0.0;
  for (const std::vector<double>& value : rows) {
    const agi::Vector<3> specific_force(value[14], value[15],
                                        value[16] + agi::G);
    const double norm = specific_force.norm();
    const double thrust = value[20] + value[21] + value[22] + value[23];
    numerator += thrust * norm;
    denominator += norm * norm;
  }
  if (!(denominator > 0.0)) {
    throw std::runtime_error("trajectory has no usable acceleration data");
  }
  const double mass = numerator / denominator;
  if (!std::isfinite(mass) || mass <= 0.0) {
    throw std::runtime_error("could not estimate the trajectory source mass");
  }
  return mass;
}

/// \param start_yaw heading the vehicle hovers at when the trajectory takes
///        over.  The CSV is rotated onto it, see `alignment` below.
inline agi::SetpointVector loadTrajectory(const std::vector<std::vector<double>>& rows,
                                   const double start_time,
                                   const agi::Vector<3>& start_position,
                                   const double start_yaw,
                                   const double source_mass,
                                   const double min_altitude) {
  const double file_start_time = rows.front()[0];
  agi::Vector<3> file_start_position(rows.front()[1], rows.front()[2],
                                     rows.front()[3]);

  // Fly the trajectory in the heading the vehicle is already hovering at.
  //
  // The position is translated onto the takeoff point, so leaving the heading
  // alone would be inconsistent: the datasets start at yaw 0 while the Aeroloop
  // world spawns the vehicle at yaw 90 deg, and the outer loop then answers the
  // first setpoint by slewing 90 deg of yaw at over 3 rad/s.  That transient is
  // not part of the trajectory being scored, and it is violent enough to throw
  // the flight controller's own heading estimate off by more than the MSP
  // pipeline's drift budget.
  const agi::Quaternion file_start_attitude =
    agi::Quaternion(rows.front()[4], rows.front()[5], rows.front()[6],
                    rows.front()[7])
      .normalized();
  const Eigen::AngleAxis<agi::Scalar> alignment(
    start_yaw - std::atan2(2.0 * (file_start_attitude.w() * file_start_attitude.z() + file_start_attitude.x() * file_start_attitude.y()), 1.0 - 2.0 * (file_start_attitude.y() * file_start_attitude.y() + file_start_attitude.z() * file_start_attitude.z())), agi::Vector<3>::UnitZ());

  // Keep the whole trajectory a safe distance above the takeoff point.  The
  // datasets dip to within 0.1 m of their own origin, and translating that
  // straight onto the takeoff altitude would fly the vehicle into the ground.
  double lowest = rows.front()[3];
  for (const std::vector<double>& value : rows)
    lowest = std::min(lowest, value[3]);
  const double lowest_altitude =
    start_position.z() + (lowest - file_start_position.z());
  const double lift = std::max(0.0, min_altitude - lowest_altitude);
  const agi::Vector<3> origin = start_position + agi::Vector<3>(0.0, 0.0, lift);
  if (lift > 0.0) {
    std::cout << "Raising the trajectory by " << lift
              << " m to keep it above the requested ground clearance.\n";
  }

  agi::SetpointVector setpoints;
  std::size_t line_number = 1;
  for (const std::vector<double>& value : rows) {
    ++line_number;
    agi::QuadState state;
    state.setZero();
    state.t = start_time + value[0] - file_start_time;
    state.p = origin + alignment * (agi::Vector<3>(value[1], value[2],
                                                   value[3]) -
                                    file_start_position);
    const agi::Quaternion attitude(value[4], value[5], value[6], value[7]);
    if (!attitude.coeffs().allFinite() || attitude.norm() < 1e-9) {
      throw std::runtime_error("invalid quaternion in trajectory line " +
                               std::to_string(line_number));
    }
    // Everything the CSV expresses in its own world frame is rotated onto the
    // takeoff heading.  `w` and `tau` are body-frame quantities and `u_1..u_4`
    // are rotor forces, so all three are invariant under that rotation.
    state.q(agi::Quaternion(alignment) * attitude.normalized());
    state.v = alignment * agi::Vector<3>(value[8], value[9], value[10]);
    state.w << value[11], value[12], value[13];
    state.a = alignment * agi::Vector<3>(value[14], value[15], value[16]);
    state.tau << value[17], value[18], value[19];
    state.j = alignment * agi::Vector<3>(value[24], value[25], value[26]);
    state.s = alignment * agi::Vector<3>(value[27], value[28], value[29]);

    // CSV u_1..u_4 are rotor forces for the source vehicle.  Express their
    // sum as mass-normalized collective thrust so MPC can apply the same
    // feed-forward acceleration to the Betaloop vehicle.
    const double collective_thrust =
      (value[20] + value[21] + value[22] + value[23]) / source_mass;
    const agi::Command input(state.t, collective_thrust, state.w);
    if (!state.valid() || !input.valid()) {
      throw std::runtime_error("invalid trajectory state in line " +
                               std::to_string(line_number));
    }
    if (!setpoints.empty() && state.t <= setpoints.back().state.t) {
      throw std::runtime_error("trajectory timestamps are not increasing at "
                               "line " + std::to_string(line_number));
    }
    setpoints.emplace_back(state, input);
  }

  if (setpoints.size() < 2) {
    throw std::runtime_error("trajectory must contain at least two samples");
  }
  return setpoints;
}


}
