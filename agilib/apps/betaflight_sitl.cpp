#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <gz/msgs/model.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>

#include "agilib/bridge/betaflight_udp/betaflight_udp_bridge.hpp"
#include "agilib/bridge/betaflight_udp/betaflight_udp_bridge_params.hpp"
#include "agilib/math/gravity.hpp"
#include "agilib/pilot/pilot.hpp"
#include "agilib/pilot/pilot_params.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/utils/timer.hpp"
#include "agilib/utils/yaml.hpp"

namespace {

using SteadyClock = std::chrono::steady_clock;

volatile std::sig_atomic_t stop_requested = 0;

void signalHandler(int) { stop_requested = 1; }

struct Options {
  std::string pilot_config;
  std::string params_dir;
  std::string quad;
  std::string bridge_config;
  std::string odom_topic{"/model/iris/odometry"};
  std::string joint_topic{"/world/betaloop_demo/model/iris/joint_state"};
  std::string trajectory;
  std::string log_file;
  // Zero means "estimate the source mass from the trajectory itself".
  double trajectory_source_mass{0.0};
  double ground_clearance{0.8};
  double disarmed_seconds{6.0};
  double prearm_seconds{2.0};
  double duration{0.0};
  bool arm{false};
  bool help{false};
};

void printUsage(const char* executable) {
  std::cout
    << "Usage: " << executable << " [options]\n\n"
    << "Required:\n"
    << "  --pilot-config FILE   Agilib Pilot configuration\n"
    << "  --params-dir DIR      Agilib parameter directory\n"
    << "  --quad FILE           Quadrotor parameter file\n"
    << "  --bridge-config FILE  Betaflight UDP bridge configuration\n\n"
    << "Optional:\n"
    << "  --odom-topic TOPIC    Gazebo odometry topic"
       " (default: /model/iris/odometry)\n"
    << "  --joint-topic TOPIC   Gazebo joint-state diagnostic topic\n"
    << "  --trajectory FILE     CSV trajectory to run after takeoff\n"
    << "  --trajectory-source-mass KG Mass used to generate CSV motor thrusts"
       " (default: estimate it from the file)\n"
    << "  --ground-clearance M  Minimum trajectory altitude above the takeoff"
       " point (default: 0.8)\n"
    << "  --log FILE            Write a reference-vs-state CSV for analysis\n"
    << "  --arm                 Arm after odometry becomes valid, then take off\n"
    << "  --disarmed-seconds SEC Keep AUX1 low before arming (default: 6.0)\n"
    << "  --prearm-seconds SEC  Armed low-throttle time before takeoff"
       " (default: 2.0)\n"
    << "  --duration SEC        Stop after SEC seconds; 0 runs until interrupted\n"
    << "  -h, --help            Show this help\n\n"
    << "Without --arm, the program only publishes disarmed, low-throttle RC "
       "packets.\n";
}

double parseNumber(const std::string& option, const std::string& value) {
  std::size_t parsed = 0;
  double result = 0.0;
  try {
    result = std::stod(value, &parsed);
  } catch (const std::exception&) {
    throw std::runtime_error(option + " expects a number, got: " + value);
  }

  if (parsed != value.size() || !std::isfinite(result)) {
    throw std::runtime_error(option + " expects a finite number, got: " +
                             value);
  }
  return result;
}

Options parseOptions(int argc, char** argv) {
  Options options;

  auto valueAfter = [&](int& index, const std::string& option) -> std::string {
    if (index + 1 >= argc) {
      throw std::runtime_error("missing value after " + option);
    }
    return argv[++index];
  };

  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    if (argument == "-h" || argument == "--help") {
      options.help = true;
    } else if (argument == "--pilot-config") {
      options.pilot_config = valueAfter(i, argument);
    } else if (argument == "--params-dir") {
      options.params_dir = valueAfter(i, argument);
    } else if (argument == "--quad") {
      options.quad = valueAfter(i, argument);
    } else if (argument == "--bridge-config") {
      options.bridge_config = valueAfter(i, argument);
    } else if (argument == "--odom-topic") {
      options.odom_topic = valueAfter(i, argument);
    } else if (argument == "--joint-topic") {
      options.joint_topic = valueAfter(i, argument);
    } else if (argument == "--trajectory") {
      options.trajectory = valueAfter(i, argument);
    } else if (argument == "--trajectory-source-mass") {
      options.trajectory_source_mass =
        parseNumber(argument, valueAfter(i, argument));
    } else if (argument == "--ground-clearance") {
      options.ground_clearance = parseNumber(argument, valueAfter(i, argument));
    } else if (argument == "--log") {
      options.log_file = valueAfter(i, argument);
    } else if (argument == "--disarmed-seconds") {
      options.disarmed_seconds =
        parseNumber(argument, valueAfter(i, argument));
    } else if (argument == "--prearm-seconds") {
      options.prearm_seconds =
        parseNumber(argument, valueAfter(i, argument));
    } else if (argument == "--duration") {
      options.duration = parseNumber(argument, valueAfter(i, argument));
    } else if (argument == "--arm") {
      options.arm = true;
    } else {
      throw std::runtime_error("unknown option: " + argument);
    }
  }

  if (options.help) return options;

  if (options.pilot_config.empty())
    throw std::runtime_error("--pilot-config is required");
  if (options.params_dir.empty())
    throw std::runtime_error("--params-dir is required");
  if (options.quad.empty())
    throw std::runtime_error("--quad is required");
  if (options.bridge_config.empty())
    throw std::runtime_error("--bridge-config is required");
  if (options.odom_topic.empty())
    throw std::runtime_error("--odom-topic must not be empty");
  if (options.joint_topic.empty())
    throw std::runtime_error("--joint-topic must not be empty");
  if (options.prearm_seconds < 0.0)
    throw std::runtime_error("--prearm-seconds must be >= 0");
  if (options.disarmed_seconds < 0.0)
    throw std::runtime_error("--disarmed-seconds must be >= 0");
  if (options.duration < 0.0)
    throw std::runtime_error("--duration must be >= 0");
  if (options.trajectory_source_mass < 0.0)
    throw std::runtime_error("--trajectory-source-mass must be > 0");
  if (options.ground_clearance < 0.0)
    throw std::runtime_error("--ground-clearance must be >= 0");
  if (!options.trajectory.empty() && !options.arm)
    throw std::runtime_error("--trajectory requires --arm");

  return options;
}

std::filesystem::path resolvePath(const std::filesystem::path& base,
                                  const std::string& value) {
  const std::filesystem::path path(value);
  return path.is_absolute() ? path : base / path;
}

std::filesystem::path resolveQuadFromYaml(
  const std::filesystem::path& params_dir, const std::string& value) {
  const std::filesystem::path path(value);
  if (path.is_absolute()) return path;
  if (std::filesystem::is_regular_file(params_dir / path))
    return params_dir / path;
  return params_dir / "quads" / path;
}

void requireFile(const std::filesystem::path& path,
                 const std::string& option) {
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error(option + " is not a readable file: " +
                             path.string());
  }
}

std::vector<double> parseCsvRow(const std::string& line,
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

std::vector<std::vector<double>> readTrajectoryRows(
  const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("could not open trajectory: " + path.string());
  }

  std::string line;
  if (!std::getline(file, line) ||
      line != "t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z,"
              "a_lin_x,a_lin_y,a_lin_z,a_rot_x,a_rot_y,a_rot_z,u_1,u_2,"
              "u_3,u_4,jerk_x,jerk_y,jerk_z,snap_x,snap_y,snap_z") {
    throw std::runtime_error("unsupported trajectory CSV header: " +
                             path.string());
  }

  std::vector<std::vector<double>> rows;
  std::size_t line_number = 1;
  while (std::getline(file, line)) {
    ++line_number;
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
double estimateSourceMass(const std::vector<std::vector<double>>& rows) {
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

agi::SetpointVector loadTrajectory(const std::vector<std::vector<double>>& rows,
                                   const double start_time,
                                   const agi::Vector<3>& start_position,
                                   const double source_mass,
                                   const double min_altitude) {
  const double file_start_time = rows.front()[0];
  agi::Vector<3> file_start_position(rows.front()[1], rows.front()[2],
                                     rows.front()[3]);

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
    state.p = origin + (agi::Vector<3>(value[1], value[2], value[3]) -
                        file_start_position);
    const agi::Quaternion attitude(value[4], value[5], value[6], value[7]);
    if (!attitude.coeffs().allFinite() || attitude.norm() < 1e-9) {
      throw std::runtime_error("invalid quaternion in trajectory line " +
                               std::to_string(line_number));
    }
    state.q(attitude.normalized());
    state.v << value[8], value[9], value[10];
    state.w << value[11], value[12], value[13];
    state.a << value[14], value[15], value[16];
    state.tau << value[17], value[18], value[19];
    state.j << value[24], value[25], value[26];
    state.s << value[27], value[28], value[29];

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

/// Reference-versus-state recorder, used by betaflight_sitl/validate.py.
class FlightLog {
 public:
  explicit FlightLog(const std::filesystem::path& path) : file_(path) {
    if (!file_) {
      throw std::runtime_error("could not open log file: " + path.string());
    }
    file_ << "t,mode,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z,"
             "ref_p_x,ref_p_y,ref_p_z,ref_v_x,ref_v_y,ref_v_z,"
             "ref_w_x,ref_w_y,ref_w_z,ref_thrust,"
             "cmd_thrust,cmd_w_x,cmd_w_y,cmd_w_z,"
             "rc_a,rc_e,rc_t,rc_r,rc_aux1,has_reference,in_trajectory\n";
    file_ << std::setprecision(9);
  }


  /// \param state vehicle state with `v` already rotated into the world frame,
  ///        matching the frame the reference setpoints use.
  void write(const double time, const char* mode, const agi::QuadState& state,
             const agi::SetpointVector& reference, const agi::Command& command,
             const agi::BetaflightUdpBridge::Channels& channels) {
    const bool has_reference = !reference.empty();
    const agi::QuadState& target =
      has_reference ? reference.front().state : state;
    const agi::Command& target_input =
      has_reference ? reference.front().input : command;

    if (!std::isfinite(start_time_)) start_time_ = time;
    file_ << (time - start_time_) << ',' << mode;
    write(state.p);
    file_ << ',' << state.q().w() << ',' << state.q().x() << ','
          << state.q().y() << ',' << state.q().z();
    write(state.v);
    write(state.w);
    write(target.p);
    write(target.v);
    write(target.w);
    file_ << ',' << (target_input.isRatesThrust() ? target_input.collective_thrust
                                                  : 0.0);
    file_ << ','
          << (command.isRatesThrust() ? command.collective_thrust : 0.0);
    write(command.isRatesThrust() ? command.omega : agi::Vector<3>::Zero());
    for (std::size_t i = 0; i < 5; ++i) file_ << ',' << channels[i];
    file_ << ',' << (has_reference ? 1 : 0) << ','
          << (time >= trajectory_start_ && time <= trajectory_end_ ? 1 : 0)
          << '\n';
  }

  /// Mark the window in which the sampled CSV trajectory is the reference, so
  /// scoring does not average takeoff and the trailing hover into the result.
  void setTrajectoryWindow(const double start, const double end) {
    trajectory_start_ = start;
    trajectory_end_ = end;
  }

 private:
  void write(const agi::Vector<3>& vector) {
    file_ << ',' << vector.x() << ',' << vector.y() << ',' << vector.z();
  }

  std::ofstream file_;
  double start_time_{NAN};
  double trajectory_start_{std::numeric_limits<double>::infinity()};
  double trajectory_end_{-std::numeric_limits<double>::infinity()};
};

struct LatestOdometry {
  std::mutex mutex;
  gz::msgs::Odometry message;
  SteadyClock::time_point received_at{};
  bool received{false};

  void update(const gz::msgs::Odometry& odometry) {
    std::lock_guard<std::mutex> lock(mutex);
    message = odometry;
    received_at = SteadyClock::now();
    received = true;
  }

  bool snapshot(gz::msgs::Odometry* odometry,
                SteadyClock::time_point* receipt_time) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!received) return false;
    *odometry = message;
    *receipt_time = received_at;
    return true;
  }
};

struct LatestRotorVelocity {
  std::mutex mutex;
  std::array<double, 4> velocity{};
  bool received{false};

  void update(const gz::msgs::Model& model) {
    static const std::array<std::string, 4> names{
      "rotor_0_joint", "rotor_1_joint", "rotor_2_joint", "rotor_3_joint"};
    std::lock_guard<std::mutex> lock(mutex);
    bool found_any = false;
    for (const auto& joint : model.joint()) {
      if (!joint.has_axis1()) continue;
      for (std::size_t i = 0; i < names.size(); ++i) {
        if (joint.name().find(names[i]) != std::string::npos) {
          velocity[i] = joint.axis1().velocity();
          found_any = true;
        }
      }
    }
    received = received || found_any;
  }

  bool snapshot(std::array<double, 4>* output) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!received) return false;
    *output = velocity;
    return true;
  }
};

/// Simulated time carried by a Gazebo odometry message, or NaN if unstamped.
///
/// The whole control loop is driven off this rather than off the wall clock.
/// Gazebo does not always hold a real-time factor of one -- the GUI alone costs
/// intermittent dips to 0.3 -- and a wall-clock reference would then advance
/// faster than the vehicle can fly, which loses the trajectory outright instead
/// of merely simulating it slowly.
double simulatedTime(const gz::msgs::Odometry& odometry) {
  if (!odometry.has_header() || !odometry.header().has_stamp()) return NAN;
  const auto& stamp = odometry.header().stamp();
  const double seconds =
    static_cast<double>(stamp.sec()) + 1e-9 * static_cast<double>(stamp.nsec());
  return seconds > 0.0 ? seconds : NAN;
}

bool toQuadState(const gz::msgs::Odometry& odometry, const double time,
                 agi::QuadState* state, std::string* error) {
  if (!odometry.has_pose() || !odometry.has_twist()) {
    *error = "odometry is missing pose or twist";
    return false;
  }

  const auto& position = odometry.pose().position();
  const auto& attitude = odometry.pose().orientation();
  const auto& velocity = odometry.twist().linear();
  const auto& angular_velocity = odometry.twist().angular();

  const agi::Quaternion quaternion(attitude.w(), attitude.x(), attitude.y(),
                                   attitude.z());
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-9) {
    *error = "odometry contains an invalid quaternion";
    return false;
  }

  state->setZero();
  state->t = time;
  state->p << position.x(), position.y(), position.z();
  state->q(quaternion.normalized());
  // Gazebo OdometryPublisher expresses Twist in the child (body) frame.
  // PilotParams::velocity_in_bodyframe should therefore be true for this app.
  state->v << velocity.x(), velocity.y(), velocity.z();
  state->w << angular_velocity.x(), angular_velocity.y(),
    angular_velocity.z();

  if (!state->valid()) {
    *error = "odometry contains non-finite state values";
    return false;
  }
  return true;
}

void sendSafe(const std::shared_ptr<agi::BetaflightUdpBridge>& bridge) {
  if (!bridge) return;
  bridge->deactivate();
  bridge->reset();
  for (int i = 0; i < 3; ++i) {
    bridge->send(agi::Command(agi::ChronoTime()));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void stopSafely(agi::Pilot* pilot,
                const std::shared_ptr<agi::BetaflightUdpBridge>& bridge) {
  if (pilot) pilot->off();
  sendSafe(bridge);
}

struct SafeStopGuard {
  agi::Pilot* pilot;
  std::shared_ptr<agi::BetaflightUdpBridge> bridge;

  ~SafeStopGuard() { stopSafely(pilot, bridge); }
};

void printStatus(const char* mode, const agi::QuadState& state,
                 const agi::Command& command,
                 const agi::BetaflightUdpBridge::Channels& channels,
                 const double odom_age,
                 const std::array<double, 4>& rotor_velocity,
                 const bool have_rotor_velocity) {
  std::cout << std::fixed << std::setprecision(3) << '[' << mode << "] "
            << "p=[" << state.p.transpose() << "] "
            << "v_body=[" << state.v.transpose() << "] "
            << "w=[" << state.w.transpose() << "] ";

  if (command.valid()) {
    std::cout << "thrust=" << command.collective_thrust << " omega_cmd=["
              << command.omega.transpose() << "] ";
  } else {
    std::cout << "command=unavailable ";
  }
  std::cout << "RC(AETR,AUX1)=[" << channels[0] << ' ' << channels[1] << ' '
            << channels[2] << ' ' << channels[3] << ' ' << channels[4]
            << "] ";
  if (have_rotor_velocity) {
    std::cout << "rotor_w=[" << rotor_velocity[0] << ' ' << rotor_velocity[1]
              << ' ' << rotor_velocity[2] << ' ' << rotor_velocity[3]
              << "] ";
  }
  std::cout << "odom_age=" << 1e3 * odom_age << " ms\n";
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  try {
    options = parseOptions(argc, argv);
  } catch (const std::exception& exception) {
    std::cerr << "Argument error: " << exception.what() << "\n\n";
    printUsage(argv[0]);
    return 2;
  }

  if (options.help) {
    printUsage(argv[0]);
    return 0;
  }

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  std::shared_ptr<agi::BetaflightUdpBridge> bridge;
  std::unique_ptr<agi::Pilot> pilot;
  int exit_code = 0;

  try {
    const std::filesystem::path params_dir =
      std::filesystem::absolute(options.params_dir);
    if (!std::filesystem::is_directory(params_dir)) {
      throw std::runtime_error("--params-dir is not a directory: " +
                               params_dir.string());
    }

    const std::filesystem::path pilot_config =
      resolvePath(params_dir, options.pilot_config);
    const std::filesystem::path quad = resolvePath(params_dir, options.quad);
    const std::filesystem::path bridge_config =
      resolvePath(params_dir, options.bridge_config);
    const std::filesystem::path trajectory = options.trajectory.empty()
                                               ? std::filesystem::path{}
                                               : std::filesystem::absolute(
                                                   options.trajectory);
    requireFile(pilot_config, "--pilot-config");
    requireFile(quad, "--quad");
    requireFile(bridge_config, "--bridge-config");
    if (!trajectory.empty()) requireFile(trajectory, "--trajectory");

    // Read and validate the trajectory before anything is armed, so a bad file
    // cannot strand the vehicle mid-flight.
    std::vector<std::vector<double>> trajectory_rows;
    double source_mass = options.trajectory_source_mass;
    if (!trajectory.empty()) {
      trajectory_rows = readTrajectoryRows(trajectory);
      if (source_mass <= 0.0) {
        source_mass = estimateSourceMass(trajectory_rows);
        std::cout << "Estimated trajectory source mass: " << source_mass
                  << " kg\n";
      }
    }

    std::unique_ptr<FlightLog> flight_log;
    if (!options.log_file.empty()) {
      flight_log =
        std::make_unique<FlightLog>(options.log_file);
      std::cout << "Logging reference vs. state to " << options.log_file
                << '\n';
    }

    std::unique_ptr<agi::PilotParams> pilot_params;
    std::string yaml_quad;
    const agi::Yaml pilot_yaml(pilot_config);
    if (pilot_yaml["quadrotor"].getIfDefined(yaml_quad)) {
      const std::filesystem::path yaml_quad_path =
        resolveQuadFromYaml(params_dir, yaml_quad);
      requireFile(yaml_quad_path, "pilot config quadrotor");
      if (std::filesystem::weakly_canonical(yaml_quad_path) !=
          std::filesystem::weakly_canonical(quad)) {
        throw std::runtime_error(
          "--quad conflicts with pilot config quadrotor: " +
          yaml_quad_path.string());
      }
      // PilotParams intentionally rejects setting the same quad both through
      // YAML and its constructor override, so use the already-validated YAML
      // entry in this case.
      pilot_params =
        std::make_unique<agi::PilotParams>(pilot_config, params_dir);
    } else {
      pilot_params =
        std::make_unique<agi::PilotParams>(pilot_config, params_dir, quad);
    }
    // Shared with the Pilot's clock below; updated once per control cycle from
    // the odometry stamp.
    std::atomic<double> latest_sim_time{0.0};

    agi::BetaflightUdpBridgeParams bridge_params;
    if (!bridge_params.load(bridge_config)) {
      throw std::runtime_error("invalid Betaflight UDP bridge parameters: " +
                               bridge_config.string());
    }

    // The Pilot stamps its own references -- takeoff polynomial, hover, the
    // sampled trajectory -- with this clock, and the sampler then looks them up
    // by the state's timestamp.  Both have to be simulated time or the lookup
    // lands somewhere else entirely.  The bridge keeps the wall clock: its
    // watchdog is a real safety timer, not part of the simulated dynamics.
    const agi::TimeFunction simulated_clock =
      [&latest_sim_time]() -> agi::Scalar {
      return latest_sim_time.load(std::memory_order_relaxed);
    };
    pilot = std::make_unique<agi::Pilot>(*pilot_params, simulated_clock);
    bridge = std::make_shared<agi::BetaflightUdpBridge>(bridge_params,
                                                        agi::ChronoTime);
    if (!pilot->registerExternalBridge(bridge)) {
      throw std::runtime_error("failed to register Betaflight UDP bridge");
    }

    LatestOdometry latest_odometry;
    LatestRotorVelocity latest_rotor_velocity;
    gz::transport::Node node;
    const bool subscribed = node.Subscribe<gz::msgs::Odometry>(
      options.odom_topic, [&latest_odometry](const gz::msgs::Odometry& msg) {
        latest_odometry.update(msg);
      });
    if (!subscribed) {
      throw std::runtime_error("failed to subscribe to Gazebo topic: " +
                               options.odom_topic);
    }
    const bool subscribed_joints = node.Subscribe<gz::msgs::Model>(
      options.joint_topic,
      [&latest_rotor_velocity](const gz::msgs::Model& msg) {
        latest_rotor_velocity.update(msg);
      });
    if (!subscribed_joints) {
      throw std::runtime_error("failed to subscribe to Gazebo topic: " +
                               options.joint_topic);
    }
    // This is deliberately declared after the Gazebo node.  C++ destroys
    // locals in reverse order, so AUX1 is lowered before transport teardown
    // can delay shutdown.
    SafeStopGuard safe_stop_guard{pilot.get(), bridge};

    std::cout << "Subscribed to " << options.odom_topic << " at 100 Hz.\n";
    if (options.arm) {
      std::cout << "ARMING ENABLED: valid odometry will be followed by "
                << options.disarmed_seconds << " s with AUX1 low, then "
                << options.prearm_seconds
                << " s of armed low throttle, then Pilot::start().\n";
    } else {
      std::cout << "SAFE MONITOR MODE: --arm was not supplied; Betaflight "
                   "will remain disarmed.\n";
    }

    constexpr auto control_period = std::chrono::milliseconds(10);
    constexpr double stale_limit_seconds = 0.25;
    constexpr auto startup_timeout = std::chrono::seconds(30);
    const SteadyClock::time_point wait_started = SteadyClock::now();
    SteadyClock::time_point next_cycle = wait_started;
    double first_sim_time = NAN;
    double armed_from_sim = NAN;
    double prearm_from_sim = NAN;
    SteadyClock::time_point next_status = wait_started;
    bool have_valid_state = false;
    bool enabled = false;
    bool flight_started = false;

    while (!stop_requested) {
      next_cycle += control_period;
      const SteadyClock::time_point steady_now = SteadyClock::now();

      gz::msgs::Odometry odometry;
      SteadyClock::time_point receipt_time;
      if (!latest_odometry.snapshot(&odometry, &receipt_time)) {
        bridge->send(agi::Command(agi::ChronoTime()));
        if (steady_now >= next_status) {
          std::cout << "[WAIT] No odometry received on "
                    << options.odom_topic << "\n";
          next_status = steady_now + std::chrono::seconds(1);
        }
        if (steady_now - wait_started > startup_timeout) {
          std::cerr << "No odometry received for 30 s; stopping safely.\n";
          exit_code = 1;
          break;
        }
        std::this_thread::sleep_until(next_cycle);
        continue;
      }

      const double odom_age =
        std::chrono::duration<double>(steady_now - receipt_time).count();
      if (odom_age > stale_limit_seconds) {
        std::cerr << "Odometry is stale (" << odom_age
                  << " s > 0.25 s); stopping safely.\n";
        exit_code = 1;
        break;
      }

      // Simulated time, so a real-time factor below one slows the reference
      // down with the vehicle instead of running away from it.  The loop still
      // wakes on the wall clock, which keeps the bridge watchdog fed.
      const double sim_time = simulatedTime(odometry);
      if (!std::isfinite(sim_time)) {
        std::cerr << "Odometry carries no simulated-time stamp; the overlay "
                     "model must publish it. Stopping safely.\n";
        exit_code = 1;
        break;
      }
      if (!std::isfinite(first_sim_time)) first_sim_time = sim_time;
      const double sim_elapsed = sim_time - first_sim_time;
      latest_sim_time.store(sim_time, std::memory_order_relaxed);

      agi::QuadState state;
      std::string conversion_error;
      if (!toQuadState(odometry, sim_time, &state, &conversion_error)) {
        std::cerr << "Invalid Gazebo odometry: " << conversion_error
                  << "; stopping safely.\n";
        exit_code = 1;
        break;
      }

      pilot->odometryCallback(state);
      pilot->guardOdometryCallback(state);
      if (!have_valid_state) {
        have_valid_state = true;
        armed_from_sim = sim_elapsed;
      }

      if (options.arm) {
        const double disarmed_elapsed = sim_elapsed - armed_from_sim;
        if (!enabled && disarmed_elapsed >= options.disarmed_seconds) {
          pilot->enable(true);
          if (!pilot->enabled()) {
            std::cerr << "Betaflight UDP bridge did not activate.\n";
            exit_code = 1;
            break;
          }
          enabled = true;
          prearm_from_sim = sim_elapsed;
        }

        if (enabled) {
          // With no reference installed, Pipeline emits zero collective
          // thrust while the bridge keeps the configured ARM channel high.
          pilot->runPipeline(sim_time);
          if (bridge->locked() || !bridge->active() || !pilot->enabled()) {
            std::cerr << "Betaflight UDP bridge lost its active state"
                      << (bridge->locked() ? " after watchdog lockout" : "")
                      << "; stopping safely.\n";
            exit_code = 1;
            break;
          }
        } else {
          // Betaflight must observe ARM low after RX starts, and its default
          // five-second power-on arming grace must expire before takeoff.
          bridge->send(agi::Command(sim_time));
        }

        if (enabled && !flight_started &&
            sim_elapsed - prearm_from_sim >= options.prearm_seconds) {
          if (!pilot->start()) {
            std::cerr << "Pilot::start() failed; stopping safely.\n";
            exit_code = 1;
            break;
          }
          if (!trajectory.empty()) {
            const double takeoff_duration =
              pilot_params->takeoff_heigth_ / pilot_params->start_land_speed_;
            agi::Vector<3> trajectory_start = state.p;
            trajectory_start.z() += pilot_params->takeoff_heigth_;
            const agi::SetpointVector setpoints = loadTrajectory(
              trajectory_rows, sim_time + takeoff_duration, trajectory_start,
              source_mass, state.p.z() + options.ground_clearance);
            if (!pilot->addSampledTrajectory(setpoints)) {
              std::cerr << "Could not append sampled trajectory; stopping "
                           "safely.\n";
              exit_code = 1;
              break;
            }
            std::cout << "Queued trajectory " << trajectory << " ("
                      << setpoints.size() << " samples, "
                      << setpoints.back().state.t - setpoints.front().state.t
                      << " s) after takeoff.\n";
            if (flight_log) {
              flight_log->setTrajectoryWindow(setpoints.front().state.t,
                                              setpoints.back().state.t);
            }
          }
          flight_started = true;
          std::cout << "Takeoff reference started.\n";
        }
      } else {
        bridge->send(agi::Command(sim_time));
      }

      const char* mode = !options.arm
                           ? "SAFE"
                           : (!enabled ? "DISARM"
                                       : (flight_started ? "FLIGHT"
                                                         : "PREARM"));
      if (flight_log) {
        agi::QuadState logged = state;
        // The pipeline works in the world frame; match it so the log can be
        // differenced against the reference directly.
        logged.v = state.q() * state.v;
        flight_log->write(sim_time, mode, logged,
                          pilot->getReferenceSetpoints(), pilot->getCommand(),
                          bridge->lastChannels());
      }

      if (steady_now >= next_status) {
        std::array<double, 4> rotor_velocity{};
        const bool have_rotor_velocity =
          latest_rotor_velocity.snapshot(&rotor_velocity);
        printStatus(mode, state,
                    options.arm ? pilot->getCommand()
                                : agi::Command(sim_time),
                    bridge->lastChannels(),
                    odom_age, rotor_velocity, have_rotor_velocity);
        next_status = steady_now + std::chrono::seconds(1);
      }

      if (options.duration > 0.0 &&
          sim_elapsed - armed_from_sim >= options.duration) {
        std::cout << "Requested duration reached; stopping safely.\n";
        break;
      }

      if (next_cycle < SteadyClock::now()) {
        next_cycle = SteadyClock::now();
      } else {
        std::this_thread::sleep_until(next_cycle);
      }
    }
  } catch (const std::exception& exception) {
    std::cerr << "Fatal error: " << exception.what() << '\n';
    exit_code = 1;
  }

  std::cout << "Betaflight SITL bridge stopped with disarmed, low-throttle RC.\n";
  return exit_code;
}
