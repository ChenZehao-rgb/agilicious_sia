#include "AerodynamicsModel.hh"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

using agilicious::aero::AerodynamicsModel;
using agilicious::aero::BodyDragConfig;
using agilicious::aero::Vec3;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kMass = 0.700;
constexpr double kGravity = 9.80665;
constexpr double kMaxOmega = 3108.6;

struct Trim {
  double pitch{0.0};
  double omega{0.0};
  double horizontalResidual{0.0};
  double thrustPerRotor{0.0};
  double torquePerRotor{0.0};
  double drag{0.0};
  double tipMach{0.0};
  bool valid{false};
};

// Gazebo uses FLU axes. Positive pitch here means nose-down, so a positive
// body-z rotor force has a positive world-x component.
Trim atPitch(const AerodynamicsModel &model, double speed, double pitch) {
  const Vec3 velocity{speed * std::cos(pitch), 0.0,
                      speed * std::sin(pitch)};
  const auto body = model.evaluateBodyDrag(velocity, BodyDragConfig{});

  auto loads = [&](double omega, double *worldX, double *worldZ,
                   double *thrust, double *torque, double *drag,
                   double *tipMach) {
    const auto rotor = model.evaluateRotor(omega, velocity);
    const double fx = body.x - 4.0 * rotor.hForce;
    const double fz = body.z + 4.0 * rotor.thrust;
    *worldX = std::cos(pitch) * fx + std::sin(pitch) * fz;
    *worldZ = -std::sin(pitch) * fx + std::cos(pitch) * fz;
    *thrust = rotor.thrust;
    *torque = rotor.torque;
    // Component of body plus in-plane rotor drag opposite world-forward
    // velocity. At trim this equals the forward component of rotor thrust.
    *drag = -(std::cos(pitch) * (body.x - 4.0 * rotor.hForce) +
              std::sin(pitch) * body.z);
    *tipMach = rotor.tipMach;
  };

  double wx = 0.0, wz = 0.0, thrust = 0.0, torque = 0.0;
  double drag = 0.0, mach = 0.0;
  loads(kMaxOmega, &wx, &wz, &thrust, &torque, &drag, &mach);
  if (wz < kMass * kGravity) return {};

  double low = 1.0;
  double high = kMaxOmega;
  for (int i = 0; i < 28; ++i) {
    const double middle = 0.5 * (low + high);
    loads(middle, &wx, &wz, &thrust, &torque, &drag, &mach);
    if (wz < kMass * kGravity)
      low = middle;
    else
      high = middle;
  }
  const double omega = 0.5 * (low + high);
  loads(omega, &wx, &wz, &thrust, &torque, &drag, &mach);
  return {pitch, omega, wx, thrust, torque, drag, mach, true};
}

Trim solveTrim(const AerodynamicsModel &model, double speed) {
  Trim previous = atPitch(model, speed, 0.0);
  if (speed == 0.0) return previous;
  for (int degree = 1; degree <= 85; ++degree) {
    const Trim current = atPitch(model, speed, degree * kPi / 180.0);
    if (current.valid && previous.valid &&
        previous.horizontalResidual * current.horizontalResidual <= 0.0) {
      double low = previous.pitch;
      double high = current.pitch;
      Trim middle;
      for (int i = 0; i < 24; ++i) {
        middle = atPitch(model, speed, 0.5 * (low + high));
        if (!middle.valid || middle.horizontalResidual > 0.0)
          high = 0.5 * (low + high);
        else
          low = 0.5 * (low + high);
      }
      return atPitch(model, speed, 0.5 * (low + high));
    }
    if (current.valid) previous = current;
  }
  return {};
}
}  // namespace

int main() {
  AerodynamicsModel model;
  std::cout << "speed_mps,pitch_deg,rpm,thrust_N_per_rotor,torque_Nm_per_rotor,"
               "forward_drag_N,tip_mach,status\n";
  std::cout << std::fixed << std::setprecision(3);
  for (double speed = 0.0; speed <= 100.0; speed += 5.0) {
    const Trim trim = solveTrim(model, speed);
    std::cout << speed << ',';
    if (!trim.valid) {
      std::cout << "nan,nan,nan,nan,nan,nan,no_trim\n";
      continue;
    }
    std::cout << trim.pitch * 180.0 / kPi << ','
              << trim.omega * 60.0 / (2.0 * kPi) << ','
              << trim.thrustPerRotor << ',' << trim.torquePerRotor << ','
              << trim.drag << ','
              << trim.tipMach << ",trim\n";
  }
}
