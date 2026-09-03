#include "AerodynamicsModel.hh"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace agilicious::aero {
namespace {

template<typename T>
void loadIfPresent(const std::shared_ptr<const sdf::Element> &sdf,
                   const std::string &name, T *value) {
  if (sdf->HasElement(name)) *value = sdf->Get<T>(name);
}

gz::math::Vector3d toGz(const Vec3 &v) { return {v.x, v.y, v.z}; }
Vec3 fromGz(const gz::math::Vector3d &v) { return {v.X(), v.Y(), v.Z()}; }

struct RotorBinding {
  gz::sim::Entity joint{gz::sim::kNullEntity};
  gz::sim::Link link{gz::sim::kNullEntity};
  Vec3 offset;
  double direction{1.0};
  Vec3 cachedForce;
  Vec3 cachedTorque;
  RotorResult cachedResult;
};

class AgiliciousAerodynamicsPlugin final
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    model_ = gz::sim::Model(entity);
    baseEntity_ = model_.CanonicalLink(ecm);
    baseLink_ = gz::sim::Link(baseEntity_);
    if (!model_.Valid(ecm) || !baseLink_.Valid(ecm)) {
      gzerr << "[AgiliciousAero] invalid model or canonical link\n";
      return;
    }

    RotorConfig rotorConfig = AerodynamicsModel::defaultRotorConfig();
    loadIfPresent(sdf, "air_density", &rotorConfig.airDensity);
    loadIfPresent(sdf, "speed_of_sound", &rotorConfig.speedOfSound);
    loadIfPresent(sdf, "propeller_radius", &rotorConfig.radius);
    loadIfPresent(sdf, "propeller_pitch", &rotorConfig.pitch);
    loadIfPresent(sdf, "hub_radius", &rotorConfig.hubRadius);
    loadIfPresent(sdf, "chord_root", &rotorConfig.chordRoot);
    loadIfPresent(sdf, "chord_tip", &rotorConfig.chordTip);
    loadIfPresent(sdf, "num_blades", &rotorConfig.blades);
    loadIfPresent(sdf, "h_force_scale", &rotorConfig.hForceScale);
    modelCore_ = std::make_unique<AerodynamicsModel>(rotorConfig);

    bodyDrag_.airDensity = rotorConfig.airDensity;
    gz::math::Vector3d cdArea(bodyDrag_.cdArea.x, bodyDrag_.cdArea.y,
                              bodyDrag_.cdArea.z);
    loadIfPresent(sdf, "body_cd_area", &cdArea);
    bodyDrag_.cdArea = fromGz(cdArea);
    loadIfPresent(sdf, "wind_velocity", &windWorld_);
    loadIfPresent(sdf, "telemetry_rate", &telemetryRate_);
    loadIfPresent(sdf, "aerodynamics_rate", &aerodynamicsRate_);
    std::string telemetryTopic{"/model/iris/aerodynamics"};
    loadIfPresent(sdf, "telemetry_topic", &telemetryTopic);
    telemetryPublisher_ = node_.Advertise<gz::msgs::StringMsg>(telemetryTopic);

    auto mutableSdf = std::const_pointer_cast<sdf::Element>(sdf);
    if (sdf->HasElement("rotor")) {
      auto rotorSdf = mutableSdf->GetElement("rotor");
      while (rotorSdf) {
        const std::string jointName = rotorSdf->Get<std::string>("joint_name");
        const std::string linkName = rotorSdf->Get<std::string>("link_name");
        RotorBinding rotor;
        rotor.joint = model_.JointByName(ecm, jointName);
        rotor.link = gz::sim::Link(model_.LinkByName(ecm, linkName));
        const auto position = rotorSdf->Get<gz::math::Vector3d>("position");
        rotor.offset = fromGz(position);
        const std::string direction = rotorSdf->Get<std::string>("direction");
        rotor.direction = direction == "cw" ? -1.0 : 1.0;
        if (rotor.joint == gz::sim::kNullEntity || !rotor.link.Valid(ecm)) {
          gzerr << "[AgiliciousAero] missing rotor joint/link " << jointName
                << '/' << linkName << "\n";
          rotors_.clear();
          return;
        }
        if (!ecm.Component<gz::sim::components::JointVelocity>(rotor.joint))
          ecm.CreateComponent(rotor.joint,
                              gz::sim::components::JointVelocity());
        rotors_.push_back(rotor);
        rotorSdf = rotorSdf->GetNextElement("rotor");
      }
    }

    if (!ecm.Component<gz::sim::components::WorldLinearVelocity>(baseEntity_))
      ecm.CreateComponent(baseEntity_,
                          gz::sim::components::WorldLinearVelocity());
    if (!ecm.Component<gz::sim::components::WorldAngularVelocity>(baseEntity_))
      ecm.CreateComponent(baseEntity_,
                          gz::sim::components::WorldAngularVelocity());
    configured_ = rotors_.size() == 4;
    gzlog << "[AgiliciousAero] Qianfeng 5136 three-blade model, "
          << rotors_.size() << " rotors, CdA=" << cdArea << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (!configured_ || info.paused || info.dt.count() <= 0) return;
    const auto *linear =
      ecm.Component<gz::sim::components::WorldLinearVelocity>(baseEntity_);
    const auto *angular =
      ecm.Component<gz::sim::components::WorldAngularVelocity>(baseEntity_);
    if (!linear || !angular) return;

    const gz::math::Pose3d pose = gz::sim::worldPose(baseEntity_, ecm);
    const gz::math::Vector3d airWorld = linear->Data() - windWorld_;
    const gz::math::Vector3d velocityBody =
      pose.Rot().RotateVectorReverse(airWorld);
    const gz::math::Vector3d angularBody =
      pose.Rot().RotateVectorReverse(angular->Data());
    const double simTime = std::chrono::duration<double>(info.simTime).count();
    if (lastAerodynamicsTime_ < 0.0 ||
        simTime - lastAerodynamicsTime_ >= 1.0 / aerodynamicsRate_) {
      cachedBodyDrag_ =
        modelCore_->evaluateBodyDrag(fromGz(velocityBody), bodyDrag_);
      cachedTotalForce_ = cachedBodyDrag_;
      cachedTotalTorque_ = {};
      for (auto &rotor : rotors_) {
        const auto *jointVelocity =
          ecm.Component<gz::sim::components::JointVelocity>(rotor.joint);
        const double signedOmega = jointVelocity && !jointVelocity->Data().empty()
          ? jointVelocity->Data()[0] : 0.0;
        const gz::math::Vector3d localVelocity = velocityBody +
          angularBody.Cross(toGz(rotor.offset));
        rotor.cachedResult =
          modelCore_->evaluateRotor(std::abs(signedOmega), fromGz(localVelocity));
        const double vh = std::hypot(localVelocity.X(), localVelocity.Y());
        rotor.cachedForce = {
          vh > 1e-6 ? -rotor.cachedResult.hForce * localVelocity.X() / vh : 0.0,
          vh > 1e-6 ? -rotor.cachedResult.hForce * localVelocity.Y() / vh : 0.0,
          rotor.cachedResult.thrust};
        rotor.cachedTorque = {0.0, 0.0,
                              -rotor.direction * rotor.cachedResult.torque};
        cachedTotalForce_.x += rotor.cachedForce.x;
        cachedTotalForce_.y += rotor.cachedForce.y;
        cachedTotalForce_.z += rotor.cachedForce.z;
        cachedTotalTorque_.x += rotor.offset.y * rotor.cachedForce.z;
        cachedTotalTorque_.y -= rotor.offset.x * rotor.cachedForce.z;
        cachedTotalTorque_.z += rotor.offset.x * rotor.cachedForce.y -
                                rotor.offset.y * rotor.cachedForce.x +
                                rotor.cachedTorque.z;
      }
      lastAerodynamicsTime_ = simTime;
    }

    // Wrenches last one physics step, so re-apply the cached 200 Hz solution.
    baseLink_.AddWorldForce(
      ecm, pose.Rot().RotateVector(toGz(cachedBodyDrag_)));
    std::vector<RotorResult> results;
    results.reserve(rotors_.size());
    for (const auto &rotor : rotors_) {
      rotor.link.AddWorldWrench(
        ecm, pose.Rot().RotateVector(toGz(rotor.cachedForce)),
        pose.Rot().RotateVector(toGz(rotor.cachedTorque)));
      results.push_back(rotor.cachedResult);
    }

    if (telemetryRate_ > 0.0 &&
        simTime - lastTelemetryTime_ >= 1.0 / telemetryRate_) {
      publishTelemetry(simTime, velocityBody, cachedBodyDrag_,
                       cachedTotalForce_, cachedTotalTorque_, results);
      lastTelemetryTime_ = simTime;
    }
  }

 private:
  void publishTelemetry(double time, const gz::math::Vector3d &velocity,
                        const Vec3 &bodyDrag, const Vec3 &force,
                        const Vec3 &torque,
                        const std::vector<RotorResult> &rotors) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(5)
           << "{\"time\":" << time
           << ",\"air_velocity_body\":[" << velocity.X() << ','
           << velocity.Y() << ',' << velocity.Z() << "]"
           << ",\"body_drag_body\":[" << bodyDrag.x << ',' << bodyDrag.y
           << ',' << bodyDrag.z << "]"
           << ",\"force_body\":[" << force.x << ',' << force.y << ',' << force.z
           << "] ,\"torque_body\":[" << torque.x << ',' << torque.y << ','
           << torque.z << "],\"rotors\":[";
    for (std::size_t i = 0; i < rotors.size(); ++i) {
      if (i) stream << ',';
      const auto &r = rotors[i];
      stream << "{\"omega\":" << r.omegaRadS
             << ",\"thrust\":" << r.thrust << ",\"torque\":" << r.torque
             << ",\"h_force\":" << r.hForce << ",\"inflow\":"
             << r.inducedVelocity << ",\"mu\":" << r.advanceRatio
             << ",\"tip_mach\":" << r.tipMach << ",\"converged\":"
             << (r.inflowConverged ? "true" : "false") << '}';
    }
    stream << "]}";
    gz::msgs::StringMsg message;
    message.set_data(stream.str());
    telemetryPublisher_.Publish(message);
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity baseEntity_{gz::sim::kNullEntity};
  gz::sim::Link baseLink_{gz::sim::kNullEntity};
  std::vector<RotorBinding> rotors_;
  std::unique_ptr<AerodynamicsModel> modelCore_;
  BodyDragConfig bodyDrag_;
  gz::math::Vector3d windWorld_{0, 0, 0};
  gz::transport::Node node_;
  gz::transport::Node::Publisher telemetryPublisher_;
  double telemetryRate_{100.0};
  double aerodynamicsRate_{200.0};
  double lastTelemetryTime_{-1.0};
  double lastAerodynamicsTime_{-1.0};
  Vec3 cachedBodyDrag_;
  Vec3 cachedTotalForce_;
  Vec3 cachedTotalTorque_;
  bool configured_{false};
};

}  // namespace
}  // namespace agilicious::aero

GZ_ADD_PLUGIN(agilicious::aero::AgiliciousAerodynamicsPlugin,
              gz::sim::System,
              agilicious::aero::AgiliciousAerodynamicsPlugin::ISystemConfigure,
              agilicious::aero::AgiliciousAerodynamicsPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(agilicious::aero::AgiliciousAerodynamicsPlugin,
                    "agilicious::aero::AgiliciousAerodynamicsPlugin")
