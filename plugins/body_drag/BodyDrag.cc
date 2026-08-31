#include "BodyDrag.hh"

#include <mutex>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Wind.hh>

using namespace gz;
using namespace sim;

namespace px4_gz_plugins
{

class BodyDragPrivate
{
  /// \brief Drag area per BODY axis [m^2], i.e. Cd_i * A_i.
  public: math::Vector3d cdA = math::Vector3d::Zero;

  /// \brief Centre of pressure, offset from the link's CENTRE OF MASS,
  /// in the body frame [m]. Zero gives a force-only model with no moment.
  public: math::Vector3d cp = math::Vector3d::Zero;

  public: double rho = 1.2041;

  public: Model model{kNullEntity};
  public: Entity linkEntity{kNullEntity};
  public: std::string linkName;
  public: bool validConfig{false};
  public: bool initialized{false};

  /// \brief Optional diagnostic publisher for the applied wrench.
  public: transport::Node node;
  public: transport::Node::Publisher wrenchPub;
  public: bool publishWrench{false};
};

//////////////////////////////////////////////////
BodyDrag::BodyDrag()
  : System(), dataPtr(std::make_unique<BodyDragPrivate>())
{
}

//////////////////////////////////////////////////
BodyDrag::~BodyDrag() = default;

//////////////////////////////////////////////////
void BodyDrag::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "BodyDrag must be attached to a model entity. Not loading.\n";
    return;
  }

  auto sdfClone = _sdf->Clone();

  if (!sdfClone->HasElement("link_name"))
  {
    gzerr << "BodyDrag requires the <link_name> parameter. Not loading.\n";
    return;
  }
  this->dataPtr->linkName = sdfClone->Get<std::string>("link_name");

  this->dataPtr->cdA =
      sdfClone->Get<math::Vector3d>("cda", this->dataPtr->cdA).first;
  this->dataPtr->cp =
      sdfClone->Get<math::Vector3d>("cp", this->dataPtr->cp).first;
  this->dataPtr->rho =
      sdfClone->Get<double>("air_density", this->dataPtr->rho).first;

  if (this->dataPtr->cdA.Min() < 0.0)
  {
    gzerr << "BodyDrag <cda> must be non-negative on every axis. "
          << "Not loading.\n";
    return;
  }
  if (this->dataPtr->rho <= 0.0)
  {
    gzerr << "BodyDrag <air_density> must be positive. Not loading.\n";
    return;
  }

  if (sdfClone->HasElement("topic"))
  {
    const auto topic = transport::TopicUtils::AsValidTopic(
        sdfClone->Get<std::string>("topic"));
    if (topic.empty())
    {
      gzerr << "BodyDrag <topic> is not a valid topic name. "
            << "Wrench will not be published.\n";
    }
    else
    {
      this->dataPtr->wrenchPub =
          this->dataPtr->node.Advertise<msgs::Wrench>(topic);
      this->dataPtr->publishWrench = true;
      gzmsg << "BodyDrag publishing the applied wrench on [" << topic << "]\n";
    }
  }

  this->dataPtr->validConfig = true;

  gzmsg << "BodyDrag on link [" << this->dataPtr->linkName << "]: cdA = "
        << this->dataPtr->cdA << " m^2, cp = " << this->dataPtr->cp
        << " m, rho = " << this->dataPtr->rho << " kg/m^3\n";
}

//////////////////////////////////////////////////
void BodyDrag::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("BodyDrag::PreUpdate");

  if (!this->dataPtr->validConfig || _info.paused)
    return;

  // Resolve the link late: the model's links do not exist yet at Configure
  // time when the model is spawned into a running world, which is exactly how
  // PX4 SITL starts.
  if (!this->dataPtr->initialized)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
    if (this->dataPtr->linkEntity == kNullEntity)
      return;

    Link link(this->dataPtr->linkEntity);
    // World linear/angular velocity are not populated by default.
    link.EnableVelocityChecks(_ecm, true);
    this->dataPtr->initialized = true;
    return;
  }

  const auto worldPose =
      _ecm.Component<components::WorldPose>(this->dataPtr->linkEntity);
  const auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->linkEntity);
  const auto worldAngVel =
      _ecm.Component<components::WorldAngularVelocity>(
          this->dataPtr->linkEntity);

  if (!worldPose || !worldLinVel || !worldAngVel)
    return;

  // Wind, if the world declares any. Read unconditionally, matching
  // LiftDrag and AdvancedLiftDrag: neither consults the link's WindMode.
  math::Vector3d windVel = math::Vector3d::Zero;
  const Entity windEntity = _ecm.EntityByComponents(components::Wind());
  if (windEntity != kNullEntity)
  {
    const auto windLinVel =
        _ecm.Component<components::WorldLinearVelocity>(windEntity);
    if (windLinVel)
      windVel = windLinVel->Data();
  }

  const auto &pose = worldPose->Data();
  const auto cpWorld = pose.Rot().RotateVector(this->dataPtr->cp);

  // Velocity of the centre of pressure relative to the airmass, world frame.
  const math::Vector3d vRelWorld =
      worldLinVel->Data() + worldAngVel->Data().Cross(cpWorld) - windVel;

  // Per-axis quadratic drag is only meaningful in the body frame.
  const math::Vector3d vBody = pose.Rot().RotateVectorReverse(vRelWorld);

  math::Vector3d fBody;
  for (int i = 0; i < 3; ++i)
  {
    // v*|v| rather than v^2: keeps the sign right in both directions.
    fBody[i] = -0.5 * this->dataPtr->rho * this->dataPtr->cdA[i] *
               vBody[i] * std::abs(vBody[i]);
  }

  const math::Vector3d fWorld = pose.Rot().RotateVector(fBody);
  // AddWorldWrench applies the force at the centre of mass, so the moment the
  // offset centre of pressure produces has to be handed over explicitly.
  const math::Vector3d tWorld = cpWorld.Cross(fWorld);

  if (!fWorld.IsFinite() || !tWorld.IsFinite())
    return;

  Link link(this->dataPtr->linkEntity);
  link.AddWorldWrench(_ecm, fWorld, tWorld);

  if (this->dataPtr->publishWrench)
  {
    msgs::Wrench msg;
    msgs::Set(msg.mutable_force(), fWorld);
    msgs::Set(msg.mutable_torque(), tWorld);
    auto stamp = msg.mutable_header()->mutable_stamp();
    const auto s =
        std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
    stamp->set_sec(s.count());
    stamp->set_nsec(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            _info.simTime - s).count());
    this->dataPtr->wrenchPub.Publish(msg);
  }
}

}  // namespace px4_gz_plugins

GZ_ADD_PLUGIN(px4_gz_plugins::BodyDrag,
              gz::sim::System,
              px4_gz_plugins::BodyDrag::ISystemConfigure,
              px4_gz_plugins::BodyDrag::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(px4_gz_plugins::BodyDrag, "px4_gz_plugins::BodyDrag")
