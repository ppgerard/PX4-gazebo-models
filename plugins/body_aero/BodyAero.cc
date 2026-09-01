#include "BodyAero.hh"

#include <cmath>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Wind.hh>

using namespace gz;
using namespace sim;

namespace px4_gz_plugins
{

/// \brief One lifting surface. Parameter names mirror
/// gz-sim-lift-drag-system so values copy across verbatim.
struct Surface
{
  std::string name{"surface"};
  double area{0.0};
  double alpha0{0.0};
  double cla{0.0};
  double alphaStall{GZ_PI_2};
  math::Vector3d cp = math::Vector3d::Zero;      // from the CENTRE OF MASS
  math::Vector3d forward{1, 0, 0};
  math::Vector3d upward{0, 0, 1};
};

class BodyAeroPrivate
{
  public: math::Vector3d cdA = math::Vector3d::Zero;
  /// \brief One centre of pressure per drag axis, from the centre of mass.
  public: math::Vector3d cpDrag[3] = {math::Vector3d::Zero,
                                      math::Vector3d::Zero,
                                      math::Vector3d::Zero};
  public: double rho{1.2041};
  /// \brief Sharpness of the linear <-> flat-plate stall blend.
  public: double stallBlend{15.0};
  public: std::vector<Surface> surfaces;

  public: Model model{kNullEntity};
  public: Entity linkEntity{kNullEntity};
  public: std::string linkName;
  public: bool validConfig{false};
  public: bool initialized{false};

  public: transport::Node node;
  public: transport::Node::Publisher wrenchPub;
  public: bool publishWrench{false};
};

//////////////////////////////////////////////////
BodyAero::BodyAero()
  : System(), dataPtr(std::make_unique<BodyAeroPrivate>())
{
}

//////////////////////////////////////////////////
BodyAero::~BodyAero() = default;

//////////////////////////////////////////////////
void BodyAero::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "BodyAero must be attached to a model entity. Not loading.\n";
    return;
  }

  auto sdfClone = _sdf->Clone();

  if (!sdfClone->HasElement("link_name"))
  {
    gzerr << "BodyAero requires the <link_name> parameter. Not loading.\n";
    return;
  }
  this->dataPtr->linkName = sdfClone->Get<std::string>("link_name");

  this->dataPtr->cdA =
      sdfClone->Get<math::Vector3d>("cda", this->dataPtr->cdA).first;
  this->dataPtr->rho =
      sdfClone->Get<double>("air_density", this->dataPtr->rho).first;
  this->dataPtr->stallBlend =
      sdfClone->Get<double>("stall_blend", this->dataPtr->stallBlend).first;

  // A single <cp_drag> sets all three; <cp_drag_x/y/z> override individually.
  const auto cpAll =
      sdfClone->Get<math::Vector3d>("cp_drag", math::Vector3d::Zero).first;
  const char *axisTag[3] = {"cp_drag_x", "cp_drag_y", "cp_drag_z"};
  for (int i = 0; i < 3; ++i)
    this->dataPtr->cpDrag[i] = sdfClone->Get<math::Vector3d>(axisTag[i],
                                                             cpAll).first;

  if (this->dataPtr->cdA.Min() < 0.0)
  {
    gzerr << "BodyAero <cda> must be non-negative on every axis. "
          << "Not loading.\n";
    return;
  }
  if (this->dataPtr->rho <= 0.0)
  {
    gzerr << "BodyAero <air_density> must be positive. Not loading.\n";
    return;
  }

  for (auto elem = sdfClone->GetElement("surface");
       elem && elem->GetName() == "surface";
       elem = elem->GetNextElement("surface"))
  {
    Surface s;
    s.name       = elem->Get<std::string>("name", s.name).first;
    s.area       = elem->Get<double>("area", s.area).first;
    s.alpha0     = elem->Get<double>("a0", s.alpha0).first;
    s.cla        = elem->Get<double>("cla", s.cla).first;
    s.alphaStall = elem->Get<double>("alpha_stall", s.alphaStall).first;
    s.cp         = elem->Get<math::Vector3d>("cp", s.cp).first;
    s.forward    = elem->Get<math::Vector3d>("forward", s.forward).first;
    s.upward     = elem->Get<math::Vector3d>("upward", s.upward).first;

    if (s.area <= 0.0)
    {
      gzerr << "BodyAero surface [" << s.name << "] has area <= 0. Skipped.\n";
      continue;
    }
    if (s.forward.Length() < 1e-9 || s.upward.Length() < 1e-9 ||
        s.forward.Cross(s.upward).Length() < 1e-6)
    {
      gzerr << "BodyAero surface [" << s.name << "]: <forward> and <upward> "
            << "must be non-zero and not parallel. Skipped.\n";
      continue;
    }
    s.forward.Normalize();
    s.upward.Normalize();
    this->dataPtr->surfaces.push_back(s);
  }

  if (sdfClone->HasElement("topic"))
  {
    const auto topic = transport::TopicUtils::AsValidTopic(
        sdfClone->Get<std::string>("topic"));
    if (topic.empty())
    {
      gzerr << "BodyAero <topic> is not a valid topic name. "
            << "Wrench will not be published.\n";
    }
    else
    {
      this->dataPtr->wrenchPub =
          this->dataPtr->node.Advertise<msgs::Wrench>(topic);
      this->dataPtr->publishWrench = true;
      gzmsg << "BodyAero publishing the applied wrench on [" << topic << "]\n";
    }
  }

  this->dataPtr->validConfig = true;

  gzmsg << "BodyAero on link [" << this->dataPtr->linkName << "]: cdA = "
        << this->dataPtr->cdA << " m^2, rho = " << this->dataPtr->rho
        << " kg/m^3, " << this->dataPtr->surfaces.size()
        << " lifting surface(s)\n";
  for (const auto &s : this->dataPtr->surfaces)
  {
    gzmsg << "  surface [" << s.name << "] area=" << s.area
          << " a0=" << s.alpha0 << " cla=" << s.cla
          << " alpha_stall=" << s.alphaStall << " cp=" << s.cp << "\n";
  }
}

//////////////////////////////////////////////////
void BodyAero::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("BodyAero::PreUpdate");

  if (!this->dataPtr->validConfig || _info.paused)
    return;

  // Resolve the link late: a model spawned into a running world (which is how
  // PX4 SITL starts) has no links yet at Configure time.
  if (!this->dataPtr->initialized)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
    if (this->dataPtr->linkEntity == kNullEntity)
      return;
    Link link(this->dataPtr->linkEntity);
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

  // Wind, if the world declares any. Read unconditionally, matching LiftDrag
  // and AdvancedLiftDrag: neither consults the link's WindMode.
  math::Vector3d windVel = math::Vector3d::Zero;
  const Entity windEntity = _ecm.EntityByComponents(components::Wind());
  if (windEntity != kNullEntity)
  {
    const auto w =
        _ecm.Component<components::WorldLinearVelocity>(windEntity);
    if (w)
      windVel = w->Data();
  }

  const auto &pose = worldPose->Data();
  const auto &q = pose.Rot();
  const auto &omega = worldAngVel->Data();

  math::Vector3d totalForce = math::Vector3d::Zero;
  math::Vector3d totalTorque = math::Vector3d::Zero;

  // Velocity of a body-frame point r, relative to the airmass, in the body
  // frame. r is measured from the centre of mass.
  auto vAtBody = [&](const math::Vector3d &r) {
    const auto rWorld = q.RotateVector(r);
    const auto vWorld = worldLinVel->Data() + omega.Cross(rWorld) - windVel;
    return q.RotateVectorReverse(vWorld);
  };

  // ---- Drag: per body axis, at that axis' own centre of pressure ----
  for (int i = 0; i < 3; ++i)
  {
    if (this->dataPtr->cdA[i] <= 0.0)
      continue;
    const auto &r = this->dataPtr->cpDrag[i];
    const double vi = vAtBody(r)[i];
    // v*|v|, so the sign is right in both directions on this axis.
    const double fi =
        -0.5 * this->dataPtr->rho * this->dataPtr->cdA[i] * vi * std::abs(vi);

    math::Vector3d fBody = math::Vector3d::Zero;
    fBody[i] = fi;
    const auto fWorld = q.RotateVector(fBody);
    totalForce += fWorld;
    totalTorque += q.RotateVector(r).Cross(fWorld);
  }

  // ---- Lift: per surface, valid over the full +-180 deg ----
  for (const auto &s : this->dataPtr->surfaces)
  {
    const auto vBody = vAtBody(s.cp);

    // Span-normal of this surface's lift-drag plane.
    const auto span = s.forward.Cross(s.upward).Normalized();
    const auto vLD = vBody - vBody.Dot(span) * span;
    const double speed = vLD.Length();
    if (speed < 1e-6)
      continue;

    // v is the body's velocity relative to the air, so the relative wind is
    // -v: a downward v (sinking) means air arriving from below, i.e. alpha > 0.
    double alpha = std::atan2(-vLD.Dot(s.upward), vLD.Dot(s.forward)) + s.alpha0;
    // Wrap to [-pi, pi]. sin(2*alpha) is unaffected; the stall blend is not.
    while (alpha >  GZ_PI) alpha -= 2.0 * GZ_PI;
    while (alpha < -GZ_PI) alpha += 2.0 * GZ_PI;

    // Blend the linear law into the flat-plate law around alpha_stall. Same
    // sigmoid AdvancedLiftDrag uses, and it saturates to 1 well before 180 deg.
    const double M = this->dataPtr->stallBlend;
    const double eNeg = std::exp(-M * (alpha - s.alphaStall));
    const double ePos = std::exp( M * (alpha + s.alphaStall));
    const double sigma =
        (1.0 + eNeg + ePos) / ((1.0 + eNeg) * (1.0 + ePos));

    const double cl = (1.0 - sigma) * (s.cla * alpha) +
                      sigma * std::sin(2.0 * alpha);

    const double qDyn = 0.5 * this->dataPtr->rho * speed * speed;
    // Lift acts perpendicular to the in-plane relative flow.
    const auto liftDir = span.Cross(vLD / speed);
    const auto fBody = cl * qDyn * s.area * liftDir;

    const auto fWorld = q.RotateVector(fBody);
    totalForce += fWorld;
    totalTorque += q.RotateVector(s.cp).Cross(fWorld);
  }

  if (!totalForce.IsFinite() || !totalTorque.IsFinite())
    return;

  Link link(this->dataPtr->linkEntity);
  link.AddWorldWrench(_ecm, totalForce, totalTorque);

  if (this->dataPtr->publishWrench)
  {
    msgs::Wrench msg;
    msgs::Set(msg.mutable_force(), totalForce);
    msgs::Set(msg.mutable_torque(), totalTorque);
    auto stamp = msg.mutable_header()->mutable_stamp();
    const auto sec =
        std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
    stamp->set_sec(sec.count());
    stamp->set_nsec(std::chrono::duration_cast<std::chrono::nanoseconds>(
        _info.simTime - sec).count());
    this->dataPtr->wrenchPub.Publish(msg);
  }
}

}  // namespace px4_gz_plugins

GZ_ADD_PLUGIN(px4_gz_plugins::BodyAero,
              gz::sim::System,
              px4_gz_plugins::BodyAero::ISystemConfigure,
              px4_gz_plugins::BodyAero::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(px4_gz_plugins::BodyAero, "px4_gz_plugins::BodyAero")
