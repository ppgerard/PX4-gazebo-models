#include "BodyAero.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Wind.hh>

#include <sdf/Model.hh>
#include <sdf/Plugin.hh>

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
  double cd0{0.0};        // profile drag, ALWAYS present (never blended out)
  double ar{0.0};         // aspect ratio; 0 disables the induced term
  double eff{0.85};       // Oswald efficiency
  // (v3) Normal-force coefficient of the surface broadside to the flow. 2.0 is
  // the 2-D flat plate the plugin always used; a finite plate is ~1.2-1.3
  // (Hoerner, aspect ratio 2-6), i.e. 2.0 overstated every stalled surface.
  double cd90{2.0};
  math::Vector3d cp = math::Vector3d::Zero;      // from the CENTRE OF MASS
  math::Vector3d forward{1, 0, 0};
  math::Vector3d upward{0, 0, 1};

  // Control surface (v3): the joint's angle shifts this surface's angle of
  // attack by controlEff * angle -- a flap-effectiveness factor, signed so the
  // joint's own sign convention needs no further handling.
  std::string controlJoint;
  double controlEff{0.0};
  Entity controlJointEntity{kNullEntity};

  // Slipstream (v3): the fraction <slipstream_fraction> of this surface's span
  // sits in the wake of rotor <slipstream_rotor>. Free-stream and wake flows
  // are both evaluated and blended by that fraction. Only the part of the wake
  // that runs along the chord is applied (weight (t . forward)^2), so a
  // hovering rotor's downwash does not become a flat-plate download that this
  // model has no calibration for.
  std::string slipRotor;
  double slipFraction{0.0};
  double slipGain{1.5};     // wake speed at the surface, in induced velocities
  int slipRotorIndex{-1};
};

/// \brief (v3) A rotor whose thrust the motor model computes as k*w^2 with no
/// dependence on the inflow. This plugin applies the missing part: a force
/// along the rotor axis equal to -(1 - f(J)) * k * w^2, at the rotor.
struct Rotor
{
  std::string linkName;
  std::string jointName;
  double motorConstant{0.0};
  double momentConstant{0.0};
  double slowdown{1.0};
  bool warnedSkip{false};
  double spin{1.0};          // +1 ccw, -1 cw (about the rotor's +z)
  double diameter{0.254};
  double j0{0.72};           // advance ratio of zero thrust
  double ja{0.25};           // f(J) = 1 - ja*x - (1-ja)*x^2,  x = J/j0
  Entity linkEntity{kNullEntity};
  Entity jointEntity{kNullEntity};
  // Filled every step.
  double thrust{0.0};        // CORRECTED thrust [N]
  double vInduced{0.0};      // induced velocity at the disc [m/s]
  math::Vector3d axisWorld{0, 0, 1};
};

/// \brief (v3) First-order Gauss-Markov turbulence per axis (u along the mean
/// horizontal wind, v across it, w vertical), time constant L/V with V the
/// vehicle's airspeed floored at vFloor. A zero sigma disables that axis.
struct Turbulence
{
  math::Vector3d sigma = math::Vector3d::Zero;   // u, v, w [m/s]
  math::Vector3d length{100.0, 100.0, 15.0};     // L_u, L_v, L_w [m]
  double vFloor{3.0};
  unsigned int seed{0};
  bool enabled{false};
  math::Vector3d state = math::Vector3d::Zero;   // u, v, w
  std::mt19937 rng;
  std::normal_distribution<double> normal{0.0, 1.0};
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
  public: std::vector<Rotor> rotors;
  public: Turbulence turb;
  public: bool inflowCorrection{false};

  public: Model model{kNullEntity};
  public: Entity linkEntity{kNullEntity};
  public: std::string linkName;
  public: bool validConfig{false};
  public: bool initialized{false};

  public: transport::Node node;
  public: transport::Node::Publisher wrenchPub;
  public: transport::Node::Publisher windPub;
  public: bool publishWrench{false};
  public: bool publishWind{false};
};

namespace
{
/// Lift and drag of one surface at relative velocity vBody (body frame, the
/// BODY's velocity relative to the air), with an extra angle of attack.
math::Vector3d SurfaceForce(const Surface &s, const math::Vector3d &vBody,
                            double dAlpha, double rho, double M)
{
  const auto span = s.forward.Cross(s.upward).Normalized();
  const auto vLD = vBody - vBody.Dot(span) * span;
  const double speed = vLD.Length();
  if (speed < 1e-6)
    return math::Vector3d::Zero;

  // v is the body's velocity relative to the air, so the relative wind is
  // -v: a downward v (sinking) means air arriving from below, i.e. alpha > 0.
  double alpha = std::atan2(-vLD.Dot(s.upward), vLD.Dot(s.forward))
                 + s.alpha0 + dAlpha;
  // Wrap to [-pi, pi]. sin(2*alpha) is unaffected; the stall blend is not.
  while (alpha >  GZ_PI) alpha -= 2.0 * GZ_PI;
  while (alpha < -GZ_PI) alpha += 2.0 * GZ_PI;

  // Blend the linear law into the flat-plate law around alpha_stall. Same
  // sigmoid AdvancedLiftDrag uses, and it saturates to 1 well before 180 deg.
  const double eNeg = std::exp(-M * (alpha - s.alphaStall));
  const double ePos = std::exp( M * (alpha + s.alphaStall));
  const double sigma = (1.0 + eNeg + ePos) / ((1.0 + eNeg) * (1.0 + ePos));

  const double clLin = s.cla * alpha;
  const double k = 0.5 * s.cd90;   // flat plate: CN = cd90*sin(a)
  const double cl = (1.0 - sigma) * clLin + sigma * k * std::sin(2.0 * alpha);

  // Drag must come from the SAME normal force as the lift, or the two are
  // separate copies of one physical effect. Flat-plate theory gives a single
  // normal force CN = 2*sin(alpha), whose components are lift = sin(2*alpha)
  // perpendicular to the flow and drag = 2*sin^2(alpha) along it.
  // cd0 sits OUTSIDE the blend: AdvancedLiftDrag multiplies it by (1-sigma),
  // so its parasitic drag vanishes exactly where a stalled wing has most.
  double cd = s.cd0 + sigma * k * (1.0 - std::cos(2.0 * alpha));
  if (s.ar > 0.0 && s.eff > 0.0)
    cd += (1.0 - sigma) * (clLin * clLin) / (GZ_PI * s.ar * s.eff);

  const double qDyn = 0.5 * rho * speed * speed;
  const auto vHat = vLD / speed;
  // Lift perpendicular to the in-plane flow, drag opposing it.
  const auto liftDir = span.Cross(vHat);
  return qDyn * s.area * (cl * liftDir - cd * vHat);
}

/// Thrust fraction a fixed-pitch propeller keeps at advance ratio J. J <= 0
/// (a rotor descending into its own wake) is left at 1: no vortex-ring model.
double InflowFactor(const Rotor &r, double J)
{
  if (J <= 0.0)
    return 1.0;
  const double x = J / r.j0;
  return std::max(0.0, 1.0 - r.ja * x - (1.0 - r.ja) * x * x);
}
}  // namespace

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

  // ---- Rotors (v3). Read from every MulticopterMotorModel of the parent
  // model, so the motor constants cannot drift from the ones the motor model
  // actually uses (the Monte-Carlo harness perturbs them in place).
  this->dataPtr->inflowCorrection =
      sdfClone->Get<bool>("inflow_correction", false).first;
  const double propDiameter =
      sdfClone->Get<double>("prop_diameter", 0.254).first;
  const double propJ0 = sdfClone->Get<double>("prop_j0", 0.72).first;
  const double propJa = sdfClone->Get<double>("prop_ja", 0.25).first;
  {
    // The motor plugins' parameters, as (filename, child elements). From the
    // model's own SDF in the ECM: in gz-sim 8 the plugin element handed to
    // Configure is rebuilt without a parent, so _sdf->GetParent() is null and
    // the old search found NO rotors -- the inflow correction and every
    // surface's slipstream silently never ran.
    std::vector<std::pair<std::string, std::vector<sdf::ElementPtr>>> motors;
    if (const auto *ms = _ecm.Component<components::ModelSdf>(_entity))
    {
      for (const auto &p : ms->Data().Plugins())
        motors.emplace_back(p.Filename(), p.Contents());
    }
    if (motors.empty())
    {
      auto modelElem = _sdf->GetParent();
      if (modelElem && modelElem->HasElement("plugin"))
      {
        for (auto p = modelElem->GetElement("plugin"); p;
             p = p->GetNextElement("plugin"))
        {
          std::vector<sdf::ElementPtr> c;
          for (auto e = p->GetFirstElement(); e; e = e->GetNextElement())
            c.push_back(e);
          motors.emplace_back(p->Get<std::string>("filename"), c);
        }
      }
    }
    for (const auto &[fname, contents] : motors)
    {
      if (fname.find("multicopter-motor-model") == std::string::npos &&
          fname.find("MulticopterMotorModel") == std::string::npos)
        continue;
      Rotor r;
      std::string dir = "ccw";
      for (const auto &e : contents)
      {
        const auto n = e->GetName();
        if (n == "linkName") r.linkName = e->Get<std::string>();
        else if (n == "jointName") r.jointName = e->Get<std::string>();
        else if (n == "motorConstant") r.motorConstant = e->Get<double>();
        else if (n == "momentConstant") r.momentConstant = e->Get<double>();
        else if (n == "rotorVelocitySlowdownSim") r.slowdown = e->Get<double>();
        else if (n == "turningDirection") dir = e->Get<std::string>();
      }
      r.spin = (dir == "cw") ? -1.0 : 1.0;
      r.diameter = propDiameter;
      r.j0 = propJ0;
      r.ja = propJa;
      if (!r.linkName.empty() && !r.jointName.empty() && r.motorConstant > 0)
        this->dataPtr->rotors.push_back(r);
    }
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
    s.cd0        = elem->Get<double>("cd0", s.cd0).first;
    s.ar         = elem->Get<double>("ar", s.ar).first;
    s.eff        = elem->Get<double>("eff", s.eff).first;
    s.cd90       = elem->Get<double>("cd90", s.cd90).first;
    s.cp         = elem->Get<math::Vector3d>("cp", s.cp).first;
    s.forward    = elem->Get<math::Vector3d>("forward", s.forward).first;
    s.upward     = elem->Get<math::Vector3d>("upward", s.upward).first;
    s.controlJoint = elem->Get<std::string>("control_joint", "").first;
    s.controlEff   = elem->Get<double>("control_eff", 0.0).first;
    s.slipRotor    = elem->Get<std::string>("slipstream_rotor", "").first;
    s.slipFraction = elem->Get<double>("slipstream_fraction", 0.0).first;
    s.slipGain     = elem->Get<double>("slipstream_gain", s.slipGain).first;

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
    if (!s.slipRotor.empty())
    {
      for (size_t i = 0; i < this->dataPtr->rotors.size(); ++i)
        if (this->dataPtr->rotors[i].linkName == s.slipRotor)
          s.slipRotorIndex = static_cast<int>(i);
      if (s.slipRotorIndex < 0)
        gzerr << "BodyAero surface [" << s.name << "]: slipstream rotor ["
              << s.slipRotor << "] is not a motor-model link. Ignored.\n";
    }
    this->dataPtr->surfaces.push_back(s);
  }

  // ---- Turbulence (v3) ----
  if (sdfClone->HasElement("turbulence"))
  {
    auto t = sdfClone->GetElement("turbulence");
    auto &T = this->dataPtr->turb;
    T.sigma = t->Get<math::Vector3d>("sigma", T.sigma).first;
    T.length = t->Get<math::Vector3d>("length", T.length).first;
    T.vFloor = t->Get<double>("v_floor", T.vFloor).first;
    T.seed = t->Get<unsigned int>("seed", 0u).first;
    T.enabled = T.sigma.Max() > 0.0;
    T.rng.seed(T.seed);
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
      // The wind the vehicle actually flew in (mean + gusts + turbulence),
      // world frame, for validating a wind estimator against truth.
      const auto windTopic =
          transport::TopicUtils::AsValidTopic(topic + "/wind");
      if (!windTopic.empty())
      {
        this->dataPtr->windPub =
            this->dataPtr->node.Advertise<msgs::Vector3d>(windTopic);
        this->dataPtr->publishWind = true;
      }
    }
  }

  this->dataPtr->validConfig = true;

  gzmsg << "BodyAero on link [" << this->dataPtr->linkName << "]: cdA = "
        << this->dataPtr->cdA << " m^2, rho = " << this->dataPtr->rho
        << " kg/m^3, " << this->dataPtr->surfaces.size()
        << " lifting surface(s), " << this->dataPtr->rotors.size()
        << " rotor(s), inflow correction "
        << (this->dataPtr->inflowCorrection ? "ON" : "off")
        << ", turbulence "
        << (this->dataPtr->turb.enabled ? "ON" : "off") << "\n";
  for (const auto &s : this->dataPtr->surfaces)
  {
    gzmsg << "  surface [" << s.name << "] area=" << s.area
          << " a0=" << s.alpha0 << " cla=" << s.cla
          << " alpha_stall=" << s.alphaStall << " cd0=" << s.cd0
          << " AR=" << s.ar << " cp=" << s.cp;
    if (!s.controlJoint.empty())
      gzmsg << " control=" << s.controlJoint << " x" << s.controlEff;
    if (s.slipRotorIndex >= 0)
      gzmsg << " slipstream=" << s.slipRotor << " frac " << s.slipFraction;
    gzmsg << "\n";
  }
  if (this->dataPtr->turb.enabled)
    gzmsg << "  turbulence sigma(u,v,w)=" << this->dataPtr->turb.sigma
          << " L=" << this->dataPtr->turb.length
          << " seed=" << this->dataPtr->turb.seed << "\n";
}

//////////////////////////////////////////////////
void BodyAero::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("BodyAero::PreUpdate");

  if (!this->dataPtr->validConfig || _info.paused)
    return;

  auto &d = *this->dataPtr;

  // Resolve entities late: a model spawned into a running world (which is how
  // PX4 SITL starts) has no links yet at Configure time.
  if (!d.initialized)
  {
    d.linkEntity = d.model.LinkByName(_ecm, d.linkName);
    if (d.linkEntity == kNullEntity)
      return;
    Link link(d.linkEntity);
    link.EnableVelocityChecks(_ecm, true);
    for (auto &s : d.surfaces)
    {
      if (s.controlJoint.empty())
        continue;
      s.controlJointEntity = d.model.JointByName(_ecm, s.controlJoint);
      if (s.controlJointEntity == kNullEntity)
      {
        gzerr << "BodyAero: control joint [" << s.controlJoint
              << "] not found; surface [" << s.name << "] is fixed.\n";
        continue;
      }
      Joint(s.controlJointEntity).EnablePositionCheck(_ecm, true);
    }
    for (auto &r : d.rotors)
    {
      r.linkEntity = d.model.LinkByName(_ecm, r.linkName);
      r.jointEntity = d.model.JointByName(_ecm, r.jointName);
      if (r.jointEntity != kNullEntity)
        Joint(r.jointEntity).EnableVelocityCheck(_ecm, true);
      // The physics system only updates WorldPose on links that carry the
      // component, and nothing else creates it for a rotor link. Without it
      // every rotor was skipped below and the inflow correction never ran:
      // the props kept full static thrust at any airspeed.
      if (r.linkEntity != kNullEntity &&
          !_ecm.Component<components::WorldPose>(r.linkEntity))
        _ecm.CreateComponent(r.linkEntity, components::WorldPose());
    }
    d.initialized = true;
    return;
  }

  const auto worldPose = _ecm.Component<components::WorldPose>(d.linkEntity);
  const auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(d.linkEntity);
  const auto worldAngVel =
      _ecm.Component<components::WorldAngularVelocity>(d.linkEntity);
  if (!worldPose || !worldLinVel || !worldAngVel)
    return;

  // Wind, if the world declares any. Read unconditionally, matching LiftDrag
  // and AdvancedLiftDrag: neither consults the link's WindMode.
  math::Vector3d windVel = math::Vector3d::Zero;
  const Entity windEntity = _ecm.EntityByComponents(components::Wind());
  if (windEntity != kNullEntity)
  {
    const auto w = _ecm.Component<components::WorldLinearVelocity>(windEntity);
    if (w)
      windVel = w->Data();
  }

  const auto &pose = worldPose->Data();
  const auto &q = pose.Rot();
  const auto &omega = worldAngVel->Data();
  const auto &vLink = worldLinVel->Data();
  const double dt = std::chrono::duration<double>(_info.dt).count();

  // ---- Turbulence: first-order Gauss-Markov per axis, in mean-wind axes ----
  if (d.turb.enabled && dt > 0.0)
  {
    auto &T = d.turb;
    const double airspeed = std::max((vLink - windVel).Length(), T.vFloor);
    for (int i = 0; i < 3; ++i)
    {
      if (T.sigma[i] <= 0.0)
        continue;
      const double tau = std::max(T.length[i] / airspeed, 1e-3);
      const double a = std::exp(-dt / tau);
      T.state[i] = a * T.state[i] +
                   T.sigma[i] * std::sqrt(1.0 - a * a) * T.normal(T.rng);
    }
    math::Vector3d uHat(1, 0, 0);
    const math::Vector3d wh(windVel.X(), windVel.Y(), 0.0);
    if (wh.Length() > 0.5)
      uHat = wh.Normalized();
    const math::Vector3d vHat(-uHat.Y(), uHat.X(), 0.0);
    windVel += T.state.X() * uHat + T.state.Y() * vHat +
               T.state.Z() * math::Vector3d(0, 0, 1);
  }

  math::Vector3d totalForce = math::Vector3d::Zero;
  math::Vector3d totalTorque = math::Vector3d::Zero;

  // Velocity of a body-frame point r, relative to the airmass, in the body
  // frame. r is measured from the centre of mass (the link origin is within
  // millimetres of it on this model, as before).
  auto vAtBody = [&](const math::Vector3d &r) {
    const auto rWorld = q.RotateVector(r);
    const auto vWorld = vLink + omega.Cross(rWorld) - windVel;
    return q.RotateVectorReverse(vWorld);
  };

  // ---- Rotors: thrust lost to axial inflow, and the induced velocity ----
  for (auto &r : d.rotors)
  {
    r.thrust = 0.0;
    r.vInduced = 0.0;
    if (r.linkEntity == kNullEntity || r.jointEntity == kNullEntity)
      continue;
    const auto rp = _ecm.Component<components::WorldPose>(r.linkEntity);
    const auto jv = _ecm.Component<components::JointVelocity>(r.jointEntity);
    if (!rp || !jv || jv->Data().empty())
    {
      if (!r.warnedSkip)
      {
        gzwarn << "BodyAero: rotor [" << r.linkName << "] has no "
               << (!rp ? "WorldPose" : "joint velocity")
               << "; its inflow correction is NOT applied.\n";
        r.warnedSkip = true;
      }
      continue;
    }
    const double w = std::abs(jv->Data()[0]) * r.slowdown;   // rad/s
    const auto axis = rp->Data().Rot().RotateVector(math::Vector3d(0, 0, 1));
    r.axisWorld = axis;
    const auto rRotor = rp->Data().Pos() - pose.Pos();
    const auto vRotor = vLink + omega.Cross(rRotor) - windVel;
    const double vAx = vRotor.Dot(axis);          // > 0: moving along thrust
    const double n = w / (2.0 * GZ_PI);
    const double J = (n > 1.0) ? vAx / (n * r.diameter) : 0.0;
    const double t0 = r.motorConstant * w * w;
    const double f = d.inflowCorrection ? InflowFactor(r, J) : 1.0;
    r.thrust = f * t0;
    const double area = 0.25 * GZ_PI * r.diameter * r.diameter;
    const double vAxIn = std::max(vAx, 0.0);
    r.vInduced = -0.5 * vAxIn + std::sqrt(0.25 * vAxIn * vAxIn +
                 r.thrust / (2.0 * d.rho * area));
    if (d.inflowCorrection && f < 1.0)
    {
      // Remove the thrust (and, with the same factor, the drag torque) the
      // motor model applied but the propeller does not make at this inflow.
      const auto dF = -(1.0 - f) * t0 * axis;
      const auto dQ = (1.0 - f) * t0 * r.momentConstant * r.spin * axis;
      totalForce += dF;
      totalTorque += rRotor.Cross(dF) + dQ;
    }
  }

  // ---- Drag: per body axis, at that axis' own centre of pressure ----
  for (int i = 0; i < 3; ++i)
  {
    if (d.cdA[i] <= 0.0)
      continue;
    const auto &r = d.cpDrag[i];
    const double vi = vAtBody(r)[i];
    // v*|v|, so the sign is right in both directions on this axis.
    const double fi = -0.5 * d.rho * d.cdA[i] * vi * std::abs(vi);
    math::Vector3d fBody = math::Vector3d::Zero;
    fBody[i] = fi;
    const auto fWorld = q.RotateVector(fBody);
    totalForce += fWorld;
    totalTorque += q.RotateVector(r).Cross(fWorld);
  }

  // ---- Lift: per surface, valid over the full +-180 deg ----
  for (const auto &s : d.surfaces)
  {
    double dAlpha = 0.0;
    if (s.controlJointEntity != kNullEntity)
    {
      const auto jp =
          _ecm.Component<components::JointPosition>(s.controlJointEntity);
      if (jp && !jp->Data().empty())
        dAlpha = s.controlEff * jp->Data()[0];
    }
    const auto vBody = vAtBody(s.cp);
    auto fBody = SurfaceForce(s, vBody, dAlpha, d.rho, d.stallBlend);

    if (s.slipRotorIndex >= 0 && s.slipFraction > 0.0)
    {
      const auto &r = d.rotors[s.slipRotorIndex];
      const auto axisBody = q.RotateVectorReverse(r.axisWorld);
      const double along = std::max(0.0, axisBody.Dot(s.forward));
      const double w = along * along * along * along;   // wake along the chord only
      if (w > 0.0 && r.vInduced > 0.0)
      {
        // The body moves forward relative to the wake air: add the wake speed
        // along the thrust axis to the relative velocity.
        const auto vWake = vBody + s.slipGain * r.vInduced * axisBody;
        const auto fWake = SurfaceForce(s, vWake, dAlpha, d.rho, d.stallBlend);
        fBody = fBody + (s.slipFraction * w) * (fWake - fBody);
      }
    }

    const auto fWorld = q.RotateVector(fBody);
    totalForce += fWorld;
    totalTorque += q.RotateVector(s.cp).Cross(fWorld);
  }

  if (!totalForce.IsFinite() || !totalTorque.IsFinite())
    return;

  Link link(d.linkEntity);
  link.AddWorldWrench(_ecm, totalForce, totalTorque);

  const auto sec =
      std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
  const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime - sec).count();
  if (d.publishWrench)
  {
    msgs::Wrench msg;
    msgs::Set(msg.mutable_force(), totalForce);
    msgs::Set(msg.mutable_torque(), totalTorque);
    auto stamp = msg.mutable_header()->mutable_stamp();
    stamp->set_sec(sec.count());
    stamp->set_nsec(nsec);
    d.wrenchPub.Publish(msg);
  }
  if (d.publishWind)
  {
    msgs::Vector3d msg;
    msgs::Set(&msg, windVel);
    auto stamp = msg.mutable_header()->mutable_stamp();
    stamp->set_sec(sec.count());
    stamp->set_nsec(nsec);
    d.windPub.Publish(msg);
  }
}

}  // namespace px4_gz_plugins

GZ_ADD_PLUGIN(px4_gz_plugins::BodyAero,
              gz::sim::System,
              px4_gz_plugins::BodyAero::ISystemConfigure,
              px4_gz_plugins::BodyAero::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(px4_gz_plugins::BodyAero, "px4_gz_plugins::BodyAero")
