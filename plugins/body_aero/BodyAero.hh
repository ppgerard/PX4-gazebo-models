/*
 * BodyAero - lift and drag valid at EVERY relative-flow angle, for gz-sim 8.
 *
 * Written because both stock aerodynamic plugins fail on a multirotor hovering
 * in wind, in ways that silently produce zero force:
 *
 *   - gz-sim-lift-drag-system returns early on `forwardI.Dot(vel) <= 0`
 *     (LiftDrag.cc:313), so every surface contributes exactly nothing whenever
 *     the relative flow arrives from the rear hemisphere. A hard
 *     discontinuity, not a rolloff.
 *   - gz-sim-advanced-lift-drag-system has no such gate, but a level vehicle in
 *     horizontal wind sits at alpha ~ 0 or 180 deg, where its post-stall drag
 *     term CD_fp*(0.5 - 0.5*cos(2*alpha)) is ZERO and the stall sigmoid has
 *     already scaled CD0 out; and at alpha = 180 the sideslip is beta = 180 deg,
 *     which the unbounded linear term CY = CYb*beta turns into hundreds of
 *     newtons that do not exist.
 *   - gz-sim-wind-effects-system applies k*m*(v_wind - v_link) at the centre of
 *     mass: linear in velocity, proportional to MASS, and producing no
 *     aerodynamic moment at all.
 *
 * MODEL
 *
 * Drag, per BODY axis, everywhere:
 *
 *     f_i = -0.5 * rho * cdA_i * v_i * |v_i|        i in {x,y,z}
 *
 *   v*|v| rather than v^2 keeps the sign right in both directions. The per-axis
 *   cdA carries the airframe's anisotropy, and each axis has its OWN centre of
 *   pressure, because the area that faces sideways is not in the same place as
 *   the area that faces down.
 *
 * Lift, per surface, over the full +-180 deg:
 *
 *     alpha = atan2(-(v.u), v.f) + alpha0        wrapped to [-pi, pi]
 *     CL    = (1-sigma)*(cla*alpha) + sigma*sin(2*alpha)
 *     f     = CL * q * area * (s_hat x v_hat),    s = f x u
 *
 *   The linear branch is the ordinary thin-airfoil law and is what acts in a
 *   head-on flow. The sin(2*alpha) branch is the flat-plate/Newtonian law,
 *   which is what a wing in reversed or heavily separated flow actually does,
 *   and it is bounded and correctly signed everywhere - including alpha = 180,
 *   where a rigged wing gets a genuine DOWNforce rather than nothing. sigma is
 *   the usual stall blend, so the two join smoothly at alpha_stall.
 *
 *   Surfaces contribute LIFT ONLY. All drag is the per-axis term above, so
 *   there is exactly one drag model and nothing double-counts. This is the
 *   mistake that shipping BodyDrag alongside LiftDrag made: in the forward
 *   hemisphere the two drags added, giving 1.66x the force of the rear
 *   hemisphere for no physical reason.
 *
 * Every surface parameter mirrors gz-sim-lift-drag-system's own SDF element
 * names, so values can be copied across from an existing <plugin> block
 * verbatim.
 *
 * INTERACTION
 *
 *   - Disable gz-sim-lift-drag-system on the same link. This plugin subsumes it.
 *   - KEEP gz-sim-wind-effects-system loaded: it is what turns the world's
 *     <wind> seed into the gusting wind vector this plugin reads. But set its
 *     <force_approximation_scaling_factor> to 0, verified to produce exactly
 *     zero force (the x/y fields vanish entirely from a free body's odometry
 *     over a 25 m/s wind, while z falls under gravity).
 */
#ifndef GZ_SIM_SYSTEMS_BODYAERO_HH_
#define GZ_SIM_SYSTEMS_BODYAERO_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace px4_gz_plugins
{
  class BodyAeroPrivate;

  class BodyAero:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public: BodyAero();
    public: ~BodyAero() override;

    public: void Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr) override;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    private: std::unique_ptr<BodyAeroPrivate> dataPtr;
  };
}

#endif
