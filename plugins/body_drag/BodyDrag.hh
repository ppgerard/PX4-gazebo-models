/*
 * BodyDrag - quadratic bluff-body aerodynamic drag for gz-sim 8.
 *
 * Exists because neither gz-sim-lift-drag-system nor
 * gz-sim-advanced-lift-drag-system produces a usable force on a MULTIROTOR
 * hovering in wind:
 *
 *   - LiftDrag returns early on `forwardI.Dot(vel) <= 0` (LiftDrag.cc:313), so
 *     every surface contributes exactly zero whenever the relative flow arrives
 *     from the rear hemisphere. That is a hard discontinuity, not a rolloff.
 *   - AdvancedLiftDrag has no such gate, but a level vehicle in horizontal wind
 *     sits at alpha ~ 0 or 180 deg, where its post-stall drag term
 *     CD_fp*(0.5 - 0.5*cos(2*alpha)) is ZERO and the stall sigmoid has already
 *     scaled CD0 out; and at alpha=180 the sideslip is beta=180 deg, which the
 *     unbounded linear term CY = CYb*beta turns into hundreds of newtons of
 *     force that does not exist.
 *
 * Both are fixed-wing models evaluated far outside their valid envelope. This
 * plugin is the complementary piece: a pure bluff-body term, valid at every
 * relative-flow angle, with no gates and no linearisation.
 *
 *   v_rel  = v_link - v_wind  (+ omega x r_cp),  expressed in the body frame
 *   f_i    = -0.5 * rho * cdA_i * v_i * |v_i|          for i in {x, y, z}
 *   torque = r_cp x f
 *
 * v*|v| rather than v^2 so the sign is correct in both directions on each axis.
 * The per-axis cdA vector carries the airframe's anisotropy (a wing is cheap
 * chordwise and very expensive face-on), and r_cp offsets the centre of
 * pressure from the centre of mass, which is what produces weathervaning and
 * gust-induced attitude upset. Set cp to 0 0 0 for a force-only model.
 *
 * It does NOT replace LiftDrag: keep those surfaces for the fixed-wing phase.
 * It DOES overlap gz-sim-wind-effects-system's own `k*m*(v_wind - v_link)`
 * force approximation, so set that plugin's
 * <force_approximation_scaling_factor> to 0 to avoid double counting. Keep the
 * plugin itself loaded - it is what generates the gusting wind vector this
 * plugin reads.
 */
#ifndef GZ_SIM_SYSTEMS_BODYDRAG_HH_
#define GZ_SIM_SYSTEMS_BODYDRAG_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace px4_gz_plugins
{
  class BodyDragPrivate;

  class BodyDrag:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public: BodyDrag();
    public: ~BodyDrag() override;

    public: void Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr) override;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    private: std::unique_ptr<BodyDragPrivate> dataPtr;
  };
}

#endif
