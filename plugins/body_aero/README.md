# BodyAero — lift and drag at every flow angle, for gz-sim 8

## Why this exists

Neither stock aerodynamic plugin produces a usable wind force on a multirotor
hovering in wind.

**`gz-sim-lift-drag-system`** returns before computing anything when the
relative flow arrives from the rear hemisphere:

```cpp
if (forwardI.Dot(vel) <= 0.0){          // LiftDrag.cc:313
  return;                               // no lift, no drag, no moment
}
```

`vel` is `v_body - v_wind`. A T2 hovering at yaw 0 in the apriltag world's
`+5 0 0` wind has `vel = (-5,0,0)` and `forward = (1,0,0)`, so **all four
surfaces contribute exactly zero**. Reverse the wind and they contribute in
full — a hard discontinuity, and the likely mechanism behind the documented
nose-on/tail-on landing asymmetry.

**`gz-sim-advanced-lift-drag-system`** has no such gate and is the right answer
for forward flight, but it is still a linearised fixed-wing model and a level
vehicle in horizontal wind sits at `alpha ~ 0 or 180 deg`, outside its envelope:

- its post-stall drag term `CD_fp*(0.5 - 0.5*cos(2*alpha))` is **zero** at
  `alpha = 0` and `alpha = 180`, and the stall sigmoid has already scaled `CD0`
  out by then;
- at `alpha = 180` the sideslip is `beta = 180 deg`, and `CY = CYb*beta` is
  unbounded — with a typical `CYb = -0.3/rad` that is a fictitious ~350 N of
  sideforce at 25 m/s.

**`gz-sim-wind-effects-system`** applies `k * m * (v_wind - v_link)` at the
centre of mass: linear in velocity, proportional to **mass**, and producing **no
aerodynamic moment at all**. On the T2 (`k=0.1`, `m=2.37 kg`) that is 0.237 N
per m/s.

Net effect before this plugin: a 25 m/s wind produced **5.9 N** where quadratic
drag demands **22.5 N**, and the aircraft landed normally.

## What it does

**Drag**, per body axis, everywhere:

```
f_i = -0.5 * rho * cdA_i * v_i * |v_i|        i in {x,y,z}
```

`v*|v|` keeps the sign right in both directions. Each axis has its **own**
centre of pressure, because the area that faces sideways is not in the same
place as the area that faces down.

**Lift**, per surface, over the full ±180°:

```
alpha = atan2(-(v.u), v.f) + a0            wrapped to [-pi, pi]
CL    = (1-sigma)*(cla*alpha) + sigma*sin(2*alpha)
f     = CL * q * area * (s_hat x v_hat),   s = f x u
```

The linear branch is the ordinary thin-airfoil law and dominates head-on. The
`sin(2*alpha)` branch is the flat-plate/Newtonian law — what a wing in reversed
or heavily separated flow actually does — and it is **bounded and correctly
signed everywhere**, including `alpha = 180°`, where a rigged wing gets a
genuine *down*force rather than nothing. `sigma` is the usual stall sigmoid, so
the two join smoothly at `alpha_stall`.

**Surfaces contribute lift only.** All drag is the per-axis term, so there is
exactly one drag model and nothing double-counts. That was the defect in
shipping `BodyDrag` alongside `LiftDrag`: in the forward hemisphere the two
drags added, giving 1.66× the rear-hemisphere force for no physical reason.

Every surface parameter mirrors `gz-sim-lift-drag-system`'s own SDF element
names, so values copy across from an existing block verbatim.

## Parameters

| element | type | default | meaning |
|---|---|---|---|
| `<link_name>` | string | — | **required** |
| `<cda>` | Vector3 | `0 0 0` | drag area `Cd*A` per **body** axis [m²] |
| `<cp_drag>` | Vector3 | `0 0 0` | centre of pressure for all three drag axes |
| `<cp_drag_x/y/z>` | Vector3 | `<cp_drag>` | per-axis override |
| `<air_density>` | double | `1.2041` | ρ [kg/m³] |
| `<stall_blend>` | double | `15.0` | sharpness of the linear↔flat-plate blend |
| `<topic>` | string | — | optional; publishes the applied `gz.msgs.Wrench` |
| `<surface>` | element | — | repeated; `name`, `area`, `a0`, `cla`, `alpha_stall`, `cp`, `forward`, `upward` |

All `cp` values are relative to the link's **centre of mass**. `LiftDrag`'s are
relative to the **link origin**, so on this airframe they shift by +0.008481 m.

## Build and use

```bash
./build.sh                                  # -> build/libBodyAero.so
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

The landing harnesses in `px4-smc-control-ws/tools/` do this automatically and
warn if the library is missing. **Gazebo loads a model whose plugin cannot be
found without failing**, so a forgotten path looks exactly like a world with no
wind in it.

Standalone CMake, deliberately **not** wired into the PX4 build.

## Interaction with the other plugins

- **`LiftDrag` is disabled** on `base_link` (the four blocks are commented out
  in `model.sdf`). This plugin subsumes them and carries their parameters.
  Their `JointPositionController` siblings are untouched — the servos still move.
- **Keep `gz-sim-wind-effects-system` loaded.** It is what turns the world's
  `<wind>` seed into the gusting wind vector this plugin reads. Its
  `<force_approximation_scaling_factor>` is **0**, which was verified
  empirically: a free body in a 25 m/s wind shows the `x` and `y` fields
  *absent from every odometry message* — protobuf omits exactly-zero fields —
  while `z` falls under gravity proving the sim ran. The control at `k=0.1`
  drifts 3.52 m.

## Verification

An independent Python implementation of the same equations
(`scratchpad/ref.py`) reproduces the plugin to three decimals in every case.

Anchored body, gravity off, 10 m/s:

| case | plugin force [N] | reference |
|---|---|---|
| headwind | `[-3.612, 0, +16.416]` | identical |
| tailwind | `[+3.612, 0, −6.954]` | identical |
| crosswind | `[0, +3.552, 0]` | identical |
| from below | `[−6.954, 0, +90.910]` | identical |

Pitched attitudes (exercising the body↔world rotations), 10 m/s headwind:
+30°, +90°, +135°, −45° all match the reference to three decimals.

**Continuity**: sweeping the flow angle through the full 360° in 0.1° steps,
the largest force change between adjacent steps is **0.53 N**. A gate like
`LiftDrag`'s would show a step of tens of newtons.

Note the drag is now symmetric (`±3.612 N`) while the lift is not
(`+16.4` vs `−6.95 N`). That asymmetry is real aerodynamics — a wing rigged at
+3.43° makes far more lift in forward flow than in reversed flow — not a gate.

## Where the numbers come from

Projected collision areas, measured from `model.sdf`:
`A_x 0.1130`, `A_y 0.0333`, `A_z 1.2760 m²`.

| | value | basis |
|---|---|---|
| `cdA_x` | 0.060 | `LiftDrag`'s own profile drag area `Σ|cda·a0|·area` = 0.0397, + 0.006 rotors/nacelles from collision geometry, + the fuselage and gear the collision model omits. Independently, §05's closed form implies 0.0598. |
| `cdA_y` | 0.059 | `(A_y 0.0333 + rudder 0.020) × Cd 1.1`. **A lower bound** — this model has no fuselage or tail-boom collision geometry at all. |
| `cdA_z` | 1.510 | `A_z 1.2760 × Cd 1.18`, flat plate normal to flow at aspect ratio 3.9. **Acts on climb and descent too**: 3.6 N against a 2 m/s climb. |
| `cp_drag_*` | ±0.009 m | area-weighted centroids of the same collision geometry, per axis |
| surfaces | verbatim | `area`, `a0`, `cla`, `alpha_stall`, `cp`, `forward`, `upward` copied from the disabled `LiftDrag` blocks |

`cdA_y` is the weakest number and it is a lower bound; `cdA_z` is the one most
likely to change behaviour that was validated without it.

## Why the corrected model still does not fly, and what that means

Three separate defects were found. Two are fixed; the third is a real property
of the aircraft, not a bug.

**1. The T2 carried an X8's aerodynamics (fixed).** Every aero number in
`model.sdf` — `a0`, `cla`, `alpha_stall`, the areas `0.5/0.5/0.01`, the
collision box `0.55 x 2.144 x 0.05`, even the control-surface link poses
(±0.3, −0.85) — was inherited verbatim from PX4's `standard_vtol`, an X8 flying
wing of **5.03 kg and 2.144 m span**. The T2 is **2.49 kg and 1.246 m span**
(measured from `fuselage.dae`; confirmed against the real airframe at
1.2 × 1.1 m). Wing loading 2.41 kg/m² against `standard_vtol`'s 4.98.

`LiftDrag`'s rear-hemisphere gate hid this for the entire project: the apriltag
world's wind is a tailwind, so those surfaces produced nothing at all. Removing
the gate made an X8's aerodynamics act on half an X8.

**2. Surfaces produced lift but no drag (fixed).** Flat-plate theory gives one
normal force `CN = 2 sin(a)`, whose components are `sin(2a)` across the flow and
`2 sin^2(a)` along it. Modelling only the lift and leaving the rest to a
per-axis `<cda>` counted the wing twice at intermediate angles and not at all at
90°. Surfaces now carry the complete normal force and `<cda>` covers only
non-lifting bodies.

**3. The airframe is statically unstable in pitch in a tailwind (not a bug).**

```
dM/dtheta  =  +1.06 N.m/rad      divergence doubling time 0.24 s
```

Positive `dM/dtheta` is divergence. In forward flight a tail behind the CG is
what makes an aircraft stable; flying **backwards** the same tail is
destabilising, and this mission pins yaw to the platform heading by project
constraint, so the vehicle cannot weathervane out of it. Moving the wing's
normal force onto the CG only slows it to a 0.36 s doubling time — the tail
surfaces at x = −0.642 m carry the rest.

The vertical force is well behaved and properly damping in both directions
(`Fz` = +6.83 N at 2 m/s descent, −8.63 N at 2 m/s climb). **It is the moment,
not the force.**

### Measured, `stsmc`, 5 m/s, one run each

| configuration | result |
|---|---|
| `main` (old model, no aero moment at all) | lands, **1.9 cm** / 3.3 cm |
| BodyAero, zero force | lands, 5.0 cm |
| BodyAero, drag only (no surfaces) | lands, 17.7 cm |
| BodyAero, surfaces at 30% | climbs to 2.95 m, falls back, never recovers |
| BodyAero, full measured | never leaves the ground; ends inverted |
| BodyAero, X8 coefficients | flies 138 m away, ABORT |

The graded response across those rows is the signature of a real instability
rather than a coding error.

### What this means

**The old wind model applied no aerodynamic moment whatsoever** — WindEffects
acts at the centre of mass and LiftDrag was gated off — so the controller has
never once had to stabilise one. Every gain in this project was tuned in that
world. Turning on honest aerodynamics is therefore a controller problem, not a
plugin problem, and it is the genuinely useful finding here: a real T2 hovering
in a 5 m/s tailwind has this instability, and the current SMC does not reject it.

Next step is a controller question, not an aerodynamic one: either the attitude
loop is retuned against a known `dM/dtheta`, or the aerodynamic moment is
deliberately scaled down (`cp` toward the CG) and that scaling is declared as a
modelling assumption in the write-up.
