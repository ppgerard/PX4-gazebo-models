# BodyDrag — quadratic bluff-body drag for gz-sim 8

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

```
v_rel  = v_link + omega x r_cp - v_wind        (world frame)
v_body = R^T * v_rel
f_i    = -0.5 * rho * cdA_i * v_i * |v_i|      i in {x,y,z}, body frame
torque = r_cp x f
```

`v*|v|` keeps the sign correct in both directions on each axis; there are no
gates and no small-angle linearisation, so it is valid at every relative-flow
angle. The per-axis `cdA` carries the airframe's anisotropy, and `cp` offset
from the centre of mass is what produces weathervaning and gust-induced
attitude upset.

## Parameters

| element | type | default | meaning |
|---|---|---|---|
| `<link_name>` | string | — | **required**, link to apply the wrench to |
| `<cda>` | Vector3 | `0 0 0` | drag area `Cd*A` per **body** axis [m²] |
| `<cp>` | Vector3 | `0 0 0` | centre of pressure, relative to the link's **centre of mass**, body frame [m]. `0 0 0` = force only, no moment |
| `<air_density>` | double | `1.2041` | ρ [kg/m³] |
| `<topic>` | string | — | optional; publishes the applied `gz.msgs.Wrench` |

## Build and use

```bash
./build.sh                                  # -> build/libBodyDrag.so
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

The landing harnesses in `px4-smc-control-ws/tools/` do this automatically and
warn if the library is missing. **Gazebo loads a model whose plugin cannot be
found without failing**, so a forgotten path looks exactly like a world with no
wind in it — hence the warning.

It is standalone CMake and deliberately **not** wired into the PX4 build, so it
cannot break `make px4_sitl`.

## Interaction with the other plugins

- **Keep `gz-sim-wind-effects-system` loaded** — it is what turns the world's
  `<wind>` seed into the gusting wind vector that this plugin reads. But set its
  `<force_approximation_scaling_factor>` to **0**, or its force double-counts
  with this one. Already done in `worlds/apriltag.sdf`.
- **Keep the `LiftDrag` surfaces.** They still matter for the fixed-wing phase.
  This plugin is the complementary bluff-body term, not a replacement.

## Verification

Anchored body, gravity off, wind ±25 m/s, `cdA = (0.06, 0.12, 1.30)`:

| case | measured | closed form `½ρ·CdA·V²` |
|---|---|---|
| wind `+25 0 0`, nose +x | **+22.5769 N** | 22.5769 N |
| wind `-25 0 0`, nose +x | **−22.5769 N** | −22.5769 N (exact mirror — no gate) |
| wind `+25 0 0`, yaw 90° | **+45.1537 N** | 45.1537 N (`cdA_y` = 2×`cdA_x`) |

Torque follows `r_cp x f`: zero when the force is along `cp`, `0.1598 N·m` in
the yawed case (`0.15 m × 1.0653 N`).

## Choosing cdA — read this before trusting a result

Only the **x** value is anchored to anything independent. The review's §05
closed-form estimate of 0.9 N at 5 m/s implies `CdA = 0.0598 m²`, which is where
`0.06` comes from. **`y` and `z` are geometric estimates and are not confirmed.**

`cdA_z = 1.30` acts on **climb and descent**, not only on wind — it is worth
3.1 N (13% of weight) against a 2 m/s climb, so it will change takeoff and
descent behaviour that was validated without it. Set `<cda>0.06 0.12 0</cda>` to
isolate the horizontal wind question first.
