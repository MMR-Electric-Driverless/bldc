# Why `vd`/`vq` are computed twice in `control_current` (dead-time compensation)

Reference: `motor/mcpwm_foc.c`, function `control_current` (4547-4696) and
`update_valpha_vbeta` (5033+).

The lines that prompted this note (`mcpwm_foc.c:4717-4719`):

```c
// Dead time compensated values for vd and vq. Note that these are not used to control the switching times.
state_m->vd = c * motor->m_motor_state.v_alpha + s * motor->m_motor_state.v_beta;
state_m->vq = c * motor->m_motor_state.v_beta  - s * motor->m_motor_state.v_alpha;
```

These are **not** a redundant recompute. They replace the controller's *demanded*
`vd/vq` with the *actually applied* `vd/vq`, and the round-trip through αβ is how VESC
injects **dead-time compensation** into the reported values.

## The two `vd`/`vq` are different quantities

**First computation** (PI + decoupling + saturation, `mcpwm_foc.c:4627-4688`) = what the
controller **demands**. These are what actually drive the hardware: they become
`mod_d/mod_q` → inverse Park → `foc_svm` → PWM timings. The switching is committed from
here.

**Second computation** (`mcpwm_foc.c:4717-4719`) = what is **actually applied** to the
windings, transformed back to dq. Between the two, `update_valpha_vbeta` runs and sets
`state_m->v_alpha/v_beta`. For the maxim at speed (phase filters are off above
`foc_phase_filter_max_erpm = 4000`, and the operating point is ~30000 erpm), that path is:

```c
mod_alpha -= mod_alpha_comp;                       // 5095: subtract dead-time compensation
mod_beta  -= mod_beta_comp;                        // mod_comp_fact = foc_dt_us*1e-6 * foc_f_zv
...
state_m->v_alpha = mod_alpha * (2.0/3.0) * v_bus;  // 5163: reconstruct applied voltage
state_m->v_beta  = mod_beta  * (2.0/3.0) * v_bus;
```

Then lines 4717-4719 Park-transform that back to dq. The comment *"not used to control the
switching times"* means exactly this: the PWM was already set from the first `vd/vq`; this
second pair is only for **downstream consumers**.

## Why bother

The applied voltage differs from the demand because of **MOSFET dead-time**: for
`foc_dt_us` of every switching period, both transistors are off and the true terminal
voltage is pulled by the current sign, not the command. `update_valpha_vbeta` models that
(`mod_*_comp = sign(phase current) × foc_dt_us × foc_f_zv`) and subtracts it. The corrected
`vd/vq` are then used for:

- **Telemetry** — this is the `Vd/Vq` seen in VESC Tool / CAN / PlotJuggler logs
  (`Vd Dx`, `Vq Dx`). So the logged Vd/Vq are the *dead-time-compensated* values, not the
  raw PI demand.
- **The sensorless observer** — `foc_observer_update` needs the actual applied
  `v_alpha/v_beta` for an accurate flux estimate.
- Power / efficiency estimates.

### Low speed vs high speed (phase filters)

`update_valpha_vbeta` has two running branches:

- **Phase filters active** (`foc_phase_filter_enable` and `abs_rpm <
  foc_phase_filter_max_erpm`, i.e. below 4000 erpm): uses the *measured* phase-voltage
  magnitude (`v_mag_filter` from the ADC) with the modulation *direction*
  (`mcpwm_foc.c:5152-5154`).
- **Otherwise** (the maxim's high-speed case): reconstructs the applied voltage from the
  dead-time-compensated modulation (`mcpwm_foc.c:5163-5164`).

## The size of the difference

Without dead-time, the second computation is an exact inverse of the first:
`v = mod·(2/3)·v_bus` and `mod = v·1.5/v_bus` cancel, so `vd/vq` would come back identical.
The **only** delta is the dead-time term:

```
mod_comp_fact = foc_dt_us × foc_f_zv = 0.12e-6 × 30000 ≈ 0.0036
Δv ≈ mod_comp_fact × (2/3) × v_bus ≈ 0.0036 × 0.667 × 57 ≈ 0.14 V   (sign-dependent)
```

So ~0.1-0.15 V, sign set by phase-current direction (`foc_dt_us = 0.12`, `foc_f_zv = 30000`,
`v_bus ≈ 57` from `maxim_150.xml`).

## Do we need it in simverter?

**No.** simverter does not model dead-time or SVM/ADC quantization, so this back-transform
would be a pure identity round-trip — it would hand back exactly the `vd/vq` the sim
already logs. Adding it changes nothing about the physics.

The one caveat, relevant when comparing against real logs: the sim logs the
**controller-demanded** `vd/vq`, while real logs show the **dead-time-compensated**
`Vd/Vq`. That is a ~0.1 V systematic offset between them — negligible for the
duty/saturation story, but it is why sim `vd/vq` will not match logged `Vd/Vq` to the last
decimal.

For bit-for-bit log matching one could add an optional dead-time-compensation step
(`foc_dt_us` is already in the XML at 0.12) reproducing this ~0.14 V correction. Otherwise
it is not worth the complexity.
