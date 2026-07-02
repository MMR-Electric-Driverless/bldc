# simverter

Offline, **closed-loop** simulator of the VESC FOC current controller.

It replays *your firmware's* current-control chain (not an idealized model) against a
prescribed RPM ramp, a fixed commanded q-axis current, and a fixed battery voltage, then
writes every internal signal to a CSV so you can open it in PlotJuggler (there is a
`time` column for exactly that).

Unlike a steady-state solve, this runs the actual PI current loop at the real control
frequency and closes the loop with a small dq motor plant. So the voltage budget genuinely
runs out as RPM rises, **vq genuinely saturates**, field weakening kicks in, and iq droops
— the same causal chain that happens on the bench.

## Inputs

- `maxim_150.xml` — the mcconf. Motor electricals, PI gains, limits, field-weakening
  params and the switching frequency are all read from here.
- Firmware behavior — the control chain is ported 1:1 from the C, with `file:line`
  comments in `simverter.py` pointing at the source of every step.

## Usage

```bash
# accelerating ramp, 95 A demand, 57 V battery, 0 -> 7500 MECHANICAL rpm at 5000 rpm/s
python3 simverter.py --iq 95 --vbus 57 --rpm-start 0 --rpm-end 7500 --accel 5000

# fixed speed (accel = 0): hold 3250 mechanical rpm for 0.3 s
python3 simverter.py --iq 95 --vbus 57 --rpm-start 3250 --accel 0 --sim-time 0.3

# same ramp expressed directly in ELECTRICAL erpm instead of mechanical rpm
python3 simverter.py --iq 95 --vbus 57 --rpm-end 30000 --accel 20000 --erpm-input

# also render a PNG overview (needs matplotlib)
python3 simverter.py --iq 95 --vbus 57 --rpm-end 7500 --accel 5000 --plot
```

Everything defaults from `maxim_150.xml`; CLI flags override. Output CSV defaults to
`sim.csv` in this folder.

> **rpm is MECHANICAL (physical) by default.** The electrical speed the firmware
> controls with is `erpm = rpm * pole_pairs` (`pole_pairs = si_motor_poles / 2 = 4`).
> Pass `--erpm-input` if your numbers are already electrical. This is the usual gotcha:
> a VESC "rpm" telemetry channel is normally erpm, and the electrical omega is what sets
> back-EMF and therefore duty.

### Key flags

| flag | meaning |
|------|---------|
| `--iq` | fixed commanded q-axis current [A] (the "current required"); negative = braking |
| `--vbus` | fixed battery voltage [V] |
| `--rpm-start` / `--rpm-end` | ramp endpoints, **mechanical rpm** (or erpm with `--erpm-input`) |
| `--accel` | rpm per second; **0 = fixed speed** at `--rpm-start` |
| `--sim-time` | override total sim time [s] (needed for fixed-speed runs) |
| `--hold-time` | extra seconds held at `--rpm-end` after the ramp |
| `--erpm-input` | interpret the ramp numbers as electrical erpm instead of mechanical rpm |
| `--control-freq` | override the control loop frequency [Hz] (else derived from config) |
| `--phase-shunts` | board has inline phase shunts (maxim does **not**); only then does V0_V7 give 30 kHz |
| `--out` | output CSV path |
| `--plot` | also write a PNG next to the CSV |

## The control chain (one tick = 1/control_freq)

Ported in firmware order, each with the source location:

| step | what | source |
|------|------|--------|
| 1 | duty low-pass filter (`UTILS_LP_FAST(..., 0.01)`) | `mcpwm_foc.c:3357-3364` |
| 2 | `foc_run_fw` — field weakening (duty-driven, ramp-limited) | `foc_math.c:702-742` |
| 3 | MTPA + FW fold + current limits → `id_target`, `iq_target` | `mcpwm_foc.c:3614-3658` |
| 4 | `control_current` — PI loop + **voltage saturation** | `mcpwm_foc.c:4547-4696` |
| 5 | motor plant (dq `di/dt`) — *the only non-firmware part*, closes the loop | — |
| 6 | speed estimator — `m_speed_est_fast` (LP of dφ/dt) | `mcpwm_foc.c:3819-3822` |

### Electric omega: true vs estimated

The ramp gives **mechanical rpm**. The three speed representations and where the firmware
relates them:

- `erpm = mech_rpm × pole_pairs` — inverse done in `mc_interface.c:1646`
- `we = RPM2RADPS_f(erpm) = erpm × 2π/60` [rad/s] — `utils_math.h:77`; inverse
  `erpm = RADPS2RPM_f(we)` (`utils_math.h:78`), as used by `mcpwm_foc_get_rpm()` at
  `mcpwm_foc.c:1159`

From `we` the sim integrates the true electrical angle. But the firmware's decoupling
feed-forward does **not** use the true speed — it uses `m_speed_est_fast`, a heavily
low-passed (k=0.01) estimate of `d(phase)/dt` (`mcpwm_foc.c:3819-3822`). The sim runs that
same estimator and feeds its output (`we_est` in the CSV) into the controller, so the
decoupling inherits the estimator's lag during acceleration (a few rad/s, ≈
`dt/0.01 × ramp_rate`). The motor plant uses the true `we`. Compare the `we` and `we_est`
columns to see the lag.

> The firmware never actually has the true speed — only estimates (`m_speed_est_fast`,
> `m_pll_speed`). In the sim `we` is the imposed ground truth; that's why the value handed
> to `control_current` is named `we_est`, not `we`.

## Control frequency (the maxim runs at 15 kHz, not 30 kHz)

`dt` is taken directly from config at the top of the FOC interrupt
(`mcpwm_foc.c:2996-3005`):

```c
#ifdef HW_HAS_PHASE_SHUNTS
    if (foc_control_sample_mode == FOC_CONTROL_SAMPLE_MODE_V0_V7)
        dt = 1.0 / foc_f_zv;            // 30 kHz  (sample twice per PWM period)
    else
        dt = 1.0 / (foc_f_zv / 2.0);    // 15 kHz
#else                                   // no phase shunts
    dt = 1.0 / (foc_f_zv / 2.0);        // 15 kHz  (sample once per period)
#endif
```

The `foc_control_sample_mode` (V0_V7) **only matters when `HW_HAS_PHASE_SHUNTS` is
defined**. The maxim (`hwconf/vesc/maxim/hw_maxim_core.h`) defines `HW_HAS_3_SHUNTS` and
`HW_HAS_PHASE_FILTERS` but **not** `HW_HAS_PHASE_SHUNTS` (those are different: low-side leg
shunts + phase-voltage RC filters vs. inline phase-current shunts). So it takes the `#else`
branch: `dt = 1/(30000/2) = 1/15000` → **15 kHz** (66.7 µs), regardless of the sample mode.

So simverter **defaults to 15 kHz** for the maxim. Pass `--phase-shunts` only for a board
that actually has inline phase shunts (then V0_V7 gives 30 kHz), or `--control-freq <Hz>` to
force any value. This matters because both the FW ramp and the 0.01 duty filter integrate
per tick.

## Where vq is limited / saturated

All of it is inside `control_current` (`mcpwm_foc.c`):

- `max_v_mag = 1/sqrt(3) * max_duty * v_bus * overmod` — the voltage budget (`:4666`)
- vd (and `vd_int`) truncated to ±`max_v_mag` — **d-axis has priority** (`:4675-4676`)
- `max_vq = sqrt(max_v_mag^2 - vd^2)` — whatever budget is left (`:4681`)
- **vq (and `vq_int`) truncated to ±`max_vq`** — the clamp that starves iq (`:4683-4684`)
- `utils_saturate_vector_2d(vd, vq, max_v_mag)` — final vector clamp (`:4688`)

In the CSV, watch `max_v_mag`, `max_vq`, `vq`, `bemf` (= `we*lambda`) and the boolean
`vq_saturated`. vq saturates roughly when `bemf + Rs*iq` starts to exceed `max_vq`; that is
the RPM at which the motor runs out of voltage and field weakening takes over.

## CSV columns

`time, rpm, erpm, we, we_est, iq_cmd, i_fw_set, mtpa_id, id_target, iq_target,
id, iq, i_abs, vd, vq, vd_int, vq_int, bemf, max_v_mag, max_vq, mod_d, mod_q, mod_q_filter,
duty, duty_filtered, vd_saturated, vq_saturated`

- `rpm` = mechanical, `erpm` = electrical (`rpm × pole_pairs`)
- `we` = true electrical omega [rad/s]; `we_est` = estimated (`m_speed_est_fast`)

## Notes / limitations

- The motor plant is a standard salient-PMSM electrical model, forward-Euler integrated at
  the control `dt` (`dt << L/R`, so stable and accurate). It stands in for the measured,
  Park-transformed currents.
- HFI, audio, dead-time compensation, the observer/PLL and thermal/throttle derating are
  intentionally omitted — the sim assumes `m_speed_est_fast == we` (RPM is prescribed) and
  that `lo_*` override limits equal the base `l_*` limits (no derating).
- This models the *current setpoint + current loop*, not mechanical dynamics: RPM is an
  imposed input, not a result of torque.
