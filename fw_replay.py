#!/usr/bin/env python3
"""
Offline replay of the VESC FOC current-setpoint generator + field weakening.

Goal: isolate YOUR firmware's setpoint logic from the motor, feed it the logged
torque command (iq) and rpm, and see whether the *setpoints* diverge toward the
-150 A you observe on the bench. If they DON'T (they shouldn't, given the clamps),
then the -150 A is measured current under current-loop voltage saturation, not a
setpoint-generation runaway -- which points the fix at axis decoupling / the
current loop, not at clamping m_i_fw_set.

Ports faithfully (branch dev/release_6_06):
  - mcpwm_foc.c:3539-3586  (MTPA + FW fold + current limits)
  - foc_math.c:702-742     (foc_run_fw, duty-driven, ramp-limited)
  - voltage/duty model from mcpwm_foc.c:3718-3730, 4594

Three tests in one:
  #1 setpoint replay (clamped vs unclamped m_i_fw_set, A/B)
  #2 voltage-budget crossing  ->  rpm where sqrt(vd^2+vq^2) hits Vbus/sqrt(3)
  #3 foc_run_fw unit assertion (m_i_fw_set <= foc_fw_current_max when clamped)

Usage:
  python3 fw_replay.py                       # synthetic rpm ramp (no data needed)
  python3 fw_replay.py log.csv               # replay a PlotJuggler CSV export
  python3 fw_replay.py log.csv --t0 35 --t1 45   # zoom analysis to a time window [s]
Columns expected in CSV (rename in COLMAP below to match your export):
  time, rpm, iq_cmd   (iq_cmd = "Torque Strat" channel, the commanded iq)
"""

import math
import numpy as np

try:
    import matplotlib.pyplot as plt
    HAVE_PLT = True
except ImportError:
    HAVE_PLT = False

# ----------------------------------------------------------------------------
# CONFIG -- FILL THESE FROM YOUR ACTUAL mcconf (VESC Tool -> motor cfg).
# Defaults below are the luna/bbshd reference values, NOT your motor. CHANGE THEM.
# ----------------------------------------------------------------------------
class Conf:
    # Motor electrical -- maxim150_37.xml (the real config)
    foc_motor_r            = 0.03          # Rs [ohm]
    foc_motor_l            = 9e-05         # L (average) [H]  -> Ld=L-diff/2, Lq=L+diff/2
    foc_motor_ld_lq_diff   = 4e-05         # Lq - Ld [H] (saliency)
    foc_motor_flux_linkage = 0.0115        # lambda [Wb]
    si_motor_poles         = 8             # pole COUNT (pairs = poles/2 = 4)

    # Current limits
    lo_current_max         = 200.0         # l_current_max
    lo_current_min         = -200.0
    lo_in_current_max      = 200.0         # l_in_current_max
    lo_in_current_min      = -200.0

    # Field weakening
    foc_fw_current_max     = 70.0          # A
    foc_fw_duty_start      = 0.9
    foc_fw_ramp_time       = 0.2           # s
    foc_fw_q_current_factor= 0.0
    cc_min_current         = 0.0

    # Modulation / bus
    l_max_duty             = 0.95
    foc_overmod_factor     = 1.0
    v_bus                  = 50.0          # 14S nominal ~50V -- SET TO THE LOGGED Vbus

    # MTPA: 0=OFF, 1=MTPA_MODE_IQ (commanded), 2=MTPA_MODE_IQ_MEASURED
    foc_mtpa_mode          = 1

    # NOTE: rpm input to electrical() is MECHANICAL motor rpm. erpm = rpm * poles/2.

    dt                     = 1.0/1000.0    # foc_run_fw runs from the 1 kHz timer

C = Conf()

MTPA_MODE_OFF = 0
MTPA_MODE_IQ_MEASURED = 2

# ----------------------------------------------------------------------------
# small utils ported 1:1 from util/utils_math.h
# ----------------------------------------------------------------------------
def utils_map(x, in_min, in_max, out_min, out_max):           # NON-clamping
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def utils_step_towards(val, goal, step):
    if val < goal:
        return min(val + step, goal)
    elif val > goal:
        return max(val - step, goal)
    return val

def truncate_abs(x, lim):
    return max(-lim, min(lim, x))

def truncate(x, lo, hi):
    return max(lo, min(hi, x))

def sign(x):
    return 0.0 if x == 0 else (1.0 if x > 0 else -1.0)

# ----------------------------------------------------------------------------
# Quasi-steady-state electrical model (closes the duty feedback loop).
# Ignores di/dt -> this is the STEADY part. The transient Ld*(di/dt) accelerant
# is deliberately NOT modelled here; if the SS setpoints already diverge, di/dt
# is not required to explain it. If they don't, di/dt / control loss is implicated.
# ----------------------------------------------------------------------------
def electrical(id_, iq_, rpm):
    pole_pairs = C.si_motor_poles / 2.0
    we = rpm / 60.0 * 2.0 * math.pi * pole_pairs          # elec rad/s
    Ld = C.foc_motor_l - C.foc_motor_ld_lq_diff / 2.0
    Lq = C.foc_motor_l + C.foc_motor_ld_lq_diff / 2.0
    vd = C.foc_motor_r * id_ - we * Lq * iq_
    vq = C.foc_motor_r * iq_ + we * (C.foc_motor_flux_linkage + Ld * id_)
    v_mod = math.hypot(vd, vq)
    # duty = sqrt(3)*v_mod/(Vbus*overmod);  duty_abs clamped to 1.0 (mcpwm_foc.c:3293)
    duty = math.sqrt(3.0) * v_mod / (C.v_bus * C.foc_overmod_factor)
    return vd, vq, v_mod, min(abs(duty), 1.0)

def v_budget():
    # max linear-SVM voltage magnitude (mcpwm_foc.c:4594)
    return (1.0 / math.sqrt(3.0)) * C.l_max_duty * C.v_bus * C.foc_overmod_factor

# ----------------------------------------------------------------------------
# foc_run_fw  (foc_math.c:702-742).  clamp=True applies the proposed one-line fix.
# ----------------------------------------------------------------------------
def foc_run_fw(state, duty_abs, clamp, dt):
    if C.foc_fw_current_max < max(C.cc_min_current, 0.001):
        return
    fw_current_now = 0.0
    if C.foc_fw_duty_start < 0.99 and duty_abs > C.foc_fw_duty_start * C.l_max_duty:
        fw_current_now = utils_map(duty_abs,
                                   C.foc_fw_duty_start * C.l_max_duty,
                                   C.l_max_duty,
                                   0.0, C.foc_fw_current_max)
        if clamp:                                    # <-- proposed fix
            fw_current_now = min(fw_current_now, C.foc_fw_current_max)
    if C.foc_fw_ramp_time < dt:
        state['i_fw'] = fw_current_now
    else:
        state['i_fw'] = utils_step_towards(state['i_fw'], fw_current_now,
                                           (dt / C.foc_fw_ramp_time) * C.foc_fw_current_max)

# ----------------------------------------------------------------------------
# setpoint block (mcpwm_foc.c:3539-3586). Returns id_target, iq_target.
# iq_filter/mod_q come from the model (feedback) -- approximated by last step.
# ----------------------------------------------------------------------------
def setpoint_block(iq_cmd, state, mod_q, iq_filter):
    id_set = 0.0
    iq_set = iq_cmd
    ld_lq_diff = C.foc_motor_ld_lq_diff

    if C.foc_mtpa_mode != MTPA_MODE_OFF and ld_lq_diff != 0.0:
        lam = C.foc_motor_flux_linkage
        iq_ref = iq_set
        if C.foc_mtpa_mode == MTPA_MODE_IQ_MEASURED:
            iq_ref = iq_set if abs(iq_set) < abs(iq_filter) else iq_filter  # utils_min_abs
        id_set = (lam - math.sqrt(lam*lam + 8.0*(ld_lq_diff*iq_ref)**2)) / (4.0*ld_lq_diff) - state['i_fw']
        i_diff = iq_set*iq_set - id_set*id_set
        if i_diff < 0.0:
            i_diff = 0.0
        iq_set = sign(iq_set) * math.sqrt(i_diff)
    else:
        id_set -= state['i_fw']
        iq_set -= sign(mod_q) * state['i_fw'] * C.foc_fw_q_current_factor

    # input-current limit (skipped if lo_in_current huge)
    if mod_q > 0.001:
        iq_set = truncate(iq_set, C.lo_in_current_min/mod_q, C.lo_in_current_max/mod_q)
    elif mod_q < -0.001:
        iq_set = truncate(iq_set, C.lo_in_current_max/mod_q, C.lo_in_current_min/mod_q)

    if mod_q > 0.0:
        iq_set = truncate(iq_set, C.lo_current_min, C.lo_current_max)
    else:
        iq_set = truncate(iq_set, -C.lo_current_max, -C.lo_current_min)

    current_max_abs = abs(max(C.lo_current_max, C.lo_current_min, key=abs))
    id_set = truncate_abs(id_set, current_max_abs)                      # line 3580
    iq_set = truncate_abs(iq_set, math.sqrt(max(0.0, current_max_abs**2 - id_set**2)))
    return id_set, iq_set

# ----------------------------------------------------------------------------
# driver
# ----------------------------------------------------------------------------
# Physical sanity bounds for input channels. Samples outside these are treated as
# telemetry/decode glitches (e.g. a uint read as int -> ~1e9 rpm) and interpolated over,
# which keeps the time base / dt intact (m_i_fw_set is a stateful ramp, so dropping or
# zeroing samples would distort the FW dynamics).
RPM_SANE_MAX = 30000.0          # mechanical motor rpm; anything above is garbage
IQ_SANE_MAX  = 400.0            # A; above l_abs_current_max -> garbage
IQ_CMD_SCALE = 0.001            # Torque Strat packed as scaled int over CAN (35000 -> 35.0 A)

GAP_MAX_S = 0.05      # interpolate telemetry gaps up to this long; leave longer ones as breaks

def _sanitize(t, y, lim, name, gap_max_s=None):
    """Replace |y|>lim (and NaN/inf) by interpolation. If gap_max_s is set, only fill
    bad runs shorter than that and leave longer runs as NaN -- so a real telemetry
    dropout shows as a line break instead of a fake straight line that looks misaligned."""
    bad = ~(np.abs(y) <= lim)           # also catches NaN/inf
    n = int(bad.sum())
    if not n:
        return y
    good = ~bad
    if good.sum() < 2:
        raise ValueError(f"{name}: too few valid samples to interpolate ({good.sum()})")
    y = y.copy()
    filled = np.interp(t, t[good], y[good])
    if gap_max_s is None:
        y[bad] = filled[bad]
        print(f"sanitized {name}: {n}/{len(y)} glitch samples interpolated (|{name}| > {lim:g})")
    else:
        edges = np.diff(np.r_[0, bad.view(np.int8), 0])  # run starts (+1) / ends (-1)
        starts, ends = np.where(edges == 1)[0], np.where(edges == -1)[0]
        kept = 0
        for s, e in zip(starts, ends):                   # [s, e) is a bad run
            if t[e - 1] - t[s] <= gap_max_s:
                y[s:e] = filled[s:e]
            else:
                y[s:e] = np.nan                          # honest break
                kept += e - s
        print(f"sanitized {name}: {n}/{len(y)} bad samples; filled {n - kept}, "
              f"left {kept} as gap breaks (gap > {gap_max_s*1e3:.0f} ms)")
    return y

def load_csv(path):
    COLMAP = {'time':    'Time',
              'rpm':     'Hybrid Control System Left RPM Sx',
              'iq_cmd':  'Hybrid Control System Left Torque Strat',
              'id_meas': 'Hybrid Control System Left Id Sx',   # measured id, CAN status 7
              'iq_meas': 'Hybrid Control System Left Iq Sx'}   # measured iq, CAN status 7
    import csv
    def fnum(v, blank):                      # parse cell, blank -> blank fill
        return float(v) if v not in ('', None) else blank
    cols = {k: [] for k in COLMAP}
    with open(path) as f:
        r = csv.DictReader(f)
        # measured id/iq are optional -- only overlay them if both columns exist
        have_meas = (COLMAP['id_meas'] in r.fieldnames and COLMAP['iq_meas'] in r.fieldnames)
        for row in r:
            cols['time'].append(fnum(row[COLMAP['time']], 0.0))
            cols['rpm'].append(fnum(row[COLMAP['rpm']], 0.0))
            cols['iq_cmd'].append(fnum(row[COLMAP['iq_cmd']], 0.0))
            if have_meas:                    # blank -> NaN so _sanitize interpolates it
                cols['id_meas'].append(fnum(row[COLMAP['id_meas']], np.nan))
                cols['iq_meas'].append(fnum(row[COLMAP['iq_meas']], np.nan))
    t   = np.array(cols['time'])
    rpm = _sanitize(t, np.array(cols['rpm']),    RPM_SANE_MAX, 'rpm')
    # Torque Strat is packed as a scaled int over CAN (e.g. 35000 == 35.0 A) -> rescale
    # BEFORE sanitizing, otherwise the raw values trip IQ_SANE_MAX and get interpolated away.
    iqc = _sanitize(t, np.array(cols['iq_cmd']) * IQ_CMD_SCALE, IQ_SANE_MAX, 'iq_cmd')
    meas = None
    if have_meas:   # gap-aware: real status-7 dropouts stay as breaks, not interpolation
        meas = {'id': _sanitize(t, np.array(cols['id_meas']), IQ_SANE_MAX, 'id_meas', GAP_MAX_S),
                'iq': _sanitize(t, np.array(cols['iq_meas']), IQ_SANE_MAX, 'iq_meas', GAP_MAX_S)}
    return t, rpm, iqc, meas

def synth():
    # mimics the PlotJuggler trace: rpm ramps 0->8000 then falls; iq_cmd ~ -45 A
    t = np.arange(0, 12, C.dt)
    rpm = np.interp(t, [0, 6, 9, 12], [0, 8000, 4000, 1500])
    iqc = np.full_like(t, -45.0)
    return t, rpm, iqc, None       # no measured channels in synthetic mode

def run(t, rpm, iqc, clamp):
    state = {'i_fw': 0.0}
    out = {k: np.zeros_like(t) for k in
           ('id', 'iq', 'ifw', 'vmod', 'duty', 'vd', 'vq')}
    # real per-sample dt from the time column (FW ramp must integrate in real time,
    # not a fixed 1 kHz tick -- otherwise the reconstructed setpoint lags the log).
    dts = np.diff(t, prepend=t[0] - (t[1] - t[0] if len(t) > 1 else C.dt))
    dts = np.clip(dts, 1e-6, None)
    mod_q, iq_filter, duty_abs = 0.0, 0.0, 0.0
    for i in range(len(t)):
        foc_run_fw(state, duty_abs, clamp, dts[i])
        id_set, iq_set = setpoint_block(iqc[i], state, mod_q, iq_filter)
        vd, vq, vmod, duty_abs = electrical(id_set, iq_set, rpm[i])
        # feedback approximations for next iter
        mod_q = vq * (1.5 / C.v_bus)
        iq_filter = iq_set
        out['id'][i], out['iq'][i], out['ifw'][i] = id_set, iq_set, state['i_fw']
        out['vmod'][i], out['duty'][i] = vmod, duty_abs
        out['vd'][i], out['vq'][i] = vd, vq
    return out

def main():
    import argparse
    p = argparse.ArgumentParser(description="VESC FOC setpoint + FW offline replay")
    p.add_argument('csv', nargs='?', help="PlotJuggler CSV export (omit for synthetic ramp)")
    p.add_argument('--t0', type=float, default=None, help="window start time [s]")
    p.add_argument('--t1', type=float, default=None, help="window end time [s]")
    p.add_argument('--t-shift', type=float, default=0.0, dest='t_shift',
                   help="add a constant offset [s] to the time axis (to match a live "
                        "PlotJuggler view whose clock origin differs from the CSV export)")
    args = p.parse_args()

    if args.csv:
        t, rpm, iqc, meas = load_csv(args.csv); src = args.csv
    else:
        t, rpm, iqc, meas = synth(); src = "synthetic ramp"

    if args.t_shift:
        t = t + args.t_shift

    # time-base sanity: the FW ramp integrates in real time, so a slow/irregular
    # telemetry rate (vs the firmware's 1 kHz) directly affects reconstruction.
    d = np.diff(t)
    if len(d):
        nonmono = int((d <= 0).sum())
        print(f"time: {t[0]:.3f}..{t[-1]:.3f} s  median dt={np.median(d)*1e3:.2f} ms  "
              f"(firmware tick=1.00 ms)  non-monotonic steps={nonmono}")
        if nonmono:
            print("  WARNING: time column is not strictly increasing -- "
                  "samples may be misordered/duplicated, which misaligns the overlay.")

    # Run the FULL trace (FW state is a stateful ramp -- never crop before running),
    # then restrict the view/stats to the requested window.
    a = run(t, rpm, iqc, clamp=False)   # firmware as-is
    b = run(t, rpm, iqc, clamp=True)    # with m_i_fw_set clamp fix
    vbud = v_budget()

    t0 = args.t0 if args.t0 is not None else t[0]
    t1 = args.t1 if args.t1 is not None else t[-1]
    w = (t >= t0) & (t <= t1)
    if not w.any():
        print(f"window [{t0}, {t1}] s contains no samples (data spans "
              f"{t[0]:.3f}..{t[-1]:.3f} s)"); return

    print(f"source: {src}   samples: {int(w.sum())}/{len(t)} in window "
          f"[{t0:.3f}, {t1:.3f}] s   Vbus={C.v_bus} V")
    print(f"voltage budget  Vbus/sqrt(3)*max_duty*overmod = {vbud:.2f} V")
    # #2 crossing rpm (first crossing inside the window)
    cross = np.where(w & (a['vmod'] >= vbud))[0]
    if len(cross):
        print(f"#2 budget crossed first at t={t[cross[0]]:.3f}s  rpm={rpm[cross[0]]:.0f}  "
              f"(this is the FW-onset / saturation rpm)")
    else:
        print("#2 budget never crossed in this window")
    # #1 divergence (windowed)
    print(f"#1 unclamped: id min={a['id'][w].min():.1f} A  iq min={a['iq'][w].min():.1f} A  "
          f"m_i_fw_set max={a['ifw'][w].max():.1f} A")
    print(f"#1 clamped:   id min={b['id'][w].min():.1f} A  iq min={b['iq'][w].min():.1f} A  "
          f"m_i_fw_set max={b['ifw'][w].max():.1f} A")
    # #3 assertion (windowed)
    viol = a['ifw'][w].max() - C.foc_fw_current_max
    print(f"#3 foc_run_fw assertion: max m_i_fw_set exceeds foc_fw_current_max by "
          f"{viol:+.1f} A {'(FAIL: utils_map extrapolation)' if viol > 1e-3 else '(ok)'}")
    cmax = abs(max(C.lo_current_max, C.lo_current_min, key=abs))
    print(f"NOTE: id setpoint is hard-clamped to current_max_abs={cmax:.0f} A (mcpwm_foc.c:3580). "
          f"If the bench shows |i| >> the offline id above, the runaway is MEASURED current "
          f"(current-loop voltage saturation), not setpoint generation.")
    # measured id/iq from CAN status 7 -- the decisive overlay
    if meas is not None:
        imeas_abs = np.hypot(meas['id'][w], meas['iq'][w])
        ngap = int(np.isnan(meas['id'][w]).sum())
        print(f"#0 MEASURED (status7): id min={np.nanmin(meas['id'][w]):.1f} A  "
              f"iq min={np.nanmin(meas['iq'][w]):.1f} A  |i| max={np.nanmax(imeas_abs):.1f} A"
              f"  ({ngap} samples in telemetry gaps)")
        gap = abs(np.nanmin(meas['id'][w])) - abs(a['id'][w].min())
        print(f"   measured |id| exceeds reconstructed setpoint |id| by {gap:+.1f} A "
              f"{'<-- control loss: measured >> setpoint' if gap > 10 else '(tracks setpoint)'}")

    if HAVE_PLT:
        # plot only the windowed slice so the y-axes autoscale to the section
        fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        ax[2].plot(t[w], rpm[w], 'g'); ax[2].set_ylabel('rpm')
        ax[1].plot(t[w], a['id'][w], 'b', label='id (unclamped fw)')
        ax[1].plot(t[w], b['id'][w], 'b--', label='id (clamped fw)')
        ax[1].plot(t[w], a['iq'][w], 'r', label='iq (unclamped fw)')
        ax[1].plot(t[w], a['ifw'][w], 'm', label='m_i_fw_set')
        if meas is not None:        # measured id/iq from CAN status 7
            ax[1].plot(t[w], meas['id'][w], color='tab:cyan',   lw=1.3, label='id MEASURED (status7)')
            ax[1].plot(t[w], meas['iq'][w], color='tab:orange', lw=1.3, label='iq MEASURED (status7)')
        ax[1].axhline(-cmax, color='k', ls=':', lw=0.8, label='id clamp')
        ax[1].set_ylabel('A'); ax[1].legend(fontsize=7)
        ax[0].plot(t[w], a['vmod'][w], 'c', label='|v| = sqrt(vd^2+vq^2)')
        ax[0].plot(t[w], a['vd'][w], 'b', lw=0.7, label='vd')
        ax[0].plot(t[w], a['vq'][w], 'r', lw=0.7, label='vq')
        ax[0].axhline(vbud, color='k', ls='--', label='Vbus/sqrt(3) budget')
        ax[0].set_ylabel('V'); ax[2].set_xlabel('time [s]'); ax[0].legend(fontsize=7)
        ax[2].set_title(f"{src}   window [{t0:.2f}, {t1:.2f}] s", fontsize=9)
        import os
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fw_replay.png')
        plt.tight_layout(); plt.savefig(out, dpi=110)
        print(f"plot -> {out}")

if __name__ == '__main__':
    main()
