#!/usr/bin/env python3
"""
simverter -- an offline, closed-loop simulator of the VESC FOC current controller.

WHAT IT DOES
------------
It replays *your firmware's* current-control chain (not an idealized model) against a
prescribed RPM ramp, a fixed commanded q-axis current and a fixed battery voltage, and
writes every internal signal to a CSV so you can open it in PlotJuggler (there is a
`time` column for exactly that).

Unlike the older `fw_replay.py` (which solved the electrical equations algebraically at
each RPM, i.e. steady-state), this one runs the *actual* PI current loop
(`control_current`, mcpwm_foc.c:4547) at the real control frequency and closes the loop
with a small dq motor plant. That means the voltage budget genuinely runs out as RPM
rises, vq genuinely saturates, field weakening genuinely kicks in, and iq genuinely
droops -- the same causal chain that happens on the bench. If you only want the setpoint
math without the loop dynamics, the steady-state approach in git history is simpler; this
file exists to *see the saturation happen*.

THE CONTROL CHAIN THAT IS PORTED (one control tick = 1/control_freq seconds)
---------------------------------------------------------------------------
Each tick reproduces, in firmware order:

  1. Duty low-pass filter          mcpwm_foc.c:3357-3364   UTILS_LP_FAST(..., 0.01)
  2. foc_run_fw (field weakening)  foc_math.c:702-742      duty-driven, ramp-limited
  3. MTPA + FW fold + current lims mcpwm_foc.c:3614-3658   -> id_target, iq_target
  4. control_current (PI loop)     mcpwm_foc.c:4547-4696   -> vd, vq (SATURATED here)
  5. motor plant (dq di/dt)        not in firmware         closes the loop -> new id, iq
  6. speed estimator               mcpwm_foc.c:3819-3822   -> m_speed_est_fast

SPEED: the ramp is given as MECHANICAL (physical) rpm. The three representations:
  erpm = mech_rpm * pole_pairs         (inverse in mc_interface.c:1646)
  we   = RPM2RADPS_f(erpm)             (utils_math.h:77; = erpm*2*pi/60)  [rad/s]
The true electrical omega `we` is integrated into an electrical angle and fed through
VESC's own speed estimator (a k=0.01 low-pass of d(phase)/dt, mcpwm_foc.c:3819-3822) to
get the ESTIMATED electric omega `we_est` (= m_speed_est_fast). The controller's
decoupling uses `we_est` (so it inherits the estimator's lag during acceleration); the
motor plant uses the true `we`. CSV columns: `we` (true) and `we_est` (estimate).

----------------------------------------------------------------------
All of it happens inside `control_current`, mcpwm_foc.c:

  * max_v_mag = ONE_BY_SQRT3 * max_duty * v_bus * overmod           (line 4666)
        -> the length of the largest voltage vector the inverter can synthesise
           without overmodulation. This is the whole "voltage budget".
  * vd (and vd_int) truncated to +/-max_v_mag                       (lines 4675-4676)
        -> the D axis gets FIRST pick of the budget ("d-axis has priority", because it
           does field weakening).
  * max_vq = sqrt(max_v_mag^2 - vd^2)                               (line 4681)
        -> whatever budget is LEFT after vd is the ceiling for vq. As RPM rises and FW
           pushes vd more negative, this ceiling SHRINKS.
  * vq (and vq_int) truncated to +/-max_vq                          (lines 4683-4684)
        -> THIS is the clamp that stops vq from following the PI demand. Once it bites,
           the q current can no longer be forced and iq droops below iq_target.
  * utils_saturate_vector_2d(vd, vq, max_v_mag)                     (line 4688)
        -> final belt-and-suspenders: scales the (vd,vq) vector back onto the circle of
           radius max_v_mag if it still pokes outside.

In the CSV, watch: `max_v_mag`, `max_vq`, `vq`, `bemf` (= we*lambda), and the boolean
`vq_saturated`. vq saturates roughly when bemf + Rs*iq starts to exceed max_vq; that is
the RPM at which the motor runs out of voltage and field weakening takes over.

USAGE
-----
    python3 simverter.py                          # defaults: 0 -> 8000 mech rpm ramp
    python3 simverter.py --iq 150 --vbus 50       # 150 A demand, 50 V battery
    python3 simverter.py --rpm-start 0 --rpm-end 7500 --accel 5000   # mechanical rpm
    python3 simverter.py --rpm-start 6000 --rpm-end 6000 --accel 0 --sim-time 0.5  # fixed speed
    python3 simverter.py --rpm-end 30000 --accel 20000 --erpm-input  # ramp given as erpm
    python3 simverter.py --out run.csv --plot     # also render a PNG if matplotlib is present

Everything defaults from simverter/maxim_150.xml; CLI flags override.
"""

import argparse
import math
import os
import random
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# Small math helpers ported 1:1 from util/utils_math.h so the numerics match
# the firmware exactly (same clamping semantics, same LP filter form).
# ---------------------------------------------------------------------------
ONE_BY_SQRT3 = 1.0 / math.sqrt(3.0)
TWO_BY_SQRT3 = 2.0 / math.sqrt(3.0)


def SQ(x):
    return x * x


def NORM2(a, b):                                   # NORM2_f
    return math.sqrt(a * a + b * b)


def SIGN(x):                                       # SIGN() macro
    return 1.0 if x >= 0.0 else -1.0


def utils_map(x, in_min, in_max, out_min, out_max):        # NON-clamping map
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def utils_step_towards(val, goal, step):           # utils_step_towards
    if val < goal:
        return min(val + step, goal)
    elif val > goal:
        return max(val - step, goal)
    return val


def truncate_abs(x, lim):                          # utils_truncate_number_abs
    return max(-lim, min(lim, x))


def truncate(x, lo, hi):                           # utils_truncate_number
    return max(lo, min(hi, x))


def utils_min_abs(a, b):                            # utils_min_abs
    return a if abs(a) < abs(b) else b


def utils_max_abs(a, b):                            # utils_max_abs
    return a if abs(a) > abs(b) else b


def lp_fast(state, sample, filter_const):          # UTILS_LP_FAST(state, sample, k)
    # #define UTILS_LP_FAST(v, s, k)  v -= (k) * ((v) - (s))
    return state - filter_const * (state - sample)


def saturate_vector_2d(x, y, mag_max):             # utils_saturate_vector_2d
    mag = NORM2(x, y)
    if mag < 1e-20:
        return x, y, False
    if mag > mag_max:
        f = mag_max / mag
        return x * f, y * f, True
    return x, y, False


def norm_angle_rad(a):                             # utils_norm_angle_rad -> (-pi, pi]
    two_pi = 2.0 * math.pi
    a = math.fmod(a, two_pi)
    if a < -math.pi:
        a += two_pi
    elif a > math.pi:
        a -= two_pi
    return a


def angle_difference_rad(a, b):                    # utils_angle_difference_rad
    return norm_angle_rad(a - b)


# MTPA_MODE enum (datatypes.h)
MTPA_MODE_OFF = 0
MTPA_MODE_IQ = 1
MTPA_MODE_IQ_MEASURED = 2

# FOC_CC_DECOUPLING enum (datatypes.h)
FOC_CC_DECOUPLING_DISABLED = 0
FOC_CC_DECOUPLING_CROSS = 1
FOC_CC_DECOUPLING_BEMF = 2
FOC_CC_DECOUPLING_CROSS_BEMF = 3

# foc_control_sample_mode enum (datatypes.h:71-73)
FOC_CONTROL_SAMPLE_MODE_V0 = 0
FOC_CONTROL_SAMPLE_MODE_V0_V7 = 1


# ---------------------------------------------------------------------------
# Configuration -- loaded from the mcconf XML, mirrors the fields of
# mc_configuration (datatypes.h) that the control chain actually reads.
# ---------------------------------------------------------------------------
class Conf:
    # Fields we need, with the XML tag they come from. Anything not present in
    # the XML keeps the default below.
    _FIELDS = {
        # motor electrical
        "foc_motor_r":            0.03,
        "foc_motor_l":            9e-05,
        "foc_motor_ld_lq_diff":   4e-05,
        "foc_motor_flux_linkage": 0.0115,
        "si_motor_poles":         8,
        # current PI loop
        "foc_current_kp":         0.4,
        "foc_current_ki":         400.0,
        "foc_current_filter_const": 0.15,
        "foc_cc_decoupling":      3,
        "foc_d_gain_scale_start": 0.9,
        "foc_d_gain_scale_max_mod": 0.9,
        # switching / control rate
        "foc_f_zv":               30000.0,
        "foc_control_sample_mode": 1,
        "foc_overmod_factor":     1.0,
        # limits
        "l_current_max":          200.0,
        "l_current_min":         -200.0,
        "l_in_current_max":       200.0,
        "l_in_current_min":      -200.0,
        "l_max_duty":             0.95,
        # field weakening
        "foc_mtpa_mode":          1,
        "foc_fw_current_max":     70.0,
        "foc_fw_duty_start":      0.9,
        "foc_fw_q_current_factor": 0.0,
        "foc_fw_ramp_time":       0.2,
        "cc_min_current":         0.0,
    }

    def __init__(self, xml_path):
        root = ET.parse(xml_path).getroot()
        for name, default in self._FIELDS.items():
            node = root.find(name)
            if node is not None and node.text is not None:
                # keep ints as ints, floats as floats (matches C type usage loosely)
                val = float(node.text)
                if isinstance(default, int):
                    val = int(round(val))
                setattr(self, name, val)
            else:
                setattr(self, name, default)

        # --- lo_* limits ---
        # In the firmware lo_current_* are the "override" limits derived from the
        # l_current_* base limits (update_override_limits). With no throttle/temp
        # derating they equal the base limits, which is the case we simulate.
        self.lo_current_max = self.l_current_max
        self.lo_current_min = self.l_current_min
        self.lo_in_current_max = self.l_in_current_max
        self.lo_in_current_min = self.l_in_current_min

        # --- precalculated values (foc_precalc_values, foc_math.c:758-766) ---
        self.p_lq = self.foc_motor_l + self.foc_motor_ld_lq_diff * 0.5
        self.p_ld = self.foc_motor_l - self.foc_motor_ld_lq_diff * 0.5
        self.p_duty_norm = TWO_BY_SQRT3 / self.foc_overmod_factor
        self.pole_pairs = self.si_motor_poles / 2.0

    def control_dt(self, phase_shunts=False):
        """Control-loop timestep, derived exactly as mcpwm_foc.c:2996-3005.

            #ifdef HW_HAS_PHASE_SHUNTS
                V0_V7 sample mode -> dt = 1/foc_f_zv        (sample twice per PWM period)
                otherwise         -> dt = 1/(foc_f_zv/2)
            #else   (no phase shunts)
                dt = 1/(foc_f_zv/2)                         (sample once per period)

        The foc_control_sample_mode (V0_V7) ONLY matters when HW_HAS_PHASE_SHUNTS is
        defined. The maxim (hwconf/vesc/maxim/hw_maxim_core.h) defines HW_HAS_3_SHUNTS
        and HW_HAS_PHASE_FILTERS but NOT HW_HAS_PHASE_SHUNTS, so it takes the #else
        branch: dt = 1/(30000/2) = 1/15000 -> 15 kHz (66.7 us), regardless of sample
        mode. That is why the real board runs at 15 kHz, not 30 kHz.

        phase_shunts defaults to False to match the maxim. Pass --phase-shunts only for
        boards with inline phase shunts (there V0_V7 gives 30 kHz).
        """
        if phase_shunts and self.foc_control_sample_mode == FOC_CONTROL_SAMPLE_MODE_V0_V7:
            return 1.0 / self.foc_f_zv
        return 1.0 / (self.foc_f_zv / 2.0)


# ---------------------------------------------------------------------------
# Mutable motor state -- mirrors the parts of motor_state_t / motor_all_state_t
# (datatypes.h) that flow through the control chain, plus the true dq currents
# of the plant (which stand in for the measured/Park-transformed currents).
# ---------------------------------------------------------------------------
class State:
    def __init__(self):
        # measured currents (in firmware: Park transform of ADC samples, here: plant)
        self.id = 0.0
        self.iq = 0.0
        self.id_filter = 0.0        # UTILS_LP_FAST of id, foc_current_filter_const
        self.iq_filter = 0.0
        # setpoints
        self.id_target = 0.0
        self.iq_target = 0.0
        # PI integrators and outputs
        self.vd_int = 0.0
        self.vq_int = 0.0
        self.vd = 0.0
        self.vq = 0.0
        # true-frame voltage the plant actually integrates. Equals vd/vq unless an
        # observer angle error is injected, in which case the controller's (vd,vq)
        # are rotated out of the estimated frame into the true rotor frame.
        self.vd_applied = 0.0
        self.vq_applied = 0.0
        # modulation
        self.mod_d = 0.0
        self.mod_q = 0.0
        self.mod_q_filter = 0.0
        self.duty_now = 0.0         # signed instantaneous duty (NORM2(mod)*p_duty_norm)
        self.duty_abs_filtered = 0.0  # m_duty_abs_filtered (LP 0.01), drives FW
        # field weakening
        self.i_fw_set = 0.0         # m_i_fw_set
        # speed estimation (VESC m_speed_est_fast estimator, mcpwm_foc.c:3819-3822)
        self.phase = 0.0            # true electrical angle [rad], integrated from mech rpm
        self.phase_before_speed_est = 0.0
        self.speed_est_fast = 0.0   # m_speed_est_fast: FILTERED electric omega estimate [rad/s]
        # debug snapshots
        self.max_v_mag = 0.0
        self.max_vq = 0.0
        self.bemf = 0.0
        self.mtpa_id = 0.0          # pure MTPA id target (before -i_fw_set)
        self.vq_saturated = False
        self.vd_saturated = False


# ---------------------------------------------------------------------------
# foc_run_fw  --  faithful port of foc_math.c:702-742
# Field-weakening current is driven purely by the FILTERED duty estimate: once
# the inverter starts running out of voltage the duty climbs, and past
# foc_fw_duty_start * l_max_duty this ramps m_i_fw_set up toward foc_fw_current_max.
# ---------------------------------------------------------------------------
def foc_run_fw(conf, st, dt):
    # foc_math.c:703 -- FW disabled if fw current budget is (near) zero
    if conf.foc_fw_current_max < max(conf.cc_min_current, 0.001):
        return

    # We are always in CONTROL_MODE_CURRENT + MC_STATE_RUNNING here, so the guard
    # at foc_math.c:711-715 is always satisfied.
    fw_current_now = 0.0
    duty_abs = st.duty_abs_filtered

    # foc_math.c:719-724
    if (conf.foc_fw_duty_start < 0.99 and
            duty_abs > conf.foc_fw_duty_start * conf.l_max_duty):
        fw_current_now = utils_map(
            duty_abs,
            conf.foc_fw_duty_start * conf.l_max_duty,
            conf.l_max_duty,
            0.0, conf.foc_fw_current_max)
        # NOTE: utils_map does NOT clamp, so if duty_abs somehow exceeds l_max_duty
        # this extrapolates ABOVE foc_fw_current_max. (m_duty_abs_filtered is clamped
        # to 1.0 at mcpwm_foc.c:3364, and l_max_duty=0.95, so it can overshoot a bit.)

    # foc_math.c:735-740 -- ramp-limit the FW current toward the target
    if conf.foc_fw_ramp_time < dt:
        st.i_fw_set = fw_current_now
    else:
        st.i_fw_set = utils_step_towards(
            st.i_fw_set, fw_current_now,
            (dt / conf.foc_fw_ramp_time) * conf.foc_fw_current_max)


# ---------------------------------------------------------------------------
# Setpoint block  --  faithful port of mcpwm_foc.c:3614-3658
# Applies MTPA (moves some current onto the d-axis to exploit saliency), folds
# in the field-weakening current, then applies input- and motor-current limits.
# Produces st.id_target / st.iq_target that control_current will chase.
# ---------------------------------------------------------------------------
def setpoint_block(conf, st, iq_cmd):
    # For CONTROL_MODE_CURRENT the incoming targets are the commanded values:
    # id_set_tmp = m_id_set (0 here), iq_set_tmp = m_iq_set (the fixed demand).
    id_set_tmp = 0.0
    iq_set_tmp = iq_cmd

    ld_lq_diff = conf.foc_motor_ld_lq_diff
    mod_q = st.mod_q_filter                                  # mcpwm_foc.c:3616

    # --- MTPA ---  mcpwm_foc.c:3618-3630
    if conf.foc_mtpa_mode != MTPA_MODE_OFF and ld_lq_diff != 0.0:
        lambda_ = conf.foc_motor_flux_linkage
        iq_ref = iq_set_tmp
        if conf.foc_mtpa_mode == MTPA_MODE_IQ_MEASURED:
            iq_ref = utils_min_abs(iq_set_tmp, st.iq_filter)  # mcpwm_foc.c:3624

        # Pure MTPA d-current (logged as mtpa_id), mcpwm_foc.c:3627 without the -i_fw:
        st.mtpa_id = (lambda_ - math.sqrt(SQ(lambda_) + 8.0 * SQ(ld_lq_diff * iq_ref))) / (4.0 * ld_lq_diff)
        # ... then field weakening subtracts i_fw_set from the d target:
        id_set_tmp = st.mtpa_id - st.i_fw_set

        # ================== for future tests ==================
        #max_iq = SQ(min(abs(conf.l_current_min), abs(conf.l_current_max))) - SQ(id_set_tmp)             # mcpwm_foc.c:3628
        #truncate(iq_set_tmp, -math.sqrt(max_iq), math.sqrt(max_iq))  # mcpwm_foc.c:3629
        #iq_set_tmp = SIGN(iq_set_tmp) * iq_set_tmp    # mcpwm_foc.c:3630
        # ======================================================

        i_diff = SQ(conf.l_current_max) - SQ(id_set_tmp)             # mcpwm_foc.c:3628
        if i_diff < 0.0:
            i_diff = 0.0
        i_diff = min(iq_set_tmp, i_diff)
        iq_set_tmp = SIGN(iq_set_tmp) * math.sqrt(i_diff)    # mcpwm_foc.c:3630
    else:
        # No MTPA: FW current goes straight onto the d-axis.  mcpwm_foc.c:3632-3633
        st.mtpa_id = 0.0
        id_set_tmp -= st.i_fw_set
        iq_set_tmp -= SIGN(mod_q) * st.i_fw_set * conf.foc_fw_q_current_factor

    # --- input (battery) current limit ---  mcpwm_foc.c:3639-3643
    # i_bus ~= mod_q * iq, so bounding iq by lo_in_current/mod_q bounds bus current.
    if mod_q > 0.001:
        iq_set_tmp = truncate(iq_set_tmp,
                              conf.lo_in_current_min / mod_q,
                              conf.lo_in_current_max / mod_q)
    elif mod_q < -0.001:
        iq_set_tmp = truncate(iq_set_tmp,
                              conf.lo_in_current_max / mod_q,
                              conf.lo_in_current_min / mod_q)

    # --- motor current limit on iq ---  mcpwm_foc.c:3645-3648
    if mod_q > 0.0:
        iq_set_tmp = truncate(iq_set_tmp, conf.lo_current_min, conf.lo_current_max)
    else:
        iq_set_tmp = truncate(iq_set_tmp, -conf.lo_current_max, -conf.lo_current_min)

    # --- vector magnitude limit (id has priority, iq gets the remainder) ---
    # mcpwm_foc.c:3651-3653
    current_max_abs = abs(utils_max_abs(conf.lo_current_max, conf.lo_current_min))
    id_set_tmp = truncate_abs(id_set_tmp, current_max_abs)
    iq_set_tmp = truncate_abs(iq_set_tmp, math.sqrt(max(0.0, SQ(current_max_abs) - SQ(id_set_tmp))))

    st.id_target = id_set_tmp                                # mcpwm_foc.c:3656
    st.iq_target = iq_set_tmp                                # mcpwm_foc.c:3658


# ---------------------------------------------------------------------------
# control_current  --  faithful port of mcpwm_foc.c:4547-4696 (the parts that
# matter for a running current-controlled motor; HFI / audio / dead-time
# compensation are intentionally omitted).
#
# This is the PI current loop AND the place where vd/vq saturate against the
# available voltage. `we_est` is the ESTIMATED electrical speed in rad/s -- exactly
# motor->m_speed_est_fast in the firmware (the LP-filtered estimate, not the true
# speed), so the decoupling feed-forward here inherits the estimator's lag. It is
# deliberately NOT named `we`: in the firmware the decoupling equations use
# m_speed_est_fast, an estimate, never the true speed (which the firmware never has).
# ---------------------------------------------------------------------------
def control_current(conf, st, we_est, v_bus, dt, id_noise=0.0, iq_noise=0.0, mag_vd_max=0.98,
                    aw_mode="truncate", angle_err=0.0):
    # state_m->max_duty is set to l_max_duty upstream (mcpwm_foc.c:3357), then
    # clamped to [0, l_max_duty] here (mcpwm_foc.c:4589-4590).
    max_duty = truncate(abs(conf.l_max_duty), 0.0, conf.l_max_duty)

    # The PI (and the id/iq low-pass, the decoupling and MTPA that read it) act on
    # the MEASURED current -- what the ADC captured this tick -- not the true plant
    # current. id_noise/iq_noise (from --current-noise) is additive measurement
    # ripple on that sample: the true dq currents (st.id/st.iq) are untouched, only
    # what the controller *sees* is perturbed. Default 0.0 keeps the clean loop.
    # --- observer angle error injection (NON-firmware; models a mis-tracked rotor
    # estimate) --- the controller sees currents projected onto its ESTIMATED dq
    # frame, rotated by angle_err from the true rotor frame:
    #   [id_c]   [ cos  sin][id_true]
    #   [iq_c] = [-sin  cos][iq_true]
    # With angle_err=0 this is the identity and the loop is byte-for-byte unchanged.
    ce, se = math.cos(angle_err), math.sin(angle_err)
    id_meas = (ce * st.id + se * st.iq) + id_noise
    iq_meas = (-se * st.id + ce * st.iq) + iq_noise

    # --- d-axis gain scaling near full modulation ---  mcpwm_foc.c:4600-4613
    # Reduces the d-axis PI gain as modulation approaches the limit, to keep the
    # (priority) d-axis loop from fighting the saturation. Uses the PREVIOUS tick's
    # duty_now (pipelined, as in firmware).
    d_gain_scale = 1.0
    if conf.foc_d_gain_scale_start < 0.99:
        # How far into the modulation budget we are, normalized to [0,1].
        max_mod_norm = abs(st.duty_now / max_duty) if max_duty >= 0.01 else 1.0
        # If max_mod_norm > 0.855, scale the d-axis gain down toward 0.9 (or conf.foc_d_gain_scale_max_mod).
        if max_mod_norm > conf.foc_d_gain_scale_start:
            d_gain_scale = utils_map(max_mod_norm, conf.foc_d_gain_scale_start, 1.0,
                                     1.0, conf.foc_d_gain_scale_max_mod)
            if d_gain_scale < conf.foc_d_gain_scale_max_mod:
                d_gain_scale = conf.foc_d_gain_scale_max_mod

    # --- low-pass the measured currents (used by decoupling, MTPA) ---
    # mcpwm_foc.c:4597-4598
    st.id_filter = lp_fast(st.id_filter, id_meas, conf.foc_current_filter_const)
    st.iq_filter = lp_fast(st.iq_filter, iq_meas, conf.foc_current_filter_const)

    # --- PI controller ---  mcpwm_foc.c:4615-4628
    Ierr_d = st.id_target - id_meas
    Ierr_q = st.iq_target - iq_meas
    ki = conf.foc_current_ki

    st.vd_int += Ierr_d * (ki * d_gain_scale * dt)
    st.vq_int += Ierr_q * (ki * dt)

    st.vd = st.vd_int + Ierr_d * conf.foc_current_kp * d_gain_scale
    st.vq = st.vq_int + Ierr_q * conf.foc_current_kp

    # --- decoupling / feed-forward ---  mcpwm_foc.c:4630-4662
    # NOTE: the firmware uses motor->m_speed_est_fast (the estimate) as the omega here.
    # vd = Rs*id + Ld*did/dt - we*Lq*iq        (cross term  -we*Lq*iq)
    # vq = Rs*iq + Lq*diq/dt + we*Ld*id + we*psi   (cross +we*Ld*id, bemf +we*psi)
    dec_vd = dec_vq = dec_bemf = 0.0
    dec = conf.foc_cc_decoupling
    if dec == FOC_CC_DECOUPLING_CROSS:
        dec_vd = st.iq_filter * we_est * conf.p_lq
        dec_vq = st.id_filter * we_est * conf.p_ld
    elif dec == FOC_CC_DECOUPLING_BEMF:
        dec_bemf = we_est * conf.foc_motor_flux_linkage
    elif dec == FOC_CC_DECOUPLING_CROSS_BEMF:      # maxim_150: foc_cc_decoupling=3
        dec_vd = st.iq_filter * we_est * conf.p_lq
        dec_vq = st.id_filter * we_est * conf.p_ld
        dec_bemf = we_est * conf.foc_motor_flux_linkage

    st.vd -= dec_vd                                # mcpwm_foc.c:4661
    st.vq += dec_vq + dec_bemf                     # mcpwm_foc.c:4662

    # Largest voltage vector without overmodulation.  mcpwm_foc.c:4666
    max_v_mag = ONE_BY_SQRT3 * max_duty * v_bus * conf.foc_overmod_factor   # currently 1.0, so this is just ONE_BY_SQRT3 * max_duty * v_bus

    # d-axis takes its share of the budget first (priority), but capped BELOW the
    # full circle by mag_vd_max so a sliver is always reserved for the q-axis.
    # Mirrors v7's foc_mag_vd_max.  mcpwm_foc.c:4675-4676
    vd_limit = max_v_mag * mag_vd_max
    vd_presat = st.vd
    st.vd = truncate_abs(st.vd, vd_limit)
    # Anti-windup on the d integrator: "truncate" hard-clamps vd_int to the same
    # rail (current firmware, mcpwm_foc.c:4710); "backcalc" instead subtracts the
    # clamped overshoot -- incl. the Kp/decoupling term -- from vd_int, forcing
    # vd_int + P' = vd_limit (the commented alternative, mcpwm_foc.c:4714).
    if aw_mode == "backcalc":
        st.vd_int += (st.vd - vd_presat)
    else:
        st.vd_int = truncate_abs(st.vd_int, vd_limit)
    st.vd_saturated = (abs(vd_presat) > vd_limit)

    # q-axis is capped by whatever budget is LEFT.  mcpwm_foc.c:4681-4684
    max_vq = math.sqrt(max(0.0, SQ(max_v_mag) - SQ(st.vd)))
    vq_presat = st.vq
    st.vq = truncate_abs(st.vq, max_vq)
    if aw_mode == "backcalc":
        st.vq_int += (st.vq - vq_presat)
    else:
        st.vq_int = truncate_abs(st.vq_int, max_vq)
    st.vq_saturated = (abs(vq_presat) > max_vq)

    # Final vector clamp onto the circle of radius max_v_mag.  mcpwm_foc.c:4688
    st.vd, st.vq, sat_vec = saturate_vector_2d(st.vd, st.vq, max_v_mag)
    if sat_vec:
        st.vq_saturated = True

    # snapshots for logging / CAN debug (m_debug_max_v_mag, m_debug_bemf)
    st.max_v_mag = max_v_mag
    st.max_vq = max_vq
    st.bemf = dec_bemf

    # --- normalise to modulation and compute duty ---  mcpwm_foc.c:4694-4698, 3801
    voltage_normalize = 1.5 / v_bus
    st.mod_d = st.vd * voltage_normalize
    st.mod_q = st.vq * voltage_normalize
    st.mod_q_filter = lp_fast(st.mod_q_filter, st.mod_q, 0.2)
    st.mod_q_filter = truncate_abs(st.mod_q_filter, 1.0)

    # duty_now = |mod| * p_duty_norm  (mcpwm_foc.c:3801). Signed in firmware; here we
    # only ever need its magnitude for the FW duty filter, so store the magnitude.
    # duty is computed from the controller-frame (vd,vq), matching firmware telemetry.
    st.duty_now = NORM2(st.mod_d, st.mod_q) * conf.p_duty_norm

    # The controller synthesised (vd,vq) in its ESTIMATED frame; the inverter applies
    # them to the machine, so the plant sees them rotated back into the TRUE rotor
    # frame by angle_err:
    #   [vd_true]   [cos  -sin][vd_c]
    #   [vq_true] = [sin   cos][vq_c]
    # This is what projects the q-axis back-EMF onto the true d-axis (~bemf*sin(err))
    # and loads vd_int, reproducing the bench discrepancy the ideal sim cannot show.
    st.vd_applied = ce * st.vd - se * st.vq
    st.vq_applied = se * st.vd + ce * st.vq


# ---------------------------------------------------------------------------
# Motor plant -- the ONLY part not taken from the firmware. It turns the applied
# (saturated) vd/vq into the next-tick dq currents, closing the loop. Standard
# salient-PMSM electrical model, forward-Euler integrated at the control dt
# (dt << L/R, so this is stable and accurate):
#     did/dt = (vd - Rs*id + we*Lq*iq) / Ld
#     diq/dt = (vq - Rs*iq - we*Ld*id - we*psi) / Lq
# This is exactly the inverse of the decoupling equations above, so a perfectly
# decoupled loop would track; where vq saturates, iq can no longer be driven and
# droops -- which is the behaviour we want to observe.
# ---------------------------------------------------------------------------
def sat_inductance(L0, i_mag, sat_frac, i_ref):
    """TRUE (saturated) inductance of the iron as a function of stator current.

    Linear droop that saturates: L = L0*(1 - sat_frac) once |i| >= i_ref, ramping
    linearly from L0 at 0 A. sat_frac=0 (or i_ref<=0) disables it -> L stays L0.

    This is the 'apparent/secant' inductance approximation: we drop L in the di/dt
    equation but ignore the dL/di*(di/dt) incremental term. That is deliberate --
    it is enough to create a REALISTIC feed-forward mismatch (the controller's
    decoupling uses the constant config Lq/Ld) without pretending to be a full
    saturating flux-map model.
    """
    if sat_frac <= 0.0 or i_ref <= 0.0:
        return L0
    droop = sat_frac * min(1.0, abs(i_mag) / i_ref)
    return L0 * (1.0 - droop)


def plant_step(conf, st, we, dt, args):
    R = conf.foc_motor_r
    psi = conf.foc_motor_flux_linkage

    # TRUE plant inductances. When --plant-*-sat is set these deliberately DIFFER
    # from the controller's configured conf.p_ld / conf.p_lq: the FOC decoupling
    # feed-forward (control_current) uses the constant config values, so any droop
    # here is a feed-forward MISMATCH that the PI integrators must absorb. That is
    # what loads up vd_int/vq_int on a real bench (where Lq/Ld sag with current)
    # while the ideal sim -- controller and plant sharing one parameter set -- keeps
    # the integrators near the tiny Rs*i residual. Saturation is driven by the total
    # stator current magnitude (the core saturates on total MMF, not per-axis).
    i_mag = NORM2(st.id, st.iq)
    Ld = sat_inductance(conf.p_ld, i_mag, args.plant_ld_sat, args.plant_sat_current)
    Lq = sat_inductance(conf.p_lq, i_mag, args.plant_lq_sat, args.plant_sat_current)

    # Use the TRUE-frame applied voltages (vd_applied/vq_applied). These equal
    # st.vd/st.vq when no observer angle error is injected.
    did = (st.vd_applied - R * st.id + we * Lq * st.iq) / Ld
    diq = (st.vq_applied - R * st.iq - we * Ld * st.id - we * psi) / Lq
    st.id += did * dt
    st.iq += diq * dt


# ---------------------------------------------------------------------------
# VESC speed estimator  --  faithful port of mcpwm_foc.c:3819-3822
# m_speed_est_fast is NOT the true speed: it is a heavily low-passed (k=0.01)
# estimate of d(electrical phase)/dt. During acceleration this filter LAGS the
# true speed, so the decoupling feed-forward (which uses m_speed_est_fast) is
# slightly behind -- an effect this reproduces. We feed it the TRUE electrical
# phase (assuming a perfect observer, i.e. no angle error), so the only
# estimation artifact modelled here is the speed-filter lag itself.
# ---------------------------------------------------------------------------
def speed_est_update(st, dt):
    diff = angle_difference_rad(st.phase, st.phase_before_speed_est)
    diff = truncate(diff, -math.pi / 3.0, math.pi / 3.0)   # mcpwm_foc.c:3820
    st.speed_est_fast = lp_fast(st.speed_est_fast, diff / dt, 0.01)  # mcpwm_foc.c:3822
    st.phase_before_speed_est = st.phase


# ---------------------------------------------------------------------------
# RPM ramp: rpm(t) = rpm_start + accel*t, clamped to rpm_end (accel=0 -> fixed).
# `rpm` is the MECHANICAL (physical) motor rpm. The electrical speed the firmware
# actually controls with is derived from it: erpm = rpm * pole_pairs, and the
# TRUE electrical omega is fed through VESC's speed estimator (speed_est_update)
# to obtain the ESTIMATED electric omega used by the controller. Pass
# --erpm-input to instead interpret the ramp numbers directly as electrical erpm.
# ---------------------------------------------------------------------------
def rpm_at(t, rpm_start, rpm_end, accel):
    if accel == 0.0:
        return rpm_start
    rpm = rpm_start + accel * t
    if rpm_end >= rpm_start:
        return min(rpm, rpm_end)
    return max(rpm, rpm_end)


# CSV columns, in order. Kept explicit so PlotJuggler sees stable names.
CSV_COLUMNS = [
    "time",             # s -- x-axis for PlotJuggler
    "rpm",              # MECHANICAL rpm (erpm / pole_pairs; mc_interface.c:1646)
    "erpm",             # ELECTRICAL rpm = mech rpm * pole_pairs = RADPS2RPM_f(we)
    "we",               # TRUE electric omega [rad/s] = RPM2RADPS_f(erpm) (utils_math.h:77)
    "we_est",           # ESTIMATED electric omega [rad/s] = m_speed_est_fast (mcpwm_foc.c:3822)
    "iq_cmd",           # fixed commanded q current [A]
    "i_fw_set",         # field-weakening current magnitude [A] (foc_run_fw)
    "mtpa_id",          # pure MTPA d-current target [A] (before FW)
    "id_target",        # final d-current target [A] (MTPA - i_fw, clamped)
    "iq_target",        # final q-current target [A]
    "id",               # actual d current [A] (plant / "measured")
    "iq",               # actual q current [A]
    "i_abs",            # sqrt(id^2 + iq^2) [A]
    "vbus",             # DC-link voltage actually used this tick [V] (with --vbus-noise)
    "vd",               # applied d voltage [V] (post-saturation)
    "vq",               # applied q voltage [V] (post-saturation)
    "vd_int",           # d integrator [V]
    "vq_int",           # q integrator [V]
    "bemf",             # we*psi back-EMF [V]
    "max_v_mag",        # voltage budget [V]  (mcpwm_foc.c:4666)
    "max_vq",           # remaining q budget after vd [V] (mcpwm_foc.c:4681)
    "mod_d",            # normalized d modulation
    "mod_q",            # normalized q modulation
    "mod_q_filter",     # filtered mod_q (used by MTPA/limits)
    "duty",             # |mod| * p_duty_norm  (instantaneous duty)
    "duty_filtered",    # m_duty_abs_filtered (LP 0.01) -- drives FW
    "vd_saturated",     # 1 when vd hit +/-max_v_mag
    "vq_saturated",     # 1 when vq hit +/-max_vq (voltage-limited q current!)
    "plant_ld",         # TRUE (saturated) plant Ld [H] -- differs from config p_ld when --plant-ld-sat set
    "plant_lq",         # TRUE (saturated) plant Lq [H] -- differs from config p_lq when --plant-lq-sat set
]


def simulate(conf, args):
    dt = args.control_dt
    st = State()

    # optional speed-ripple noise on the rpm ramp (reproducible with --noise-seed).
    rng = random.Random(args.noise_seed)

    # commanded q current (fixed). Signed: positive = motoring in +erpm direction.
    iq_cmd = args.iq
    v_bus = args.vbus

    # total sim time: enough to reach rpm_end at `accel`, plus a hold, unless a
    # fixed sim-time is requested (needed for the accel=0 / fixed-speed case).
    if args.sim_time is not None:
        t_end = args.sim_time
    elif args.accel != 0.0:
        t_ramp = abs(args.rpm_end - args.rpm_start) / abs(args.accel)
        t_end = t_ramp + args.hold_time
    else:
        t_end = 1.0  # fixed speed default window

    rows = []
    n_steps = int(round(t_end / dt))
    for k in range(n_steps + 1):
        t = k * dt

        # --- prescribed kinematics: physical rpm ramp -> TRUE electrical omega ---
        # The three speed representations and how the firmware relates them:
        #   erpm = mech_rpm * pole_pairs   (mc_interface.c:1646 does the inverse:
        #                                   mech = erpm / (si_motor_poles/2))
        #   we   = RPM2RADPS_f(erpm) = erpm * 2*pi/60   (utils_math.h:77); inverse
        #          erpm = RADPS2RPM_f(we) (utils_math.h:78), as used by
        #          mcpwm_foc_get_rpm() -> RADPS2RPM_f(m_pll_speed) (mcpwm_foc.c:1159).
        # rpm is MECHANICAL by default; with --erpm-input the ramp numbers are erpm.
        if args.erpm_input:
            erpm = rpm_at(t, args.rpm_start, args.rpm_end, args.accel)
            rpm = erpm / conf.pole_pairs
        else:
            rpm = rpm_at(t, args.rpm_start, args.rpm_end, args.accel)
            erpm = rpm * conf.pole_pairs
        # Optional speed ripple: additive Gaussian noise on the MECHANICAL rpm, std =
        # args.rpm_noise. It propagates into the true `we` (plant) and, filtered, into
        # `we_est` (decoupling); the plant/estimator mismatch it creates is what makes
        # the loop -- and therefore the duty -- ripple, the way a real bench trace does.
        if args.rpm_noise > 0.0:
            rpm += rng.gauss(0.0, args.rpm_noise)
            erpm = rpm * conf.pole_pairs
        we = erpm / 60.0 * 2.0 * math.pi          # true electric omega [rad/s], RPM2RADPS_f

        # Optional DC-link ripple: additive Gaussian noise on v_bus (std = args.vbus_noise),
        # drawn from the SAME rng as the rpm noise. It shrinks/grows max_v_mag tick to tick
        # (max_v_mag = ONE_BY_SQRT3*max_duty*v_bus) so the voltage budget itself ripples --
        # on the real bench the regen-pumped link is far from a clean rail. Floored above 0
        # to keep max_v_mag / voltage_normalize well-defined.
        v_bus_now = v_bus
        if args.vbus_noise > 0.0:
            v_bus_now = max(1e-3, v_bus + rng.gauss(0.0, args.vbus_noise))
        # Optional current-measurement ripple: additive Gaussian noise (std =
        # args.current_noise) on the id/iq the PI SAMPLES this tick, same rng as the
        # other noise. Unlike --rpm/--vbus noise it perturbs the loop's *measurement*,
        # not the plant: it de-saturates the odd tick (the q-error momentarily flips),
        # so a voltage-limited duty stops pinning dead-flat at max and its filtered
        # mean -- which drives field weakening -- rides below the peak, as on a bench.
        # It is a LUMPED stand-in for ADC/shunt noise; the deterministic switching- and
        # 6th-harmonic ripples are the larger real sources but are not modelled here.
        id_noise = rng.gauss(0.0, args.current_noise) if args.current_noise > 0.0 else 0.0
        iq_noise = rng.gauss(0.0, args.current_noise) if args.current_noise > 0.0 else 0.0
        # NB: the firmware never has this true `we` -- it only ever has estimates
        # (m_speed_est_fast, m_pll_speed). Here `we` is the sim's ground truth, used
        # by the motor plant; the controller gets the estimate (st.speed_est_fast).

        # Integrate the TRUE electrical angle (drives the speed estimator below).
        st.phase = norm_angle_rad(st.phase + we * dt)

        # --- firmware control chain, in order ---
        # 1. duty low-pass filter (mcpwm_foc.c:3363-3364): m_duty_abs_filtered tracks
        #    the instantaneous duty from the PREVIOUS control tick.
        st.duty_abs_filtered = lp_fast(st.duty_abs_filtered, abs(st.duty_now), 0.01)
        st.duty_abs_filtered = truncate_abs(st.duty_abs_filtered, 1.0)

        # 2. field weakening (foc_math.c:702)
        foc_run_fw(conf, st, dt)

        # 3. MTPA + FW fold + current limits (mcpwm_foc.c:3614)
        setpoint_block(conf, st, iq_cmd)

        # 4. PI current loop + voltage saturation (mcpwm_foc.c:4547).
        #    The controller uses the ESTIMATED omega (m_speed_est_fast, one tick old,
        #    as in the firmware pipeline), NOT the true speed.
        control_current(conf, st, st.speed_est_fast, v_bus_now, dt, id_noise, iq_noise,
                        args.mag_vd_max, args.aw_mode, math.radians(args.angle_error))

        # log AFTER control_current so vd/vq/mod are the values actually applied
        rows.append({
            "time": t, "rpm": rpm, "erpm": erpm,
            "we": we, "we_est": st.speed_est_fast,
            "iq_cmd": iq_cmd, "i_fw_set": st.i_fw_set,
            "mtpa_id": st.mtpa_id, "id_target": st.id_target, "iq_target": st.iq_target,
            "id": st.id, "iq": st.iq, "i_abs": NORM2(st.id, st.iq),
            "vbus": v_bus_now,
            "vd": st.vd, "vq": st.vq, "vd_int": st.vd_int, "vq_int": st.vq_int,
            "bemf": st.bemf, "max_v_mag": st.max_v_mag, "max_vq": st.max_vq,
            "mod_d": st.mod_d, "mod_q": st.mod_q, "mod_q_filter": st.mod_q_filter,
            "duty": st.duty_now, "duty_filtered": st.duty_abs_filtered,
            "vd_saturated": 1 if st.vd_saturated else 0,
            "vq_saturated": 1 if st.vq_saturated else 0,
            # TRUE plant inductances the NEXT plant_step will use (st.id/st.iq are
            # unchanged between here and plant_step), so they line up with this row.
            "plant_ld": sat_inductance(conf.p_ld, NORM2(st.id, st.iq), args.plant_ld_sat, args.plant_sat_current),
            "plant_lq": sat_inductance(conf.p_lq, NORM2(st.id, st.iq), args.plant_lq_sat, args.plant_sat_current),
        })

        # 5. motor plant: advance the true dq currents using the TRUE electrical
        #    omega (this is physics, not the controller's estimate).
        plant_step(conf, st, we, dt, args)

        # 6. VESC speed estimator: update m_speed_est_fast from the electrical-phase
        #    advance, for the NEXT tick's decoupling (mcpwm_foc.c:3819-3822).
        speed_est_update(st, dt)

    return rows


def write_csv(rows, path):
    import csv
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def maybe_plot(rows, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available -- skipping plot")
        return
    t = [r["time"] for r in rows]
    fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)

    ax[0].plot(t, [r["rpm"] for r in rows], "g")
    ax[0].set_ylabel("rpm"); ax[0].grid(True, alpha=0.3)

    ax[1].plot(t, [r["id_target"] for r in rows], "b--", label="id_target")
    ax[1].plot(t, [r["id"] for r in rows], "b", label="id")
    ax[1].plot(t, [r["iq_target"] for r in rows], "r--", label="iq_target")
    ax[1].plot(t, [r["iq"] for r in rows], "r", label="iq")
    ax[1].plot(t, [r["mtpa_id"] for r in rows], "c", lw=0.8, label="mtpa_id")
    ax[1].plot(t, [-r["i_fw_set"] for r in rows], "m", lw=0.8, label="-i_fw_set")
    ax[1].set_ylabel("A"); ax[1].legend(fontsize=7); ax[1].grid(True, alpha=0.3)

    ax[2].plot(t, [r["vd"] for r in rows], "b", label="vd")
    ax[2].plot(t, [r["vq"] for r in rows], "r", label="vq")
    ax[2].plot(t, [r["max_v_mag"] for r in rows], "k--", lw=0.8, label="max_v_mag")
    ax[2].plot(t, [r["max_vq"] for r in rows], "r:", lw=0.8, label="max_vq")
    ax[2].plot(t, [r["bemf"] for r in rows], "g", lw=0.8, label="bemf")
    ax[2].set_ylabel("V"); ax[2].legend(fontsize=7); ax[2].grid(True, alpha=0.3)

    ax[3].plot(t, [r["duty"] for r in rows], "0.6", label="duty")
    ax[3].plot(t, [r["duty_filtered"] for r in rows], "k", label="duty_filtered")
    ax[3].plot(t, [r["vq_saturated"] for r in rows], "r", lw=0.8, label="vq_saturated")
    ax[3].set_ylabel("duty / flag"); ax[3].set_xlabel("time [s]")
    ax[3].legend(fontsize=7); ax[3].grid(True, alpha=0.3)

    fig.tight_layout()
    png = os.path.splitext(path)[0] + ".png"
    fig.savefig(png, dpi=110)
    print(f"plot -> {png}")


def main():
    p = argparse.ArgumentParser(
        description="Offline closed-loop simulator of the VESC FOC current controller.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--config", default=os.path.join(HERE, "maxim_150.xml"),
                   help="mcconf XML file")
    p.add_argument("--iq", type=float, default=150.0,
                   help="fixed commanded q-axis current [A] (the 'current required')")
    p.add_argument("--vbus", type=float, default=50.0,
                   help="fixed battery voltage [V]")
    p.add_argument("--rpm-start", type=float, default=0.0,
                   help="start MECHANICAL rpm (physical)")
    p.add_argument("--rpm-end", type=float, default=8000.0,
                   help="end MECHANICAL rpm (physical)")
    p.add_argument("--accel", type=float, default=4000.0,
                   help="mechanical rpm per second (0 = fixed speed at rpm-start)")
    p.add_argument("--rpm-noise", type=float, default=0.0,
                   help="std dev of Gaussian speed ripple added to the MECHANICAL rpm "
                        "each tick [rpm] (0 = clean ramp). Propagates to bemf -> duty.")
    p.add_argument("--vbus-noise", type=float, default=0.0,
                   help="std dev of Gaussian DC-link ripple added to v_bus each tick [V] "
                        "(0 = clean rail). Ripples max_v_mag; shares --noise-seed rng.")
    p.add_argument("--current-noise", type=float, default=0.0,
                   help="std dev of Gaussian ripple [A] on the id/iq the PI SAMPLES each tick "
                        "(0 = ideal measurement). Lumped ADC/shunt-noise stand-in; de-saturates "
                        "the loop so voltage-limited duty stops pinning flat. Shares --noise-seed rng.")
    p.add_argument("--noise-seed", type=int, default=None,
                   help="RNG seed for --rpm-noise / --vbus-noise (reproducible; default: random)")
    p.add_argument("--hold-time", type=float, default=0.1,
                   help="extra seconds held at rpm-end after the ramp")
    p.add_argument("--sim-time", type=float, default=None,
                   help="override total sim time [s] (needed for accel=0 fixed speed)")
    p.add_argument("--erpm-input", action="store_true",
                   help="interpret rpm-start/end/accel as ELECTRICAL erpm instead of mechanical")
    p.add_argument("--control-freq", type=float, default=None,
                   help="override control loop frequency [Hz] (else derived from config)")
    p.add_argument("--mag-vd-max", type=float, default=0.98,
                   help="fraction of the voltage circle vd may consume (v7 foc_mag_vd_max); "
                        "the rest is reserved for the q-axis so the current loop keeps q-axis "
                        "authority under field weakening. 1.0 = old behaviour (vd can take all)")
    p.add_argument("--aw-mode", choices=["truncate", "backcalc"], default="truncate",
                   help="integrator anti-windup on saturation: 'truncate' hard-clamps "
                        "vd_int/vq_int to the rail (current firmware); 'backcalc' subtracts the "
                        "clamped overshoot (incl. Kp/decoupling) from the integrator instead")
    p.add_argument("--angle-error", type=float, default=0.0,
                   help="observer rotor-angle error [deg]: the controller's estimated dq frame "
                        "is rotated by this from the true rotor frame. Projects bemf*sin(err) "
                        "onto the d-axis and loads vd_int (0 = perfect observer, ideal sim)")
    p.add_argument("--phase-shunts", action="store_true",
                   help="board has HW_HAS_PHASE_SHUNTS (maxim does NOT). Only then does "
                        "V0_V7 sample mode give 30 kHz; otherwise the loop is 15 kHz")
    p.add_argument("--plant-lq-sat", type=float, default=0.0,
                   help="fractional drop of the TRUE plant Lq at --plant-sat-current "
                        "(0.3 = plant Lq is 30%% below the configured value there, ramping "
                        "linearly from 0 A). The controller keeps decoupling with the config "
                        "Lq, so this is the feed-forward mismatch that loads vd_int/vq_int the "
                        "way a real (saturating) motor does. 0 = ideal/matched (default)")
    p.add_argument("--plant-ld-sat", type=float, default=0.0,
                   help="fractional drop of the TRUE plant Ld at --plant-sat-current. 0 = ideal")
    p.add_argument("--plant-sat-current", type=float, default=None,
                   help="stator current magnitude |i|=sqrt(id^2+iq^2) [A] at which the above "
                        "droops are fully reached (default: l_current_max from the config)")
    p.add_argument("-o", "--out", default=None,
                   help="output CSV path (default: sim.csv, or <name>.csv if --name given)")
    p.add_argument("-n", "--name", default=None,
                   help="short run name -> writes <name>.csv (and <name>.png) in this folder")
    p.add_argument("--plot", action="store_true", help="also render a PNG (needs matplotlib)")
    args = p.parse_args()

    # Resolve the output path: explicit --out wins; else --name; else default sim.csv.
    if args.out is None:
        base = args.name if args.name else "sim"
        if not base.lower().endswith(".csv"):
            base += ".csv"
        # a bare name goes in this folder; a path (has a separator) is used as-is
        args.out = base if os.path.dirname(base) else os.path.join(HERE, base)

    conf = Conf(args.config)
    args.control_dt = (1.0 / args.control_freq) if args.control_freq else conf.control_dt(args.phase_shunts)
    if args.plant_sat_current is None:
        args.plant_sat_current = conf.l_current_max

    print(f"config           : {args.config}")
    print(f"control frequency: {1.0/args.control_dt:.0f} Hz  (dt = {args.control_dt*1e6:.2f} us)")
    print(f"iq command       : {args.iq} A")
    print(f"battery voltage  : {args.vbus} V")
    rpm_kind = "erpm" if args.erpm_input else "mech rpm"
    print(f"rpm ramp         : {args.rpm_start} -> {args.rpm_end} {rpm_kind} @ {args.accel} rpm/s "
          f"(pole pairs = {conf.pole_pairs:.0f})")
    print(f"voltage budget   : max_v_mag = 1/sqrt(3) * l_max_duty * vbus = "
          f"{ONE_BY_SQRT3 * conf.l_max_duty * args.vbus * conf.foc_overmod_factor:.2f} V")
    if args.plant_lq_sat > 0.0 or args.plant_ld_sat > 0.0:
        print(f"plant saturation : Lq -{args.plant_lq_sat*100:.0f}%  Ld -{args.plant_ld_sat*100:.0f}%  "
              f"at |i|={args.plant_sat_current:.0f} A  (config Lq={conf.p_lq*1e6:.1f} uH -> "
              f"{conf.p_lq*(1-args.plant_lq_sat)*1e6:.1f} uH, Ld={conf.p_ld*1e6:.1f} uH -> "
              f"{conf.p_ld*(1-args.plant_ld_sat)*1e6:.1f} uH)  [feed-forward mismatch ENABLED]")
    else:
        print("plant saturation : off (plant Lq/Ld == config -> integrators stay near Rs*i)")

    # ==============================
    #          START SIM
    # ==============================
    rows = simulate(conf, args)

    # ==============================
    #          WRITE CSV
    # ==============================
    write_csv(rows, args.out)

    # ==============================
    #            SUMMARY
    # ==============================
    print(f"csv  -> {args.out}   ({len(rows)} rows)")

    # quick text summary. Two distinct events can saturate vq:
    #   (a) the startup current-ramp transient (huge iq error at t~0, bemf~0) -- benign,
    #   (b) the speed-driven voltage limit (bemf approaches max_v_mag) -- the real one.
    # We report (b): the first vq saturation where the back-EMF is actually large.
    speed_sat = [r for r in rows if r["vq_saturated"] and r["bemf"] > 0.5 * r["max_v_mag"]]
    if speed_sat:
        r0 = speed_sat[0]
        print(f"vq voltage-limited from t={r0['time']*1e3:.1f} ms  erpm={r0['erpm']:.0f}  "
              f"bemf={r0['bemf']:.1f} V vs budget {r0['max_v_mag']:.1f} V  "
              f"(iq can no longer be forced -> field weakening)")
    else:
        print("vq never voltage-limited by speed in this run (budget never exceeded)")
    fw_on = next((r for r in rows if r["i_fw_set"] > 1.0), None)
    if fw_on:
        print(f"field weakening engaged at t={fw_on['time']*1e3:.1f} ms  erpm={fw_on['erpm']:.0f}  "
              f"(duty_filtered crossed {conf.foc_fw_duty_start * conf.l_max_duty:.3f})")
    print(f"peak field-weakening current: {max(r['i_fw_set'] for r in rows):.1f} A")

    # ==============================
    #             PLOT
    # ==============================
    if args.plot:
        maybe_plot(rows, args.out)


if __name__ == "__main__":
    main()
