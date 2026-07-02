# Voltage-saturation breakdown order: vq → vd → instability

Why, once field-weakening current reaches its cap, **vq saturates first, then vd, then the
current loop goes unstable**.

Reference operating point (a simverter run, cursor at t=1.558 s, high-speed motoring):

```
bemf = 59.9 V     max_v_mag = 31.5 V     (bemf ~1.9x the budget)
vq = 31.46 V (saturated)   vd = -2.28 V (not saturated)
vq_saturated = 1           vd_saturated = 0
i_fw = 69.9 A (capped at foc_fw_current_max = 70)
id = -78 A    iq ~ 0    iq_target = 0 (MTPA zeroed it)
we = 5208 rad/s   erpm = 49847   rpm = 12461 (mech)
```

## Why vq saturates first

vq is the axis that carries the **back-EMF**. The steady q-axis voltage is:

```
vq = R*iq + we*Ld*id + we*lambda      <- we*lambda (bemf) is the dominant term
```

vd only has to cover the small cross term `-we*Lq*iq` (and iq ~ 0 here). So as speed rises,
**vq climbs toward the budget while vd stays small** — vq hits the ceiling long before vd.
That is exactly the snapshot: `vq_saturated = 1`, `vd_saturated = 0`.

## Why FW-max is the tipping point

Before FW saturates, there is a release valve: pushing `id` more negative makes `we*Ld*id`
*subtract* from the vq demand. At the knife's edge (t=1.558):

```
vq_needed = bemf + we*Ld*id = 59.9 + (5208*7e-5)*(-78) = 59.9 - 28.4 = 31.5 = max_v_mag
```

FW pulled `id` to -78 A, and that -28.4 V of field weakening is *exactly* what lets vq=31.5
still hold the point with iq ~ 0. While FW has headroom, the loop keeps driving `id` more
negative to keep vq pinned at the ceiling — stable, but only just.

The moment **`i_fw` hits its 70 A cap**, `id` can no longer go more negative. The release
valve is gone. Any further speed increase raises bemf with nothing to offset it, so
`vq_needed` climbs back above the 31.5 V ceiling. **This is the trigger.**

## Why vd saturates next, then instability

Once vq is maxed and FW cannot compensate, the q-current can no longer be held, so `iq`
starts to move. As soon as the currents deviate, the **cross-coupling terms explode**,
because `we` is huge (5200 rad/s):

```
vd = R*id - we*Lq*iq      <- we*Lq ~ 0.57, so any iq swing throws vd around violently
```

So vd, which was tiny, gets driven hard and **saturates too**. Now **both axes are railed**:
the applied voltage vector is stuck at magnitude `max_v_mag` and can only rotate; the
controller can no longer independently set vd and vq to regulate id and iq. Zero authority.

## Why "both railed" means instability

With the input saturated, the dq plant runs open-loop:

```
eigenvalues ~ -R/L +/- j*we*sqrt(...)   -> damping ratio ~ R/(we*L)
                                        ~ 0.03/(5200*9e-5) ~ 0.06
```

At this speed that is a **very lightly damped resonator** (~6% damping, ringing near `we`).
Undamped dynamics + a saturating nonlinearity = a **limit cycle**: the currents oscillate.
(In the fully developed oscillation the opposite extreme appears — vd pinned at max, vq -> 0
— because the saturated vector is now *rotating* through the oscillation.)

## The one-line causal chain

```
speed up -> vq carries bemf -> vq saturates FIRST
        -> FW drives id negative to offset it (holds the edge)
        -> i_fw hits 70 A cap        <- THE TRIGGER
        -> bemf keeps rising, unoffset -> iq breaks loose
        -> cross-terms explode -> vd saturates TOO
        -> both axes railed -> no control authority
        -> underdamped dq plant rings -> instability
```

## Takeaway

The order is not arbitrary: **vq first because it holds the back-EMF; FW-max removes the
only compensator; vd second because the breakdown throws the cross-coupling into it;
instability once both axes rail and the loop goes open at a speed where the plant is barely
damped.**

The real fix is upstream — keep bemf under the budget so you never reach the FW-max trigger:
cap `l_max_erpm`, raise `Vbus`, or use a lower-Kv / higher-Ld motor. Throttling the setpoint
(regen cut, current limits) does not help once the loop is voltage-saturated. See also
`doc_dead_time.md` and the "Where vq is limited / saturated" section of `README.md`.
