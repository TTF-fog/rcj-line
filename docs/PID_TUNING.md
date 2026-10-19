# Gyro Turn PID Tuning Guide

Applies to the in-place gyro turns (`GTURN / GLEFT / GRIGHT / GTL90 / GTR90 /
GTL45 / GTR45`) in `src/GyroTurn.cpp`, driven from `MotorController::updateTurn()`
at the loop cadence (~50 Hz).

## The controller in one screen

```
error    = wrap180(target - heading)          # deg, -180..+180, shortest path
deltaE   = clamped(error - lastError)          # wrap-safe deg
integral = integral + error * dt               # deg*s (measured loop dt)
drive    = Kp*|error| + |Ki*integral|          # toward-target push
brake    = opposing dE/dt (IIR-filtered)       # closing on the target
mag      = drive - Kd*brake, floored at 10% of drive
pwm      = clamp(mag, [GTMIN if |err| > band], speed)   # band = 4*GTTOL (min 8 deg)
direction: ALWAYS toward the target (sign of error)
```

Two structural anti-wobble properties (not gains — they can't be tuned away):

- **Kd is brake-only.** The D term can only *reduce* the toward-target drive
  while the heading closes on the target; it can never reverse or boost the
  spin (its punch is capped at 90% of the drive, 10% headroom always kept).
  A signed D that subtracts hundreds of PWM mid-approach flips the motors —
  that was the left/right/left wobble source.
- **The `GTMIN` floor sleeps near the target.** Beyond `band = max(4*GTTOL, 8°)`
  the floor still rescues a stalled robot; inside the band the small PID
  output and the finish short-brake land the stop, so nothing shoves you
  back and forth across the landing window.

| Term | Command | GUI | Unit | Meaning |
|---|---|---|---|---|
| Kp | `GTKP <0..50>` | Turn PID "Kp" | PWM / deg | Pull toward target |
| Ki | `GTKI <0..10>` | Turn PID "Ki" | PWM / (deg*s) | Fixes systematic bias |
| Kd | `GTKD <0..20>` | Turn PID "Kd" | PWM*s / deg | Brake-only damping (anti-wiggle) |
| iMax | `GTIMAX <0..255>` | Turn PID "iMax" | PWM | Clamp on the Ki term (anti-windup) |
| floor | `GTMIN <0..255>` | Turn spd area | PWM | Keeps creeping near the target (far band only) |
| tol | `GTTOL <0.5..20>` | — | deg | "Done" window (then 80 ms short-brake); also scales the floor band |

Defaults flashed in `src/main.cpp`: `Kp=3, Ki=1, Kd=1.0, iMax=20, GTMIN=60,
GTTOL=2`. All speeds are additionally clamped by the global `MAX_SPEED` (90).

The bug-shaped behaviors you might see are actually the safeguards:

- **Finish = tolerance crossing.** Inside `GTTOL` the controller latches done,
  brakes 80 ms (`TURN_BRAKE_MS`) and publishes `TDONE`. It will never
  micro-hunt (jitter) inside the window.
- **Stall floor (`GTMIN`, far band only).** Well away from the target, below
  the floor the robot keeps *driving toward the target* (error sign). Close
  in, the floor steps aside (see above) — that's what kills the wiggle.
- **Integral term clamp (`GTIMAX`).** A stuck robot cannot charge more than
  `iMax` PWM of bias; the integrator zeroes on start/finish/abort, so a
  lost-IMU abort never carries a stale bias into the next turn.
- **Derivative filter.** D reads `dE/dt` low-pass filtered 50/50 per step at
  ~50 Hz, deltas are wrap-safe across the 0/360 seam, and the brake punch is
  capped — gyro noise or a fast approach cannot spike the motors.

## Symptoms → fixes

| Symptom | Likely cause | First move |
|---|---|---|
| Wiggling around the target before `TDONE` | Floor still fighting inside the band, or Kp too hot | Lower turn speed one notch (GUI slider); then `GTTOL` 2 → 2.5 |
| Overshoots 90°, comes back swinging | Approach too fast for the brake floor | Lower turn speed (biggest lever); `GTKD` up (+0.25) |
| Stops 2–4° short consistently (far side) | `GTMIN` too high for the glide zone | `GTMIN` 60 → 50, or `GTTOL` a touch wider |
| Creeps forever, `TDONE` late | `GTMIN` too low to move the wheels (stall, far) | `GTMIN` up (60 → 70/80) |
| Slow drift after `TDONE` (brake grip) | Integrator fighting wheel bias | `GTKI` up in 0.5 steps (max 10) |
| Different turn quality L vs R | Mechanical/surface bias | `GTKI`, and `GTTOL` a touch wider |
| Turns worse as battery drains | PWM floor/headroom shifted | Re-validate after charging; retune `GTMIN` if needed |

Rule of thumb for ranges on this drivetrain: `Kp` 2–6, `Kd` 0.5–2.0,
`Ki` 0–2, `GTMIN` 50–80, `speed` 70–90, `GTTOL` 2–3.

## Procedure (15 minutes, wheels off the ground first)

Setup: `gstatus` to view current gains; poll `T,...` telemetry (turn lines,
5 Hz) and the `TDONE err=` lines to judge each attempt. Charge the battery.
Repeat each `gtl90` **3 times** and look at the *worst* of the three. If any
attempt misbehaves, you don't have the constant yet.

### Step 0 — wheels off the ground
1. `gtl90` × 3. Expect plainly instable symmetry (no floor, low stress).
   This validates wiring/sign of the heading: a *left* button must spin it
   CCW seen from above and the error must shrink monotonically.

### Step 1 — Kp first (D and I still zero)
1. `gtki 0`, `gtkd 0`, `gtkp 3` → `gtl90` × 3.
2. Overshoots or rings (the "wiggle") → `gtkp 2` and repeat.
3. Sluggish/stalls mid-turn → `gtkp 5`–`6` and repeat.
4. Land on the highest Kp with **no ringing**. Note the Kp.

### Step 2 — D second (kills the wiggle for good)
1. `gtkd 1.0` → `gtl90` × 3.
2. Move D up in +0.25 steps while keeping Kp. D lets you re-raise Kp that
   was previously unstable: each 0.25 of D usually buys ~1 of Kp headroom.
3. Stop when: reaches tolerance once, no re-cross visible in `T,...` lines
   (`err` monotonic to sign change may be OK, but only *once*).
4. If `TDONE err` sits positive (short-stop bias) → drop `gtkd` a notch;
   if it orbits 90 → raise `gttd` or drop Kp a notch.

### Step 3 — Ki last (only if needed)
Add Ki **only** if the final `TDONE err` is consistently biased (e.g.
always +1.5°) — that's a mechanical/surface asymmetry, not dynamics.
1. `gtki 1.0` → repeat. 2-step increments up to 3.0; never above 5 without
   re-checking `gtimax`. A wrongly-signed Ki oscillates *slowly* (seconds
   scale) — if it does, reset to 2 or 0.

### Step 4 — validate on the floor
1. Wheels down on the actual competition surface.
2. `gtl90` / `gtr90` / `gtl45` / `gtr45` × 3 each.
3. Field-crossing checks: after `gtl90`, drive `fwd 90` and confirm the path
   is straight (no residual drift); after a full `gtl90`+`gtr90` pair the
   heading must return to start within tolerance.
4. Capture final numbers and write them into `src/main.cpp` `setup()` so a
   reflash keeps the tuned robot (values are not persisted to flash).

## Reading the telemetry while tuning

- `T,<head>,<target>,<err>,L=..,R=..,Kp=..` @ 5 Hz while turning: `err`
  should shrink monotonically-ish toward `GTTOL`.
- `TDONE err=.. target=.. head=..` — the one-shot completion line; the
  `err` here is your *quality* metric (aim |err| < 1).
- `GSTATUS` while idle prints live `turnKp/turnKi/turnKd/turniMax/turnTol`.
- `TABORT imu` means heading reads failed 5× — retighten IMU mounting/wiring;
  not a PID problem.

## Quick backdoor: turn-ish wiggle but not the turn it is wiggling

If a `gtl90` is damped nicely but the *robot body* still wiggles while
**driving straight** (`gfwd`), that's the straight-hold controller, not this
one — tune `GKP` (default 2 PWM/deg) and `GMAX` (default 80 PWM), same
procedure-style reasoning but on Kp alone.
