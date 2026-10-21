# Serial protocol

USB serial, any baud rate. One command per line, ending in `\n` (`\r\n` is fine).
Commands and setting names are case-insensitive. Speeds and powers are percent.
Angles are degrees, and a positive angle or steer means "to the right" (clockwise).

## Replies

Every command gets exactly one reply line, after any data lines it prints:

```
OK <NAME>
ERR <NAME> <reason>
```

Moves that have an end (a `DRIVE`/`STRAIGHT` with a duration, every `TURN`, and `GRAB`)
report it later with one more line:

```
DONE DRIVE
DONE STRAIGHT
DONE TURN err=-2.1        heading error left over, inside turn.tol
FAIL TURN timeout err=37.0
FAIL STRAIGHT imu err=1.2  the IMU stopped answering
CANCEL TURN               replaced by another move, or STOP
DONE GRAB
CANCEL GRAB               ESTOP button
```

Open-ended moves (no duration) never print DONE or CANCEL. Starting any move replaces
the one that's running.

Other lines the robot sends on its own:

```
READY imu=1 tof=2         after boot
EVT ESTOP                 user button pressed: motors braked, GRAB cancelled
EVT TOF 3 init_failed     a ToF slot failed discovery (init_failed / address_failed)
EVT TOF 1 lost            a working ToF gave no reading for 0.5 s
EVT TOF stray device at 0x29 on I2C1, check XSHUT wiring
```

## Commands

| Command | What it does |
|---|---|
| `PING` | Just answers `OK PING`. |
| `DRIVE <speed> <steer> [ms]` | Steering drive. steer 0 = straight, 1 = spin right in place, -1 = spin left. |
| `STRAIGHT <speed> [ms]` | Drive holding the heading the robot had when the move started. Re-sending an open-ended `STRAIGHT` only changes the speed. |
| `TURN <deg> [power]` | Spin in place by deg (any size, e.g. 360). Power defaults to `turn.max`. |
| `STOP` | Brake and end the current move. Does not touch a running GRAB. |
| `MOTORS <m1> <m2> <m3> <m4>` | Drive each motor directly, for checking directions. |
| `GRAB <L\|C\|R>` | Pickup: arm down, jaws open, jaws neutral, jaws biased to L/R (C = none), arm up, jaws creep open. |
| `ARM <UP\|DOWN>`, `JAW <OPEN\|NEUTRAL>` | Move to one stored claw pose. |
| `SERVO <1\|2\|3> <deg>` | Raw servo move: 1 = left jaw, 2 = right jaw, 3 = arm. |
| `LIGHT <r> <g> <b>`, `LIGHT OFF` | Light bar color, 0..255. Boots white. |
| `STATUS`, `IMU` | Print one `STAT` or `IMU` line. |
| `TOF` | Print one `TOF <slot> state=.. bus=.. addr=.. xshut=.. mm=..` line per slot. |
| `TOF SCAN` | Rediscover all ToF sensors, then print the slots. Refused while moving (blocks ~0.5 s). |
| `STREAM <hz>` | Send `IMU` + `STAT` lines continuously, 0 = off. Same as `SET stream.hz`. |
| `SET <key> <value>` | Change a setting until reboot. |
| `GET [key]` | Print one or all settings as `CFG key=value` lines. |
| `SAVE` | Write the current settings to flash. |
| `DEFAULTS` | Restore the built-in settings (then `SAVE` to keep them). |
| `BOOTLOAD` | Reboot into the UF2 bootloader. |
| `HELP` | List commands. |

## Stream lines

```
IMU heading=93.25 roll=-0.50 pitch=1.06 gx=0.00 gy=-0.06 gz=12.50 qw=0.7310 qx=0.0040 qy=-0.0060 qz=-0.6820 cal=3,3,1,0
STAT mode=TURN left=45 right=-45 target=183.3 err=12.4 remain=0 grab=IDLE tof=312,-1,8190,-1
```

- `heading` is 0..360 clockwise from an arbitrary zero (the BNO055's last power-up,
  not the last firmware reboot). The magnetometer is off, so `cal` mag is always 0.
- `gx/gy/gz` are deg/s. `cal` is system, gyro, accel, mag calibration, 0..3.
- `left`/`right` are the motor outputs right now, after the ramp.
- `target`/`err` are `nan` unless in STRAIGHT or TURN.
- `remain` is ms left on a timed move, 0 otherwise.
- `tof` is mm for sensors 1..4. -1 = no reading (slot not `ok`, see `TOF`), 8190 or more = nothing in range.

Stream lines are dropped, never queued, when the Pi isn't reading fast enough.

## Settings

| Key | Default | Meaning |
|---|---|---|
| `motor.invert1..4` | 0, 1, 1, 1 | Reverse one motor. |
| `motor.swap_sides` | 0 | Swap which motor pair is the left side. |
| `motor.ramp` | 10 | Max output change per 10 ms, in %. Softens current spikes. 0 = off. |
| `heading.invert` | 0 | Flip the heading direction. |
| `straight.kp`, `straight.kd` | 1.5, 0.05 | Heading hold gains (% per deg, % per deg/s). |
| `straight.max` | 30 | Max steering correction, %. |
| `turn.kp`, `turn.kd` | 1.0, 0.05 | Turn gains. |
| `turn.min`, `turn.max` | 25, 60 | Power floor (so it doesn't stall) and cap, %. |
| `turn.tol` | 5 | A turn is done within this many degrees. |
| `turn.timeout` | 5000 | ms before a turn gives up with FAIL. |
| `claw.arm_up`, `claw.arm_down` | 130, 40 | Arm servo angles. |
| `claw.open_l`, `claw.open_r` | 20, 160 | Jaws open. |
| `claw.neutral_l`, `claw.neutral_r` | 80, 100 | Jaws neutral (closed on the object). |
| `claw.bias` | 15 | Degrees both jaws shift for GRAB L/R. Negative if L/R come out swapped. |
| `claw.arm_ms`, `claw.jaw_ms` | 450, 300 | Wait after each arm / jaw move in GRAB. |
| `claw.creep` | 120 | Speed of the final slow open, deg/s. |
| `stream.hz` | 20 | Stream rate, 0 = off. |
