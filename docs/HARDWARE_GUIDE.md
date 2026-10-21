# Hardware guide

How to bring up, operate and tune each part of the robot from a serial terminal.
[PROTOCOL.md](PROTOCOL.md) has the exact syntax of every command, reply and setting.

## Connecting

```sh
python tools/robot_cli.py               # /dev/ttyACM0, stream lines hidden
python tools/robot_cli.py --stream      # also show IMU/STAT lines
```

Type commands, one per line. Every command answers `OK <NAME>` or `ERR <NAME> <reason>`.
Quitting the CLI sends `STOP`.

On power-up the robot prints `READY imu=<0|1> tof=<working sensors>`. Keep it still while
it boots: heading 0 is wherever it points when the IMU starts. The CLI may connect after
`READY` was sent, so use `IMU` (`IMU none` = no BNO055) and `TOF` to check the same things.

## Status pixels and ESTOP

The two onboard pixels (GP23) show what the robot is doing, first match wins:

| Color | Meaning |
|---|---|
| red, 1.5 s | ESTOP pressed |
| red, blinking | BNO055 not found at boot. Heading moves will be refused. |
| purple | GRAB running |
| amber | TURN |
| cyan | STRAIGHT |
| blue | DRIVE |
| dim green | idle |

User button 1 (GP20) is the ESTOP. It brakes the motors, cancels any GRAB, and prints
`EVT ESTOP`. It doesn't latch, so the next command drives again.

## Settings workflow

Everything tunable is a setting. `SET` changes it immediately, but only until reboot.

```
GET                       list everything
GET turn.kp               one value
SET turn.kp 1.2           try a value
SAVE                      keep the current values across reboots
DEFAULTS                  back to built-in values (SAVE to make that stick)
```

Tune with `SET`, test, repeat, then `SAVE` once at the end. A firmware update that
changes the settings layout resets them to defaults.

## Motors

Left side is M1 + M2, right side is M3 + M4. Speeds are percent, -100..100.

### First-time direction check

Put the robot on a stand, wheels in the air.

1. `MOTORS 30 0 0 0`, then `MOTORS 0 30 0 0`, and so on for each motor. Each wheel
   should turn the way that moves the robot forward. Use `STOP` between tests.
2. Fix a wheel that spins backwards with `SET motor.invert<N> 1`, or `0` if it is already 1.
3. `DRIVE 30 0` should drive all four forward. `DRIVE 30 1` should spin the robot right
   (clockwise from above). If it spins left, use `SET motor.swap_sides 1`.
4. `SAVE`.

### Driving

```
DRIVE 50 0                straight ahead at 50 %, until the next command
DRIVE 50 0.3              arc right
DRIVE 50 -1               spin left in place
DRIVE -40 0 1500          reverse for 1.5 s, then brake and print DONE DRIVE
STOP                      brake now
```

Send `DRIVE` as often as you like, for example every camera frame. Each one replaces
the last. The outputs ramp by `motor.ramp` % every 10 ms (0 to 100 in 100 ms by default),
which limits current spikes. Lower it for gentler starts, or set 0 to turn the ramp off.

## Gyro moves (BNO055)

### Heading

`IMU` prints one reading. `heading` is 0..360, increasing clockwise, and 0 is wherever
the robot pointed at boot. The magnetometer is off on purpose (motors disturb it), so
the heading slowly drifts over minutes. Use it for relative moves, not as a compass.

`cal=sys,gyro,accel,mag`: gyro should reach 3 after a few seconds standing still. Mag
always shows 0.

### Straight

```
STRAIGHT 50               hold the current heading, until the next command
STRAIGHT 60               already running: only the speed changes, heading kept
STRAIGHT 50 2000          2 s, then DONE STRAIGHT
STRAIGHT -40 1000         also works in reverse
```

Tuning: if it drifts and never quite corrects, raise `straight.kp`. If it snakes
left and right, lower `straight.kp` or raise `straight.kd`. `straight.max` caps how
hard it may steer.

### Turns

```
TURN 90                   right 90°
TURN -45                  left 45°
TURN 180 40               cap the power at 40 % instead of turn.max
```

A turn ends with `DONE TURN err=<deg>` once it is within `turn.tol` (5°). It ends with
`FAIL TURN timeout err=<deg>` after `turn.timeout` ms.

First-time check: `TURN 90`. If the robot spins away and times out with a growing
error, the heading runs backwards for your mounting: `SET heading.invert 1`.

Tuning, from `STAT` lines or the final `err`:

| Symptom | Fix |
|---|---|
| Overshoots past the tolerance | lower `turn.kp`, or raise `turn.kd` |
| Stops short and times out with a small err | raise `turn.min`, it can't overcome friction |
| Too slow overall | raise `turn.max` |
| Rocks back and forth at the end | lower `turn.min` |

If the IMU stops answering mid-move, the motors brake. A `TURN` or timed `STRAIGHT`
reports `FAIL <MODE> imu`, an open-ended `STRAIGHT` just stops.

## Distance sensors (VL53L0X ToF)

There are four slots. Each sensor needs its XSHUT pin wired, because that is how the
firmware gives every sensor its own address at boot.

| Slot | Bus | XSHUT | Address |
|---|---|---|---|
| 1 | I2C0 (GP16/GP17) | GP3 | 0x30 |
| 2 | I2C0 | GP4 | 0x31 |
| 3 | I2C1 (GP26/GP27) | GP6 | 0x32 |
| 4 | I2C1 | GP19 | 0x33 |

Fill slots in any order. Empty slots are fine.

```
TOF                       state of each slot
TOF SCAN                  power-cycle and rediscover all sensors (only while stopped)
```

```
TOF 1 state=ok bus=I2C0 addr=0x30 xshut=GP3 mm=312
TOF 2 state=missing bus=I2C0 addr=0x31 xshut=GP4 mm=-1
```

Distances also stream in `STAT ... tof=312,-1,-1,-1`. `-1` means there is no
reading from that slot, and 8190 or more means nothing is in range (about 1.2 m+).

| State | Meaning | What to check |
|---|---|---|
| `ok` | working | |
| `missing` | nothing answered | normal for an empty slot, otherwise power, SDA/SCL, XSHUT |
| `init_failed` | answered, wouldn't start | loose wiring, a damaged sensor |
| `address_failed` | didn't accept its address | the same, then `TOF SCAN` |
| `lost` | worked, then stopped giving readings for 0.5 s | connector shaken loose, then `TOF SCAN` |

A slot that fails discovery is held off the bus with XSHUT, so it can't block the
sensors after it. A `lost` slot is no longer read.
Faults print `EVT TOF <slot> <state>` when they happen, but empty slots stay quiet.
`EVT TOF stray device at 0x29 ...` during a scan means something answers at 0x29 while all
XSHUT lines are low. Usually a sensor's XSHUT isn't connected, or the BNO055's ADR pin
is high (it must be at 0x28).

## Claw

Three servos: the left jaw (GP2), the right jaw (GP7), and the arm (GP5) that lowers
and raises them. They need their own 5-6 V supply. At boot the arm goes up and the
jaws go to neutral.

### Pickup

```
GRAB C                    centered object
GRAB L                    object sits to the left: grip shifted left
GRAB R                    ... right
```

The sequence prints `DONE GRAB` at the end:

1. Arm down, wait `claw.arm_ms`.
2. Jaws open, wait `claw.jaw_ms`.
3. Jaws neutral (closed on the object), wait `claw.jaw_ms`.
4. L/R only: both jaws shift by `claw.bias` degrees, wait `claw.jaw_ms`.
5. Arm up, wait `claw.arm_ms`.
6. Jaws creep open at `claw.creep` deg/s.

While it runs, manual claw commands answer `ERR ... busy grabbing`. Driving still works,
`STOP` doesn't touch the claw, and ESTOP cancels the GRAB.

### Tuning the poses

1. Find each angle by hand with raw moves: `SERVO 3 40` (arm), `SERVO 1 25` (left jaw),
   `SERVO 2 155` (right jaw).
2. Store them: `SET claw.arm_down 40`, `SET claw.open_l 25`, and so on for `arm_up`,
   `open_r`, `neutral_l` and `neutral_r`.
3. Check each pose: `ARM DOWN`, `ARM UP`, `JAW OPEN`, `JAW NEUTRAL`.
4. Run `GRAB L`. If the grip shifts right instead, flip the sign: `SET claw.bias -15`.
5. If the arm hasn't arrived before the jaws move, raise `claw.arm_ms`. If the jaws
   haven't arrived before the next step, raise `claw.jaw_ms`. If the object gets flung
   on release, lower `claw.creep`.
6. `SAVE`.

## Light bar

The WS2812 bar on GP18 boots white, to light the line for the camera.

```
LIGHT 255 0 0             red
LIGHT OFF
```

The color isn't saved.

## Flashing

```sh
pio run -t upload         from the computer the board is plugged into
```

Or send `BOOTLOAD`: the board reboots into its UF2 drive, and you copy
`.pio/build/robot/firmware.uf2` onto it. If nothing responds, hold BOOTSEL while
plugging in USB. Saved settings live in their own flash area and should survive
reflashing. Run `GET` afterwards to check.
