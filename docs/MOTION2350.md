# Cytron Motion 2350 Pro — Firmware Guide & Wiring

The robot brain is a single **Cytron Motion 2350 Pro** (RP2350A,
arduino-pico / earlephilhower Arduino core). All board specifics live in
`include/BoardPins.h`; the motor-driver logic and gyro control code are
hardware-agnostic.

```
pio run         → .pio/build/motion2350pro/firmware.uf2   (default env)
pio test -e native → 18/18 gyro logic tests (host, no hardware)
```

> Note on numbers: `Serial.printf("%.2f")` on this toolchain prints nothing
> (newlib is linked integer-only). All float telemetry therefore uses
> `Serial.print(value, digits)` — same text on the wire.

## Pin map (board defaults)

| Function | Pin(s) | Notes |
|---|---|---|
| Motor 1 IN A/B | **GP8 / GP9** | terminal block M1 |
| Motor 2 IN A/B | **GP10 / GP11** | terminal block M2 |
| Motor 3 IN A/B | **GP12 / GP13** | terminal block M3 |
| Motor 4 IN A/B | **GP14 / GP15** | terminal block M4 |
| BNO055 IMU I²C | **SDA=GP16, SCL=GP17** | board default `Wire` = I2C0, 100 kHz |
| Claw jaw servos | **GP2 (left), GP7 (right)** | PIO 50 Hz, `CLAW OPEN/CLOSE` |
| Pickup arm servo | **GP5** | PIO 50 Hz, `CLAW PICK/DROP` |
| LF white light bar | **GP18** | LIGHT-command sink, WS2812-type |
| Onboard status LEDs | **GP23** | 2× WS2812, driven by the status engine |
| Feature UART | **TX=GP0, RX=GP1** | `Serial1`, 115200 |
| Onboard buzzer | **GP22** | passive piezo, has a mute switch |
| User buttons | **BT1=GP20, BT2=GP21** | low-active, internal pull-ups |
| USB host port | GP24/GP25 | unused |
| VIN sense | GP29/ADC3 | solder jumper, off by default, unused |

Motor driver: onboard 4-channel driver, **3 A continuous / 5 A peak per
channel**, supply **3.6–16 V**.

## Motor wiring
- Plug each motor pair into terminal blocks **M1..M4**. Polarity per motor is
  absorbed by `setInverted(true, true, true, false)`; if a motor drives
  backwards in the test below, swap its two wires at the terminal (or flip its
  invert flag) — no firmware change needed.
- Verify with startup menu `[2]` (four-motor forward/reverse test) or the GUI.

## I²C / BNO055 wiring
- **SDA → GP16**, **SCL → GP17**, 3V3, GND. The arduino-pico core enables the
  RP2350's internal I²C pull-ups; Adafruit-style BNO055 breakouts also carry
  their own. Run menu `[1]` (I2C scanner) after wiring: the BNO should show at
  **0x28** (or 0x29 when ADR pulled high).
- Firmware init: `bno.begin(Wire, 16, 17, true, 100000)` — external-crystal
  flag and 100 kHz clock unchanged. A vendored patch in `lib/7Semi_BNO055/`
  adapts the library's I2C init to this core.

## Serial
- USB-CDC serial at 115200 = console (`Serial`): menus, commands, telemetry.
- `Serial1` on **GP0 (TX) / GP1 (RX)** at 115200 = secondary command UART
  (the same command set, both UARTs accepted concurrently).

## Claw & pickup servos (GP2 / GP7 / GP5)
Three 50 Hz hobby servos, driven through the core's PIO `Servo` library, so
they are completely independent of the 20 kHz motor PWM:

| Servo | Pin | Action commands |
|---|---|---|
| Claw jaw, left | GP2 | `CLAW OPEN` / `CLAW CLOSE` |
| Claw jaw, right | GP7 | `CLAW OPEN` / `CLAW CLOSE` |
| Pickup arm | GP5 | `CLAW PICK` (down/grab) / `CLAW DROP` (up/release) |

Each action has its own stored angle in degrees (0..180). **Pass the angles to
tune and move in one shot** — the value is saved and applied immediately:

```
CLAW OPEN 15 165     # set open pose (left=15, right=165) and go there
CLAW OPEN            # just move to the stored open pose
CLAW CLOSE 80 100    # set + go to the closed pose
CLAW PICK 40         # pickup arm down at 40 deg
CLAW DROP 130        # pickup arm up at 130 deg
CLAW RAW 2 90        # nudge one servo directly (1=L, 2=R, 3=pickup)
CLAW SEQ 60 120      # scoop: full close -> Pi grip angles -> pickup -> slow open
CLAW                 # status: stored angles + live pose
CLAW RESET           # back to the compiled-in defaults
CLAW DETACH / ATTACH # stop pulsing (frees the PIO state machines) / re-arm
```

`CLAW` status also prints a paste-ready `# defaults: OPEN_L=.. OPEN_R=..` line:
once the mechanics feel right, put those numbers into `ServoClaw::DEF_*`
(`include/ServoClaw.h`) and rebuild so the tuned pose is the power-on pose.
Angles are RAM-only, so a reset/reflash returns to the `DEF_*` values.

Power-on pose is **jaws closed, pickup arm up** (stowed). Servos need their own
5–6 V supply — do not run three servos off the board's 3V3 rail.

## The scoop sequence (`CLAW SEQ [piL] [piR]`)

Runs the real pickup as a non-blocking stage machine on the robot (serviced
from `loop()`; drive + telemetry keep running while it works):

1. **FULL CLOSE** — jaws to the calibrated close pose, ~300 ms grip dwell.
2. **ANGLE** — jaws settle to the grip angles the Pi computed for the object
   (`piL`/`piR`, 0..180). With no angles given this step is skipped.
3. **PICKUP** — arm down to the grab angle, ~450 ms lift-settle dwell.
4. **OPEN CAREFULLY** — jaws creep toward the calibrated open pose in 6°
   steps every 40 ms (~0.6 s) so the object settles instead of kicking.

Total ≈1.5 s with Pi angles. The command acks with `OK: CLAW SEQ …` and the
robot publishes `CLAWSEQ done` on **both** UARTs when finished; `CLAW` status
shows the running stage (`seq=CLOSE|ANGLE|PICKUP|OPEN`). While it runs the
actions answer `ERR: CLAW busy (SEQ running)`. Pi integration: send
`CLAW SEQ <left> <right>\r\n` over GP0/GP1 and watch for `CLAWSEQ done`.

## Status LEDs (2 onboard WS2812, GP23) — priority top-down

| Display | Meaning |
|---|---|
| 🔴 solid red 1.5 s | ESTOP (user button pressed → all motors stopped) |
| 🟢 green flash | gyro turn completed (`TDONE`) |
| 🟣 magenta flash | gyro turn aborted (IMU glitch) |
| 🟡 amber solid | gyro spin turn in progress |
| 🔵 cyan solid | gyro straight-hold in progress |
| 🟣 magenta blink | BNO055 not detected |
| user color | idle → mirrors the last `LIGHT` command |

## LIGHT (LF white bar, GP18)
Boot = solid white. `LIGHT <r> <g> <b> [bri]`, presets `RED/GREEN/BLUE/
YELLOW/CYAN/MAGENTA/ORANGE/WHITE`, `LIGHT OFF/ON`, `LIGHT BRI <0..255>`,
`LIGHT ?` — all target the GP18 bar. (Onboard pixels only mirror the user
color while idle; the status engine owns them otherwise.)

## Buzzer (GP22)
- `BEEP [ms=150] [hz=2730]` — non-blocking chirp (10–2000 ms, 100–5000 Hz).
- Turn completes → one chirp at 3135 Hz (`TDONE`).
- ESTOP button → three urgent chirps at 2730 Hz.
- Boot with a dead/missing BNO055 → one long low tone (990 Hz).
- The onboard **mute switch** silences it physically.

## Flashing
From the PC the board is plugged into:

```
pio run                                  # → firmware.uf2
pio run -t upload                        # resets into the bootloader + flashes
```

The board advertises **1200 bps touch reset**, so `pio run -t upload` (or
PlatformIO's Upload button) flashes without anyone holding BOOTSEL — the host
opens the port at 1200 baud, the RP2350 reboots into its ROM bootloader and
`picotool` writes the image. From the robot's own firmware the `BOOTLOAD`
serial command does the same reset and stops, leaving the `RP2350` UF2 drive
mounted to drag `firmware.uf2` onto. Hard fallback (first-ever flash or dead
firmware): hold **BOOTSEL** while plugging in USB, then drag the `.uf2` onto
the `RP2350` drive.

## Bring-up order (hardware)
1. Power off; wire motors M1..M4 + BNO055 (SDA=16, SCL=17) + light bar (GP18)
   + servos (GP2/GP7/GP5, on their own 5–6 V supply).
2. Flash (`pio run -t upload`, or BOOTSEL the first time).
3. Menu `[1]` → confirm BNO at 0x28.  Menu `[2]` → motor direction check.
4. Menu `[4]` → `CLAW OPEN` / `CLAW CLOSE` / `CLAW PICK` / `CLAW DROP`; pass
   angles to tune, then bake the printed `# defaults:` line into
   `include/ServoClaw.h` and rebuild.
5. `GSTATUS` → `GTL90` → watch status LEDs (amber→green) + chirp.
6. Tune per `docs/PID_TUNING.md` if the chassis changes mass distribution.
