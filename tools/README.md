# Robot Monitor & Motor Control Tools

This directory contains tools for visualizing the Robocup Line robot telemetry and interacting with its motor controller.

## Prerequisites

You can install all required dependencies using:
```bash
pip install -r requirements.txt
```

Or use the environment at `/home/pi/rcj-line/.venv`:
```bash
source ~/rcj-line/.venv/bin/activate
```

> **Board note:** the robot runs on a **Cytron Motion 2350 Pro** (RP2350).
> See [`docs/MOTION2350.md`](../docs/MOTION2350.md) for wiring and flashing.

---

## 1. Rerun Visualizer (`tools/visualizer.py`)

A live dashboard featuring:
- **3D Robot Orientation**: Real-time rendering of robot pose based on BNO055 quaternion data (`Q,w,x,y,z`), with 3D chassis, wheels, and coordinate axes.
- **Motor Speed Time-Series Graph**: Real-time scalar plots of left and right motor speeds.
- **Motor Commands Dataframe**: Live history table showing timestamp, command, left/right speeds, and status/acknowledgments.
- **Interactive Terminal Input**: You can type motor commands directly into the terminal while watching Rerun update live!

### Usage Examples

1. **Hardware Mode (Default / Auto-detect)**:
   ```bash
   ~/rcj-line/.venv/bin/python tools/visualizer.py
   ```
   *Auto-detects `/dev/ttyACM0`.*

2. **Separate IMU and Motor UART Ports**:
   ```bash
   ~/rcj-line/.venv/bin/python tools/visualizer.py --imu-port /dev/ttyACM0 --motor-port /dev/ttyUSB0
   ```

3. **Demo / Simulation Mode (No hardware required)**:
   ```bash
   ~/rcj-line/.venv/bin/python tools/visualizer.py --demo
   ```

---

## 2. Standalone Motor CLI (`tools/motor_cli.py`)

A dedicated command-line interface for sending commands over the serial bus to the RP2350 motor controller.

### Usage Examples

1. **Interactive REPL Prompt**:
   ```bash
   ~/rcj-line/.venv/bin/python tools/motor_cli.py --port /dev/ttyACM0
   ```
   In the prompt:
   ```text
   motor> fwd 180
   motor> steer 150 0.5
   motor> stop
   motor> status
   ```

2. **One-Shot Command Execution**:
   ```bash
   ~/rcj-line/.venv/bin/python tools/motor_cli.py --port /dev/ttyACM0 fwd 90
   ~/rcj-line/.venv/bin/python tools/motor_cli.py --port /dev/ttyACM0 stop
   ```

### Guided shortcuts (interactive prompt)

Besides raw passthrough, the prompt expands these (defaults in brackets):

```text
motor> fwd [90] | back [90] | steer <spd> <ratio> | left [90] | right [90]
motor> speed <L> <R> | stop | brake | status
motor> gfwd [90] [kp] | gback [90] [kp]   # gyro straight-hold, heading latched
motor> gstop | gkp <0..20> | gmax <0..255> | gstatus
```

All drive speeds (and turn speeds) are capped at **90 PWM** by both the tools
and the firmware (`MAX_SPEED`), so passing a larger value is silently clamped.

### Tuning gyro turns (PID)

Turn PID gains are tuned live with `GTKP <0..50>` / `GTKI <0..10>` /
`GTKD <0..20>` / `GTIMAX <0..255>` (defaults flashed in `src/main.cpp`:
Kp=3, Ki=1, Kd=1, iMax=20; `GTMIN` 60, `GTTOL` 2). If a turn wiggles,
rings, or overshoots before `TDONE`, raise `GTKD` first, then re-try.
**Full step-by-step procedure:** see [`docs/PID_TUNING.md`](../docs/PID_TUNING.md).

### Claw & pickup servos (tunable)

Three servos (claw jaws GP2/GP7, pickup arm GP5) are driven with the `CLAW`
command. Every action reads a stored angle; passing angles tunes **and** moves
in one go:

```text
motor> claw open          # jaws apart at the stored open pose
motor> claw open 15 165   # set open pose (left=15, right=165) then move
motor> claw close 80 100  # set + go to the closed pose
motor> claw pick 40       # pickup arm down (grab) at 40 deg
motor> claw drop 130      # pickup arm up (release) at 130 deg
motor> claw raw 2 90      # nudge one servo: 1=left, 2=right, 3=pickup
motor> claw seq 60 120    # scoop: full close -> Pi grip angles -> pickup -> slow open
motor> claw               # status: stored angles + live pose + paste-ready line
motor> claw reset         # restore compiled-in defaults
motor> claw detach|attach # stop pulsing / re-arm the servos
```

Angles are RAM-only: run `claw` once the mechanics feel right, then bake the
printed `# defaults: OPEN_L=.. OPEN_R=..` line into `include/ServoClaw.h`.
See [`docs/MOTION2350.md`](../docs/MOTION2350.md#claw--pickup-servos-gp2--gp7--gp5).

`GSTATUS` replies are parsed by the CLI (the response filter accepts the
`GSTATUS:` prefix).

## 3. Motor Control Panel (`tools/motor_gui.py`)

A CustomTkinter desktop panel (requires a display) with drive buttons +
speed slider (0..90), steer ratio slider, a Light panel (bar GP18 presets,
ON/OFF, brightness slider), a **Claw / pickup panel** (action buttons
`OPEN`/`CLOSE`/`PICK`/`DROP`, the scoop sequence `SEQ`, per-angle calibration
entries + `Set+…` buttons, and a `Get` button that reads the MCU's calibrated
angles back into the fields), gyro straight-hold buttons (`GFWD` /
`GBACK` / `GSTOP`), Kp slider, `GMAX` entry, `GSTATUS` readout, keyboard
driving (arrows or WASD: `w`/`s` forward/back, `a`/`d` pivot, Space = STOP,
`q`/`e`/`z`/`c` = gyro turns), and a TX/RX log. Reuses the `MotorCLI` serial
layer, so commands behave identically to the CLI.

```bash
~/rcj-line/.venv/bin/python tools/motor_gui.py --port /dev/ttyACM0
```

## Firmware flashing (Cytron Motion 2350 Pro)

Build on Windows, flash from the Pi — **no BOOTSEL button, no loader tool**:

```bash
# Windows (this machine):
pio run -e motion2350pro          # -> .pio/build/motion2350pro/firmware.uf2
# ...upload + flash + verify in one step:
#    C:\...\Temp\opencode\pi_flash_rp2350.py  (uses paramiko, needs Pi online)

# Pi (manual, equivalent):
~/rcj-line/.venv/bin/python tools/flash_rp2350.py [--uf2 ~/rcj-line/firmware.uf2]
```

What `flash_rp2350.py` does: probes the robot's serial port (`PING`→`PONG`),
sends the `BOOTLOAD` command (firmware reboots into the RP2350 UF2
mass-storage bootloader), copies `firmware.uf2` onto the `RP2350` drive the
board exposes over USB, and verifies the robot answers again afterwards.

Manual fallback (one-time bring-up or dead firmware): hold **BOOTSEL** while
plugging in USB, drag `firmware.uf2` onto the `RP2350` drive.
