# Robot Monitor & Motor Control Tools

This directory contains tools for visualizing the Robocup Line robot telemetry and interacting with the TB6612 motor driver.

## Prerequisites

You can install all required dependencies using:
```bash
pip install -r requirements.txt
```

Or activate the existing environment at `/home/ttf/.venv`:
```bash
source /home/ttf/.venv/bin/activate
```

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

A dedicated command-line interface for sending commands over the UART bus to the TB6612 motor controller.

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
Kp=4, Ki=2, Kd=0.5, iMax=30; `GTMIN` 60, `GTTOL` 2). If a turn wiggles,
rings, or overshoots before `TDONE`, raise `GTKD` first, then re-try.
**Full step-by-step procedure:** see [`docs/PID_TUNING.md`](../docs/PID_TUNING.md).

`GSTATUS` replies are parsed by the CLI (the response filter accepts the
`GSTATUS:` prefix).

## 3. Motor Control Panel (`tools/motor_gui.py`)

A CustomTkinter desktop panel (requires a display) with drive buttons +
speed slider (0..90), steer ratio slider, gyro straight-hold buttons (`GFWD` /
`GBACK` / `GSTOP`), Kp slider, `GMAX` entry, `GSTATUS` readout, keyboard
driving (arrows or WASD: `w`/`s` forward/back, `a`/`d` pivot, Space = STOP,
`q`/`e`/`z`/`c` = gyro turns), and a TX/RX log. Reuses the `MotorCLI` serial
layer, so commands behave identically to the CLI.

```bash
~/rcj-line/.venv/bin/python tools/motor_gui.py --port /dev/ttyACM0
```
