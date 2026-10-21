# rcj-line firmware

Firmware for the Cytron Motion 2350 Pro (RP2350) that drives the RCJ Line robot.
The Raspberry Pi controls it over USB serial (`/dev/ttyACM0`) with the text
protocol in [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Build and flash

```sh
pio run                 # build
pio run -t upload       # flash (the board resets itself into the bootloader)
pio test -e native      # host tests for the control math
```

If the board doesn't show up at all, hold BOOTSEL while plugging in USB and drag
`.pio/build/robot/firmware.uf2` onto the `RP2350` drive.

Control it from a laptop with the GUI, `python tools/robot_gui.py`, or type commands with
`python tools/robot_cli.py`. Both need `pip install -r requirements.txt`.
[docs/HARDWARE_GUIDE.md](docs/HARDWARE_GUIDE.md) explains how to bring up and tune every part.

## Wiring

All pins live in [src/BoardPins.h](src/BoardPins.h).

| What | Pins |
|---|---|
| Motors | M1, M2 = left side, M3, M4 = right side (onboard driver) |
| I2C0: BNO055 + ToF 1, 2 | SDA GP16, SCL GP17 |
| I2C1: ToF 3, 4 | SDA GP26, SCL GP27 |
| ToF XSHUT 1..4 | GP3, GP4, GP6, GP19 |
| Jaw servos | left GP2, right GP7 |
| Arm servo | GP5 |
| Light bar | GP18, 8 WS2812 |
| Status pixels | GP23 (onboard) |
| ESTOP button | GP20 (onboard user button 1) |

The BNO055 must be at 0x28 (ADR low) because 0x29 is the ToF power-on address.
Servos need their own 5-6 V supply.
