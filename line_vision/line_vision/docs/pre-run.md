# Pre-run guide

Work through this before every official run. The rules forbid sending anything to the robot during a run, so everything here happens before you go to the field. Once the run starts, only the robot's own buttons are used.

## At the venue, once

1. Calibrate for the venue lighting: the line camera, then zone steps `green red triangle`. See `docs/calibration.md`.
2. Do one full practice run on a practice field if there is one. Watch for weaving, missed markers and missed balls.
3. Back up `calibration.json` and `robot.json`.

## Before each run (about 10 minutes before)

### Power and hardware

- [ ] Batteries charged. Swap them if you're unsure; low voltage changes speed and turns.
- [ ] Pi supply is solid: no "USB overcurrent" or low-voltage warning on the desktop.
- [ ] Cooling fan or heatsink in place. The Pi reaches 80 °C under load and throttles above that.
- [ ] Hardware ESTOP released and working.
- [ ] Both cameras clean and firmly mounted; the down camera hasn't been knocked.
- [ ] Claw and storage stick move freely and are empty.
- [ ] Wheels clean, no hair or tape on them.

### Software

1. Check the robot program is running and starts at boot:
   ```sh
   ./robot.sh status
   ```
   It should say `running` and `starts at boot`. If not, `./robot.sh start`.
2. Open the dashboard: the browser at `http://<pi address>:8080/` or the desktop app:
   ```sh
   ~/rcj-line/.venv/bin/python robot/ctk_dash.py --host <pi address>:8080
   ```
3. On the dashboard, check:
   - [ ] Phase shows `READY`.
   - [ ] `board` shows a `/dev/ttyACM*` port, not `dry run`.
   - [ ] `down cam` shows `ov5647`, not `missing, using front`.
   - [ ] All four TOF values change when you move a hand near each sensor.
   - [ ] The camera picture looks right side up and in focus.
   - [ ] CPU temperature is below 70 °C.
   - [ ] Victims all show `missing` and LoP shows 0. If not, hold the button for a new run.
   - [ ] No warnings at the end of the log.
4. Look at the light bar: it should be white. The line camera was calibrated with it on.

### Quick function test

On a practice line, press the button once to start and once more for Lack of Progress. Check it follows the line and stops on the second press. Then hold the button to clear the run so the clock and LoP count start from zero.

## At the field

1. Close the dashboard buttons or step away from the laptop. Watching is fine, pressing is not.
2. Put the robot on the start tile as the referee says.
3. Press the robot's button once to start.

During the run:

| Situation | Do |
| --- | --- |
| Referee calls Lack of Progress | Press the button once. The robot stops. Put it on the checkpoint, then press again to restart. |
| Robot stops by itself on the line (`stuck` on the dashboard) | Call Lack of Progress, then as above. |
| Robot drives dangerously | Hit the hardware ESTOP. |

After a Lack of Progress the robot keeps victims it has already delivered and only looks for the missing ones. Victims it was holding are forgotten, because the referee puts them back on the floor.

## After the run

- Note the score, the LoPs and where they happened.
- The run log is in `runs/logs/` (newest file), and the robot's output is in `runs/logs/robot.out`. `./robot.sh log` shows the end of it.
- Hold the button to start a new run before the next attempt.
- Charge the batteries.
