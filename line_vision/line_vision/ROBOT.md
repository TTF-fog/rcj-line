# Robot program

`python -m robot` runs a whole RCJ Rescue Line attempt: line following on the down camera (`line_vision`), the evacuation zone on the front camera (YOLO balls, colour triangles, TOF wall follow), the goal stop, and Lack of Progress restarts. Board commands are the ones in `~/rcj-line/docs/HARDWARE_GUIDE.md`.

```sh
.venv-yolo/bin/python -m robot                     # competition: wait for the button, run, repeat
.venv-yolo/bin/python -m robot --view              # plus the dashboard on port 8080
.venv-yolo/bin/python -m robot --from collect      # every attempt starts at a zone step (bench testing)
.venv-yolo/bin/python -m robot --dry --debug-dir runs/test1   # print board commands, save a frame per event
```

Settings: `robot.json` (only the keys you change, defaults in `robot/config.py`) and the line follower's `calibration.json`. Every session writes a log to `runs/logs/`.

## Bring-up: plug in, calibrate, run

Step-by-step guides: `docs/calibration.md` (all calibration) and `docs/pre-run.md` (checklist before each run).

Stop the boot copy first (`./robot.sh stop`), because only one program can hold the cameras and the serial port.

1. Flash firmware that streams the TOF line below. The startup log shows `TOF: {...}` with the four distances, or a warning that none arrived.
2. Check the firmware's sides match the robot: in `python -m robot.calibrate tof`, cover each sensor with your hand and watch that side drop.
3. Plug the OV5647 line camera into the second camera port (22-pin cable on the Pi 5). `rpicam-hello --list-cameras` should list `imx500` and `ov5647`.
4. Turn the light bar on, then calibrate the line camera (see CALIBRATION.md):
   ```sh
   ~/rcj-line/.venv/bin/python ~/rcj-line/tools/motor_cli.py light white
   python3 -m line_vision.calibrate
   ```
5. Calibrate the zone: `.venv-yolo/bin/python -m robot.calibrate`. It walks through `tof wall move grab far green red triangle`, shows a live view in the browser, and saves accepted values to `robot.json`. Re-run single steps with e.g. `python -m robot.calibrate green red`.
6. Set `buttons` in `robot.json` to the button GPIO pins.
7. Bench test each part: `--from line`, `--from collect`, `--from green`, `--from exit`, all with `--view`.
8. Tune `line_pwm` / `line_turn_pwm` and `approach_kp` by watching runs.
9. Start it for real with `./robot.sh start` (it already starts at boot, see below).

At a new venue repeat steps 4 and 5 (`green red triangle` at least).

When the storage stick firmware exists, put its commands into `robot.json`, e.g. `{"drop_cmds": {"L": "CLAW RAW 4 30", "R": "CLAW RAW 4 150", "N": "CLAW RAW 4 90"}}`. Until then the robot sends `DROP L|R|N`, which the board answers with an error.

## Firmware TOF line

The Pi expects the board to stream, at 20 Hz or more, on USB:

```
TOF,<front>,<back>,<left>,<right>
```

Distances are whole millimetres. Send `-1` when a sensor sees nothing in range. The Pi treats `-1` and anything at or above 2000 as "far", and stops trusting TOF if no line arrives for 0.5 s. Example: `TOF,412,655,-1,98`.

## Layout

| File | Job |
| --- | --- |
| `robot/__main__.py` | Opens board, cameras, buttons, viewer. Loops: wait for start, run an attempt, handle LoP. |
| `robot/run.py` | `Run`: state kept across LoPs (delivered victims, LoP count, run clock) and one attempt: line follow, zone, line follow, goal. |
| `robot/zone.py` | `Zone`: enter, collect, green, red, exit. |
| `robot/board.py` | Serial link to the Motion 2350. Checks `PING` at startup. Every call raises `Aborted` once LoP is pressed. |
| `robot/balls.py`, `robot/zone_vision.py` | YOLO detector, triangle masks, exit strip check. |
| `robot/tof.py` | Latest `TOF,...` line from the board. |
| `robot/calibrate.py` | Guided zone calibration into `robot.json`. |
| `robot/log.py` | Timestamped log to the terminal and `runs/logs/`. |
| `robot/dash.py`, `robot/dash.html` | Browser dashboard. |
| `robot/ctk_dash.py` | Desktop dashboard (CustomTkinter) that talks to the browser dashboard's server. |
| `robot.sh` | Start, stop and start-at-boot. |

## Dashboard

`--view [PORT]` serves a dashboard at the printed URL (default port 8080). It shows the live camera with the line or zone overlay, the phase and zone step, the run clock against 8:00, LoP count, the three victims (missing, held, rescued), TOF distances, cameras, board port, CPU temperature, and the log with LoPs, warnings and zone events coloured. The Start / Lack of Progress and New run buttons do the same as the robot's buttons.

The rules forbid sending anything to the robot during an official run, so use the dashboard buttons on the bench only. Watching is fine. The dashboard has no external dependencies, so it works on a field network with no internet.

The server's routes are `/` (the page), `/state` (JSON), `/stream` (MJPEG), `/frame.jpg` (latest frame, 204 before the first one), and POST `/press` and `/hold`.

### Desktop dashboard

The same view as a CustomTkinter app. It only needs `customtkinter` and `Pillow`, so `robot/ctk_dash.py` can be copied to a laptop on its own. The robot must run with `--view`.

```sh
~/rcj-line/.venv/bin/python robot/ctk_dash.py --host localhost:8080      # on the Pi
python ctk_dash.py --host 192.168.1.4:8080                               # on a laptop
```

Space presses Start / Lack of Progress. The host box and Connect switch robots without restarting.

## Buttons

Set `buttons` in `robot.json` to Pi GPIO (BCM) pins, one or two, all doing the same:

| Action | Idle | Running |
| --- | --- | --- |
| Press | Start an attempt | Lack of Progress: stop everything, wait |
| Hold `hold_s` | New run (clears rescued victims, LoPs, clock) | nothing |

Enter in the terminal acts as a press, `r` + Enter as a hold. Reaching the goal finishes the run, so the next start is a new run.

## Lack of Progress

The referee always puts the robot on the previous checkpoint, so every attempt starts with line following.

- On the line: the next attempt follows the line from the checkpoint.
- In the zone: held victims are forgotten (the referee puts them on the floor), delivered ones stay counted. The next attempt follows the line, sees silver again, and the zone only looks for what is still missing. With everything delivered it goes straight to finding the exit.
- After the zone: the same rule covers both cases. If the checkpoint is after the zone, the robot follows the line to the goal. If it is before, it goes through the zone again.

If the zone gives up (exit not found, no TOF), or a step crashes, the robot stops, logs why, and waits for a LoP press.

## Zone flow

| Step | What happens |
| --- | --- |
| enter | Gyro straight `enter_cm`, or with TOF until front and back distances match. |
| collect | 30 deg gyro steps. After each step, `look_frames` frames taken after the robot settles. A wanted ball on all of them gets a P approach on box centre x until its bottom edge reaches `grab_line`. Then creep `grab_creep_cm` and `CLAW GRAB L` (silver) or `R` (black). The class is a vote over the whole approach. |
| green | Only if silver is held. Step-scan for the green mask, P approach until it covers `triangle_close_area`, turn 180, back `drop_back_cm`, `DROP L`, `DROP N`, drive away. |
| red | Same with red and `DROP R`. |
| exit | Drive to a wall, keep it on the right with TOF, turn left at corners. At a gap, turn in and check the floor: black strip = drive through and return to line following, silver = the entrance, back out and carry on. |

Collecting stops at 2 silver plus 1 black, after `collect_budget_s`, or after `max_empty_scans` empty full turns. Classes that are already full are ignored. Boxes whose bottom sits above `ball_min_bottom` are on a wall (LEDs, pictures) and are ignored.

## Start at boot

```sh
./robot.sh enable     # adds "@reboot sleep 10 && /home/pi/line_vision/robot.sh start" to the user crontab (done)
./robot.sh disable    # removes it
./robot.sh start      # runs python -m robot --view in the background; extra args replace --view
./robot.sh stop       # SIGTERM, which halts the motors before exiting
./robot.sh status
./robot.sh log        # follows runs/logs/robot.out
```

The script waits for `/dev/ttyACM*` before starting and restarts the program 5 s after it exits, so a board plugged in after boot or a crash is picked up on its own. It uses a pid file, so starting it twice does nothing.

## Still to do

- Firmware: stream the `TOF,...` line.
- Storage stick firmware, then `drop_cmds` in `robot.json`.
- Silver entry model. Today the navigator's `silver_strip` from `line_vision` triggers the zone.
- Check the triangle wall height (6 cm) against the TOF mounting height.
