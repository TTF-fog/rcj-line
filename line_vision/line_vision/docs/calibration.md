# Calibration guide

This covers everything the robot needs calibrating: the line camera, the zone (TOF, driving, the claw and the triangles), and a check of the victim model. Run it all on the Pi from `~/line_vision`, over SSH or on its desktop.

## When to calibrate

| Situation | Do |
| --- | --- |
| First time, or after changing a camera, the light bar or a mount | Everything, in order |
| New venue or new lighting | Line camera, then zone steps `green red triangle` |
| New tape or new floor | Line camera |
| Changed wheels, motors or batteries feel different | Zone step `move` |
| Moved the claw | Zone steps `grab far` |

## Before you start

1. Stop the copy that starts at boot, because only one program can use the cameras and the board:
   ```sh
   ./robot.sh stop
   ```
2. Plug in the board and check both cameras are there:
   ```sh
   ls /dev/ttyACM*
   rpicam-hello --list-cameras      # should list imx500 (front) and ov5647 (down)
   ```
3. Turn the light bar on. The line camera's exposure is calibrated with it on, and the robot turns it on at startup.
   ```sh
   ~/rcj-line/.venv/bin/python ~/rcj-line/tools/motor_cli.py light white
   ```
4. Have the field ready: a straight line, a curve, a junction with a green marker, the red goal strip, the silver entrance tape, and the evacuation zone with its triangles and a silver and a black ball.

Both calibration tools print a URL for a live camera view. Open it in a browser on the same network.

## 1. Line camera (down, OV5647)

```sh
python3 -m line_vision.calibrate
```

It walks through focus, flips, exposure, line size, green, red, silver and a final check. Each step tells you where to put the robot. Press Enter to accept, `r` to redo, `s` to skip. It saves `calibration.json` at the end. `CALIBRATION.md` explains each step and what to change if a result is wrong.

Things that matter most:

- Focus the lens for the floor distance. The stock lens is focused for about 1 m.
- Do the exposure step in the venue's light, with the light bar on, over white floor.
- Read the warnings in the line size step. If the view is too narrow for a marker beside the line, junction decisions will be guesses.
- In the final check, every count should be near 0.

Then push the robot by hand over each case and compare with the table in `CALIBRATION.md` section 3:

```sh
python3 -m line_vision --view
```

## 2. Zone (front camera, TOF, drive, claw)

```sh
.venv-yolo/bin/python -m robot.calibrate                 # all steps
.venv-yolo/bin/python -m robot.calibrate green red       # only some steps
```

Each step prints what to do. Press Enter to measure, `s` to skip, then Enter to accept or `r` to redo. Nothing is saved until the end, when it shows the new values and asks before writing `robot.json`. Ctrl+C stops without saving.

| Step | Put the robot | It sets |
| --- | --- | --- |
| `tof` | Anywhere in the zone. Cover each sensor with your hand and check the matching side drops. | nothing, it's a check |
| `wall` | Parallel to a wall, right side about 10 cm from it | `wall_mm`, `gap_mm` |
| `move` | 1 m of clear floor ahead, mark the front. It drives for 2 s; type the distance in cm. | `move_cm_per_s` |
| `grab` | A ball straight ahead, `grab_creep_cm` (default shown in the prompt) in front of where the claw picks it up | `grab_line` |
| `far` | A ball against the far wall, about 110 cm away | `ball_min_bottom` |
| `green` | Facing the green triangle from about 50 cm | green colour range |
| `red` | Facing the red triangle from about 50 cm | red colour range |
| `triangle` | Where it should stop in front of the green triangle before turning to back in | `triangle_close_area` |

If a step fails, it says why and asks again. Common ones:

- `tof` shows no readings: the firmware isn't streaming the `TOF,...` line. See ROBOT.md.
- A side reads wrong when covered: the firmware's front/back/left/right order doesn't match the robot.
- `far` says the ball is below the grab line: the ball is too close. Move it further away.
- `triangle` says green covers too little: run `green` again first.

## 3. Victim model check

The robot uses `models/victims_320_ncnn_model` by default. Check it on the real balls in the zone lighting:

```sh
.venv-yolo/bin/python yolo_camera.py --model models/victims_320_ncnn_model --seconds 0 --conf 0.25 --view
```

Press q to quit. Look for:

- Every silver and black ball boxed with the right class, near and far.
- Nothing else boxed above 0.45, which is the confidence the robot acts on (`ball_conf`). Walls, LEDs, triangles and your hands are the usual false hits.

If far balls are missed, try the 416 model (`"model": "models/victims_416_ncnn_model"` in `robot.json`). It is slower, so the robot reacts later. If something that isn't a ball keeps getting boxed above 0.45, raise `ball_conf`.

## 4. Bench test

Run each part on its own and watch the dashboard. Use `--dry` first to see the commands without driving.

```sh
.venv-yolo/bin/python -m robot --view --from line
.venv-yolo/bin/python -m robot --view --from collect
.venv-yolo/bin/python -m robot --view --from green
.venv-yolo/bin/python -m robot --view --from exit
```

Press Enter in the terminal to start an attempt and Enter again for Lack of Progress. If it weaves or is slow on the line, tune `line_pwm` and `line_turn_pwm`. If it overshoots balls, lower `approach_kp`. All of these go in `robot.json`.

## 5. Finish

```sh
./robot.sh start
```

Keep `calibration.json` and `robot.json` backed up off the Pi. A copy of the last good pair saves a lot of time if a venue calibration goes wrong.
