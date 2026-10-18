# Calibrating and running line_vision

Everything below runs on the Pi over SSH. To see the camera, open the URL the tools print (for example `http://10.42.0.230:8080/`) in a browser on the same network.

## 1. Hardware, once

- **Focus.** The stock OV5647 lens is focused for about 1 m. At floor distance it is badly blurred: the bench camera scored a sharpness of 6. Turn the lens to fix this; on many v1.3 boards it is glued, so crack the glue first. Watch the "sharpness" number in the calibration view and stop at the highest value.
- **Light.** With the camera close to the floor, the robot shades its own view. The bench camera needed maximum gain. Add a constant, diffused white LED strip or ring that lights the floor under the camera, and keep it on in every run. Without it, black/white separation changes from venue to venue.
- **Mount.** It must be rigid. If you can't raise the camera, tilting it forward is the cheapest way to see more floor. Calibration step 5 tells you whether the view is large enough for green markers.

## 2. Calibrate, at every venue and after any camera or light change

```
cd ~/line_vision
python3 -m line_vision.calibrate
```

Each step tells you where to put the robot, captures for a few seconds, and prints what it measured. Press Enter to accept, `r` to redo, `s` to skip. If a capture is unusable, it says why and asks again. Nothing is saved until the end, when it writes `calibration.json`. Re-running it keeps your hand-edited `nav` settings.

| Step | What you do | What it sets |
|---|---|---|
| 1 Focus | Turn the lens for maximum sharpness | nothing |
| 2 Left/right | Line under the robot's left side | `camera.hflip` |
| 3 Forward/back | Robot across the line, push it forward ~2 cm | `camera.vflip` (it errors if the camera is mounted rotated 90 degrees) |
| 4 Exposure | Venue lighting, robot LEDs on, over white floor | fixed exposure, gain and white balance |
| 5 Line size and black | Enter your measured tape width, then slide along a straight line | size-dependent settings and black thresholds; prints the field of view in mm |
| 6 Green | Marker in view | green HSV range |
| 7 Red | Goal strip in view | red HSV range |
| 8 Silver | Entrance tape in view | silver detection mode |
| 9 Check | Move along plain line, curves, seams and shadow | nothing; counts false detections |

Read the warnings in step 5. If it says the view is too narrow for a marker beside the line, the robot can follow lines and gaps, but intersection decisions will be guesses.

In step 9 every count should be near 0. A count marked "too many" names the setting to change.

## 3. Verify by hand

```
python3 -m line_vision --view --debug-dir runs/test1
```

Push the robot by hand over each case and check the printed action and the live view:

| Case | Expected |
|---|---|
| straight line | `FOLLOW`, turn near 0 |
| line off to the left | `FOLLOW`, negative turn |
| sharp corner | `FOLLOW`, strong turn toward the corner |
| junction, no marker | `SLOW` then `STRAIGHT` |
| junction, left or right marker | `SLOW` then `TURN_LEFT` / `TURN_RIGHT` |
| two markers | `U_TURN` |
| marker after the crossing | ignored (grey box in the view) |
| gap | `GAP_FORWARD`, then `FOLLOW` after the gap |
| red strip | `GOAL STOP`, "run finished" after 6 s |
| silver strip | `EVACUATION_ZONE STOP` |

In the view, magenta is line, blue is the tracked line, green boxes are valid markers, orange arrows are exits, and the white circle is where the line enters.

## 4. Tune with recordings

`--debug-dir` saves `raw_*.jpg` frames plus annotated `dbg_*.jpg`. Edit `calibration.json` and replay the same frames until the decisions are right:

```
python3 -m line_vision --input runs/test1 --debug-dir runs/test1_replay --save-every 1
```

| Symptom | Setting in `calibration.json` |
|---|---|
| Line flickers or breaks up | `vision.black_rel` up (for example 0.6 to 0.7) |
| Shadows or seams count as line | `vision.black_rel` down, `vision.black_rel_max_gray` down |
| Markers missed | `vision.green_lo` saturation and brightness down; `vision.marker_min_area_frac` down |
| Turns happen too early or late at junctions | `nav.commit_y_frac` (0-1, how far down the view the crossing must be), `nav.commit_advance_s` |
| Junction flagged on noise | `nav.junction_enter_frames` up, `vision.min_exit_px` up |
| Weaving on straights | `nav.kp_lateral`, `nav.kp_angle` down, `nav.kd` up |
| Too slow in corners | `nav.kp_angle` up |
| U-turn stops on the crossing line | `nav.u_turn_min_s` up |
| Gives up on gaps too soon | `nav.gap_max_s` up |

To choose the area used for detection, set `vision.roi_top/bottom/left/right` as fractions of the image. Cut off anything that isn't floor, such as the chassis or bumpers.

## 5. Run

```
python3 -m line_vision
```

It prints one line per frame: `frame state action speed turn note`. `speed` runs from 0 to 1; `turn` runs from -1 (full left) to +1 (full right). This is the interface for the chassis code. If the camera stops delivering frames, it prints `STOP` until frames return.
