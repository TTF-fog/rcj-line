# line_vision

Camera line following for RCJ Rescue Line on a Pi 5 with the OV5647 camera. It prints one drive command per frame and does not control the chassis.

```
python3 -m line_vision.calibrate                         # guided calibration, writes calibration.json
python3 -m line_vision                                   # live camera
python3 -m line_vision --view                            # plus a live debug view in the browser
python3 -m line_vision --debug-dir runs/1                # also save raw frames and overlays
python3 -m line_vision --input runs/1                    # replay saved frames, an image or a video
python3 -m unittest                                      # tests on synthetic scenes
```

Requires `python3-opencv`, `python3-numpy` and `python3-picamera2` (apt).

Output format: `frame state action speed turn note`. `speed` is 0..1. `turn` is -1 (full left) to +1 (full right). `U_TURN` with `turn=+1` and `speed=0` means pivot in place.

See [CALIBRATION.md](CALIBRATION.md) for the full calibrate, verify, tune and run procedure. Settings live in `calibration.json`; defaults are in `line_vision/config.py`.

For the separate CPU YOLO26n experiment with the IMX500 camera, see [YOLO.md](YOLO.md).

For the full robot run (line, zone, goal, Lack of Progress, buttons), see [ROBOT.md](ROBOT.md).
