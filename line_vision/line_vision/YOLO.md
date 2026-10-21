# YOLO on this Pi

YOLO26n, NCNN FP16 weights, CPU only. The IMX500 supplies ordinary 640x480 BGR frames through Picamera2 at 30 FPS. No IMX500 model is loaded. Capture reuses the project's newest-frame camera thread. Inference runs directly through NCNN without importing PyTorch or Ultralytics.

## Run

```sh
cd /home/pi/line_vision
.venv-yolo/bin/python yolo_camera.py --seconds 0 --view
```

Press q or Ctrl-C to stop. Omit `--view` for headless operation. The default uses 320x320 input and two CPU threads. To preserve more object detail at lower FPS:

```sh
.venv-yolo/bin/python yolo_camera.py --model models/yolo26n_416_ncnn_model --seconds 0 --view
```

Save benchmark statistics and one annotated frame:

```sh
.venv-yolo/bin/python yolo_camera.py --seconds 60 --output runs/yolo/check
```

FPS includes waiting for new camera frames, preprocessing, inference and postprocessing. With `--view`, it also includes drawing/display. Five warmup frames and camera startup are excluded. Sensor-to-result latency is separate from FPS. Frames can be dropped to avoid a growing processing queue.

## Measured results

Headless live-camera tests on this Pi 5 4GB, with two NCNN threads:

| Model input | Duration | Live FPS | Mean processing | Mean sensor-to-result |
| --- | --- | --- | --- | --- |
| 320x320 | 60 s | 29.06 | 32.18 ms | 78.85 ms |
| 416x416 | 60 s | 16.82 | 59.20 ms | 185.60 ms |
| 640x640 | 20 s | 5.74 | 173.68 ms | 516.30 ms |

Raw statistics and annotated camera images are in `runs/yolo/sustained/` and `runs/yolo/tuned/`. These are speed tests, not labeled accuracy evaluation. Both sustained runs produced detections.

A separate 10-second desktop preview test at 320 reached 20.87 FPS including drawing/display. Its saved frame and statistics are in `runs/yolo/preview/`. Qt emitted missing-font warnings, but the preview ran successfully.

The Pi reached 84.8-85.35 C and reported throttling. Improve cooling and verify power before robot deployment. No overclock, governor change or OS update was used. Two threads were faster than four on this running desktop; four threads caused much higher latency. Browser/desktop activity and thermal state can change results. At 416, the roughly 186 ms average sensor-to-result delay matters for a moving robot even though throughput meets 15 FPS. Prefer 320 when response time matters, if your validation accuracy permits it.

## Setup used

The existing system Picamera2, libcamera and NumPy packages are available inside a separate virtual environment. System Python packages were not changed.

```sh
python3 -m venv --system-site-packages .venv-yolo
.venv-yolo/bin/pip install torch==2.10.0 torchvision==0.25.0 --index-url https://download.pytorch.org/whl/cpu
.venv-yolo/bin/pip install ultralytics==8.4.182 ncnn==1.0.20260526 pnnx==20260526
.venv-yolo/bin/python export_yolo.py
```

NCNN export currently disables YOLO26's end-to-end head because this backend lacks top-k support. The runner uses class-aware OpenCV NMS. FP16 refers to the exported weights and NCNN's supported precision settings, not a guaranteed FP16-only execution path.

## Custom training later

Fine-tune `yolo26n.pt` on a workstation/GPU using your labeled dataset, then copy `best.pt` to this Pi:

```sh
.venv-yolo/bin/python export_yolo.py --weights models/best.pt --sizes 320 416
.venv-yolo/bin/python yolo_camera.py --model models/best_416_ncnn_model --seconds 0 --view
```

The runner reads class names and input size from export metadata. Existing export directories are skipped; use a new weights filename when replacing a model. Choose 320 or 416 using validation accuracy on your own objects, especially small ones. COCO weights do not validate accuracy for rescue-line targets. Training on this 4GB Pi is not recommended.

Ultralytics weights/software use AGPL-3.0 or an enterprise license. See [Ultralytics licensing](https://www.ultralytics.com/license). Backend selection follows the [current Raspberry Pi guide](https://docs.ultralytics.com/guides/raspberry-pi/).
