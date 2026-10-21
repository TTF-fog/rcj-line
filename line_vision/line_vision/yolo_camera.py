import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from line_vision.camera import Camera
from robot.balls import AI_CAMERA, Detector


def temperature():
    return int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000


def run(camera, args, model, threads):
    detector = Detector(model, threads)
    for _ in range(5):
        item = camera.read(2)
        if item is None:
            raise RuntimeError("Camera stopped producing frames")
        detector.predict(item[0], args.conf)
    timings, ages = [], []
    detections = 0
    last_frame = last_boxes = None
    start_temp = temperature()
    start = report = time.perf_counter()
    try:
        while args.seconds == 0 or time.perf_counter() - start < args.seconds:
            item = camera.read(2)
            if item is None:
                raise RuntimeError("Camera stopped producing frames")
            frame, timestamp = item
            before = time.perf_counter()
            boxes = detector.predict(frame, args.conf)
            after = time.perf_counter()
            timings.append((after - before) * 1000)
            ages.append((time.monotonic() - timestamp) * 1000)
            detections += len(boxes)
            last_frame, last_boxes = frame, boxes
            if args.view:
                image = detector.draw(frame, boxes)
                cv2.putText(image, f"{len(timings) / (after - start):.1f} FPS", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imshow("YOLO26n CPU, q to quit", image)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if after - report >= 5:
                print(f"{model}, {threads} threads: {len(timings) / (after - start):.1f} live FPS", flush=True)
                report = after
    except KeyboardInterrupt:
        pass
    elapsed = time.perf_counter() - start
    if not timings:
        return None
    result = {
        "model": str(model), "threads": threads, "frames": len(timings),
        "seconds": round(elapsed, 3), "live_fps": round(len(timings) / elapsed, 2),
        "processing_ms_mean": round(float(np.mean(timings)), 2),
        "processing_ms_p95": round(float(np.percentile(timings, 95)), 2),
        "sensor_to_result_ms_mean": round(float(np.mean(ages)), 2),
        "sensor_to_result_ms_p95": round(float(np.percentile(ages, 95)), 2),
        "temperature_start_c": start_temp, "temperature_end_c": temperature(),
        "total_detections": detections,
        "last_detections": [
            {"class": detector.names[int(box[5])], "confidence": round(float(box[4]), 3),
             "xyxy": [round(float(v), 1) for v in box[:4]]}
            for box in last_boxes
        ],
    }
    print(json.dumps(result), flush=True)
    if args.output:
        output = Path(args.output)
        output.mkdir(parents=True, exist_ok=True)
        label = f"{Path(model).name}_{threads}t"
        (output / f"{label}.json").write_text(json.dumps(result, indent=2) + "\n")
        cv2.imwrite(str(output / f"{label}.jpg"), detector.draw(last_frame, last_boxes))
    return result


def main():
    parser = argparse.ArgumentParser(description="YOLO26 NCNN on a normal Pi camera, CPU only")
    parser.add_argument("--model", nargs="+", default=["models/yolo26n_320_ncnn_model"])
    parser.add_argument("--threads", nargs="+", type=int, choices=[1, 2, 3, 4], default=[2])
    parser.add_argument("--seconds", type=float, default=30, help="Duration per combination; 0 runs until Ctrl-C")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--view", action="store_true")
    parser.add_argument("--output", help="Save statistics and one annotated camera frame per combination")
    args = parser.parse_args()
    if args.seconds < 0:
        parser.error("--seconds must be nonnegative")
    cv2.setNumThreads(1)
    camera = Camera(AI_CAMERA)
    try:
        camera.start()
        camera.set_auto()
        for model in args.model:
            for threads in args.threads:
                run(camera, args, model, threads)
    finally:
        camera.stop()
        if args.view:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
