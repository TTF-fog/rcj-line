from pathlib import Path

import cv2
import ncnn
import numpy as np
import yaml

from line_vision.config import CameraConfig

# The AI camera is mounted upside down.
AI_CAMERA = CameraConfig(model="imx500", width=640, height=480, fps=30, hflip=True, vflip=True)


class Detector:
    def __init__(self, model, threads):
        model = Path(model)
        metadata = yaml.safe_load((model / "metadata.yaml").read_text())
        self.names = metadata["names"]
        self.height, self.width = metadata["imgsz"]
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False
        self.net.opt.num_threads = threads
        self.net.opt.openmp_blocktime = 0
        if self.net.load_param(str(model / "model.ncnn.param")) != 0:
            raise RuntimeError("Cannot load NCNN parameters")
        if self.net.load_model(str(model / "model.ncnn.bin")) != 0:
            raise RuntimeError("Cannot load NCNN weights")

    def predict(self, frame, confidence):
        """Rows of (x1, y1, x2, y2, score, class) in frame pixels."""
        height, width = frame.shape[:2]
        scale = min(self.width / width, self.height / height)
        resized = cv2.resize(frame, (round(width * scale), round(height * scale)))
        top = (self.height - resized.shape[0]) // 2
        left = (self.width - resized.shape[1]) // 2
        image = cv2.copyMakeBorder(
            resized, top, self.height - resized.shape[0] - top,
            left, self.width - resized.shape[1] - left,
            cv2.BORDER_CONSTANT, value=(114, 114, 114),
        )
        mat = ncnn.Mat.from_pixels(image, ncnn.Mat.PixelType.PIXEL_BGR2RGB, self.width, self.height)
        mat.substract_mean_normalize([], [1 / 255.0] * 3)
        with self.net.create_extractor() as extractor:
            extractor.input(self.net.input_names()[0], mat)
            status, output = extractor.extract(self.net.output_names()[0])
            if status != 0:
                raise RuntimeError(f"NCNN inference failed: {status}")
            boxes = np.array(output)
        if boxes.ndim != 2 or boxes.shape[0] != 4 + len(self.names):
            raise RuntimeError(f"Expected YOLO detection output (4 + classes, N), got {boxes.shape}")
        predictions = boxes.T
        scores = predictions[:, 4:].max(axis=1)
        keep = scores >= confidence
        predictions, scores = predictions[keep], scores[keep]
        categories = predictions[:, 4:].argmax(axis=1)
        xywh = predictions[:, :4].copy()
        xywh[:, :2] -= xywh[:, 2:] / 2
        selected = cv2.dnn.NMSBoxesBatched(xywh.tolist(), scores.tolist(), categories.tolist(), confidence, 0.45)
        selected = np.asarray(selected, dtype=int).reshape(-1)
        xywh = xywh[selected]
        boxes = np.column_stack((xywh[:, :2], xywh[:, :2] + xywh[:, 2:], scores[selected], categories[selected]))
        boxes[:, [0, 2]] = np.clip((boxes[:, [0, 2]] - left) / scale, 0, width)
        boxes[:, [1, 3]] = np.clip((boxes[:, [1, 3]] - top) / scale, 0, height)
        return boxes

    def draw(self, frame, boxes):
        frame = frame.copy()
        for x1, y1, x2, y2, score, category in boxes:
            category = int(category)
            name = self.names[category]
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"{name} {score:.2f}", (int(x1), max(20, int(y1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return frame


def floor_balls(detector, boxes, frame_height, min_bottom):
    """[(class_name, centre_x, bottom_y, box)] for boxes that sit on the floor, closest first."""
    out = [(detector.names[int(b[5])], (b[0] + b[2]) / 2, b[3], b) for b in boxes if b[3] >= min_bottom * frame_height]
    return sorted(out, key=lambda ball: -ball[2])
