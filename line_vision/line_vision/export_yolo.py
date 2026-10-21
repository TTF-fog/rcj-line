import argparse
from pathlib import Path

import torch
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="Export YOLO26 weights to CPU NCNN")
    parser.add_argument("--weights", default="models/yolo26n.pt")
    parser.add_argument("--sizes", nargs="+", type=int, default=[320, 416, 640])
    args = parser.parse_args()
    torch.set_num_threads(4)
    weights = Path(args.weights)
    weights.parent.mkdir(parents=True, exist_ok=True)
    for size in args.sizes:
        destination = weights.parent / f"{weights.stem}_{size}_ncnn_model"
        if destination.exists():
            print(f"Already exists: {destination}")
            continue
        exported = Path(YOLO(str(weights)).export(format="ncnn", imgsz=size, device="cpu", quantize=16))
        exported.rename(destination)
        print(f"Saved {destination}")


if __name__ == "__main__":
    main()
