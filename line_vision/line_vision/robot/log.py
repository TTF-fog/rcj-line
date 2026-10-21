import time
from collections import deque
from pathlib import Path

_file = None
recent = deque(maxlen=300)


def open_file(directory):
    global _file
    Path(directory).mkdir(parents=True, exist_ok=True)
    path = Path(directory) / time.strftime("%Y%m%d_%H%M%S.log")
    _file = open(path, "a", buffering=1)
    return path


def log(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    print(line, flush=True)
    recent.append(line)
    if _file:
        _file.write(line + "\n")
