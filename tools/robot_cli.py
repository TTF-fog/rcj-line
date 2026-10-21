#!/usr/bin/env python3
"""Type commands to the robot and see its replies.

    python tools/robot_cli.py                 # /dev/ttyACM0, hides IMU/STAT stream lines
    python tools/robot_cli.py --stream        # show the stream too
    python tools/robot_cli.py --port /dev/ttyACM1
"""

import argparse
import threading

import serial

STREAM_PREFIXES = ("IMU ", "STAT ")


def print_replies(port, show_stream):
    for raw in port:
        line = raw.decode(errors="replace").rstrip()
        if show_stream or not line.startswith(STREAM_PREFIXES):
            print(line)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--stream", action="store_true", help="show IMU and STAT lines")
    args = parser.parse_args()

    port = serial.Serial(args.port, 115200, exclusive=True)
    threading.Thread(target=print_replies, args=(port, args.stream), daemon=True).start()
    print(f"connected to {args.port}, type HELP, Ctrl-D to quit")
    try:
        while True:
            port.write((input() + "\n").encode())
    except (EOFError, KeyboardInterrupt):
        port.write(b"STOP\n")


if __name__ == "__main__":
    main()
