#!/usr/bin/env python3
"""Replay XR command packets from JSONL into the teleop bridge UDP port.

Expected JSONL fields per line:
  timestamp_ns, sequence_id, teleop_enabled, clutch_pressed,
  target_position_xyz[3], target_orientation_xyzw[4], gripper_command
"""

import argparse
import json
import socket
import struct
import time

PACKET_FMT = "<QQBB3d4df3x"


def build_packet(msg: dict) -> bytes:
    return struct.pack(
        PACKET_FMT,
        int(msg["timestamp_ns"]),
        int(msg["sequence_id"]),
        1 if msg["teleop_enabled"] else 0,
        1 if msg["clutch_pressed"] else 0,
        float(msg["target_position_xyz"][0]),
        float(msg["target_position_xyz"][1]),
        float(msg["target_position_xyz"][2]),
        float(msg["target_orientation_xyzw"][0]),
        float(msg["target_orientation_xyzw"][1]),
        float(msg["target_orientation_xyzw"][2]),
        float(msg["target_orientation_xyzw"][3]),
        float(msg["gripper_command"]),
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("jsonl")
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=28080)
    parser.add_argument("--hz", type=float, default=90.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dt = 1.0 / max(args.hz, 1.0)

    seq = 0
    with open(args.jsonl, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            msg = json.loads(line)
            msg["sequence_id"] = seq
            msg["timestamp_ns"] = time.time_ns()
            sock.sendto(build_packet(msg), (args.ip, args.port))
            seq += 1
            time.sleep(dt)


if __name__ == "__main__":
    main()
