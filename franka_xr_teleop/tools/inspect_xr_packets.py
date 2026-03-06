#!/usr/bin/env python3
"""Simple UDP packet inspector for XR command stream."""

import socket
import struct
import time

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 28080
PACKET_FMT = "<QQBB3d4df3x"  # Matches teleop bridge XRWirePacket.
PACKET_SIZE = struct.calcsize(PACKET_FMT)


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print(f"Listening on udp://{LISTEN_IP}:{LISTEN_PORT} size={PACKET_SIZE}")

    while True:
        data, addr = sock.recvfrom(4096)
        if len(data) < PACKET_SIZE:
            print(f"short packet from {addr} len={len(data)}")
            continue
        values = struct.unpack(PACKET_FMT, data[:PACKET_SIZE])
        ts, seq, enabled, clutch = values[0], values[1], values[2], values[3]
        pos = values[4:7]
        quat = values[7:11]
        grip = values[11]
        age_ms = (time.time_ns() - ts) * 1e-6 if ts > 0 else -1.0
        print(
            f"seq={seq} age_ms={age_ms:.2f} deadman={enabled} clutch={clutch} "
            f"pos={tuple(round(x, 4) for x in pos)} quat={tuple(round(x, 4) for x in quat)} "
            f"grip={grip:.3f}"
        )


if __name__ == "__main__":
    main()
