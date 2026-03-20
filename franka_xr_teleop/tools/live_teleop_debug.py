#!/usr/bin/env python3
"""Live UDP observation monitor for franka_xr_teleop."""

import argparse
import json
import math
import socket
import time
from typing import Any, Dict, List


def vec_norm(v: List[float]) -> float:
    return math.sqrt(sum(x * x for x in v))


def format_faults(fault_flags: Dict[str, Any]) -> str:
    active = [k for k, v in fault_flags.items() if bool(v)]
    if not active:
        return "none"
    return ",".join(active)


def safe_get(d: Dict[str, Any], *keys: str, default: Any = None) -> Any:
    cur: Any = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Listen to franka_xr_teleop UDP observations and print live debug stats. "
            "Run this while franka_xr_teleop_bridge is active."
        )
    )
    p.add_argument("--bind-ip", default="0.0.0.0", help="IP to bind for UDP receive (default: 0.0.0.0)")
    p.add_argument("--port", type=int, default=28081, help="UDP port to listen on (default: 28081)")
    p.add_argument(
        "--print-hz",
        type=float,
        default=10.0,
        help="How often to print summaries (default: 10 Hz)",
    )
    p.add_argument(
        "--timeout-s",
        type=float,
        default=0.5,
        help="Socket timeout in seconds (default: 0.5)",
    )
    p.add_argument(
        "--raw",
        action="store_true",
        help="Print full decoded JSON payload each packet instead of compact summary",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    print_period_s = 1.0 / max(args.print_hz, 0.1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(max(args.timeout_s, 0.01))
    sock.bind((args.bind_ip, args.port))

    print(
        f"Listening on udp://{args.bind_ip}:{args.port} "
        f"(print_hz={args.print_hz:.2f}, raw={args.raw})"
    )

    last_print_t = 0.0
    last_rate_t = time.monotonic()
    packets_since_rate = 0
    rx_hz = 0.0
    last_payload_t = 0.0

    while True:
        try:
            payload, addr = sock.recvfrom(65535)
            now = time.monotonic()
            packets_since_rate += 1
            last_payload_t = now

            elapsed = now - last_rate_t
            if elapsed >= 1.0:
                rx_hz = packets_since_rate / elapsed
                packets_since_rate = 0
                last_rate_t = now

            try:
                obs = json.loads(payload.decode("utf-8", errors="replace"))
            except json.JSONDecodeError as e:
                if now - last_print_t >= print_period_s:
                    print(f"[json_error] from {addr[0]}:{addr[1]}: {e}")
                    last_print_t = now
                continue

            if args.raw:
                print(json.dumps(obs, separators=(",", ":"), sort_keys=True))
                continue

            if now - last_print_t < print_period_s:
                continue

            status = safe_get(obs, "status", default={}) or {}
            robot = safe_get(obs, "robot_state", default={}) or {}
            action = safe_get(obs, "executed_action", default={}) or {}
            faults = safe_get(status, "fault_flags", default={}) or {}

            teleop_state = safe_get(status, "teleop_state", default="N/A")
            control_mode = safe_get(status, "control_mode", default="N/A")
            teleop_active = bool(safe_get(status, "teleop_active", default=False))
            target_fresh = bool(safe_get(status, "target_fresh", default=False))
            packet_age_ms = float(safe_get(status, "packet_age_ns", default=0)) * 1e-6
            target_age_ms = float(safe_get(status, "target_age_ns", default=0)) * 1e-6
            manip = float(safe_get(status, "target_manipulability", default=0.0))

            tcp = safe_get(robot, "tcp_position_xyz", default=[0.0, 0.0, 0.0])
            dq = safe_get(robot, "dq", default=[0.0] * 7)
            gripper_width = float(safe_get(robot, "gripper_width", default=0.0))
            gripper_state = safe_get(robot, "gripper_state", default="N/A")

            dpos = safe_get(action, "cartesian_delta_translation", default=[0.0, 0.0, 0.0])
            drot = safe_get(action, "cartesian_delta_rotation", default=[0.0, 0.0, 0.0])
            grip_cmd = float(safe_get(action, "gripper_command", default=0.0))

            dpos_n = vec_norm(dpos)
            drot_n = vec_norm(drot)
            dq_max = max(abs(float(x)) for x in dq) if dq else 0.0
            stale_ms = (now - last_payload_t) * 1e3

            print(
                " | ".join(
                    [
                        f"rx={rx_hz:6.1f}Hz",
                        f"src={addr[0]}:{addr[1]}",
                        f"state={teleop_state}",
                        f"mode={control_mode}",
                        f"active={int(teleop_active)}",
                        f"fresh={int(target_fresh)}",
                        f"pkt_age={packet_age_ms:6.1f}ms",
                        f"tgt_age={target_age_ms:6.1f}ms",
                        f"tcp=({float(tcp[0]):+.3f},{float(tcp[1]):+.3f},{float(tcp[2]):+.3f})",
                        f"|dpos|={dpos_n:.4f}m",
                        f"|drot|={drot_n:.4f}rad",
                        f"dq_max={dq_max:.4f}",
                        f"grip_cmd={grip_cmd:.3f}",
                        f"grip_state={gripper_state}",
                        f"grip_w={gripper_width:.3f}m",
                        f"manip={manip:.5f}",
                        f"faults={format_faults(faults)}",
                        f"stale={stale_ms:.1f}ms",
                    ]
                )
            )
            last_print_t = now

        except socket.timeout:
            now = time.monotonic()
            if now - last_print_t >= print_period_s:
                if last_payload_t <= 0.0:
                    print("waiting_for_packets=true")
                else:
                    silent_ms = (now - last_payload_t) * 1e3
                    print(f"waiting_for_packets=true last_packet_ago_ms={silent_ms:.1f}")
                last_print_t = now
        except KeyboardInterrupt:
            print("\nStopped.")
            return 0


if __name__ == "__main__":
    raise SystemExit(main())
