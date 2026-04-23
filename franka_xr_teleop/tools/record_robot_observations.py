#!/usr/bin/env python3
"""Record franka_xr_teleop UDP robot observations to JSONL."""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import sys
import time
from typing import Any, Dict, Optional, Tuple


STOP_REQUESTED = False


def handle_signal(_signum: int, _frame: object) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


def default_output_path() -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path("recordings") / f"robot_observations_{stamp}.jsonl"


def default_episode_events_path(output_path: Path) -> Path:
    return output_path.parent / "episode_events.jsonl"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Receive UDP JSON robot observations from franka_xr_teleop_bridge "
            "and record them as newline-delimited JSON for datasets."
        )
    )
    parser.add_argument(
        "--bind-ip",
        default="0.0.0.0",
        help="Local IP address to bind for UDP receive (default: 0.0.0.0).",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=28081,
        help="UDP observation port to listen on (default: 28081).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_output_path(),
        help="Output JSONL path (default: recordings/robot_observations_<time>.jsonl).",
    )
    parser.add_argument(
        "--episode-events-output",
        type=Path,
        default=None,
        help=(
            "Episode event JSONL path. Defaults to episode_events.jsonl next to "
            "--output. Rows are written on rising edges of status.episode_start "
            "and status.episode_end."
        ),
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=0.0,
        help="Stop after this many seconds; 0 records until Ctrl-C (default: 0).",
    )
    parser.add_argument(
        "--max-packets",
        type=int,
        default=0,
        help="Stop after this many valid packets; 0 records until Ctrl-C (default: 0).",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=0.5,
        help="Socket timeout used for shutdown/progress checks (default: 0.5).",
    )
    parser.add_argument(
        "--print-hz",
        type=float,
        default=1.0,
        help="Progress print frequency; 0 disables progress (default: 1 Hz).",
    )
    parser.add_argument(
        "--flush-every",
        type=int,
        default=50,
        help="Flush output after this many valid packets (default: 50).",
    )
    parser.add_argument(
        "--fsync-every",
        type=int,
        default=0,
        help="Call fsync after this many valid packets; 0 disables fsync (default: 0).",
    )
    parser.add_argument(
        "--with-receive-metadata",
        action="store_true",
        help=(
            "Wrap each line as {receive:{...}, observation:{...}} with host receive "
            "timestamp and source address. By default, lines are raw observations."
        ),
    )
    parser.add_argument(
        "--keep-invalid",
        action="store_true",
        help="Also log malformed UDP payloads as invalid records.",
    )
    return parser.parse_args()


def make_socket(bind_ip: str, port: int, timeout_s: float) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(max(timeout_s, 0.01))
    sock.bind((bind_ip, port))
    return sock


def decode_payload(payload: bytes) -> Tuple[Optional[Dict[str, Any]], Optional[str], str]:
    text = payload.decode("utf-8", errors="replace")
    try:
        decoded = json.loads(text)
    except json.JSONDecodeError as exc:
        return None, str(exc), text
    if not isinstance(decoded, dict):
        return None, "top-level JSON value is not an object", text
    return decoded, None, text


def wrap_record(obs: Dict[str, Any], addr: Tuple[str, int], payload_len: int) -> Dict[str, Any]:
    return {
        "receive": {
            "host_time_ns": time.time_ns(),
            "source_ip": addr[0],
            "source_port": addr[1],
            "payload_bytes": payload_len,
        },
        "observation": obs,
    }


def safe_get(d: Dict[str, Any], *keys: str, default: Any = None) -> Any:
    cur: Any = d
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def make_episode_event(
    event_name: str, obs: Dict[str, Any], addr: Tuple[str, int], packet_index: int
) -> Dict[str, Any]:
    return {
        "event": event_name,
        "robot_timestamp_ns": safe_get(obs, "timestamp_ns"),
        "receive_host_time_ns": time.time_ns(),
        "packet_index": packet_index,
        "source_ip": addr[0],
        "source_port": addr[1],
        "teleop_state": safe_get(obs, "status", "teleop_state"),
        "control_mode": safe_get(obs, "status", "control_mode"),
    }


def print_progress(
    output: Path,
    count: int,
    invalid_count: int,
    bytes_written: int,
    start_mono: float,
    last_packet_mono: Optional[float],
) -> None:
    now = time.monotonic()
    elapsed = max(now - start_mono, 1e-9)
    rate_hz = count / elapsed
    if last_packet_mono is None:
        stale = "none"
    else:
        stale = f"{(now - last_packet_mono) * 1e3:.1f}ms"
    print(
        " | ".join(
            [
                f"recorded={count}",
                f"invalid={invalid_count}",
                f"rate={rate_hz:.1f}Hz",
                f"bytes={bytes_written}",
                f"last_packet_ago={stale}",
                f"output={output}",
            ]
        ),
        flush=True,
    )


def should_stop(start_mono: float, duration_s: float, count: int, max_packets: int) -> bool:
    if STOP_REQUESTED:
        return True
    if duration_s > 0.0 and (time.monotonic() - start_mono) >= duration_s:
        return True
    if max_packets > 0 and count >= max_packets:
        return True
    return False


def main() -> int:
    args = parse_args()
    if args.port <= 0 or args.port > 65535:
        print(f"Invalid UDP port: {args.port}", file=sys.stderr)
        return 2
    if args.duration_s < 0.0:
        print("--duration-s must be >= 0", file=sys.stderr)
        return 2
    if args.max_packets < 0:
        print("--max-packets must be >= 0", file=sys.stderr)
        return 2

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.episode_events_output is None:
        args.episode_events_output = default_episode_events_path(args.output)
    args.episode_events_output.parent.mkdir(parents=True, exist_ok=True)
    sock = make_socket(args.bind_ip, args.port, args.timeout_s)
    print(
        f"Recording udp://{args.bind_ip}:{args.port} -> {args.output} "
        f"(metadata={int(args.with_receive_metadata)}, "
        f"episode_events={args.episode_events_output})",
        flush=True,
    )

    count = 0
    invalid_count = 0
    bytes_written = 0
    start_mono = time.monotonic()
    last_progress_mono = start_mono
    last_packet_mono: Optional[float] = None
    last_episode_start_robot_timestamp: Optional[int] = None
    last_episode_end_robot_timestamp: Optional[int] = None
    last_episode_start = False
    last_episode_end = False
    progress_period_s = 1.0 / args.print_hz if args.print_hz > 0.0 else 0.0
    flush_every = max(args.flush_every, 1)

    with args.output.open("a", encoding="utf-8", buffering=1) as out, args.episode_events_output.open(
        "a", encoding="utf-8", buffering=1
    ) as episode_out:
        while not should_stop(start_mono, args.duration_s, count, args.max_packets):
            now = time.monotonic()
            if progress_period_s > 0.0 and (now - last_progress_mono) >= progress_period_s:
                print_progress(
                    args.output, count, invalid_count, bytes_written, start_mono, last_packet_mono
                )
                last_progress_mono = now

            try:
                payload, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue

            last_packet_mono = time.monotonic()
            obs, error, text = decode_payload(payload)
            if obs is None:
                invalid_count += 1
                if not args.keep_invalid:
                    continue
                record: Dict[str, Any] = {
                    "receive": {
                        "host_time_ns": time.time_ns(),
                        "source_ip": addr[0],
                        "source_port": addr[1],
                        "payload_bytes": len(payload),
                    },
                    "invalid": {
                        "error": error,
                        "payload": text,
                    },
                }
            elif args.with_receive_metadata:
                record = wrap_record(obs, addr, len(payload))
            else:
                record = obs

            line = json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n"
            out.write(line)
            bytes_written += len(line.encode("utf-8"))
            if obs is not None:
                robot_timestamp = safe_get(obs, "timestamp_ns")
                episode_start = bool(safe_get(obs, "status", "episode_start", default=False))
                episode_end = bool(safe_get(obs, "status", "episode_end", default=False))
                if (
                    episode_start
                    and not last_episode_start
                    and robot_timestamp != last_episode_start_robot_timestamp
                ):
                    episode = make_episode_event("episode_start", obs, addr, count)
                    episode_out.write(json.dumps(episode, separators=(",", ":"), sort_keys=True) + "\n")
                    if isinstance(robot_timestamp, int):
                        last_episode_start_robot_timestamp = robot_timestamp
                if (
                    episode_end
                    and not last_episode_end
                    and robot_timestamp != last_episode_end_robot_timestamp
                ):
                    episode = make_episode_event("episode_end", obs, addr, count)
                    episode_out.write(json.dumps(episode, separators=(",", ":"), sort_keys=True) + "\n")
                    if isinstance(robot_timestamp, int):
                        last_episode_end_robot_timestamp = robot_timestamp
                last_episode_start = episode_start
                last_episode_end = episode_end
                count += 1

            if count > 0 and count % flush_every == 0:
                out.flush()
                episode_out.flush()
            if args.fsync_every > 0 and count > 0 and count % args.fsync_every == 0:
                out.flush()
                episode_out.flush()
                os.fsync(out.fileno())
                os.fsync(episode_out.fileno())

    sock.close()
    print_progress(args.output, count, invalid_count, bytes_written, start_mono, last_packet_mono)
    print("Stopped.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
