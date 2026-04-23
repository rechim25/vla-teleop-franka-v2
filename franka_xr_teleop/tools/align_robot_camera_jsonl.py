#!/usr/bin/env python3
"""Align robot observation JSONL with camera frame timestamp JSONL."""

import argparse
import bisect
import json
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create aligned_samples.jsonl by nearest-neighbor matching robot "
            "observation timestamps to camera frame timestamps."
        )
    )
    parser.add_argument("--robot-jsonl", type=Path, required=True)
    parser.add_argument(
        "--camera",
        action="append",
        nargs=2,
        metavar=("NAME", "FRAMES_JSONL"),
        required=True,
        help="Camera name and frames.jsonl path. Repeat for multiple cameras.",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--max-dt-ms",
        type=float,
        default=35.0,
        help="Reject camera matches farther than this from robot time (default: 35 ms).",
    )
    parser.add_argument(
        "--robot-time-key",
        default="timestamp_ns",
        help="Robot observation timestamp key (default: timestamp_ns).",
    )
    parser.add_argument(
        "--camera-time-key",
        default="host_timestamp_ns",
        help="Camera frame timestamp key (default: host_timestamp_ns).",
    )
    parser.add_argument(
        "--allow-missing",
        action="store_true",
        help="Write samples even when one or more cameras miss the max-dt threshold.",
    )
    return parser.parse_args()


def iter_jsonl(path: Path) -> Iterable[Tuple[int, Dict[str, Any]]]:
    with path.open("r", encoding="utf-8") as src:
        for line_no, line in enumerate(src, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_no}: invalid JSON: {exc}") from exc
            if not isinstance(value, dict):
                raise ValueError(f"{path}:{line_no}: top-level JSON value is not an object")
            yield line_no, value


def unwrap_robot_record(record: Dict[str, Any]) -> Dict[str, Any]:
    # record_robot_observations.py can optionally wrap observations with receive metadata.
    obs = record.get("observation")
    if isinstance(obs, dict):
        return obs
    return record


def timestamp_from(record: Dict[str, Any], key: str) -> Optional[int]:
    value = record.get(key)
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value)
    return None


def load_camera_frames(path: Path, time_key: str) -> Tuple[List[int], List[Dict[str, Any]]]:
    frames: List[Dict[str, Any]] = []
    timestamps: List[int] = []
    for line_no, frame in iter_jsonl(path):
        timestamp = timestamp_from(frame, time_key)
        if timestamp is None:
            raise ValueError(f"{path}:{line_no}: missing integer {time_key}")
        frames.append(frame)
        timestamps.append(timestamp)

    order = sorted(range(len(timestamps)), key=timestamps.__getitem__)
    return [timestamps[i] for i in order], [frames[i] for i in order]


def nearest_frame(
    timestamps: List[int],
    frames: List[Dict[str, Any]],
    target_ns: int,
) -> Tuple[Optional[Dict[str, Any]], Optional[int]]:
    if not timestamps:
        return None, None

    index = bisect.bisect_left(timestamps, target_ns)
    candidates: List[int] = []
    if index < len(timestamps):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    best = min(candidates, key=lambda i: abs(timestamps[i] - target_ns))
    return frames[best], timestamps[best] - target_ns


def main() -> int:
    args = parse_args()
    if args.max_dt_ms < 0.0:
        print("--max-dt-ms must be >= 0", file=sys.stderr)
        return 2

    max_dt_ns = int(args.max_dt_ms * 1e6)
    cameras: Dict[str, Tuple[List[int], List[Dict[str, Any]]]] = {}
    for name, frames_path in args.camera:
        cameras[name] = load_camera_frames(Path(frames_path), args.camera_time_key)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    total = 0
    written = 0
    rejected = 0

    with args.output.open("w", encoding="utf-8") as out:
        for line_no, raw_record in iter_jsonl(args.robot_jsonl):
            robot = unwrap_robot_record(raw_record)
            robot_ts = timestamp_from(robot, args.robot_time_key)
            if robot_ts is None:
                raise ValueError(f"{args.robot_jsonl}:{line_no}: missing integer {args.robot_time_key}")

            total += 1
            matched: Dict[str, Any] = {}
            missing = False
            for name, (timestamps, frames) in cameras.items():
                frame, dt_ns = nearest_frame(timestamps, frames, robot_ts)
                if frame is None or dt_ns is None or abs(dt_ns) > max_dt_ns:
                    missing = True
                    matched[name] = None
                    continue
                matched[name] = {
                    "dt_ns": dt_ns,
                    "frame": frame,
                }

            if missing and not args.allow_missing:
                rejected += 1
                continue

            sample = {
                "timestamp_ns": robot_ts,
                "robot": robot,
                "cameras": matched,
            }
            out.write(json.dumps(sample, separators=(",", ":"), sort_keys=True) + "\n")
            written += 1

    print(
        f"aligned={written} rejected={rejected} total_robot={total} output={args.output}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
