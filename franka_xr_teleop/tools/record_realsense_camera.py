#!/usr/bin/env python3
"""Record an Intel RealSense camera with frame timestamps for dataset sync."""

import argparse
import json
from pathlib import Path
import signal
import sys
import time
from typing import Any, Dict, Optional, Tuple

STOP_REQUESTED = False


def handle_signal(_signum: int, _frame: object) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


def default_output_dir() -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path("recordings") / f"realsense_{stamp}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record an Intel RealSense camera to rgb.mp4 plus frames.jsonl "
            "timestamps. Optional depth and .bag backup are supported."
        )
    )
    parser.add_argument("--output-dir", type=Path, default=default_output_dir())
    parser.add_argument("--camera-name", default="third_person_d405")
    parser.add_argument("--serial", default="", help="RealSense serial number; empty uses first camera.")
    parser.add_argument("--list-devices", action="store_true", help="List connected RealSense devices and exit.")
    parser.add_argument("--color-width", type=int, default=1280)
    parser.add_argument("--color-height", type=int, default=720)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--duration-s", type=float, default=0.0, help="Stop after seconds; 0 records until Ctrl-C.")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after frames; 0 records until Ctrl-C.")
    parser.add_argument(
        "--depth",
        action="store_true",
        help="Also save per-frame depth as compressed NPZ files containing z16 depth.",
    )
    parser.add_argument(
        "--no-align-depth",
        action="store_true",
        help="Do not align depth to the color frame when --depth is used.",
    )
    parser.add_argument(
        "--bag",
        action="store_true",
        help="Also record a RealSense .bag file for SDK playback/re-extraction.",
    )
    parser.add_argument(
        "--video-codec",
        default="mp4v",
        help="OpenCV fourcc for rgb.mp4 (default: mp4v; try avc1 if your OpenCV supports it).",
    )
    parser.add_argument("--print-hz", type=float, default=1.0, help="Progress print rate; 0 disables.")
    return parser.parse_args()


def import_dependencies() -> Tuple[Any, Any, Any]:
    try:
        import cv2
    except ImportError:
        print("Missing dependency: opencv-python / cv2", file=sys.stderr)
        raise
    try:
        import numpy as np
    except ImportError:
        print("Missing dependency: numpy", file=sys.stderr)
        raise
    try:
        import pyrealsense2 as rs
    except ImportError:
        print(
            "Missing dependency: pyrealsense2. Install the RealSense Python "
            "bindings for your librealsense version.",
            file=sys.stderr,
        )
        raise
    return cv2, np, rs


def list_devices(rs: Any) -> int:
    ctx = rs.context()
    devices = ctx.query_devices()
    print(f"device_count={len(devices)}")
    for index, dev in enumerate(devices):
        serial = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)
        firmware = dev.get_info(rs.camera_info.firmware_version)
        usb_type = dev.get_info(rs.camera_info.usb_type_descriptor)
        print(f"{index}: name={name} serial={serial} firmware={firmware} usb={usb_type}")
    return 0


def frame_timestamp_domain(rs: Any, frame: Any) -> str:
    try:
        domain = frame.get_frame_timestamp_domain()
    except RuntimeError:
        return "unknown"
    mapping = {
        rs.timestamp_domain.hardware_clock: "hardware_clock",
        rs.timestamp_domain.system_time: "system_time",
        rs.timestamp_domain.global_time: "global_time",
    }
    return mapping.get(domain, str(domain))


def intrinsics_to_dict(intrinsics: Any) -> Dict[str, Any]:
    return {
        "width": intrinsics.width,
        "height": intrinsics.height,
        "ppx": intrinsics.ppx,
        "ppy": intrinsics.ppy,
        "fx": intrinsics.fx,
        "fy": intrinsics.fy,
        "model": str(intrinsics.model),
        "coeffs": list(intrinsics.coeffs),
    }


def profile_metadata(rs: Any, profile: Any, depth_scale_m: Optional[float]) -> Dict[str, Any]:
    dev = profile.get_device()
    metadata: Dict[str, Any] = {
        "serial_number": dev.get_info(rs.camera_info.serial_number),
        "camera_name": dev.get_info(rs.camera_info.name),
        "firmware_version": dev.get_info(rs.camera_info.firmware_version),
        "usb_type_descriptor": dev.get_info(rs.camera_info.usb_type_descriptor),
        "depth_scale_m": depth_scale_m,
    }

    streams: Dict[str, Any] = {}
    for stream_profile in profile.get_streams():
        video_profile = stream_profile.as_video_stream_profile()
        stream_name = str(stream_profile.stream_type()).split(".")[-1]
        streams[stream_name] = {
            "format": str(stream_profile.format()),
            "fps": stream_profile.fps(),
            "intrinsics": intrinsics_to_dict(video_profile.get_intrinsics()),
        }
    metadata["streams"] = streams
    return metadata


def configure_pipeline(args: argparse.Namespace, rs: Any, bag_path: Path) -> Tuple[Any, Any]:
    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    if args.bag:
        config.enable_record_to_file(str(bag_path))
    config.enable_stream(
        rs.stream.color,
        args.color_width,
        args.color_height,
        rs.format.bgr8,
        args.fps,
    )
    if args.depth:
        config.enable_stream(
            rs.stream.depth,
            args.depth_width,
            args.depth_height,
            rs.format.z16,
            args.fps,
        )
    return pipeline, config


def should_stop(start_mono: float, duration_s: float, frame_index: int, max_frames: int) -> bool:
    if STOP_REQUESTED:
        return True
    if duration_s > 0.0 and (time.monotonic() - start_mono) >= duration_s:
        return True
    if max_frames > 0 and frame_index >= max_frames:
        return True
    return False


def main() -> int:
    args = parse_args()
    if args.fps <= 0:
        print("--fps must be > 0", file=sys.stderr)
        return 2
    if args.duration_s < 0.0 or args.max_frames < 0:
        print("--duration-s and --max-frames must be >= 0", file=sys.stderr)
        return 2

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    cv2, np, rs = import_dependencies()

    if args.list_devices:
        return list_devices(rs)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    rgb_path = args.output_dir / "rgb.mp4"
    frames_path = args.output_dir / "frames.jsonl"
    metadata_path = args.output_dir / "metadata.json"
    bag_path = args.output_dir / "raw.bag"
    depth_dir = args.output_dir / "depth"
    if args.depth:
        depth_dir.mkdir(parents=True, exist_ok=True)

    pipeline, config = configure_pipeline(args, rs, bag_path)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color) if args.depth and not args.no_align_depth else None

    depth_scale_m: Optional[float] = None
    if args.depth:
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale_m = float(depth_sensor.get_depth_scale())

    writer = cv2.VideoWriter(
        str(rgb_path),
        cv2.VideoWriter_fourcc(*args.video_codec[:4]),
        float(args.fps),
        (args.color_width, args.color_height),
    )
    if not writer.isOpened():
        pipeline.stop()
        raise RuntimeError(f"Failed to open video writer for {rgb_path}")

    metadata = {
        "camera_name": args.camera_name,
        "created_unix_time_ns": time.time_ns(),
        "host_clock": "time.monotonic_ns",
        "rgb_video": "rgb.mp4",
        "frames": "frames.jsonl",
        "bag": "raw.bag" if args.bag else None,
        "depth_dir": "depth" if args.depth else None,
        "depth_aligned_to_color": bool(align is not None),
        "requested_fps": args.fps,
        "profile": profile_metadata(rs, profile, depth_scale_m),
        "notes": {
            "T_world_camera": "Fill this with third-person camera extrinsics if calibrated.",
            "timestamp_sync": "host_timestamp_ns uses the same host monotonic clock as robot timestamp_ns.",
        },
    }
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(
        f"Recording RealSense {args.camera_name}: rgb={rgb_path} frames={frames_path} "
        f"bag={int(args.bag)} depth={int(args.depth)}",
        flush=True,
    )

    frame_index = 0
    start_mono = time.monotonic()
    last_print = start_mono
    progress_period_s = 1.0 / args.print_hz if args.print_hz > 0.0 else 0.0

    try:
        with frames_path.open("w", encoding="utf-8", buffering=1) as frames_file:
            while not should_stop(start_mono, args.duration_s, frame_index, args.max_frames):
                frames = pipeline.wait_for_frames()
                host_timestamp_ns = time.monotonic_ns()
                if align is not None:
                    frames = align.process(frames)

                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                color = np.asanyarray(color_frame.get_data())
                if color.shape[1] != args.color_width or color.shape[0] != args.color_height:
                    color = cv2.resize(
                        color,
                        (args.color_width, args.color_height),
                        interpolation=cv2.INTER_AREA,
                    )
                writer.write(color)

                depth_file: Optional[str] = None
                if args.depth:
                    depth_frame = frames.get_depth_frame()
                    if depth_frame:
                        depth_z16 = np.asanyarray(depth_frame.get_data()).copy()
                        depth_file = f"depth_{frame_index:06d}.npz"
                        np.savez_compressed(
                            depth_dir / depth_file,
                            depth_z16=depth_z16,
                            depth_scale_m=depth_scale_m,
                        )

                record = {
                    "camera": args.camera_name,
                    "frame_index": frame_index,
                    "host_timestamp_ns": host_timestamp_ns,
                    "realsense_timestamp_ms": color_frame.get_timestamp(),
                    "realsense_timestamp_domain": frame_timestamp_domain(rs, color_frame),
                    "rgb_video": "rgb.mp4",
                    "rgb_video_frame": frame_index,
                    "depth_file": depth_file,
                    "width": args.color_width,
                    "height": args.color_height,
                }
                frames_file.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")
                frame_index += 1

                now = time.monotonic()
                if progress_period_s > 0.0 and (now - last_print) >= progress_period_s:
                    elapsed = max(now - start_mono, 1e-9)
                    print(f"frames={frame_index} rate={frame_index / elapsed:.1f}Hz output={args.output_dir}")
                    last_print = now
    finally:
        writer.release()
        pipeline.stop()

    print(f"Stopped. frames={frame_index} output={args.output_dir}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
