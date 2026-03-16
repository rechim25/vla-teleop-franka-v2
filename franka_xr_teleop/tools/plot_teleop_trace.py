#!/usr/bin/env python3
"""Plot planner + RT trace CSVs produced by franka_xr_teleop_bridge."""

import argparse
import pathlib

import matplotlib.pyplot as plt
import numpy as np


def load_csv(path: pathlib.Path):
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    if data.size == 0:
        raise RuntimeError(f"No rows in {path}")
    if data.shape == ():
        data = np.array([data], dtype=data.dtype)
    return data


def get_col(data, name):
    if name not in data.dtype.names:
        raise KeyError(f"Missing column '{name}'")
    return data[name]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trace-dir",
        type=pathlib.Path,
        required=True,
        help="Directory containing planner_trace.csv and rt_trace.csv",
    )
    parser.add_argument(
        "--joint",
        type=int,
        default=None,
        help="Joint index [0..6] for single-joint mode (default plots all joints)",
    )
    parser.add_argument(
        "--all-joints",
        action="store_true",
        help="Force plotting all 7 joints (default when --joint is not set)",
    )
    parser.add_argument(
        "--out",
        type=pathlib.Path,
        default=None,
        help="Output image path (default: <trace-dir>/trace_summary.png)",
    )
    parser.add_argument("--show", action="store_true", help="Show interactive window")
    args = parser.parse_args()

    if args.joint is not None and (args.joint < 0 or args.joint > 6):
        raise ValueError("--joint must be in [0, 6]")

    planner_path = args.trace_dir / "planner_trace.csv"
    rt_path = args.trace_dir / "rt_trace.csv"
    planner = load_csv(planner_path)
    rt = load_csv(rt_path)

    t0_ns = min(int(planner["timestamp_ns"][0]), int(rt["timestamp_ns"][0]))
    tp = (planner["timestamp_ns"].astype(np.float64) - float(t0_ns)) * 1e-9
    tr = (rt["timestamp_ns"].astype(np.float64) - float(t0_ns)) * 1e-9

    all_joints_mode = args.all_joints or args.joint is None

    fig, axes = plt.subplots(6, 1, figsize=(14, 14), sharex=True)

    axes[0].plot(tr, get_col(rt, "teleop_state"), label="teleop_state")
    axes[0].plot(tr, get_col(rt, "apply_motion"), label="apply_motion")
    axes[0].plot(tr, get_col(rt, "fault_jump_rejected"), label="jump_rejected")
    axes[0].set_ylabel("State/Fault")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(tp, get_col(planner, "xr_pos_0"), label="vr_x")
    axes[1].plot(tp, get_col(planner, "xr_pos_1"), label="vr_y")
    axes[1].plot(tp, get_col(planner, "xr_pos_2"), label="vr_z")
    axes[1].set_ylabel("VR XYZ [m]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(tp, get_col(planner, "robot_pos_2"), label="robot_z")
    axes[2].plot(tp, get_col(planner, "desired_pos_2"), label="desired_z")
    axes[2].plot(tp, get_col(planner, "safe_pos_2"), label="safe_z")
    axes[2].set_ylabel("Cartesian Z [m]")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)

    if all_joints_mode:
        cmap = plt.get_cmap("tab10")
        for joint in range(7):
            color = cmap(joint % 10)
            axes[3].plot(tr, get_col(rt, f"q_cmd_{joint}"), color=color, label=f"q_cmd_{joint}")
        axes[3].set_ylabel("All q_cmd [rad]")
        axes[3].legend(loc="upper right", ncol=4, fontsize=8)
        axes[3].grid(True, alpha=0.3)

        for joint in range(7):
            color = cmap(joint % 10)
            axes[4].plot(
                tr,
                get_col(rt, f"command_delta_{joint}"),
                color=color,
                label=f"cmd_dq_{joint}",
            )
        axes[4].set_ylabel("All cmd_delta [rad]")
        axes[4].legend(loc="upper right", ncol=4, fontsize=8)
        axes[4].grid(True, alpha=0.3)
    else:
        joint = args.joint
        axes[3].plot(tr, get_col(rt, f"q_d_{joint}"), label=f"q_d_{joint}")
        axes[3].plot(tr, get_col(rt, f"q_planned_{joint}"), label=f"q_planned_{joint}")
        axes[3].plot(tr, get_col(rt, f"q_cmd_{joint}"), label=f"q_cmd_{joint}")
        axes[3].set_ylabel(f"Joint {joint} [rad]")
        axes[3].legend(loc="upper right")
        axes[3].grid(True, alpha=0.3)

        axes[4].plot(tr, get_col(rt, f"target_delta_{joint}"), label=f"target_delta_{joint}")
        axes[4].plot(tr, get_col(rt, f"filtered_delta_{joint}"), label=f"filtered_delta_{joint}")
        axes[4].plot(tr, get_col(rt, f"command_delta_{joint}"), label=f"command_delta_{joint}")
        axes[4].plot(
            tr,
            get_col(rt, f"clamp_saturated_{joint}"),
            label=f"clamp_sat_{joint}",
            linewidth=1.0,
        )
        axes[4].set_ylabel(f"Delta J{joint} [rad]")
        axes[4].legend(loc="upper right")
        axes[4].grid(True, alpha=0.3)

    axes[5].plot(tp, get_col(planner, "packet_age_ns") * 1e-6, label="planner_packet_age_ms")
    axes[5].plot(tr, get_col(rt, "target_age_ns") * 1e-6, label="rt_target_age_ms")
    axes[5].plot(tr, get_col(rt, "loop_dt_ns") * 1e-6, label="rt_loop_dt_ms")
    axes[5].set_ylabel("Age / dt [ms]")
    axes[5].set_xlabel("Time [s]")
    axes[5].legend(loc="upper right")
    axes[5].grid(True, alpha=0.3)

    title_suffix = " (all joints + VR XYZ)" if all_joints_mode else f" (joint {args.joint} + VR XYZ)"
    fig.suptitle(f"Teleop Trace Summary: {args.trace_dir}{title_suffix}")
    fig.tight_layout()

    output_path = args.out or (args.trace_dir / "trace_summary.png")
    fig.savefig(output_path, dpi=150)
    print(f"Saved plot: {output_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
