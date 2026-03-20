#!/usr/bin/env python3
"""Plot planner + RT trace CSVs produced by franka_xr_teleop_bridge.

Default output:
- trace_overview.png: timing, teleop state, XR/cartesian context
- trace_joints.png: one row per joint with positions and error/delta signals

Single-joint mode:
- --joint N produces trace_joint_N.png with a larger detailed view
"""

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
    return np.asarray(data[name], dtype=np.float64)


def get_col_or_default(data, name, default):
    if name in data.dtype.names:
        return np.asarray(data[name], dtype=np.float64)
    return np.asarray(default, dtype=np.float64)


def add_legend(ax, ncol=1):
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(loc="upper right", ncol=ncol, fontsize=8)


def style_axis(ax, ylabel):
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)


def make_overview_figure(trace_dir, planner, rt, tp, tr):
    fig, axes = plt.subplots(5, 1, figsize=(16, 14), sharex=True)

    axes[0].plot(tr, get_col(rt, "teleop_state"), label="teleop_state")
    axes[0].plot(tr, get_col(rt, "apply_motion"), label="apply_motion")
    axes[0].plot(tr, get_col(rt, "target_fresh"), label="target_fresh")
    axes[0].plot(tr, get_col(rt, "fault_ik_rejected"), label="fault_ik_rejected")
    style_axis(axes[0], "State")
    add_legend(axes[0], ncol=4)

    axes[1].plot(tp, get_col(planner, "xr_pos_0"), label="xr_x")
    axes[1].plot(tp, get_col(planner, "xr_pos_1"), label="xr_y")
    axes[1].plot(tp, get_col(planner, "xr_pos_2"), label="xr_z")
    style_axis(axes[1], "XR Pos [m]")
    add_legend(axes[1], ncol=3)

    axes[2].plot(tp, get_col(planner, "robot_pos_0"), label="robot_x")
    axes[2].plot(tp, get_col(planner, "robot_pos_1"), label="robot_y")
    axes[2].plot(tp, get_col(planner, "robot_pos_2"), label="robot_z")
    axes[2].plot(tp, get_col(planner, "desired_pos_0"), "--", label="desired_x")
    axes[2].plot(tp, get_col(planner, "desired_pos_1"), "--", label="desired_y")
    axes[2].plot(tp, get_col(planner, "desired_pos_2"), "--", label="desired_z")
    style_axis(axes[2], "Cartesian [m]")
    add_legend(axes[2], ncol=3)

    xr_dt_ms = np.zeros_like(tp)
    if tp.size > 1:
        xr_dt_ms[1:] = np.diff(get_col(planner, "xr_timestamp_ns")) * 1e-6
    axes[3].plot(tp, get_col(planner, "packet_age_ns") * 1e-6, label="planner_packet_age_ms")
    axes[3].plot(tr, get_col(rt, "target_age_ns") * 1e-6, label="rt_target_age_ms")
    axes[3].plot(tr, get_col(rt, "loop_dt_ns") * 1e-6, label="rt_loop_dt_ms")
    axes[3].plot(tp, xr_dt_ms, label="xr_interarrival_ms")
    style_axis(axes[3], "Timing [ms]")
    add_legend(axes[3], ncol=4)

    max_abs_target_delta = get_col(rt, "max_abs_target_delta")
    max_abs_command_delta = get_col(rt, "max_abs_command_delta")
    axes[4].plot(tr, max_abs_target_delta, label="max_abs_target_delta")
    axes[4].plot(tr, max_abs_command_delta, label="max_abs_command_delta")
    style_axis(axes[4], "RT Delta [rad]")
    axes[4].set_xlabel("Time [s]")
    add_legend(axes[4], ncol=2)

    fig.suptitle(f"Teleop Trace Overview: {trace_dir}")
    fig.tight_layout()
    return fig


def make_joint_grid_figure(trace_dir, rt, tr):
    fig, axes = plt.subplots(7, 2, figsize=(18, 22), sharex="col")

    for joint in range(7):
        q = get_col(rt, f"q_{joint}")
        q_d = get_col(rt, f"q_d_{joint}")
        q_planned = get_col(rt, f"q_planned_{joint}")
        q_traj_ref = get_col_or_default(rt, f"q_traj_ref_{joint}", q_planned)
        q_cmd = get_col(rt, f"q_cmd_{joint}")
        target_delta = get_col(rt, f"target_delta_{joint}")
        filtered_delta = get_col(rt, f"filtered_delta_{joint}")
        command_delta = get_col(rt, f"command_delta_{joint}")
        q_traj_ref_minus_q = q_traj_ref - q
        q_cmd_minus_q = q_cmd - q

        ax_pos = axes[joint, 0]
        ax_err = axes[joint, 1]

        ax_pos.plot(tr, q, label="q_measured", linewidth=1.2)
        ax_pos.plot(tr, q_d, label="q_d", linewidth=1.0)
        ax_pos.plot(tr, q_planned, label="q_planned", linewidth=1.0)
        ax_pos.plot(tr, q_traj_ref, label="q_traj_ref", linewidth=1.0)
        ax_pos.plot(tr, q_cmd, label="q_cmd", linewidth=1.0)
        style_axis(ax_pos, f"J{joint} Pos [rad]")
        add_legend(ax_pos, ncol=5)

        ax_err.plot(tr, q_traj_ref_minus_q, label="q_traj_ref - q", linewidth=1.2)
        ax_err.plot(tr, q_cmd_minus_q, label="q_cmd - q", linewidth=1.0)
        ax_err.plot(tr, target_delta, label="target_delta", linewidth=1.0)
        ax_err.plot(tr, filtered_delta, label="filtered_delta", linewidth=1.0)
        ax_err.plot(tr, command_delta, label="command_delta", linewidth=1.0)
        style_axis(ax_err, f"J{joint} Err [rad]")
        add_legend(ax_err, ncol=3)

    axes[-1, 0].set_xlabel("Time [s]")
    axes[-1, 1].set_xlabel("Time [s]")
    axes[0, 0].set_title("Joint Position Signals")
    axes[0, 1].set_title("Joint Error / Delta Signals")
    fig.suptitle(f"Teleop Joint Debug View: {trace_dir}")
    fig.tight_layout()
    return fig


def make_single_joint_figure(trace_dir, rt, tr, joint):
    q = get_col(rt, f"q_{joint}")
    q_d = get_col(rt, f"q_d_{joint}")
    q_planned = get_col(rt, f"q_planned_{joint}")
    q_traj_ref = get_col_or_default(rt, f"q_traj_ref_{joint}", q_planned)
    q_cmd = get_col(rt, f"q_cmd_{joint}")
    target_delta = get_col(rt, f"target_delta_{joint}")
    filtered_delta = get_col(rt, f"filtered_delta_{joint}")
    command_delta = get_col(rt, f"command_delta_{joint}")
    clamp_sat = get_col(rt, f"clamp_saturated_{joint}")

    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)

    axes[0].plot(tr, q, label="q_measured")
    axes[0].plot(tr, q_d, label="q_d")
    axes[0].plot(tr, q_planned, label="q_planned")
    axes[0].plot(tr, q_traj_ref, label="q_traj_ref")
    axes[0].plot(tr, q_cmd, label="q_cmd")
    style_axis(axes[0], f"J{joint} Pos [rad]")
    add_legend(axes[0], ncol=5)

    axes[1].plot(tr, q_traj_ref - q, label="q_traj_ref - q")
    axes[1].plot(tr, q_cmd - q, label="q_cmd - q")
    axes[1].plot(tr, q_planned - q_traj_ref, label="q_planned - q_traj_ref")
    style_axis(axes[1], f"J{joint} Error [rad]")
    add_legend(axes[1], ncol=3)

    axes[2].plot(tr, target_delta, label="target_delta")
    axes[2].plot(tr, filtered_delta, label="filtered_delta")
    axes[2].plot(tr, command_delta, label="command_delta")
    axes[2].plot(tr, clamp_sat, label="clamp_saturated")
    style_axis(axes[2], f"J{joint} Delta [rad]")
    axes[2].set_xlabel("Time [s]")
    add_legend(axes[2], ncol=4)

    fig.suptitle(f"Teleop Joint {joint} Detail: {trace_dir}")
    fig.tight_layout()
    return fig


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
        help="Optional joint index [0..6] for an additional detailed single-joint figure",
    )
    parser.add_argument("--show", action="store_true", help="Show interactive windows")
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

    overview_fig = make_overview_figure(args.trace_dir, planner, rt, tp, tr)
    overview_path = args.trace_dir / "trace_overview.png"
    overview_fig.savefig(overview_path, dpi=150)
    print(f"Saved plot: {overview_path}")

    joints_fig = make_joint_grid_figure(args.trace_dir, rt, tr)
    joints_path = args.trace_dir / "trace_joints.png"
    joints_fig.savefig(joints_path, dpi=150)
    print(f"Saved plot: {joints_path}")

    if args.joint is not None:
        joint_fig = make_single_joint_figure(args.trace_dir, rt, tr, args.joint)
        joint_path = args.trace_dir / f"trace_joint_{args.joint}.png"
        joint_fig.savefig(joint_path, dpi=150)
        print(f"Saved plot: {joint_path}")

    if args.show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()
