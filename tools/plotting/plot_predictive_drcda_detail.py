#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/hnuter-matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


METHODS = {
    "direct": ("Direct", "#4C78A8"),
    "full": ("Full predictive", "#E45756"),
    "no_horizon": ("No horizon", "#54A24B"),
}
JOINTS = [
    ("a_l", r"$\alpha_L$"),
    ("b_l", r"$\beta_L$"),
    ("a_r", r"$\alpha_R$"),
    ("b_r", r"$\beta_R$"),
]
PHASES = ["a_to_b", "dwell_b", "b_to_a", "dwell_a", "a_to_c"]
PHASE_LABELS = [r"A$\rightarrow$B", "hold B", r"B$\rightarrow$A", "hold A", r"A$\rightarrow$C"]


def load_data(root: Path) -> dict[str, pd.DataFrame]:
    data = {}
    for method in METHODS:
        path = root / "data" / f"T_1p2_{method}_diagnostic.csv"
        frame = pd.read_csv(path)
        start = frame.loc[frame["switch_phase"] == "a_to_b"].iloc[0]
        # Mission elapsed freezes in final hold; the telemetry clock does not.
        t0 = start["time_s"] - start["switch_elapsed_s"] + 4.0
        frame["t_rel"] = frame["time_s"] - t0
        data[method] = frame
    return data


def style() -> None:
    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.titlesize": 10,
            "axes.labelsize": 9,
            "legend.fontsize": 8,
            "figure.dpi": 120,
            "savefig.dpi": 240,
            "axes.grid": True,
            "grid.alpha": 0.25,
            "lines.linewidth": 1.35,
        }
    )


def phase_bounds(frame: pd.DataFrame) -> list[tuple[float, float]]:
    result = []
    for phase in PHASES:
        values = frame.loc[frame["switch_phase"] == phase, "t_rel"].dropna()
        result.append((float(values.min()), float(values.max())))
    return result


def decorate_phases(ax: plt.Axes, frame: pd.DataFrame, labels: bool = False) -> None:
    bounds = phase_bounds(frame)
    shades = ["#DDEBF7", "#F3F3F3", "#FCE4D6", "#F3F3F3", "#E2F0D9"]
    for (lo, hi), shade, label in zip(bounds, shades, PHASE_LABELS):
        ax.axvspan(lo, hi, color=shade, alpha=0.22, linewidth=0)
        ax.axvline(lo, color="#777777", lw=0.55, alpha=0.45)
        if labels:
            ax.text((lo + hi) / 2, 0.985, label, transform=ax.get_xaxis_transform(),
                    ha="center", va="top", fontsize=7, color="#444444")
    ax.set_xlim(-0.5, 4.75)


def plot_method(ax: plt.Axes, data: dict[str, pd.DataFrame], column: str) -> None:
    for method, frame in data.items():
        if column not in frame:
            continue
        label, color = METHODS[method]
        ax.plot(frame["t_rel"], frame[column], label=label, color=color)


def save(fig: plt.Figure, output: Path, stem: str) -> None:
    fig.savefig(output / f"{stem}.png", bbox_inches="tight")
    fig.savefig(output / f"{stem}.pdf", bbox_inches="tight")
    plt.close(fig)


def tracking_figure(data: dict[str, pd.DataFrame], output: Path) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(12.5, 9.0), sharex=True)
    reference = data["full"]
    plot_method(axes[0, 0], data, "position_y_enu_m")
    axes[0, 0].plot(reference["t_rel"], reference["target_y_enu_m"], "k--", lw=1.2, label="Reference")
    axes[0, 0].set_ylabel("Lateral y (m)")
    axes[0, 0].set_title("Lateral position tracking")

    for method, frame in data.items():
        label, color = METHODS[method]
        err = frame["position_y_enu_m"] - frame["target_y_enu_m"]
        axes[0, 1].plot(frame["t_rel"], err, label=label, color=color)
    axes[0, 1].axhline(0, color="k", lw=0.7)
    axes[0, 1].set_ylabel(r"$e_y$ (m)")
    axes[0, 1].set_title("Signed lateral error")

    plot_method(axes[1, 0], data, "velocity_y_enu_mps")
    axes[1, 0].plot(reference["t_rel"], reference["target_vy_enu_mps"], "k--", lw=1.2, label="Reference")
    axes[1, 0].set_ylabel(r"$v_y$ (m/s)")
    axes[1, 0].set_title("Lateral velocity")

    for method, frame in data.items():
        label, color = METHODS[method]
        error = np.sqrt(
            (frame["position_x_enu_m"] - frame["target_x_enu_m"]) ** 2
            + (frame["position_y_enu_m"] - frame["target_y_enu_m"]) ** 2
            + (frame["position_z_rel_m"] - frame["target_z_rel_m"]) ** 2
        )
        axes[1, 1].plot(frame["t_rel"], error, label=label, color=color)
    axes[1, 1].axhline(0.75, color="#B22222", ls=":", lw=1.0, label="Pass limit")
    axes[1, 1].set_ylabel(r"$\|e_p\|$ (m)")
    axes[1, 1].set_title("3D position error norm")

    plot_method(axes[2, 0], data, "position_x_enu_m")
    axes[2, 0].axhline(0, color="k", ls="--", lw=0.8)
    axes[2, 0].set_ylabel("Cross-track x (m)")
    axes[2, 0].set_title("Cross-track deviation")

    plot_method(axes[2, 1], data, "position_z_rel_m")
    axes[2, 1].plot(reference["t_rel"], reference["target_z_rel_m"], "k--", lw=1.2, label="Reference")
    axes[2, 1].axhline(0.5, color="#B22222", ls=":", lw=1.0, label="Altitude limit")
    axes[2, 1].set_ylabel("Altitude z (m)")
    axes[2, 1].set_title("Altitude retention")

    for index, ax in enumerate(axes.flat):
        decorate_phases(ax, reference, labels=index < 2)
        ax.set_xlabel("Time from A-to-B start (s)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=4, frameon=False)
    fig.suptitle("T = 1.2 s: detailed translational tracking comparison", y=1.01, fontsize=13)
    fig.tight_layout(rect=(0, 0.045, 1, 1))
    save(fig, output, "01_tracking_detail_T1p2")


def attitude_figure(data: dict[str, pd.DataFrame], output: Path) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(12.5, 9.0), sharex=True)
    reference = data["full"]
    specs = [
        ("attitude_error_angle_deg", "SO(3) attitude error (deg)", "Attitude error", 8.0),
        ("roll_deg", "Roll (deg)", "Roll response", 5.0),
        ("pitch_deg", "Pitch (deg)", "Pitch response", 5.0),
        ("yaw_deg", "Yaw (deg)", "Yaw response", None),
        ("angular_q_frd_rps", "Pitch rate q (rad/s)", "Pitch body rate", None),
        ("angular_r_frd_rps", "Yaw rate r (rad/s)", "Yaw body rate", None),
    ]
    for index, (ax, (column, ylabel, title, limit)) in enumerate(zip(axes.flat, specs)):
        plot_method(ax, data, column)
        if limit is not None:
            ax.axhline(limit, color="#B22222", ls=":", lw=1.0)
            if column != "attitude_error_angle_deg":
                ax.axhline(-limit, color="#B22222", ls=":", lw=1.0)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.set_xlabel("Time from A-to-B start (s)")
        decorate_phases(ax, reference, labels=index < 2)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False)
    fig.suptitle("T = 1.2 s: attitude and angular-rate comparison", y=1.01, fontsize=13)
    fig.tight_layout(rect=(0, 0.045, 1, 1))
    save(fig, output, "02_attitude_rates_T1p2")


def wrench_figure(data: dict[str, pd.DataFrame], output: Path) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(12.5, 9.0), sharex=True)
    reference = data["full"]
    specs = [
        ("wrench_fx_body_n", "Requested Fx (N)"),
        ("wrench_fy_body_n", "Requested Fy (N)"),
        ("wrench_fz_body_n", "Requested Fz (N)"),
        ("wrench_tau_x_nm", "Requested tau-x (N m)"),
        ("wrench_tau_y_nm", "Requested tau-y (N m)"),
        ("wrench_tau_z_nm", "Requested tau-z (N m)"),
    ]
    for index, (ax, (column, title)) in enumerate(zip(axes.flat, specs)):
        plot_method(ax, data, column)
        ax.axhline(0, color="k", lw=0.6)
        ax.set_title(title)
        ax.set_xlabel("Time from A-to-B start (s)")
        decorate_phases(ax, reference, labels=index < 2)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False)
    fig.suptitle("T = 1.2 s: outer-loop wrench requests seen by each allocator", y=1.01, fontsize=13)
    fig.tight_layout(rect=(0, 0.045, 1, 1))
    save(fig, output, "03_requested_wrench_T1p2")


def joint_figure(data: dict[str, pd.DataFrame], output: Path) -> None:
    fig, axes = plt.subplots(3, 4, figsize=(15.5, 9.0), sharex=True)
    reference = data["full"]
    for row, (method, frame) in enumerate(data.items()):
        method_label, color = METHODS[method]
        for col, (joint, joint_label) in enumerate(JOINTS):
            ax = axes[row, col]
            cmd = f"switch_commanded_joint_{joint}_rad"
            measured = f"switch_measured_joint_{joint}_rad"
            ax.plot(frame["t_rel"], frame[cmd], color=color, ls="--", label="Command")
            ax.plot(frame["t_rel"], frame[measured], color="#222222", label="Measured")
            ax.fill_between(frame["t_rel"], frame[cmd], frame[measured], color=color, alpha=0.12)
            ax.axhline(0, color="k", lw=0.5)
            ax.set_title(f"{method_label}: {joint_label}")
            ax.set_ylabel("Joint angle (rad)")
            ax.set_xlabel("Time from A-to-B start (s)")
            decorate_phases(ax, reference, labels=row == 0)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=2, frameon=False)
    fig.suptitle("T = 1.2 s: command versus measured tilt-joint motion", y=1.01, fontsize=13)
    fig.tight_layout(rect=(0, 0.045, 1, 1))
    save(fig, output, "04_joint_tracking_T1p2")


def reversal_figure(data: dict[str, pd.DataFrame], output: Path) -> None:
    data = {key: frame.loc[frame["t_rel"].between(.75, 2.35)].copy()
            for key, frame in data.items()}
    fig, axes = plt.subplots(3, 2, figsize=(12.5, 9.0), sharex=True)
    full = data["full"]
    zoom = (-0.45, 1.15)

    for method, frame in data.items():
        label, color = METHODS[method]
        axes[0, 0].plot(frame["t_rel"] - 1.2, frame["position_y_enu_m"], label=label, color=color)
    axes[0, 0].plot(full["t_rel"] - 1.2, full["target_y_enu_m"], "k--", label="Reference")
    axes[0, 0].set_ylabel("y (m)")
    axes[0, 0].set_title("Position through B-point reversal")

    for method, frame in data.items():
        label, color = METHODS[method]
        axes[0, 1].plot(frame["t_rel"] - 1.2, frame["wrench_fy_body_n"], label=label, color=color)
    axes[0, 1].set_ylabel("Requested Fy (N)")
    axes[0, 1].set_title("Current lateral wrench request")

    axes[1, 0].plot(full["t_rel"] - 1.2, full["wrench_fy_body_n"], color="#777777", label=r"Current $w_d$")
    axes[1, 0].plot(full["t_rel"] - 1.2, full["drcda_reference_h1_wrench_fy_n"], "--", color="#F28E2B", label=r"Reference $H_1$")
    axes[1, 0].plot(full["t_rel"] - 1.2, full["drcda_predicted_h1_wrench_fy_n"], color="#F28E2B", label=r"Predicted $H_1$")
    axes[1, 0].plot(full["t_rel"] - 1.2, full["drcda_reference_h2_wrench_fy_n"], "--", color="#E45756", label=r"Reference $H_2$")
    axes[1, 0].plot(full["t_rel"] - 1.2, full["drcda_predicted_h2_wrench_fy_n"], color="#E45756", label=r"Predicted $H_2$")
    axes[1, 0].set_ylabel("Fy (N)")
    axes[1, 0].set_title("Full: future reference and prediction")

    for method, frame in data.items():
        label, color = METHODS[method]
        axes[1, 1].plot(frame["t_rel"] - 1.2, frame["switch_commanded_joint_b_l_rad"], label=label, color=color)
    axes[1, 1].set_ylabel(r"Commanded $\beta_L$ (rad)")
    axes[1, 1].set_title("Pre-reversal secondary-joint command")

    for method, frame in data.items():
        label, color = METHODS[method]
        axes[2, 0].plot(frame["t_rel"] - 1.2, frame["attitude_error_angle_deg"], label=label, color=color)
    axes[2, 0].axhline(8, color="#B22222", ls=":", lw=1.0)
    axes[2, 0].set_ylabel("SO(3) error (deg)")
    axes[2, 0].set_title("Attitude disturbance at reversal")

    for method, frame in data.items():
        label, color = METHODS[method]
        axes[2, 1].plot(frame["t_rel"] - 1.2, frame["angular_r_frd_rps"], label=label, color=color)
    axes[2, 1].set_ylabel("Yaw rate r (rad/s)")
    axes[2, 1].set_title("Yaw-rate excitation")

    for ax in axes.flat:
        ax.axvline(0, color="#444444", ls="--", lw=0.9, label="Nominal arrival B")
        ax.axvline(0.5, color="#777777", ls=":", lw=0.8, label="Nominal departure B")
        ax.set_xlim(*zoom)
        ax.set_xlabel("Time relative to nominal arrival at B (s)")
    for ax in axes.flat:
        ax.legend(loc="best", ncol=2)
    fig.suptitle("T = 1.2 s: enlarged B-point reversal mechanism", y=1.01, fontsize=13)
    fig.tight_layout()
    save(fig, output, "05_reversal_zoom_T1p2")


def prediction_error_figure(data: dict[str, pd.DataFrame], output: Path) -> None:
    full = data["full"].sort_values("t_rel")
    t = full["t_rel"].to_numpy(float)
    time_abs = full["time_s"].to_numpy(float)
    active = full["switch_phase"].isin(["a_to_b", "b_to_a", "a_to_c"]).to_numpy()
    components = [
        ("fx", "n", "Fx (N)"), ("fy", "n", "Fy (N)"), ("fz", "n", "Fz (N)"),
        ("tx", "nm", "tau-x (N m)"), ("ty", "nm", "tau-y (N m)"), ("tz", "nm", "tau-z (N m)"),
    ]
    fig, axes = plt.subplots(3, 2, figsize=(12.5, 9.0), sharex=True)
    for ax, (suffix, unit, title) in zip(axes.flat, components):
        actual = full[f"drcda_estimated_wrench_{suffix}_{unit}"].to_numpy(float)
        for horizon, h_label, color in [(0.05, r"$H_1=0.05$ s", "#F28E2B"), (0.10, r"$H_2=0.10$ s", "#E45756")]:
            predicted = full[f"drcda_predicted_h{1 if horizon == 0.05 else 2}_wrench_{suffix}_{unit}"].to_numpy(float)
            error = np.full_like(predicted, np.nan)
            for index in np.flatnonzero(active):
                target = time_abs[index] + horizon
                nearest = int(np.argmin(np.abs(time_abs - target)))
                if abs(time_abs[nearest] - target) <= 0.035 and np.isfinite(actual[nearest]):
                    error[index] = predicted[index] - actual[nearest]
            rms = np.sqrt(np.nanmean(error ** 2))
            ax.plot(t, error, color=color, label=f"{h_label}, RMSE={rms:.2f}")
        ax.axhline(0, color="k", lw=0.7)
        ax.set_title(title)
        ax.set_ylabel("Prediction error")
        ax.set_xlabel("Time from A-to-B start (s)")
        decorate_phases(ax, full)
        ax.legend(loc="best")
    fig.suptitle("Full predictive DRCDA: horizon prediction error against future estimated wrench", y=1.01, fontsize=13)
    fig.tight_layout()
    save(fig, output, "06_full_prediction_error_T1p2")


def direct_full_figure(data: dict[str, pd.DataFrame], output: Path) -> None:
    selected = {name: data[name] for name in ("direct", "full")}
    full = data["full"]
    fig, axes = plt.subplots(4, 2, figsize=(12.5, 11.2), sharex=True)

    plot_method(axes[0, 0], selected, "position_y_enu_m")
    axes[0, 0].plot(full["t_rel"], full["target_y_enu_m"], "k--", lw=1.2, label="Reference")
    axes[0, 0].set_ylabel("y (m)")
    axes[0, 0].set_title("Lateral position")

    for method, frame in selected.items():
        label, color = METHODS[method]
        error = frame["position_y_enu_m"] - frame["target_y_enu_m"]
        axes[0, 1].plot(frame["t_rel"], error, color=color, label=label)
    axes[0, 1].axhline(0, color="k", lw=0.7)
    axes[0, 1].set_ylabel(r"$e_y$ (m)")
    axes[0, 1].set_title("Signed lateral error")

    plot_method(axes[1, 0], selected, "velocity_y_enu_mps")
    axes[1, 0].plot(full["t_rel"], full["target_vy_enu_mps"], "k--", lw=1.2, label="Reference")
    axes[1, 0].set_ylabel(r"$v_y$ (m/s)")
    axes[1, 0].set_title("Lateral velocity")

    plot_method(axes[1, 1], selected, "attitude_error_angle_deg")
    axes[1, 1].axhline(8, color="#B22222", ls=":", lw=1.0, label="Pass limit")
    axes[1, 1].set_ylabel("SO(3) error (deg)")
    axes[1, 1].set_title("Attitude error")

    plot_method(axes[2, 0], selected, "angular_r_frd_rps")
    axes[2, 0].set_ylabel("Yaw rate r (rad/s)")
    axes[2, 0].set_title("Yaw-rate response")

    plot_method(axes[2, 1], selected, "wrench_fy_body_n")
    axes[2, 1].set_ylabel("Requested Fy (N)")
    axes[2, 1].set_title("Lateral wrench request")

    for method, frame in selected.items():
        label, color = METHODS[method]
        axes[3, 0].plot(frame["t_rel"], frame["switch_commanded_joint_b_l_rad"], color=color, ls="--", label=f"{label} cmd")
        axes[3, 0].plot(frame["t_rel"], frame["switch_measured_joint_b_l_rad"], color=color, label=f"{label} measured")
    axes[3, 0].set_ylabel(r"$\beta_L$ (rad)")
    axes[3, 0].set_title("Left secondary joint")

    for method, frame in selected.items():
        label, color = METHODS[method]
        axes[3, 1].plot(frame["t_rel"], frame["switch_commanded_joint_b_r_rad"], color=color, ls="--", label=f"{label} cmd")
        axes[3, 1].plot(frame["t_rel"], frame["switch_measured_joint_b_r_rad"], color=color, label=f"{label} measured")
    axes[3, 1].set_ylabel(r"$\beta_R$ (rad)")
    axes[3, 1].set_title("Right secondary joint")

    for index, ax in enumerate(axes.flat):
        decorate_phases(ax, full, labels=index < 2)
        ax.set_xlabel("Time from A-to-B start (s)")
        ax.legend(loc="best", ncol=2)
    fig.suptitle("T = 1.2 s: Direct versus Full predictive DRCDA (failure-free zoom)", y=1.005, fontsize=13)
    fig.tight_layout()
    save(fig, output, "07_direct_vs_full_zoom_T1p2")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    style()
    data = load_data(args.root)
    segment_data = {key: frame.loc[frame["t_rel"].between(-.5, 4.6)].copy()
                    for key, frame in data.items()}
    tracking_figure(segment_data, args.output)
    attitude_figure(segment_data, args.output)
    wrench_figure(segment_data, args.output)
    joint_figure(segment_data, args.output)
    reversal_figure(data, args.output)
    prediction_error_figure(data, args.output)
    direct_full_figure(segment_data, args.output)
    print(f"generated {len(list(args.output.glob('*.png')))} PNG and {len(list(args.output.glob('*.pdf')))} PDF figures")


if __name__ == "__main__":
    main()
