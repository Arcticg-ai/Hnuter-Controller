#!/usr/bin/env python3
"""Analyze no-delay Hnuter maneuver experiments and make publication figures."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

os.environ.setdefault("MPLCONFIGDIR", "/tmp/hnuter-matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from tools.plotting.trajectory_alignment import fit_planar_rotation, transform_points


METHODS = ("original_direct", "basic_da", "full", "no_horizon", "no_rate_limits")
LABELS = {
    "original_direct": "Original direct",
    "basic_da": "Basic DA",
    "full": "Full DRCDA",
    "no_horizon": "No horizon",
    "no_rate_limits": "No rate limits",
}
COLORS = {
    "original_direct": "#333333",
    "basic_da": "#D55E00",
    "full": "#0072B2",
    "no_horizon": "#CC79A7",
    "no_rate_limits": "#009E73",
}
LINESTYLES = {
    "original_direct": "-",
    "basic_da": "--",
    "full": "-",
    "no_horizon": "-.",
    "no_rate_limits": ":",
}
CORE = ("original_direct", "basic_da", "full")
ABLATIONS = ("full", "no_horizon", "no_rate_limits")


def configure_style() -> None:
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "font.size": 8.0,
        "axes.labelsize": 8.0,
        "axes.titlesize": 8.0,
        "legend.fontsize": 7.0,
        "xtick.labelsize": 7.0,
        "ytick.labelsize": 7.0,
        "axes.linewidth": 0.7,
        "lines.linewidth": 1.15,
        "grid.linewidth": 0.45,
        "grid.alpha": 0.22,
        "legend.frameon": False,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.03,
    })


def read_segment(path: Path, scenario: str) -> dict[str, np.ndarray]:
    expected_mode = "lissajous" if scenario == "aggressive" else "attitude"
    with path.open(newline="") as stream:
        rows = [row for row in csv.DictReader(stream) if row.get("auto_traj_mode") == expected_mode]
    if len(rows) < 10:
        raise ValueError(f"{path} has only {len(rows)} samples for {expected_mode}")

    def values(*columns: str) -> np.ndarray:
        return np.array([[float(row[column]) for column in columns] for row in rows], dtype=float)

    def one(column: str, default: float = math.nan) -> np.ndarray:
        return np.array([
            float(row[column]) if row.get(column, "") not in ("", None) else default
            for row in rows
        ], dtype=float)

    time_s = one("time_s")
    time_s -= time_s[0]
    data = {
        "time": time_s,
        "position": values("position_x_enu_m", "position_y_enu_m", "position_z_rel_m"),
        "target_position": values("target_x_enu_m", "target_y_enu_m", "target_z_rel_m"),
        "velocity": values("velocity_x_enu_mps", "velocity_y_enu_mps", "velocity_z_enu_mps"),
        "target_velocity": values("target_vx_enu_mps", "target_vy_enu_mps", "target_vz_enu_mps"),
        "target_acceleration": values("target_ax_enu_mps2", "target_ay_enu_mps2", "target_az_enu_mps2"),
        "attitude_error": one("attitude_error_angle_deg"),
        "roll": one("roll_deg"),
        "pitch": one("pitch_deg"),
        "continuous_pitch": one("continuous_test_pitch_deg"),
        "target_roll": one("target_roll_deg"),
        "target_pitch": one("target_pitch_deg"),
        "body_rates": values("angular_p_frd_rps", "angular_q_frd_rps", "angular_r_frd_rps"),
        "servo": values(*(f"cmd_servo_{index}" for index in range(4))),
        "motor": values(*(f"cmd_motor_{index}" for index in range(5))),
        "armed": one("armed"),
        "offboard": one("offboard"),
        "ground_contact": one("ground_contact"),
        "landed": one("landed"),
        "safety_cutoff": one("direct_safety_cutoff"),
    }
    residual_columns = [f"drcda_wrench_residual_{name}" for name in (
        "fx_n", "fy_n", "fz_n", "tx_nm", "ty_nm", "tz_nm"
    )]
    if all(column in rows[0] for column in residual_columns):
        data["wrench_residual"] = values(*residual_columns)
        data["solve_ms"] = one("drcda_solve_ms")
    return data


def rmse(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(finite ** 2))) if finite.size else math.nan


def finite_percentile(values: np.ndarray, percentile: float) -> float:
    finite = values[np.isfinite(values)]
    return float(np.percentile(finite, percentile)) if finite.size else math.nan


def finite_max(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(np.max(finite)) if finite.size else math.nan


def wrapped_error_deg(actual: np.ndarray, target: np.ndarray) -> np.ndarray:
    return (actual - target + 180.0) % 360.0 - 180.0


def calculate_metrics(
    data: dict[str, np.ndarray],
    scenario: str,
    completed_timed_segment: bool | None = None,
) -> dict[str, object]:
    pos_error = data["position"] - data["target_position"]
    pos_norm = np.linalg.norm(pos_error, axis=1)
    vel_error = data["velocity"] - data["target_velocity"]
    duration = max(float(data["time"][-1]), 1e-6)
    servo_tv = float(np.sum(np.abs(np.diff(data["servo"], axis=0))))
    metrics: dict[str, object] = {
        "samples": int(data["time"].size),
        "duration_s": duration,
        "completed_timed_segment": (
            duration >= (7.6 if scenario == "aggressive" else 54.0)
            if completed_timed_segment is None else bool(completed_timed_segment)
        ),
        "armed_fraction": float(np.mean(data["armed"] > 0.5)),
        "offboard_fraction": float(np.mean(data["offboard"] > 0.5)),
        "ground_contact_fraction": float(np.mean(data["ground_contact"] > 0.5)),
        "safety_cutoff_samples": int(np.sum(data["safety_cutoff"] > 0.5)),
        "position_rmse_m": rmse(pos_norm),
        "position_p95_m": float(np.percentile(pos_norm, 95.0)),
        "position_max_m": float(np.max(pos_norm)),
        "velocity_rmse_mps": rmse(np.linalg.norm(vel_error, axis=1)),
        "attitude_so3_rmse_deg": rmse(data["attitude_error"]),
        "attitude_so3_p95_deg": float(np.percentile(data["attitude_error"], 95.0)),
        "body_rate_peak_rps": float(np.max(np.linalg.norm(data["body_rates"], axis=1))),
        "servo_total_variation": servo_tv,
        "servo_variation_rate_per_s": servo_tv / duration,
        "motor_peak_normalized": float(np.nanmax(np.abs(data["motor"]))),
        "target_speed_peak_mps": float(np.max(np.linalg.norm(data["target_velocity"], axis=1))),
        "target_acceleration_peak_mps2": float(np.max(np.linalg.norm(data["target_acceleration"], axis=1))),
    }
    contact_indices = np.flatnonzero(data["ground_contact"] > 0.5)
    physical_contact = (data["ground_contact"] > 0.5) & (data["position"][:, 2] < 0.35)
    physical_contact_indices = np.flatnonzero(physical_contact)
    metrics["first_ground_contact_s"] = (
        float(data["time"][contact_indices[0]]) if contact_indices.size else math.nan
    )
    metrics["near_ground_contact_fraction"] = float(np.mean(physical_contact))
    metrics["first_physical_contact_s"] = (
        float(data["time"][physical_contact_indices[0]])
        if physical_contact_indices.size else math.nan
    )
    if "wrench_residual" in data:
        scales = np.array([80.0, 80.0, 100.0, 12.0, 12.0, 12.0])
        normalized = np.linalg.norm(data["wrench_residual"] / scales, axis=1)
        metrics["normalized_wrench_residual_rmse"] = rmse(normalized)
        finite_solve = data["solve_ms"][np.isfinite(data["solve_ms"])]
        metrics["solve_time_p95_ms"] = (
            float(np.percentile(finite_solve, 95.0)) if finite_solve.size else math.nan
        )
    if scenario == "attitude_80_180":
        roll_error = wrapped_error_deg(data["roll"], data["target_roll"])
        pitch_error = wrapped_error_deg(data["continuous_pitch"], data["target_pitch"])
        roll_hold = np.abs(data["target_roll"]) >= 0.98 * 80.0
        pitch_hold = np.abs(data["target_pitch"]) >= 0.98 * 180.0
        hold = roll_hold | pitch_hold
        metrics.update({
            "roll_tracking_rmse_deg": rmse(roll_error),
            "pitch_continuous_tracking_rmse_deg": rmse(pitch_error),
            "hold_samples": int(np.sum(hold)),
            "hold_attitude_so3_rmse_deg": rmse(data["attitude_error"][hold]),
            "hold_attitude_so3_p95_deg": finite_percentile(data["attitude_error"][hold], 95.0),
            "hold_position_rmse_m": rmse(pos_norm[hold]),
            "hold_position_max_m": finite_max(pos_norm[hold]),
        })
        signed_holds = {
            "plus_roll": data["target_roll"] >= 0.98 * 80.0,
            "minus_roll": data["target_roll"] <= -0.98 * 80.0,
            "plus_pitch": data["target_pitch"] >= 0.98 * 180.0,
            "minus_pitch": data["target_pitch"] <= -0.98 * 180.0,
        }
        for name, mask in signed_holds.items():
            metrics[f"{name}_hold_samples"] = int(np.sum(mask))
            metrics[f"{name}_hold_so3_rmse_deg"] = rmse(data["attitude_error"][mask])
            metrics[f"{name}_hold_position_rmse_m"] = rmse(pos_norm[mask])
    metrics["tracking_valid"] = bool(
        metrics["completed_timed_segment"]
        and metrics["armed_fraction"] >= 0.995
        and metrics["offboard_fraction"] >= 0.995
        and not physical_contact_indices.size
        and metrics["safety_cutoff_samples"] == 0
        and metrics["position_p95_m"] < (1.5 if scenario == "aggressive" else 2.0)
        and metrics["attitude_so3_p95_deg"] < (30.0 if scenario == "aggressive" else 45.0)
    )
    return metrics


def save_figure(figure: plt.Figure, figures: Path, stem: str) -> None:
    figure.savefig(figures / f"{stem}.pdf")
    figure.savefig(figures / f"{stem}.png", dpi=600)
    plt.close(figure)


def finish_axes(axes: np.ndarray | list[plt.Axes]) -> None:
    for axis in np.asarray(axes).reshape(-1):
        axis.grid(True)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)


def plot_aggressive(data: dict[str, dict[str, np.ndarray]], figures: Path, methods: tuple[str, ...], stem: str) -> None:
    figure, axes = plt.subplots(4, 1, figsize=(7.05, 6.0), sharex=True)
    labels = ("East error (m)", "North error (m)", "Altitude error (m)")
    for method in methods:
        item = data[method]
        error = item["position"] - item["target_position"]
        for index in range(3):
            axes[index].plot(item["time"], error[:, index], color=COLORS[method],
                             linestyle=LINESTYLES[method], label=LABELS[method])
        axes[3].plot(item["time"], item["attitude_error"], color=COLORS[method],
                     linestyle=LINESTYLES[method], label=LABELS[method])
    for index, label in enumerate(labels):
        axes[index].set_ylabel(label)
        axes[index].axhline(0.0, color="#999999", linewidth=0.55)
    axes[3].set_ylabel("SO(3) error (deg)")
    axes[3].set_xlabel("Maneuver time (s)")
    axes[0].legend(ncol=len(methods), loc="upper center")
    finish_axes(axes)
    figure.subplots_adjust(left=0.105, right=0.99, bottom=0.075, top=0.96, hspace=0.12)
    save_figure(figure, figures, stem)


def plot_aggressive_trajectory(data: dict[str, dict[str, np.ndarray]], figures: Path) -> None:
    reference = data["original_direct"]
    reference_origin = reference["target_position"][0]
    reference_path = reference["target_position"] - reference_origin
    aligned: dict[str, np.ndarray] = {}
    for method in METHODS:
        item = data[method]
        rotation, _ = fit_planar_rotation(reference["target_position"], item["target_position"])
        aligned[method] = transform_points(
            item["position"], item["target_position"][0], reference_origin, rotation
        ) - reference_origin

    figure = plt.figure(figsize=(7.05, 3.2))
    axis_xy = figure.add_subplot(1, 2, 1)
    axis_3d = figure.add_subplot(1, 2, 2, projection="3d")
    axis_xy.plot(reference_path[:, 0], reference_path[:, 1], color="#777777", linestyle="--", label="Reference")
    axis_3d.plot(*reference_path.T, color="#777777", linestyle="--", label="Reference")
    for method in METHODS:
        path = aligned[method]
        style = {"color": COLORS[method], "linestyle": LINESTYLES[method], "label": LABELS[method]}
        axis_xy.plot(path[:, 0], path[:, 1], **style)
        axis_3d.plot(*path.T, **style)
    axis_xy.set_xlabel("Aligned East (m)")
    axis_xy.set_ylabel("Aligned North (m)")
    axis_xy.grid(True)
    axis_xy.spines["top"].set_visible(False)
    axis_xy.spines["right"].set_visible(False)
    axis_3d.set_xlabel("East (m)", labelpad=1)
    axis_3d.set_ylabel("North (m)", labelpad=1)
    axis_3d.set_zlabel("Altitude (m)", labelpad=1)
    axis_3d.tick_params(pad=0)
    handles, labels = axis_xy.get_legend_handles_labels()
    figure.legend(handles, labels, ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.01))
    figure.subplots_adjust(left=0.085, right=0.97, bottom=0.13, top=0.84, wspace=0.18)
    save_figure(figure, figures, "aggressive_trajectory")


def plot_attitude(data: dict[str, dict[str, np.ndarray]], figures: Path, methods: tuple[str, ...], stem: str) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(7.05, 4.55), sharex=True)
    reference = data[methods[0]]
    cycle_s = float(reference["time"][-1]) / 4.0
    segments = (
        (0, "Roll +80 deg", "roll", "target_roll", (-95.0, 95.0)),
        (1, "Pitch +180 deg", "continuous_pitch", "target_pitch", (-195.0, 195.0)),
        (2, "Roll -80 deg", "roll", "target_roll", (-95.0, 95.0)),
        (3, "Pitch -180 deg", "continuous_pitch", "target_pitch", (-195.0, 195.0)),
    )
    for axis, (index, title, actual_key, target_key, limits) in zip(axes.flat, segments):
        start = index * cycle_s
        stop = (index + 1) * cycle_s + 0.2
        reference_mask = (reference["time"] >= start) & (reference["time"] <= stop)
        reference_time = reference["time"][reference_mask] - start
        reference_target = reference[target_key][reference_mask]
        axis.plot(reference_time, reference_target, color="#777777", linestyle="--", label="Reference")
        for method in methods:
            item = data[method]
            mask = (item["time"] >= start) & (item["time"] <= stop)
            target = item[target_key][mask]
            actual = target + wrapped_error_deg(item[actual_key][mask], target)
            axis.plot(item["time"][mask] - start, actual, color=COLORS[method],
                      linestyle=LINESTYLES[method], label=LABELS[method])
        axis.set_title(title)
        axis.set_ylim(*limits)
        axis.set_ylabel("Angle (deg)")
        axis.set_xlabel("Segment time (s)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(
        handles,
        labels,
        ncol=min(4, len(methods) + 1),
        loc="upper center",
        bbox_to_anchor=(0.5, 1.01),
    )
    finish_axes(axes)
    figure.subplots_adjust(left=0.085, right=0.99, bottom=0.10, top=0.85, hspace=0.34, wspace=0.20)
    save_figure(figure, figures, stem)


def plot_attitude_errors(
    data: dict[str, dict[str, np.ndarray]],
    figures: Path,
    methods: tuple[str, ...],
    stem: str,
) -> None:
    figure, axes = plt.subplots(2, 1, figsize=(7.05, 3.55), sharex=True)
    for method in methods:
        item = data[method]
        position_error = np.linalg.norm(item["position"] - item["target_position"], axis=1)
        axes[0].plot(item["time"], position_error, color=COLORS[method],
                     linestyle=LINESTYLES[method], label=LABELS[method])
        axes[1].plot(item["time"], item["attitude_error"], color=COLORS[method],
                     linestyle=LINESTYLES[method], label=LABELS[method])
    axes[0].set_ylabel("Position error (m)")
    axes[1].set_ylabel("SO(3) error (deg)")
    axes[1].set_xlabel("Test time (s)")
    axes[0].legend(ncol=len(methods), loc="upper center")
    finish_axes(axes)
    figure.subplots_adjust(left=0.10, right=0.99, bottom=0.12, top=0.95, hspace=0.12)
    save_figure(figure, figures, stem)


def plot_summary(metrics: dict[str, dict[str, dict[str, object]]], figures: Path) -> None:
    keys = ("position_rmse_m", "attitude_so3_rmse_deg", "servo_variation_rate_per_s")
    names = ("Position RMSE", "SO(3) attitude RMSE", "Servo activity rate")
    scenarios = ("aggressive", "attitude_80_180")
    figure, axes = plt.subplots(2, 3, figsize=(7.05, 3.75))
    x = np.arange(len(METHODS))
    for row, scenario in enumerate(scenarios):
        for column, (key, name) in enumerate(zip(keys, names)):
            values = [float(metrics[scenario][method][key]) for method in METHODS]
            axes[row, column].bar(x, values, color=[COLORS[method] for method in METHODS], width=0.72)
            axes[row, column].set_xticks(x, ["Direct", "Basic", "Full", "No H", "No rate"], rotation=25, ha="right")
            axes[row, column].set_title(name if row == 0 else "")
            if column == 0:
                axes[row, column].set_ylabel("Aggressive" if row == 0 else "Attitude 80/180")
    finish_axes(axes)
    figure.subplots_adjust(left=0.09, right=0.99, bottom=0.16, top=0.92, wspace=0.32, hspace=0.38)
    save_figure(figure, figures, "metric_summary")


def percent_change(reference: float, value: float) -> float:
    return 100.0 * (value - reference) / reference if abs(reference) > 1e-12 else math.nan


def write_report(root: Path, manifest: dict[str, object], metrics: dict[str, dict[str, dict[str, object]]]) -> None:
    lines = [
        "# 无延迟固件 DRCDA 对比实验",
        "",
        "## 实验约束",
        "",
        f"- 固件：`{manifest['firmware']['path']}`，提交 `{manifest['firmware']['commit']}`，分支 `{manifest['firmware']['branch']}`。",
        "- 模型检查未发现纯延迟或独立一阶执行器插件；DRCDA 使用 `ideal` 舵机预测模型。",
        "- 所有方法加载相同的 `no_delay_drcda_tuning.json` 闭环参数，差别仅在执行器分配方法。",
        "- 姿态指标采用 SO(3) 几何误差；俯仰曲线采用连续角，避免 ±180° 欧拉角跳变。",
        "- 倒置悬停会触发 PX4 land detector 的低推力启发式；只有检测标志与实际高度接近地面同时出现时才判为物理接地。",
        "- 每组仅一次确定性 SITL 运行，结果不代表统计显著性。",
        "",
    ]
    definitions = manifest.get("scenario_definitions", {})
    if "aggressive" in metrics:
        lines.insert(8, "- 高速机动为 8 s、幅值 1.2/0.8/0.30 m 的三维轨迹。")
    if "attitude_80_180" in metrics:
        tuning = definitions.get("attitude_80_180", {}).get("tuning_overrides", {})
        lines.insert(
            8,
            "- 大姿态采用旋转矩阵参考，依次执行横滚 ±80°、俯仰 ±180°；"
            f"单程 `{tuning.get('attitude_segment_time_s', 'unknown')} s`，峰值保持 "
            f"`{tuning.get('attitude_peak_hold_s', 'unknown')} s`，回到水平后稳定等待 "
            f"`{tuning.get('attitude_level_settle_s', 'unknown')} s`。",
        )
    scenario_titles = {
        "aggressive": "高速机动",
        "attitude_80_180": "大姿态 ±80°/±180°",
    }
    for scenario in metrics:
        title = scenario_titles.get(scenario, scenario)
        lines.extend([
            f"## {title}", "",
            "| 方法 | 计时完成 | 跟踪有效 | 位置 RMSE (m) | P95 (m) | SO(3) RMSE (deg) | SO(3) P95 (deg) | 舵机活动率 (/s) |",
            "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |",
        ])
        for method in METHODS:
            item = metrics[scenario][method]
            lines.append(
                f"| {LABELS[method]} | {'是' if item['completed_timed_segment'] else '否'} | "
                f"{'是' if item['tracking_valid'] else '否'} | "
                f"{item['position_rmse_m']:.4f} | {item['position_p95_m']:.4f} | "
                f"{item['attitude_so3_rmse_deg']:.3f} | {item['attitude_so3_p95_deg']:.3f} | "
                f"{item['servo_variation_rate_per_s']:.3f} |"
            )
        full = metrics[scenario]["full"]
        direct = metrics[scenario]["original_direct"]
        basic = metrics[scenario]["basic_da"]
        lines.append("")
        if full["tracking_valid"] and direct["tracking_valid"]:
            lines.append(
                f"- Full DRCDA 相对 Original Direct 的位置 RMSE 变化为 `{percent_change(float(direct['position_rmse_m']), float(full['position_rmse_m'])):+.1f}%`，SO(3) 姿态 RMSE 变化为 `{percent_change(float(direct['attitude_so3_rmse_deg']), float(full['attitude_so3_rmse_deg'])):+.1f}%`。"
            )
        else:
            lines.append(
                f"- Full DRCDA 相对 Original Direct 的描述性变化为：位置 RMSE `{percent_change(float(direct['position_rmse_m']), float(full['position_rmse_m'])):+.1f}%`，SO(3) RMSE `{percent_change(float(direct['attitude_so3_rmse_deg']), float(full['attitude_so3_rmse_deg'])):+.1f}%`；因 Original Direct 未通过有效性判据，该百分比不作为两种有效方法间的统计改善。"
            )
        if full["tracking_valid"] and basic["tracking_valid"]:
            lines.append(
                f"- Full DRCDA 相对 Basic DA 的位置 RMSE 变化为 `{percent_change(float(basic['position_rmse_m']), float(full['position_rmse_m'])):+.1f}%`，SO(3) 姿态 RMSE 变化为 `{percent_change(float(basic['attitude_so3_rmse_deg']), float(full['attitude_so3_rmse_deg'])):+.1f}%`。"
            )
        else:
            lines.append("- Full DRCDA 与 Basic DA 至少一项未通过跟踪有效性判据，不计算改善百分比。")
        if scenario == "attitude_80_180":
            lines.append(
                f"- Full DRCDA 的舵机活动率相对 Original Direct 变化 "
                f"`{percent_change(float(direct['servo_variation_rate_per_s']), float(full['servo_variation_rate_per_s'])):+.1f}%`；"
                "本项仅描述同一参考下的命令平滑程度。"
            )
        lines.append("")
        if scenario == "attitude_80_180":
            lines.extend([
                "### 分段峰值保持结果", "",
                "下表单元格为 `SO(3) RMSE / 位置 RMSE`；`--` 表示飞行在到达该保持段前已失效。", "",
                "| 方法 | +80° roll | +180° pitch | -80° roll | -180° pitch |",
                "| --- | ---: | ---: | ---: | ---: |",
            ])
            for method in METHODS:
                item = metrics[scenario][method]
                cells = []
                for prefix in ("plus_roll", "plus_pitch", "minus_roll", "minus_pitch"):
                    samples = int(item[f"{prefix}_hold_samples"])
                    cells.append(
                        f"{item[f'{prefix}_hold_so3_rmse_deg']:.2f}° / {item[f'{prefix}_hold_position_rmse_m']:.2f} m"
                        if samples else "--"
                    )
                lines.append(f"| {LABELS[method]} | " + " | ".join(cells) + " |")
            lines.extend([
                "",
                "跟踪有效性由完整计时、Armed/Offboard 连续性、真实接地、位置 P95 和 SO(3) P95 共同判定；未完成方法的局部 RMSE 不与完整方法计算改善百分比。",
                "",
            ])
    lines.extend([
        "## 消融解释", "",
        "`No horizon` 将预测时域缩短到单个离散步；`No rate limits` 去除执行器物理速率和命令斜率约束。无延迟对象上不再把 `No delay` 作为性能消融，因为它与 Full DRCDA 的理想舵机预测配置等价。应同时结合误差、舵机活动率和扳手残差判断，不能只按单一 RMSE 排名。",
        "",
        "本次慢速、无纯延迟的大姿态工况中，Full、No horizon 与 No rate limits 的位置和姿态指标非常接近，差异不足以支持预测时域或速率约束带来显著收益。该结果说明此工况主要验证旋转矩阵参考与大姿态稳定性；预测可达约束仍应在更高速、能触及执行器动态边界的工况中评价。Basic DA 在首个 +80° roll 到达保持段前失效，因此不能用其局部 RMSE 与完整方法作排名。",
        "",
        "此前采用 6 s 单程、2 s 峰值保持且无回正等待的极限姿态结果已由本轮测试取代，不再作为算法优劣依据。",
        "",
        "## 文件", "",
        "- 原始 CSV、ULog 与控制台日志：`runs/`",
        "- 矢量 PDF 和 600 dpi PNG：`figures/`",
        "- 完整指标：`reports/metrics.json` 和 `reports/metrics_summary.csv`",
    ])
    (root / "reports/experiment_report_zh.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.root / "manifest.json").read_text(encoding="utf-8"))
    figures = args.root / "figures"
    reports = args.root / "reports"
    figures.mkdir(parents=True, exist_ok=True)
    reports.mkdir(parents=True, exist_ok=True)
    configure_style()

    loaded: dict[str, dict[str, dict[str, np.ndarray]]] = {}
    metrics: dict[str, dict[str, dict[str, object]]] = {}
    rows_by_case = {(case["scenario"], case["method"]): case for case in manifest["cases"]}
    available_scenarios = [
        scenario for scenario in ("aggressive", "attitude_80_180")
        if all((scenario, method) in rows_by_case for method in METHODS)
    ]
    if not available_scenarios:
        raise ValueError("manifest does not contain a complete method matrix")
    for scenario in available_scenarios:
        loaded[scenario] = {}
        metrics[scenario] = {}
        for method in METHODS:
            case = rows_by_case[(scenario, method)]
            if not case.get("csv"):
                raise ValueError(f"missing CSV for {scenario}/{method}: {case.get('error', '')}")
            item = read_segment(Path(case["csv"]), scenario)
            loaded[scenario][method] = item
            metrics[scenario][method] = calculate_metrics(
                item, scenario, case.get("status") == "complete"
            )

    if "aggressive" in loaded:
        plot_aggressive(loaded["aggressive"], figures, CORE, "aggressive_core_tracking")
        plot_aggressive(loaded["aggressive"], figures, ABLATIONS, "aggressive_ablation_tracking")
        plot_aggressive_trajectory(loaded["aggressive"], figures)
    if "attitude_80_180" in loaded:
        plot_attitude(loaded["attitude_80_180"], figures, CORE, "attitude_core_tracking")
        plot_attitude(loaded["attitude_80_180"], figures, ABLATIONS, "attitude_ablation_tracking")
        plot_attitude_errors(loaded["attitude_80_180"], figures, CORE, "attitude_core_errors")
        plot_attitude_errors(loaded["attitude_80_180"], figures, ABLATIONS, "attitude_ablation_errors")
    if len(loaded) == 2:
        plot_summary(metrics, figures)
    (reports / "metrics.json").write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    with (reports / "metrics_summary.csv").open("w", newline="") as stream:
        fieldnames = ["scenario", "method"] + sorted({
            key for scenario in metrics.values() for item in scenario.values() for key in item
        })
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for scenario, scenario_metrics in metrics.items():
            for method, item in scenario_metrics.items():
                writer.writerow({"scenario": scenario, "method": method, **item})
    write_report(args.root, manifest, metrics)
    print(f"report={args.root / 'reports/experiment_report_zh.md'}")
    print(f"figures={figures}")


if __name__ == "__main__":
    main()
