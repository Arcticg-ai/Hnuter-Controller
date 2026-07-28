#!/usr/bin/env python3
"""Analyze the staged multirate DRCDA validation campaign."""

from __future__ import annotations

import base64
import csv
import html
import json
import os
import re
import shutil
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/hnuter-matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from plot_lissajous_comparison import load_lissajous, metrics


ROOT = Path("hnuter_logs/multirate_drcda_validation_20260728").resolve()
RAW = ROOT / "raw" / "identified_delay"
DATA = ROOT / "data"
FIGURES = ROOT / "figures"
REPORTS = ROOT / "reports"

RUNS = {
    "legacy": ("stage0_legacy_full_retry", "原终端 DRCDA"),
    "servo_model": ("stage1_servo_model", "执行器预测器"),
    "gate": ("stage2_gate_full", "预测器 + 可达性门控"),
    "final": ("stage4_final_default", "多点多速率 DRCDA"),
    "no_multirate": ("ablation_no_multirate", "去多速率电机块"),
    "no_gate": ("ablation_no_reachability_gate", "去可达性门控"),
    "no_delay": ("ablation_no_delay", "去延迟模型"),
    "no_physical_rate": ("ablation_no_physical_rate", "去物理速率约束"),
    "no_command_slew": ("ablation_no_command_slew", "去命令斜率约束"),
    "no_horizon": ("ablation_no_horizon", "去预测时域"),
    "basic_da": ("comparison_state_aware_basic_da", "状态感知基础 DA"),
    "auto_horizon": ("stage3_multirate_full", "自动 278 ms 时域"),
}

POSITION_COLUMNS = (
    "position_x_enu_m",
    "position_y_enu_m",
    "position_z_rel_m",
)
TARGET_POSITION_COLUMNS = (
    "target_x_enu_m",
    "target_y_enu_m",
    "target_z_rel_m",
)


def find_csv(run_directory: str) -> Path:
    paths = sorted((RAW / run_directory / "external_control").rglob("*.csv"))
    if not paths:
        raise FileNotFoundError(f"no CSV found for {run_directory}")
    return paths[-1]


def read_trajectory_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return [
            row
            for row in csv.DictReader(stream)
            if row.get("auto_traj_mode") == "lissajous"
        ]


def numeric(rows: list[dict[str, str]], column: str) -> np.ndarray:
    return np.array(
        [
            float(row[column])
            for row in rows
            if row.get(column, "") not in ("", "nan", "NaN")
        ],
        dtype=float,
    )


def run_metrics(path: Path) -> dict[str, float | int | bool | str]:
    rows = read_trajectory_rows(path)
    result: dict[str, float | int | bool | str] = metrics(load_lissajous(path))
    attitude = numeric(rows, "attitude_error_angle_deg")
    solve = numeric(rows, "drcda_solve_ms")
    objective = numeric(rows, "drcda_objective")
    result.update(
        {
            "source_csv": str(path.relative_to(ROOT)),
            "trajectory_rows": len(rows),
            "attitude_error_max_deg": float(np.max(attitude)),
            "attitude_error_ge_60_fraction": float(np.mean(attitude >= 60.0)),
            "solve_mean_ms": float(np.mean(solve)),
            "solve_p95_ms": float(np.percentile(solve, 95.0)),
            "solve_max_ms": float(np.max(solve)),
            "objective_mean": (
                float(np.mean(objective)) if objective.size else float("nan")
            ),
        }
    )
    complete = result["duration_s"] >= 23.0
    physical = (
        result["attitude_error_max_deg"] < 60.0
        and result["position_max_3d_m"] < 2.0
    )
    result["trajectory_complete"] = bool(complete)
    result["physically_valid"] = bool(complete and physical)
    result["validity"] = "有效" if complete and physical else "失效"
    return result


def relative(data: dict[str, np.ndarray], key: str) -> np.ndarray:
    return data[key] - data["target_position"][0]


def plot_trajectory(loaded: dict[str, dict[str, np.ndarray]]) -> None:
    figure = plt.figure(figsize=(10.5, 7.8))
    axis = figure.add_subplot(111, projection="3d")
    reference = relative(loaded["final"], "target_position")
    axis.plot(*reference.T, "--", color="#202124", linewidth=2.0, label="Reference")
    for key, color, label in (
        ("legacy", "#4c78a8", "Legacy terminal DRCDA"),
        ("gate", "#f28e2b", "Reachability-gated DRCDA"),
        ("final", "#2a9d8f", "Multirate DRCDA"),
    ):
        axis.plot(
            *relative(loaded[key], "position").T,
            color=color,
            linewidth=1.7,
            label=label,
        )
    axis.set_xlabel("East relative position (m)")
    axis.set_ylabel("North relative position (m)")
    axis.set_zlabel("Relative altitude (m)")
    axis.set_title("3D Lissajous staged comparison")
    axis.view_init(elev=27, azim=-55)
    axis.legend(loc="upper left")
    axis.grid(alpha=0.25)
    figure.tight_layout()
    figure.savefig(FIGURES / "staged_trajectory_comparison.png", dpi=180)
    plt.close(figure)


def plot_tracking(loaded: dict[str, dict[str, np.ndarray]]) -> None:
    figure, axes = plt.subplots(3, 1, figsize=(11.5, 8.5), sharex=True)
    labels = ("East", "North", "Altitude")
    for key, color, label in (
        ("gate", "#f28e2b", "Reachability-gated DRCDA"),
        ("final", "#2a9d8f", "Multirate DRCDA"),
    ):
        data = loaded[key]
        error = data["position"] - data["target_position"]
        for index, axis in enumerate(axes):
            axis.plot(
                data["time_s"],
                error[:, index],
                color=color,
                linewidth=1.35,
                label=label,
            )
    for index, axis in enumerate(axes):
        axis.axhline(0.0, color="#202124", linewidth=0.8)
        axis.set_ylabel(f"{labels[index]} error (m)")
        axis.grid(alpha=0.25)
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("Trajectory time (s)")
    figure.suptitle("Tracking error on identified-delay plant")
    figure.tight_layout()
    figure.savefig(FIGURES / "tracking_error_comparison.png", dpi=180)
    plt.close(figure)


def plot_ablation(summary: dict[str, dict]) -> None:
    keys = (
        "final",
        "no_gate",
        "no_delay",
        "no_physical_rate",
        "no_command_slew",
        "no_multirate",
        "no_horizon",
        "basic_da",
    )
    labels = (
        "Full",
        "No gate",
        "No delay",
        "No physical\nrate",
        "No command\nslew",
        "No multirate",
        "No horizon",
        "Basic DA",
    )
    rmse = np.array([summary[key]["position_rmse_3d_m"] for key in keys])
    attitude = np.array([summary[key]["attitude_error_max_deg"] for key in keys])
    valid = np.array([summary[key]["physically_valid"] for key in keys])
    colors = ["#2a9d8f" if item else "#d1495b" for item in valid]

    figure, axes = plt.subplots(2, 1, figsize=(12.0, 8.5), sharex=True)
    axes[0].bar(labels, np.minimum(rmse, 5.0), color=colors)
    axes[0].set_ylabel("3D position RMSE (m, capped at 5)")
    axes[1].bar(labels, attitude, color=colors)
    axes[1].axhline(60.0, color="#7f1d1d", linestyle="--", linewidth=1.2)
    axes[1].set_ylabel("Maximum attitude error (deg)")
    for axis in axes:
        axis.grid(axis="y", alpha=0.25)
    figure.suptitle("Ablation results (red denotes physical invalidity)")
    figure.tight_layout()
    figure.savefig(FIGURES / "ablation_summary.png", dpi=180)
    plt.close(figure)


def plot_servo(loaded: dict[str, dict[str, np.ndarray]]) -> None:
    figure, axes = plt.subplots(2, 1, figsize=(11.5, 7.5), sharex=True)
    for axis, key, title in (
        (axes[0], "final", "Full multirate DRCDA"),
        (axes[1], "no_delay", "Delay-model ablation"),
    ):
        data = loaded[key]
        for index in range(4):
            axis.plot(
                data["time_s"],
                data["servo"][:, index],
                linewidth=1.1,
                label=f"Servo {index}",
            )
        axis.set_ylabel("Normalized command")
        axis.set_title(title)
        axis.grid(alpha=0.25)
    axes[0].legend(ncol=4, loc="upper right")
    axes[-1].set_xlabel("Trajectory time (s)")
    figure.tight_layout()
    figure.savefig(FIGURES / "servo_delay_ablation.png", dpi=180)
    plt.close(figure)


def plot_horizon_failure(paths: dict[str, Path]) -> None:
    figure, axes = plt.subplots(2, 1, figsize=(11.5, 7.8), sharex=False)
    for axis, key, title in (
        (axes[0], "final", "Tuned 180 ms horizon"),
        (axes[1], "auto_horizon", "Automatic 278 ms horizon"),
    ):
        rows = read_trajectory_rows(paths[key])
        time = numeric(rows, "time_s")
        time -= time[0]
        attitude = numeric(rows, "attitude_error_angle_deg")
        axis.plot(time, attitude, color="#2a9d8f" if key == "final" else "#d1495b")
        axis.axhline(60.0, color="#7f1d1d", linestyle="--", linewidth=1.0)
        axis.set_ylabel("Attitude error (deg)")
        axis.set_title(title)
        axis.grid(alpha=0.25)
    axes[-1].set_xlabel("Trajectory time (s)")
    figure.tight_layout()
    figure.savefig(FIGURES / "horizon_stability_boundary.png", dpi=180)
    plt.close(figure)


def markdown_report(summary: dict[str, dict]) -> str:
    final = summary["final"]
    gate = summary["gate"]
    no_delay = summary["no_delay"]
    delay_penalty = (
        no_delay["position_rmse_3d_m"] / final["position_rmse_3d_m"] - 1.0
    ) * 100.0
    lines = [
        "# 多速率延迟感知 DRCDA 递进验证报告",
        "",
        "## 实验条件",
        "",
        "- 固件分支：`codex/hnuter-identified-delay-actuator-20260728`，提交 `c986907d`。",
        "- 控制代码分支：`codex/multirate-ftr-drcda`。",
        "- 任务：24 s 三维李萨如曲线，所有组使用同一延迟固件、位置环和姿态环。",
        "- 物理有效判据：轨迹时长不少于 23 s、最大姿态误差小于 60 deg、最大位置误差小于 2 m。",
        "",
        "## 逐步实现",
        "",
        "1. `ServoPredictor` 分离发布命令、静态标定目标和物理角度；延迟方向由命令增量决定，时间常数和物理速率由目标误差方向决定，并精确切分步内激活事件。",
        "2. 使用非均匀预测网格，近端 10 ms、远端 20 ms；传播 9 x 14 状态灵敏度。",
        "3. 以 4 个舵机目标、5 个近端电机推力和 5 个远端电机推力构成 14 维决策量，在 100 ms 切换电机块。",
        "4. 在 40 ms、120 ms 和终端设置多点代价；短时点提高力矩权重，加入快慢块连续性和末段配平。",
        "5. 用归一化可达权威度门控舵机；采用 LM 阻尼和 1/0.5/0.25 回溯，偏好分配仅用于热启动。",
        "6. 将物理速率约束与命令斜率约束拆成独立消融项，并把基础 DA 加强为状态感知版本。",
        "",
        "## 主要结果",
        "",
        "| 方法 | 有效性 | 3D RMSE (m) | 最大误差 (m) | 姿态 RMSE (deg) | 舵机 TV | 求解均值 / P95 (ms) |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    table_keys = (
        "legacy",
        "servo_model",
        "gate",
        "final",
        "no_multirate",
        "no_gate",
        "no_delay",
        "no_physical_rate",
        "no_command_slew",
        "no_horizon",
        "basic_da",
    )
    for key in table_keys:
        item = summary[key]
        label = RUNS[key][1]
        lines.append(
            f"| {label} | {item['validity']} | "
            f"{item['position_rmse_3d_m']:.4f} | "
            f"{item['position_max_3d_m']:.4f} | "
            f"{item['attitude_rmse_norm_deg']:.2f} | "
            f"{item['servo_total_variation_normalized']:.3f} | "
            f"{item['solve_mean_ms']:.2f} / {item['solve_p95_ms']:.2f} |"
        )
    lines.extend(
        [
            "",
            "## 结论",
            "",
            f"- 最终多速率方法达到 3D RMSE `{final['position_rmse_3d_m']:.4f} m`、姿态 RMSE `{final['attitude_rmse_norm_deg']:.2f} deg`，平均求解 `{final['solve_mean_ms']:.2f} ms`。",
            f"- 相比阶段 2，位置 RMSE 从 `{gate['position_rmse_3d_m']:.4f} m` 变为 `{final['position_rmse_3d_m']:.4f} m`，姿态误差基本持平，但平均求解时间约降低一半。",
            f"- 去掉延迟模型后位置 RMSE 增加 `{delay_penalty:.1f}%`，舵机总变差从 `{final['servo_total_variation_normalized']:.2f}` 增至 `{no_delay['servo_total_variation_normalized']:.2f}`。",
            "- 去多速率、去预测时域和状态感知基础 DA 均出现物理失效，说明慢舵机延迟下必须同时保留未来可达性和短期电机补偿自由度。",
            "- 去命令斜率在本轨迹下略微改善误差，因此该项当前应视为安全约束；后续需通过噪声、突变指令和模型失配实验评估其鲁棒价值。",
            "- 自动计算的 278 ms 时域在当前开环执行器状态预测下失稳；在线默认采用验证稳定的 180 ms。要恢复更长时域，应先接入实际舵机状态反馈或在线延迟/时间常数辨识。",
            "",
            "## 默认参数",
            "",
            "- `HNUTER_DRCDA_HORIZON_S=0.18`",
            "- `HNUTER_DRCDA_MOTOR_BLOCK_SWITCH_S=0.10`",
            "- `HNUTER_DRCDA_SERVO_MOVE_WEIGHT_SCALE=10`",
            "- `HNUTER_DRCDA_SERVO_COMMAND_SLEW_SCALE=0.5`",
            "- `HNUTER_DRCDA_LATE_TRANSITION_WEIGHT_SCALE=100`",
            "- `HNUTER_DRCDA_LATE_TRIM_WEIGHT_SCALE=25`",
            "",
            "## 代码位置",
            "",
            "- `hnuter_drcda.py`：预测器、14 维轨迹灵敏度、多点目标、可达性门控、LM 求解器和基础 DA。",
            "- `hnuter_external_direct_drcda.py`：ROS 2 控制器接入、参数、标定逆映射和诊断日志。",
            "- `test_hnuter_drcda.py`：静态标定、方向模型、步内事件、有限差分灵敏度、门控和多速率退化测试。",
            "- `analyze_multirate_drcda_validation.py`：本实验统一分析与报告生成。",
            "",
        ]
    )
    return "\n".join(lines)


def inline_markdown(text: str) -> str:
    escaped = html.escape(text)
    escaped = re.sub(r"`([^`]+)`", r"<code>\1</code>", escaped)
    escaped = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", escaped)
    return escaped


def markdown_to_html(markdown: str) -> str:
    lines = markdown.splitlines()
    output: list[str] = []
    index = 0
    while index < len(lines):
        line = lines[index].strip()
        if not line:
            index += 1
            continue
        heading = re.match(r"^(#{1,4})\s+(.+)$", line)
        if heading:
            level = len(heading.group(1))
            output.append(f"<h{level}>{inline_markdown(heading.group(2))}</h{level}>")
            index += 1
            continue
        if (
            index + 1 < len(lines)
            and "|" in line
            and re.fullmatch(r"[| :\\-]+", lines[index + 1].strip())
        ):
            headers = [cell.strip() for cell in line.strip("|").split("|")]
            index += 2
            rows: list[list[str]] = []
            while index < len(lines) and "|" in lines[index]:
                rows.append(
                    [cell.strip() for cell in lines[index].strip().strip("|").split("|")]
                )
                index += 1
            output.append("<table><thead><tr>")
            output.extend(f"<th>{inline_markdown(cell)}</th>" for cell in headers)
            output.append("</tr></thead><tbody>")
            for row in rows:
                output.append("<tr>")
                output.extend(f"<td>{inline_markdown(cell)}</td>" for cell in row)
                output.append("</tr>")
            output.append("</tbody></table>")
            continue
        bullet = re.match(r"^-\s+(.+)$", line)
        if bullet:
            items = []
            while index < len(lines):
                match = re.match(r"^-\s+(.+)$", lines[index].strip())
                if not match:
                    break
                items.append(match.group(1))
                index += 1
            output.append("<ul>")
            output.extend(f"<li>{inline_markdown(item)}</li>" for item in items)
            output.append("</ul>")
            continue
        numbered = re.match(r"^\d+\.\s+(.+)$", line)
        if numbered:
            items = []
            while index < len(lines):
                match = re.match(r"^\d+\.\s+(.+)$", lines[index].strip())
                if not match:
                    break
                items.append(match.group(1))
                index += 1
            output.append("<ol>")
            output.extend(f"<li>{inline_markdown(item)}</li>" for item in items)
            output.append("</ol>")
            continue
        output.append(f"<p>{inline_markdown(line)}</p>")
        index += 1
    return "\n".join(output)


def html_report(markdown: str) -> str:
    image_sections = []
    for path in sorted(FIGURES.glob("*.png")):
        payload = base64.b64encode(path.read_bytes()).decode("ascii")
        image_sections.append(
            f'<section><h2>{html.escape(path.stem.replace("_", " "))}</h2>'
            f'<img src="data:image/png;base64,{payload}"></section>'
        )
    return f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><style>
@page {{ size: A4; margin: 15mm; }}
body {{ font-family: "Noto Sans CJK SC", sans-serif; color: #17202a; }}
article {{ font-size: 9.5pt; line-height: 1.55; }}
section {{ page-break-before: always; }}
img {{ width: 100%; max-height: 245mm; object-fit: contain; }}
h1, h2 {{ color: #17384a; }}
table {{ width: 100%; border-collapse: collapse; font-size: 7.8pt; }}
th, td {{ border: 1px solid #aebdc4; padding: 1.3mm; }}
th {{ background: #dceaf0; }}
code {{ background: #eef3f5; padding: 0.2mm 0.6mm; }}
</style></head><body>
<article>{markdown_to_html(markdown)}</article>
{''.join(image_sections)}
</body></html>"""


def main() -> None:
    for directory in (DATA, FIGURES, REPORTS):
        directory.mkdir(parents=True, exist_ok=True)

    paths = {key: find_csv(directory) for key, (directory, _) in RUNS.items()}
    summary = {key: run_metrics(path) for key, path in paths.items()}
    loaded = {
        key: load_lissajous(paths[key])
        for key in (
            "legacy",
            "servo_model",
            "gate",
            "final",
            "no_gate",
            "no_delay",
            "no_physical_rate",
            "no_command_slew",
        )
    }

    for key, path in paths.items():
        shutil.copy2(path, DATA / f"{key}.csv")

    plot_trajectory(loaded)
    plot_tracking(loaded)
    plot_ablation(summary)
    plot_servo(loaded)
    plot_horizon_failure(paths)

    metrics_path = REPORTS / "metrics.json"
    metrics_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    report = markdown_report(summary)
    (REPORTS / "validation_report_zh.md").write_text(
        report + "\n", encoding="utf-8"
    )
    (REPORTS / "validation_report_zh.html").write_text(
        html_report(report), encoding="utf-8"
    )
    print(f"Wrote {metrics_path}")


if __name__ == "__main__":
    main()
