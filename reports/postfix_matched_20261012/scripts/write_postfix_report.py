"""Build the evidence-bound report from the completed matched matrix."""
import argparse
import json
from pathlib import Path
import shutil
import pandas as pd

ROOT=Path('/home/hnuter/px4_ws_ros2_consolidated/reports/postfix_matched_20261012')
METHODS=['direct','paper_nda','full','no_preview','no_horizon','no_nearfield']
NAMES=dict(zip(METHODS,['Direct 解析','论文 NDA 适配','Full DRCDA','无未来参考','单步时域','无近期目标']))


def table(headers,rows):
    return '\n'.join(['|'+'|'.join(headers)+'|','|'+'|'.join(['---']*len(headers))+'|']+
        ['|'+'|'.join(map(str,r))+'|' for r in rows])


def main():
    p=argparse.ArgumentParser();p.add_argument('--report',type=Path,default=ROOT);root=p.parse_args().report
    metrics=pd.read_csv(root/'metrics.csv');phases=pd.read_csv(root/'phase_metrics.csv')
    manifest=json.loads((root/'run_manifest.json').read_text())
    assert len(metrics)==18 and json.loads((root/'verification.json').read_text())['complete_matrix']
    def records(s):return metrics[metrics.scenario==s].set_index('method').loc[METHODS]
    def results(s):
        return table(['方法','位置 RMS / m','位置峰值 / m','控制 SO(3) 峰值 / deg','真值 SO(3) 峰值 / deg','全程留空','原判据通过'],[
            [NAMES[m],f'{r.position_rms_m:.3f}',f'{r.position_peak_m:.3f}',f'{r.so3_peak_deg:.2f}',
             f'{r.true_so3_peak_deg:.2f}','是' if r.completed_airborne else '否','是' if r.passed else '否']
            for m,r in records(s).iterrows()])
    nominal=records('lissajous_7');stress=records('lissajous_5');attitude=records('attitude_80_180')
    gains={m:100*(1-nominal.loc['full'].position_rms_m/nominal.loc[m].position_rms_m)
        for m in ['direct','paper_nda','no_preview','no_horizon']}
    held=[]
    for m in METHODS:
        cells=[]
        for phase in ['roll_pos_hold','pitch_pos_hold','roll_neg_hold','pitch_neg_hold']:
            q=phases[(phases.scenario=='attitude_80_180')&(phases.method==m)&(phases.phase==phase)]
            if len(q) and not q.iloc[0].after_height_loss:
                r=q.iloc[0];cells.append(f'{r.so3_rms_deg:.2f} / {r.true_so3_rms_deg:.2f}')
            else:cells.append('N/A (高度丢失)')
        held.append([NAMES[m],*cells])
    safety=[]
    for _,r in metrics.iterrows():
        safety.append([r.scenario,NAMES[r.method],f'{r.task_truth_min_relative_height_m:.3f}',
            '-' if pd.isna(r.first_physical_altitude_loss_s) else f'{r.first_physical_altitude_loss_s:.2f}',
            f'{r.task_estimator_truth_so3_peak_deg:.2f}',int(r.task_quat_reset_count),int(r.elevated_landing_disarms)])
    intro=f'''# 磁场与舵机增益修复后的匹配对比实验

日期：2026-10-12。数据：本轮重新启动 PX4/Gazebo 的 **18 次 SITL 飞行**，不是历史曲线重绘。每条件仅 1 次，不提供置信区间或统计显著性结论。本轮只运行冻结后的实现，没有中途调参、改阈值或替换失败样本。

## 结论先行

- **7 s 李萨如存在局部优势**：Full 位置 RMS 为 {nominal.loc['full'].position_rms_m:.3f} m，较 Direct 和 NDA 分别低 {gains['direct']:.1f}%、{gains['paper_nda']:.1f}%；相对无未来参考、单步版本分别低 {gains['no_preview']:.1f}%、{gains['no_horizon']:.1f}%。这些是本次单次运行的描述性差值。
- **5 s 压力测试不支持 Full 全面更优**：Full 位置 RMS / 控制姿态峰值为 {stress.loc['full'].position_rms_m:.3f} m / {stress.loc['full'].so3_peak_deg:.2f}°；单步版本为 {stress.loc['no_horizon'].position_rms_m:.3f} m / {stress.loc['no_horizon'].so3_peak_deg:.2f}°。两者均未达到原位置标准。Direct、NDA、无近期目标组的失稳及高度丢失均保留。
- 大姿态 Full 位置 RMS 为 {attitude.loc['full'].position_rms_m:.3f} m；第一次正横滚去程控制姿态峰值仅 0.75°，四个峰值保持段真值 SO(3) RMS 约 1.25–1.94°。剩余超限主要在负俯仰去程/回程，控制误差峰值约 13°。原精度判据通过 {int(attitude.passed.sum())}/6，全程留空 {int(attitude.completed_airborne.sum())}/6，不能用“轨迹时钟已结束”代替成功悬停。
- **大姿态关闭未来参考反而更好**：无未来参考位置 RMS 为 {attitude.loc['no_preview'].position_rms_m:.3f} m，控制姿态峰值 {attitude.loc['no_preview'].so3_peak_deg:.2f}°。这直接反驳“当前未来参考实现对所有任务都有增益”的表述；参数/磁场修复后的改善与未来参考自身的贡献必须分开。
- 因此当前方法的可支持结论是**部分工况下改善跟踪，部分极限工况仍不足**；不能声称已经解决激进飞行、普遍优于论文完整方法，或完成实机验证。

## 固件、控制链与本轮修复

- 唯一固件目录：`/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl`。源 HEAD `{manifest['firmware']['commit']}`，运行包含磁场及着陆保护的本地修改，不应只用 HEAD 描述运行内容。构建后二进制、SDF、相关源码 SHA256 位于 `run_manifest.json`，源码及 dirty diff 在 `provenance/`。
- Gazebo `{manifest['gazebo_version']}`，ROS 2 Jazzy。此次依赖变动触发完整重新构建；因此旧运行不作为本轮重复试验。
- 磁场插件使用 ENU 地球坐标（`use_earth_frame_ned=false`），gauss 数值输出（`use_units_gauss=true`）；桥接将机体 FLU 磁场转换为 FRD：`[x,-y,-z]`。没有用 Gazebo 真值替代 EKF 控制反馈。
- 修正舵机预测的静态增益为四关节 `K+=K-=1`，使实测平衡状态 `q=u` 不再被错误预测为继续移动。保留方向时间常数和约束。**没有添加纯延迟，也没有修改物理关节为额外一阶滤波模型**：真实响应仍由 Gazebo JointPositionController、刚体/关节动力学产生；一阶模型只在分配器内部预测。
- 各次 ULog 均检查 `direct_actuator=true`，position/velocity/acceleration/attitude/body_rate/thrust_and_torque 均未启用；Offboard 下 PX4 位置、速度、姿态、角速度、内部 allocation 标志均未启用。实际输出是 `actuator_motors` 与 `actuator_servos`。
- 保留 `vehicle_thrust_setpoint` **仅作为着陆检测辅助量**，不是以力/力矩驱动 PX4 内部分配。着陆误判检查同时使用真值高度和实际 disarm 事件。

## 任务与公平性

|任务|路径/姿态|时间条件|
|---|---|---|
|3D 李萨如 7 s|振幅 x/y/z=1.2/0.8/0.30 m，频率比 2:3:1|周期 7 s + 公共 0.75 s C2 相位启停，结束后观测约 5 s|
|3D 李萨如 5 s|相同路径和幅值|周期 5 s + 相同 0.75 s 相位启停，结束后观测约 5 s|
|大姿态定点|依次 +80° roll、+180° pitch、-80° roll、-180° pitch|每方向 15 s 去程、5 s 峰值保持、15 s 回程、5 s 水平稳定；共 160 s + 末端观测|

所有任务相对本次起点，起飞后 3 m 悬停。各组共享起飞/进入/稳定过程、提前分配器接管、12.5 s 起飞保护结束时间、相位启停和积分状态保留。路径方向沿每次初始化航向，绝对 ENU 初始点与航向不是逐样本相同；空间图显示每次实际参考，并仅平移原点、不旋转对齐。每次初始参考航向见 `metrics.csv`。

共同外环：位置 Kp=[4,3.5,8]、Kd=[3.8,3.2,4]、Ki=[0.15,0.12,3]；姿态 KR=[5,5.5,14]、D=[3,3.2,5.9]、Ki=[0.03,0.03,0.02]；力矩限幅 [2.5,2.8,5] Nm；水平/垂向加速度组件限幅 8 m/s²。共同四元数误差、倾角偏航调度等其余设置见每次相同的 `effective_tuning.json`。分配残差抗饱和增益固定为 0，未给 Full 单独改参考/外环。

共同约束：前电机各 0..25 N，尾部 -6.01991..12.777084 N；推力命令变化率 [2500,2500,2500,2500,100] N/s；关节 [185,180,185,180]°；预测物理速率 10 rad/s、命令速率 50 rad/s。Direct 发布路径也施加推力/舵机变化率界限，非只有优化器受限。

共同物理 SDF：rj2/lj2 PID=50/0/5、输出界 ±1.57；rj1/lj1 PID=80/0/6、输出界 ±π；四关节阻尼 0.5、摩擦 0.1、速度上限 10 rad/s。PID 的 `cmd_min/max` 是控制器输出限制，**不是关节位置范围**。前电机升/降时间常数 0.001/0.002 s，尾电机 0.05/0.05 s。

预测器四舵机 tau+=[0.1384,0.0891,0.1408,0.0920] s，tau-=[0.1396,0.0874,0.1422,0.0868] s，纯延迟全 0；本轮没有重新做阶跃辨识，不能把参数修复等同于跨工况辨识验证。

公平性边界：Direct 是名义解析几何；NDA/DRCDA 使用 SDF 关节化几何，包含随倾转变化的力臂。共同物理模型、外环及约束相同，**内部分配模型并非完全相同**。Direct 约每 4 ms 更新，优化分配通常约 12 ms；不宣称相同计算预算/分配频率。Full 与各 DRCDA 消融共享实现，适合判断其新增机制是否有收益。

5 s 轨迹的平面 y 分量理论峰值加速度约 11.37 m/s²（未含相位启停附加项），已超过公共 8 m/s² 组件限幅；它是能力边界压力测试，不应当解释为一般低速跟踪结论。

## 对比结果

以下位置误差和“控制 SO(3)”来自实际控制器日志；“真值 SO(3)”由期望姿态与 ULog Gazebo 姿态真值独立计算。前者较小不代表真实姿态同样准确。

### 7 s 李萨如

{results('lissajous_7')}

### 5 s 李萨如压力测试

{results('lissajous_5')}

失稳之后的数据未删除，因此失稳组全程 RMS 不能作为正常飞行效率的量化提升比例。`pre_loss_position_rms_m` 和 `pre_loss_duration_s` 单列失高前数据，但由于时长不同，也不用于等时性能排名。

### 大姿态悬停

{results('attitude_80_180')}

### 四个峰值保持阶段

单元格为“控制 SO(3) RMS / 真值 SO(3) RMS”，单位 deg，每段 5 s。某阶段重叠或晚于真值高度丢失时标 N/A，不计作成功悬停证据。

{table(['方法','+80 roll','+180 pitch','-80 roll','-180 pitch'],held)}

## 判据与独立检查

原精度条件不改：位置峰值 ≤0.75 m；控制 SO(3) 峰值 ≤12°；大姿态峰值保持段合并 RMS ≤5°；估计高度 ≥0.5 m；关节反馈覆盖 ≥95%；保持 armed、无安全截断、轨迹时钟完成。本轮额外用**真值高度 ≥0.5 m**限制通过资格。

“全程留空”只说明时钟完成、保持 armed、无截断、真值未失高，**不表示跟踪准确或稳定性证明**。图中同时保留原精度状态及物理留空状态。全程真值与估计器误差不会因超过旧阈值而从图中裁去。

{table(['任务','方法','最低真值相对高度 m','首次失高 t/s','EKF/真值峰值 deg','四元数重置数','空中误判着陆停机数'],safety)}

原始 CSV 与 ULog 使用首次 armed 边沿对齐，约有一个状态/日志采样间隔的不确定性，不用于毫秒级先后因果宣称。真值相对高度以首次 armed 时为零。三种姿态误差的定义见 `METHODS.md`。

## 图册与可复现数据

- [TRO 风格图册](publication_figures/TRO_postfix_comparison.pdf)：衬线字体，8 页。
- [SR 风格图册](publication_figures/SR_postfix_comparison.pdf)：无衬线字体，8 页。
- `publication_figures/TRO/`、`SR/`：8 幅各自 PDF、可编辑 SVG、450 dpi PNG；`overview.png` 为预览。
- 图 1：7 s 三维轨迹、水平投影、位置/姿态时间序列。图 2：5 s 压力测试及真值。图 3/4：两档李萨如消融、单次运行经验 CDF、舵机命令/实测总变差。图 5/6：大姿态对比与消融。图 7：分阶段保持误差热图。图 8：18 条件点图总览。
- 每次 `data/<scenario>/repeat_1/<method>/` 保存原始 `flight.ulg`、参数快照、CSV、console、有效配置及审计 JSON；`metrics.csv`、`phase_metrics.csv` 可复算图表。`run_manifest.json` 与 `verification.json` 保存匹配性及源码哈希核验。
- 109 项回归测试通过，日志为 `regression_tests.xml`。测试只证明所测代码行为，不替代 SITL，更不等同实机验证。
- 本轮首次启动发生过并发构建冲突，起飞前已终止，单独保留于工作目录 `postfix_startup_interrupted_20261012`。不计入 18 次飞行，也不混入效果统计。
- 绘图没有平滑、伪造重复、置信区间或删去失败样本。CDF 仅是单次轨迹样本分布，不是独立试验成功概率；关节总变差不是能耗。

重绘：

```bash
/usr/bin/python3 scripts/plot_postfix_suite.py --report "$PWD"
```

重新运行真实实验（会启动仿真并自动执行任务，不用于实机）：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hnuter/px4_ws_ros2/install/setup.bash
/usr/bin/python3 scripts/run_postfix_suite.py --output /tmp/hnuter_postfix_new_run
```

仅在上述无延迟固件及源代码快照条件下复现；运行前确保没有占用相同 DDS/Gazebo 端口的实例。脚本拒绝覆盖已有 manifest。归档：`scripts/finalize_postfix_suite.py --source <新运行目录> --output <新报告目录>`；脚本依赖相邻 `analyze_revision.py`。导出的驱动仅补充输出路径/禁止覆盖检查，飞行控制实现未在矩阵中途修改。

## 仍需解决的问题

1. 5 s 单步优于 Full，大姿态无未来参考也优于 Full，提示未来参考构造、舵机/机体短时预测与目标权重之间仍需进一步因果验证。本轮不能仅凭曲线断言是哪一项根因。
2. 磁场坐标修复没有消除高动态下所有 EKF/真值偏差；估计误差与分配误差应继续分开观察。
3. 7 s 无近期目标的位置 RMS 与 Full 接近，不能说近期项总能提高精度；5 s 去掉该项失高，又提示它在本次压力工况下有安全维持作用。单次结果尚不能证明普适必要性。
4. 大姿态按方向和真值误差解释，不把消除空中误判着陆当成控制器/分配器全面收敛。
5. 本轮 NDA 是论文微分分配思想的 Hnuter 力坐标适配，不是基于电功率/电压辨识约束的完整 PDA。未做完整论文方法优越性结论。

本次没有修改实机代码，没有启动延迟固件，没有提交或推送 Git。
'''
    (root/'REPORT.md').write_text(intro)
    if Path(__file__).resolve()!=(root/'scripts'/Path(__file__).name).resolve():
        shutil.copy2(__file__,root/'scripts'/Path(__file__).name)
    print(root/'REPORT.md')


if __name__=='__main__':main()
