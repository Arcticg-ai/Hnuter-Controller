### attitude_60_60

| 方法 | 通过 | 位置峰值/m | SO(3)峰值/deg | 偏航速度峰值/(rad/s) |
|---|---:|---:|---:|---:|
| Direct | 0/2 | 5.519–74.037 | 178.773–179.798 | 7.436–41.138 |
| NDA (Hnuter) | 0/2 | 3.146–17.634 | 24.225–179.995 | 0.796–36.499 |
| Full DRCDA | 0/2 | 3.040–13.054 | 24.218–179.957 | 0.713–16.301 |
| DRCDA no preview | 0/2 | 2.970–9.027 | 20.949–40.262 | 0.777–0.790 |
| DRCDA one step | 0/2 | 9.032–9.672 | 20.937–43.872 | 0.801–0.861 |

![attitude_60_60 main comparison](figures/attitude_60_60_comparison.png)

![attitude_60_60 ablation](figures/attitude_60_60_ablation.png)

[局部误差放大图](figures/attitude_60_60_detail.png)
[姿态参考与实测曲线](figures/attitude_60_60_attitude.png) · [上锁审计](figures/attitude_60_60_termination.png)

### attitude_80_120

| 方法 | 通过 | 位置峰值/m | SO(3)峰值/deg | 偏航速度峰值/(rad/s) |
|---|---:|---:|---:|---:|
| Direct | 0/2 | 2.009–2.901 | 70.553–74.562 | 1.014–1.266 |
| NDA (Hnuter) | 0/2 | 23.028–24.585 | 179.609–179.929 | 12.188–25.130 |
| Full DRCDA | 0/2 | 9.940–28.615 | 179.851–179.986 | 4.177–19.300 |
| DRCDA no preview | 0/2 | 68.376–140.150 | 179.943–179.969 | 22.056–25.103 |
| DRCDA one step | 0/2 | 7.711–12.800 | 179.276–179.970 | 2.157–5.631 |

![attitude_80_120 main comparison](figures/attitude_80_120_comparison.png)

![attitude_80_120 ablation](figures/attitude_80_120_ablation.png)

[局部误差放大图](figures/attitude_80_120_detail.png)
[姿态参考与实测曲线](figures/attitude_80_120_attitude.png) · [上锁审计](figures/attitude_80_120_termination.png)

### lissajous_5

| 方法 | 通过 | 位置峰值/m | SO(3)峰值/deg | 偏航速度峰值/(rad/s) |
|---|---:|---:|---:|---:|
| Direct | 0/2 | 13.625–58.307 | 179.879–179.997 | 28.084–51.185 |
| NDA (Hnuter) | 0/2 | 21.810–24.595 | 179.502–179.616 | 19.783–22.821 |
| Full DRCDA | 0/2 | 9.002–44.618 | 179.715–179.990 | 12.128–16.148 |
| DRCDA no preview | 0/2 | 5.769–10.001 | 100.487–179.760 | 1.770–8.157 |
| DRCDA one step | 0/2 | 1.818–3.039 | 12.100–13.349 | 0.666–0.672 |

![lissajous_5 main comparison](figures/lissajous_5_comparison.png)

![lissajous_5 ablation](figures/lissajous_5_ablation.png)

[局部误差放大图](figures/lissajous_5_detail.png)
[三维全轨迹](figures/lissajous_5_3d.png) · [三维局部视图](figures/lissajous_5_3d_detail.png)

### lissajous_7

| 方法 | 通过 | 位置峰值/m | SO(3)峰值/deg | 偏航速度峰值/(rad/s) |
|---|---:|---:|---:|---:|
| Direct | 0/2 | 0.796–0.806 | 16.960–20.456 | 0.928–1.178 |
| NDA (Hnuter) | 0/2 | 8.276–24.319 | 179.809–179.999 | 4.815–22.400 |
| Full DRCDA | 0/2 | 0.492–0.493 | 14.514–18.233 | 0.402–0.449 |
| DRCDA no preview | 0/2 | 0.528–0.550 | 18.346–23.494 | 0.439–0.665 |
| DRCDA one step | 2/2 | 0.500–0.523 | 6.653–9.612 | 0.305–0.513 |

![lissajous_7 main comparison](figures/lissajous_7_comparison.png)

![lissajous_7 ablation](figures/lissajous_7_ablation.png)

[局部误差放大图](figures/lissajous_7_detail.png)
[三维全轨迹](figures/lissajous_7_3d.png) · [三维局部视图](figures/lissajous_7_3d_detail.png)
