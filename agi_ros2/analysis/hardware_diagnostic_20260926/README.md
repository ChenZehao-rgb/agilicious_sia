# 2026-09-26 两次 hardware diagnostic 定位诊断

结论：16:18 存在可确认的静止 GNSS 漂移，后段另有连续拒收导航导致的 EKF 发散与重置；16:26 则全程没有完成 EKF 初始化。`diagnostic_only` 没有关闭融合。

## 数据和适用范围

- 原始输入：仓库 `bags/hardware_20260926_161859_702449`（247.707 s，531,593 条消息）、`bags/hardware_20260926_162634_732735`（165.356 s，278,910 条消息）。
- 操作者确认：两次均在室外开阔地段，开始静止，随后在约 2 m 范围内移动；第一包前约 120 s 静止；第二包早期解锁只是电机转动，机体未移动。
- MCAP 按内嵌 ROS 2 schema 解码，校验存在的 CRC，所有话题计数与 metadata 一致。没有外部位置真值，因此不能报告绝对定位 RMSE，也不能把移动时段的轨迹范围全部视为误差。
- 文中事件时间默认从 bag 首条记录起算的接收秒数；融合与 GNSS 数值比较使用传感器 acquisition stamp。拒收末期融合接收延迟增大，两种时间不可混用。
- 数据权威来源是这两个 bag。代码解释使用当前 checkout；bag 没有保存完整运行参数快照或已烧录固件身份，不能由本地源码证明远端二进制逐字一致。关键状态、门限触发时序与队列长度均得到 bag 独立支持。

## 静止稳定性和精度字段

第一包前 120 s 的 GNSS 可用局部位置从 2.798 s 开始，融合位置从 5.798 s 初始化后开始。下表各自以本话题首个有效位置为基准，不跨重置，也不是两个话题完全相同的起始时刻。

| 指标 | GNSS local_navigation | 有效 fused_state |
|---|---:|---:|
| 距本段首位置的最大水平偏移 | 5.43 m | 5.96 m |
| 末位置减首位置的高度变化 | −12.10 m | −7.70 m |
| 高度最大值减最小值 | 12.12 m | 7.73 m |
| 水平相对段内均值的半径 P95 | 3.00 m | 3.46 m |

这些是操作员确认静止段的漂移/重复性指标。原点本身没有真值，故不能称为绝对定位误差。前 60 s 的水平最大偏移约 1.28 m，延长到 120 s 增至约 5–6 m；短时间看起来平稳不能证明长时间定位稳定。

| 全包接收机自报精度的 P95 | 16:18 | 16:26 |
|---|---:|---:|
| 水平 | 1.137 m | 1.083 m |
| 垂直 | 2.066 m | 2.122 m |
| 速度 | 0.470 m/s | 0.476 m/s |

两包有效原始 Navigation 都是 `fix_type=3`；这是普通 3D GNSS fix，不是 RTK fixed。字段含义见 [MAVLink 定义](https://mavlink.io/en/messages/common.html#GPS_RAW_INT)。自报 accuracy 是接收机估计，不能当作真值误差或漂移上界。

第二包没有有效融合位置；全包局部 GNSS 的 XYZ 范围分别约 2.85、6.03、7.68 m，但其中包括实际搬动，不能直接当作静止精度指标。

## 为什么会飘

1. **GNSS 输入自己缓慢漂移。** 第一包确认静止时，原始局部 GNSS 已产生约 5.43 m 水平偏移和 12.10 m 高度变化。GNSS+IMU 没有额外独立位置约束，滤波平滑不能判断这种慢漂移是否为真实运动。当前高度来自 `GPS_RAW_INT.alt` 的 MSL 高度减固定初始原点，没有融合飞控气压高度或测距高度。
2. **IMU 存在静止重力残差，但不是本次初始化失败原因。** 第一包前 120 s 的完整 3 s 窗口加速度模长为 9.384–9.499 m/s²，低于项目重力 9.8066，残差约 0.308–0.423 m/s²；陀螺/加速度噪声均通过门限。用同时间戳 IMU、姿态和 fused acceleration 反解可见，EKF 加速度偏置 z 从 0 收敛到约 −0.35 至 −0.38 m/s²。它说明启动和后续传播需要估计非零偏置，不能把所有长时间 GNSS 漂移归因于 IMU。增大 gravity_tolerance 只改变接纳条件，不校准传感器。
3. **第一包后段又发生 GNSS 拒收后的惯性传播发散。** 该阶段应与前面的 GNSS 慢漂移分开诊断，详见下一节。

当前 ROS 接收 HIGHRES_IMU 时仅转换 FRD/FLU 的 Y/Z 符号，没有再次缩放加速度；坐标取反保持模长。配套固件源码 `/home/sia/betaflight` 的单位链路也未发现重复缩放，但需要已刷固件版本和标定参数才能确认硬件端具体原因。不能从这两个 bag 进一步唯一定位到天线、GNSS 接收机算法、环境或加速度计标定中的某一个部件。

## 第一包后段的确定故障链

| 接收时间 | 记录事实 |
|---|---|
| 5.798 s | EKF 初始化成功 |
| 217.171 s | 首次拒收导航，NIS=33.98，超过当前门限 24.322 |
| 219.481 s | 最后一次接受 GNSS，之后连续拒绝 59 次，总计拒绝 65 次 |
| 219.710 s | `navigation_valid=false`；最后被接受的导航超过 0.3 s，并非源 GNSS 断流 |
| 227.870 s | 重置前融合 Y≈−15.77 m，Z≈−21.91 m；按源时刻与 GNSS 比较，水平差约 19.0 m，高度差约 17.14 m |
| 227.872 s | `EKF prediction failed; reinitialization required`，reset_counter 2→3 |
| 244.796 s | 再次初始化成功 |

最后接受后到失败的**采集时间间隔**是 8.202381056 s，恰好有 4096 条 IMU，对应 `MAX_QUEUE_SIZE=4096`。拒收时后验时间不推进，IMU 队列不断增长；达到上限丢掉最早的 IMU 后，后验到当前时刻的历史不再完整，`propagatePrior()` 返回 false，节点重置。没有任何原始 IMU acquisition gap 超过 25 ms，也没有时间倒退。

此间 83 条 local_navigation 全部 `observation_valid=true`、`clock_aligned=true`、`heading_valid=true`、`fix_type=3`，源到 bag 延迟最大 77.8 ms。融合自身的延迟却最大达到 398.7 ms。每次拒收后重算更长历史可能增加耗时，但没有 CPU profile，计算耗时的因果归因仍是推断。

拒收过程中航向差大部分回到几度以内，而平移位置和速度持续分离。只有总 NIS，没有每个创新分量的完整协方差，不能精确判断位置、速度各轴分别贡献多少，也不能由此断言只改大 NIS 门限就能解决。

## 第二包为何 navigation 有数值而 fused_state 没有位置

第二包 82,398 条 FusedState 全部 `initialized=false`、`imu_ready=false`，XYZ 实际为 **0,0,0 占位值**，不是 NaN 或缺失字段。`/state` Odometry 为 0 条消息；local_navigation 有 1,546 条有效观测。

链路是：`/sensors/navigation` 原始 GNSS → `/sensors/local_navigation` 固定原点 ENU 观测 → EKF → `/fused_state`。GNSS adapter 不依赖 EKF 是否已初始化，因此可以正常输出数值。FusedState 在未初始化时也发布诊断元数据，但不产生可用融合位置。

| 接收时间 | 为什么尚未完成初始化 |
|---|---|
| 3.402 s | 才具备有效 RC、未解锁状态 |
| 7.282 s | 首条 local_navigation 到达，开始具备采集条件 |
| 7.983 s | GNSS 三维速度 0.3108 m/s >0.3，清空 IMU 初始化窗口 |
| 8.083–10.386 s | 最长连续窗口仅 2.302 s；虽有 1,151 个 IMU，但未满 3 s |
| 10.386 s | GNSS 速度 0.4820 m/s，再次清空窗口 |
| 10.922–163.562 s | `armed=true`，软件不允许初始化；电机转动但机体静止也仍不符合 `!armed` |
| 163.563–165.354 s | 最后收集仅 1.792 s、896 个 IMU，随后录制结束 |

第二包前 10 s 的 3,305 个完整 **IMU-only** 3 s 窗口全部通过物理检查，重力残差约 0.384–0.391 m/s²。这里忽略导航和 ARM 重置，仅用于证明 IMU 自身噪声/模长并非启动障碍。在线窗口仍被单帧 GNSS 速度阈值打断。

## 优先处理建议

1. 复测时保持未解锁和静止，确认 `fused_state.initialized=true`、`imu_ready=true`、`navigation_valid=true`，且 position/rtk_stamp 持续更新，再解锁。按固定秒数等待并不可靠，GNSS 静止速度噪声会重新启动计时。
2. 检查飞控加速度计标定及多姿态静止模长，解决约 0.3–0.4 m/s² 残差来源；不要继续仅放宽 gravity_tolerance。
3. 单独验证 GNSS 长时间静止重复性，并记录卫星数、DOP、C/N0、接收机状态与原始时间/高度。现有约 5 m 水平和 12 m 垂直静止漂移意味着不能按自报 1 m 精度评价实际表现。若目标显著优于该水平，需要经验证的独立位置/高度观测，单纯减小观测方差不能提供真值约束。
4. 后续代码改进应针对：单帧 GNSS 速度重置导致难以初始化的诊断/稳健性；连续导航拒收后的降级和重捕获；4096 帧历史边界前的处理。不要只扩大队列或无条件放宽 NIS，否则可能仅延迟故障或接纳坏观测。

当前 `max_*_accuracy=0` 与 `max_*_stddev=0` 的含义是飞行准入门限未配置，会让 `navigation_ready/estimator_ready=false`；它们不阻止普通 GNSS 初始化和实际融合，也不是第二包位置占位的根因。`navigation_accepted_updates` 也受精度准入控制，恒为 0 不等于没有 GNSS 更新。

## 可复查证据与复现

- `position_and_initialization.png`：两包的三轴位置、初始化和 ARM 对照。相同坐标轴；无效 fused 位置隐藏，没有把占位 0 当测量画出；曲线每 10 个融合样本抽取 1 个，统计使用全量数据。
- `analysis.ipynb`：可执行复查入口和图；`analyze.py`：稳定性、按采集时间对齐的一致性、加速度偏置重构。
- `extract.py`：从两个原始 MCAP 生成按话题分开的 CSV.gz；每包目录中 `manifest.json` 保存计数、schema 和 CRC 验证结果，`summary.json` 保存指标。
- `audit_initialization.py`、`initialization_audit.json`：完整 3 s 窗口、初始化条件和时序；独立直接计算的 31 个窗口/包与滚动算法最大差约 7.5e-10。
- `gate_audit.py`、`gate_audit.json`：拒收、消息新鲜度和 4096 帧历史证据。

基础分析依赖 Python、numpy、pandas、matplotlib、pyyaml；提取另需 mcap 和 mcap-ros2-support。Notebook 另需 nbformat、nbclient、ipykernel。

```bash
python3 agi_ros2/analysis/hardware_diagnostic_20260926/extract.py
python3 agi_ros2/analysis/hardware_diagnostic_20260926/analyze.py
python3 agi_ros2/analysis/hardware_diagnostic_20260926/audit_initialization.py
python3 agi_ros2/analysis/hardware_diagnostic_20260926/gate_audit.py
```

源码定位：`agi_ros2/src/state_fusion_node.cpp:253`（初始化）、`:332`（占位与发布）、`agi_ros2/include/agi_ros2/imu_initialization.h:36`（静止窗口）、`agi_ros2/scripts/shadow_support.py:25`（固定原点高度）、`agi_ros2/scripts/gnss_adapter.py:125`（观测方差）、`agilib/src/estimator/ekf_imu/ekf_imu.cpp:383`（历史覆盖）、`agilib/include/agilib/estimator/ekf_imu/ekf_imu.hpp:100`（4096 上限）。

本次仅新增离线分析产物，未修改飞控、融合程序或运行参数。

验证记录：四个分析脚本已执行，Python 语法检查通过；notebook 的 5 个代码单元顺序执行并通过 nbformat 结构验证，内嵌图像与已目视核对的 PNG 完全一致。表格数值已核对；当前环境没有交互式 notebook 阅读器，未验证特定 Jupyter 前端排版。`fused_initialized_whole_recording` 仅为跨重置的全包范围；连续段另存于 `fused_by_reset_counter`，不可把全包范围称作单段漂移。
