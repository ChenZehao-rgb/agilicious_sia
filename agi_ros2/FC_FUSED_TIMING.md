# fc_fused 状态交付与控制调度修复（2026-10-27）

本补丁修复 ROS 控制侧的独立定时器等待、重复样本计算入口，以及输出 watchdog 晚于 FC 样本期限检查的问题。15 ms 原始状态门限、8 ms 计算预算和所有接收机/输出授权检查保持原值。**软件验证与机载实时性分开验收：没有修复后的机载对照录包，不宣称完整链路满足 15 ms，也不宣称飞行就绪。**

详细数值、日志和复现入口：[验证结果](analysis/fc_fused_timing_fix_20261027/results.md)。

## 原包证据与边界

输入只读：`/home/sia/bag/20261027test/hardware_20261027_091826_461529/hardware_20261027_091826_461529_0.mcap`。
ROS 基线 HEAD 为 `87b0f5e20f6f256fa5b10a519cbdf4cc10f9c615`；开始时工作区干净。固件只读 HEAD 为 `6b92b586eeb53b2783c58348810b2b9af091e25b`。
本机是 ROS 2 Humble、rclcpp 16.0.21；这不证明机载 ROS 版本或机载二进制与本地源码一致。

完整 CRC 解码再次得到 148,476 条、102.451026522 s，与原 `decoded.pkl` 逐条相同。有效计算窗口内 6,231 个命令按 `(clock_id, control_session_start, sequence)` 去重，再按 `(clock_id, fc_session, generation, sample_time, publish_time)` 与状态一对一关联；不是按 MCAP 到达顺序猜测。旧 ComputationStatus 没有序号，因此先验证其 header 唯一再用于辅助关联。全包有一条启动期 ComputationStatus 没有对应的录制命令；窗口内全部匹配。新消息已补充直接关联字段。

窗口为 +31.341761369～+93.640569742 s：5,459 次有效计算，772 次未进入计算的 NaN（12.3897%），54 段，最长约 0.931447 s。分类重新核对为 476/247/49，合计 525 次在控制开始前已有更新的新鲜适配器发布，但不能据此称 DDS 已交付。全部 NaN 的 solve_seconds 为 NaN，样本检查年龄 15.002548～17.854962 ms。实际计算中位数 1.000769 ms，最大 2.127694 ms，没有超过 8 ms。

另有 **667 次有限结果在决策结束时已超过 15 ms**。`controller_success`/`mpc_success` 都是 `command_valid` 的别名；`solve_seconds` 包含 odometry/reference/pipeline，不是纯 acados QP 时间。有限结果可保留用于诊断，但不能获得输出许可。

首个间歇 NaN：控制 session_start=76.628394377，sequence=3222，generation=3119。同一 host monotonic 下：

| 事件 | 时间 s |
|---|---:|
| 旧样本 | 108.97795736285993 |
| 旧样本适配发布 | 108.983113185 |
| 旧样本控制回调 | 108.983389519 |
| 新样本适配发布（fc_time_usec=120576918） | 108.993806011 |
| 控制开始 | 108.994073923 |
| 旧样本检查 | 108.994089431 |
| 新样本控制回调（由后续同样本/代次命令回溯） | 108.994358308 |

新发布早于 tick 0.267912 ms，回调晚于 tick 0.284385 ms，旧样本年龄 16.132068 ms。确认的是“已发布的新帧尚未进入控制缓存”；DDS 内部、executor 就绪集合、Linux 抢占各自耗时没有独立测量，不能唯一归因于 FIFO5 辅助线程饥饿。

本包 MSP 错误事件为零，无 MSP200，permit_override/override_active 全为假。物理 AUTO 高 +83.653748792～+88.847074366 s，上升沿 warm_cycles 约 14，未满足 50 次；没有接管证据。

## 时钟与时间预算

所有延迟使用消息内同一 host monotonic 字段。FC `time_usec` 是估计快照 epoch，经 TIMESYNC 映射；不是物理 IMU 曝光/GPS PVT epoch。ROS wall stamp 单独接受检查，不能代替原始样本时间。MCAP log time 只定义录包相对位置。

下面的实际 P50/P95/P99/max 由 `baseline_audit.json` 保存；详细数值表及合成 A/B 在同目录 `results.md`。每阶段分位数不能相加当作端到端 P99。

| 阶段 | 可测量性/解释 |
|---|---|
| 物理传感器→FC 估计 | 本包不可独立分解 |
| FC 估计快照→MAVLink 入队/线发送 | 无逐帧发送 timestamp，不能独立分解 |
| FC 估计→ROS 接收 | 全部 6,275 个有效原始状态 P50 5.236307 ms，P99 6.827243 ms，max 7.342992 ms；包含映射误差、FC 发送等待、UART/接收路径 |
| ROS 接收→适配发布 | 有原始接收与发布字段；有效发布独立统计，失效事件重新发布时间不当作传输延迟 |
| 适配发布→控制回调 | 可测联合 DDS/执行器/OS 延迟，不能再拆分 |
| 控制回调→控制开始 | 旧定时器相位等待可测；窗口缓存消费 P50 5.009626 ms、P99 9.769889 ms、max 11.123649 ms |
| 控制开始→计算结束/发布 | 原包有 decision evidence.now、solve_seconds、published；新消息另加明确 control_end 和前后年龄 |
| 命令发布→输出回调→输出检查 | 原包 6,231 个序号全部匹配；发布→回调 P50 0.437821 ms、max 0.972852 ms |
| 输出检查→MSP200/UART→FC 应用 | 本包从未输出，不能评价；PTY 不代表真实串口 |

原始有效 FC 样本间隔 P50 10.001 ms、P99 10.039 ms、max 10.077 ms。仅典型接收年龄 5.24 ms + 独立控制相位等待近 10 ms 已可能超 15 ms；这是预算分析，不是 WCET 证明。

## 实现选择

- `control_main.cpp` 显式使用 SingleThreadedExecutor。构造、所有回调、HardwarePilot、参考和授权 FSM 都由主线程拥有，不添加第二个计算线程，不改变 FIFO 优先级。
- 仅 `mode=hardware && navigation_policy=FcFused` 使用新调度；ros_ekf/SITL 保留原 10 ms/ROS clock timer 路径。
- `FcControlSchedule` 先校验发布顺序、session、FC reset、validity generation、FC time 和映射样本时间。重复样本不更新缓存、不计算、不推进预热；乱序新发布构成故障，迟到的旧发布/旧 generation 丢弃。generation 变化本身不能让同一样本再次计算。重启允许新 session/reset 的时间基准，但先撤销原授权。
- 正常新样本经准入后立即由同一线程调度。距离上次计算入口不足 8 ms 的突发只保留最新样本，由 1 ms service timer 取走；不排队逐帧补算，不重复旧命令。8 ms 是**防突发的最短入口间隔**，不是把 MPC dt 改成 8 ms；正常 100 Hz 输入平均仍约 100 Hz，理论入口上限为 125 Hz。短时 8～12 ms 抖动和遗漏周期对闭环稳定性的影响仍需机载/控制模型验证。
- MPC 模型 `dt=0.01`、Q/R、质量、推力映射全部不变。FC 路径直接读取当前 ROS 时间，以正常门限拒绝未来证据，避免 `alignedRosTime` 在唯一控制线程上睡眠等待；其他模式保持原逻辑。
- 独立 1 ms control watchdog 在无新样本时仍构造并检查原始证据，健康时不调用 pipeline，不消耗预热。首次故障立即发布 NaN、清预热/参考/授权；持续同一故障保留约 100 Hz NaN 心跳。适配器原 2 ms watchdog 和 generation 失效通知保持不变。
- HardwarePilot 增加“只检查、不计算”入口参数，默认值保留现有调用行为；FC 重复/回退样本不能计算。generation 不再重置样本去重水位，session/FC reset 才重置。计算结束再检查 FC 新鲜度，包含 shadow 参考的提交/保持；超时有限结果仍可见，许可为假，预热清零。
- 输出节点原 watchdog 只在命令年龄到 25 ms 等条件下重新 processOutput，可能晚于 FC 15 ms 样本期限。FC 路径现在每 1 ms 独立重新检查命令与最新状态的原始证据，即使失效通知被 QoS depth 1 覆盖或控制进程停止也可撤权。仍然只有新命令能发 MSP200，watchdog 不重发旧命令。
- 新增诊断：ComputationStatus 关联 session/sequence/generation，记录计算是否被请求、前后样本年龄、忽略/合并计数、预热重置；ControlCommand 记录结束时刻；OutputStatus 记录最近一次 gate 检查时刻、对应命令和样本年龄。诊断时间不参与刷新安全证据。消息 schema 有增量变化，需要同一工作空间重建所有生产/消费节点；旧 MCAP 使用其内嵌 schema 解码。

选择此方案是因为证据已定位独立控制相位等待和旧缓存消费，而没有证明要改变 DDS 优先级或线程模型。保留默认串行所有权减少并发状态机风险。更彻底的 wait-set/节点内直连可以继续减少交付开销，但本补丁不假设它们能消除 FC/UART 年龄。

ROS 2 文档说明 `spin(node)` 使用单线程 executor，默认回调组共享，wait-set 只提供队列是否就绪的信息，不能据此断言发布先后等于回调顺序：[Humble executor 文档源码](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Executors.rst)。本机行为不替代机载版本与实际线程策略证据。

## 固件只读审查

`tasks.c` 中 ALTITUDE 使用 `TASK_ALTITUDE_RATE_HZ`（`position.h` 为 100）。`position.c` 调用 `positionEstimatorUpdate()`，后者更新完成后生成 snapshot。MAVLink 遥测任务在相应条件下被重调度为 1000 Hz 检查；`mavlink.c::mavlinkSendOdometry()` 取最新 snapshot，以 100 Hz 节流、sequence 去重、老化/拥塞丢弃规则发送。估计更新与发送是不同阶段，平均 100 Hz 不能证明端到端截止时间。

`launch_realtime.py` 主线程接收30/适配25/输出20/控制15，辅助线程 FIFO5 的源代码已核对，但原包没有记录实际线程策略，也没有逐线程调度 trace。脚本不调度 MCU、不锁定控制相位。本补丁没有修改它或固件。

当前证据不足以选定必须改哪段固件。后续若要缩小 FC 估计→接收年龄，应先取得每个 snapshot sequence 的生成、telemetry 尝试、串口入队/完成时刻，再与接收端关联。候选观测位置是 `position_estimator.c` snapshot 完成处与 `mavlink.c` send/serial write 处；先做 SITL/模拟拥塞与独立时钟/逻辑分析仪验证，再另行评审固件补丁。不能直接靠改 FC 估计周期或提高遥测优先级声称解决。

## 回归与复现

新跟踪文件位于 `agi_ros2/tests/`；已有 `agi_ros2/test/` 是本工作区原有的测试夹具，运行器复用其 ROS/PTY harness。若复制到缺少该目录的裁剪 checkout，需要一并提供已有夹具；新原生 C++ 调度测试可独立构建。

```bash
# 仅本机隔离构建、localhost DDS、PTY；不运行 launch_realtime.py。
bash agi_ros2/tests/run_fc_timing_tests.sh
```

GTest 新目标由 `AGI_ROS2_TIMING_TESTS=ON` 显式开启，不给正常生产构建新增必需测试依赖。运行器使用正常 `build.sh`，显式 `--cmake-force-configure` 与 `build/fc_timing_install`；不要把构建测试目标误报为已运行测试。结果、命令、源码/输入哈希和格式检查保存在 `agi_ros2/analysis/fc_fused_timing_fix_20261027/`。

核心验收包含：精确首帧事件链、100×3×2 相位/年龄/计算耗时组合、50 次独立计算→健康 AUTO 低→高、失效/停流/重复/回退/重启/未来时间、失效通知丢失后 generation 变化、ARM/KILL、保持高不自动恢复、计算结束过期，以及无失效消息时输出 PTY 独立撤权。完整的既有 FC 影子链另含 MAVLink/MSP 两个 PTY、50 Hz estimator health、协议/协方差/会话/时钟故障，并验证 ros_ekf 与 SITL 路径。

`fc_timing_budget.py` 是独立离线时间预算模型：接收年龄 3.74/5.24/7.34 ms、计算 1/2.13 ms、100 个相位，假设适配 0.2 ms、联合交付 0.4 ms。输出实际计数而不是把假设当测量。`compare_fc_delivery.py` 则顺序运行本地旧控制源码编译的 timer baseline 与新 executable，用相同年龄档/相位偏置；仅计算周期的延迟与安全通知分开统计。二者都不能复现原机载 DDS/内核排队。

## 剩余缺口及机载验证

新触发方式能让新样本及时计算，但**不能让旧样本在下一个样本送达前继续有效**。100 Hz 周期下，典型 5.24 ms 接收年龄意味着下一帧接收前已有约 0.24 ms 原始租约空窗；7.34 ms 时约 2.34 ms，尚未包含适配/交付。独立 watchdog 会继续撤权、清参考/预热；A/B 中看到更多真实故障通知不应被当成需要降低检查频率的理由。

连续无空窗至少需要逐对满足 `next_delivery_time <= previous_sample_time + 15ms`；有效新命令还必须在其自身样本截止前完成计算并经过输出检查。需要改善的可能是发送相位、UART/接收延迟、DDS/OS 延迟或多者，当前证据尚不能独立分配原因。任何 1 ms 定时器都不是硬实时期限保证：控制计算、同线程阻塞、OS 抢占和输出 MSP 遥测占用都可能延迟它。独立输出进程和 FC 端 failsafe 仍不可替代。

后续步骤仅供另行授权执行，本任务未执行：

1. 保存机载 OS/ROS/RMW 版本、CPU 配置、每线程策略/优先级与二进制/YAML 哈希；确认新消息 schema 的全部节点一致。记录实际 `chrt -p TID`/线程清单，不能只记录启动命令。
2. 先 `shadow_only=true` 室外静止对照录包，记录 FC 原始状态/health、适配有效及失效事件、authority/health、control/computation/output timing 与状态。保持配置与门限一致；比较逐段 P50/P95/P99/max、样本租约空窗、计算前后截止违约、generation/reset、预热、失效原因和控制间隔。
3. 确认无真实失效且完成 50 个独立有效样本后，再测试健康 AUTO 低→高。注入停流/重复/会话重置；保持 AUTO 高不得恢复参考或输出，需重新预热与低→高。
4. 拆桨且另获授权后验证真实输出撤权、MSP write deadline、原始样本到最终写入年龄、FC override timeout。测量控制进程停止与适配失效通知丢失时的独立撤权。用 FC/逻辑分析仪证据确认实际串口与控制接管，不以 ROS permit 位代替。
5. 若尾部预算仍不够，先补齐 FC send 与 host scheduler trace，再评审对应层修复；不放宽 15 ms/8 ms、不伪造样本、不修改控制参数来美化 NaN 数。

验证边界补充：时钟故障测试使用源时间戳/映射 epoch 注入与 SITL /clock 回退，没有修改工作站系统时间；真实机载 wall-clock 跳变及长期 CPU/内存/IRQ 压力仍需影子录包验证。
