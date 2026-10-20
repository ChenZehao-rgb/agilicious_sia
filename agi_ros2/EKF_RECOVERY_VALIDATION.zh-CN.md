# EKF 初始化与时钟恢复修改、验证方法

适用日期：2026-10-20。对应日志 `hardware_20261020_151721_924641` 和
`hardware_20261020_160210_640434` 中已确认的软件阻塞。

## 本次修改

| 问题 | 修改后的行为 |
| --- | --- |
| 导航撤销后，较旧的有效导航反复触发 EKF reset | 只有同一导航会话中更新的有效解才能触发初始化参数重置；正常融合仍可处理重排窗口内的迟到观测 |
| 本机校时跳变被归类为永久 MSP transport failure | 独立比较本机 ROS 时间与单调时钟；超过 50 ms 的偏移突变使已有证据失效，丢弃跨时钟时期的请求/响应，重新采集证据 |
| 恢复可能误用旧配置或旧开关 | 必须重新取得完整配置及新鲜 RC/STATUS，并同时确认 ARM 低、FC disarmed、AUTO 低、RC 正常、无 KILL；保留旧配置内容用于检测配置变化 |
| 真实通信故障与时钟异常混淆 | 真实 `error/timeout/transport_error` 和配置变化继续锁存，时钟恢复不清除这些故障 |
| 开机还未完成首次校时就启动飞行进程 | 所有 hardware launch 入口先等待 `timedatectl NTPSynchronized=yes`，默认最多 60 秒；失败不启动飞行组件或录包，SITL 不等待系统校时 |
| 只能看到笼统的初始化失败 | 增加最后一次 reset 原因、IMU 断档、迟到消息计数及静止初始化统计，细分 authority、速度、航向、IMU 等阻塞原因 |

硬件启动新增参数 `clock_sync_timeout:=90` 可调整等待时间；不能设为零或非有限值。
`launch_realtime.py` 的进程发现超时同步延长，避免等待校时时被提前终止。
直接手工启动单个 node 不经过 launch 校时检查。

以下策略保持原值：IMU 25 ms 断档 reset、飞行中禁止静止初始化、导航精度门限、NIS 门限、
IMU 噪声和 Q/R、观测重排延迟。**本次没有解决产生 291 ms IMU 断档的底层原因，
也没有实现飞行中丢失 IMU 后的状态恢复。**

## 新增观测项

`/fused_state.readiness_reason` 会分别报告：

- `vehicle armed`、`physical AUTO must be low`；
- authority 回调过期、receiver evidence 无效、会话/时钟不匹配、RC/STATUS 请求过期；
- 导航过期/撤销、航向无效、位置/速度非有限、导航速度超过静止门限；
- IMU 样本不足、窗口不足、陀螺零偏超限、陀螺/加速度噪声超限、重力残差超限。

`/fusion/baro/status` 原有 DiagnosticArray 中增加以下字段；即使关闭气压计也会发布：

| 字段 | 用途 |
| --- | --- |
| `reset_counter`, `last_reset_reason`, `last_reset_stamp` | 区分重启、导航会话切换、IMU 时间倒退/断档、EKF 内部失败 |
| `last_imu_gap_s`, `imu_timing_resets` | 最近触发时间重置的间隔及累计次数；不是持续更新的最大采样间隔 |
| `stale_imu_samples` | 硬件模式下因样本时间过旧/未来而丢弃的 IMU 数量 |
| `out_of_order_navigation` | 同一会话内未更新当前解的旧/重复导航数量；正常重排也会增加，不能单独判故障 |
| `imu_initialization_samples`, `imu_initialization_duration_s` | 当前初始化窗口累计情况 |
| `imu_initialization_gyro_bias_rad_s`, `imu_initialization_gyro_stddev_rad_s` | 平均角速度的模长、各轴标准差最大值 |
| `imu_initialization_acceleration_stddev_m_s2`, `imu_initialization_gravity_error_m_s2` | 各轴加速度标准差最大值、平均加速度模长与 g 的残差 |
| `initialization_navigation_speed_m_s`, `imu_initialization_reason` | 导航速度和 IMU 窗口检查结果 |

初始化统计在初始化成功后保留该窗口；不代表飞行过程中的在线 IMU 标定结果。
空窗口的标准差、重力残差为 `nan`。

`/health/status` JSON 增加 `clock`：
`epoch_resets`、`last_step_seconds`、`rejected_events`、`recovery_pending`。
若有时钟恢复等待，结合 `/msp/decoded_state.reason` 判断缺配置还是开关状态不满足。

## 软件回归

在仓库根目录执行。以下测试使用合成输入、模拟时间或 PTY，不连接实际飞控，也不修改主机时间。

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 agi_ros2/scripts/build.sh
source /opt/ros/jazzy/setup.bash
source install/agi_ros2/local_setup.bash
export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD/agi_ros2/test:$PYTHONPATH"

python3 -m unittest -v test_msp_evidence test_shadow_support test_wait_for_clock
python3 -m unittest -v \
  test_runtime_config.RuntimeProfiles.test_hardware_waits_for_clock_before_assembling_any_component \
  test_runtime_config.RuntimeProfiles.test_sitl_has_no_system_clock_gate \
  test_runtime_config.RuntimeProfiles.test_clock_gate_rejects_invalid_timeout \
  test_runtime_config.RuntimeProfiles.test_clock_process_exit_controls_actual_launch_event_flow
python3 -m unittest -v test_baro_fusion
python3 -m unittest -v \
  test_shadow_pipeline.ShadowTests.test_output_interlock_telemetry_and_readonly_parameter

g++ -std=c++17 -O1 -fno-finite-math-only \
  -Iagilib/include -Iagi_ros2/include -I/usr/include/eigen3 \
  agi_ros2/test/test_imu_initialization.cpp \
  -lgtest_main -lgtest -lpthread -o /tmp/agi_imu_initialization_tests
/tmp/agi_imu_initialization_tests
```

关键回归的预期结果：

1. 合成约 291 ms IMU 间隔，EKF 因断档重置一次；后续三条较旧有效导航不再增加 reset。
   当前有效导航恢复时可重置初始化参数一次，armed 状态仍不能初始化；disarmed 后可重新初始化。
2. 导航速度 0.31 m/s 阻塞静止初始化并明确报告原因；速度恢复后，0.2 rad/s 零偏仍阻塞；
   输入恢复静止后才能完成初始化。测试缩短了初始化窗口，硬件配置仍为 3 秒。
3. 注入本机时钟前跳 3295.33 秒或后跳，旧请求不能恢复 authority；新配置、新 RC/STATUS、
   disarmed 和 AUTO 低全部满足后，`receiver_valid` 恢复；没有自动取得控制权。
4. 真实 transport failure、跨校时配置变化仍锁存。仅输入未来/错误时钟消息，不能伪装成本机校时恢复。
5. 启动时同步成功才装配后续组件；失败以非零状态结束。测试用假的校时子进程检查真实 launch 退出事件链。

`test_runtime_config` 全量测试另有 5 个既有失败，修改前 HEAD 与本次修改均已复现：
它们假定旧的质量、角速度限制、零推力占位配置或空轨迹。
本次没有修改实际配置来满足这些旧断言。

真实 ROS 进程测试受宿主调度影响；不要与编译、压力测试并行运行。
出现断档时先保留日志，查看 `last_reset_reason` 和 `last_imu_gap_s`，不要直接放宽 25 ms 门限。
本次测试曾捕获合成发送端出现 62–79 ms 的样本跳变。测试夹具现在将循环垃圾回收推迟到
合成发送端关闭之后，避免保存大量观测消息的测试进程自行中断 IMU/虚拟飞控回复；
被测运行时子进程不受此设置影响，显式注入的 291 ms 断档也仍会触发 reset。
气压测试输入与 GNSS 频率已分开：半速模拟中的 200 ms GNSS 周期对应 400 ms 墙上时间，
若连带降低气压频率，会超过原有 250 ms 气压接收门限，混淆导航重排测试。
虚拟串口测试也改为等待实际读回完成，避免将 Python 启动/DDS 发现时间算入固定启动等待。

### 本次执行结果

| 检查 | 结果 |
| --- | --- |
| ROS2 Release 构建及安装 | 通过；有既有 CMake 最低版本弃用提示 |
| MSP/时钟/配置辅助逻辑及启动门禁 | 50 项通过 |
| C++ 静止初始化 | 4 项通过 |
| 实际融合进程与虚拟串口联调 | 26 项通过（25 项融合、1 项 MSP/输出联锁），最终串行运行约 125 秒 |
| C++ 本次修改范围的 clang-format、140 列/Tab/空白检查 | 通过 |
| Python 语法与 `git diff --check` | 通过 |

相关回归合计 **80 项通过**。另将修改前 HEAD 的融合源码和头文件编译为独立基线程序，
原有 23 项融合测试通过。初期新版本整套运行出现的输入时序问题按上文修正测试夹具后复核通过。
全量启动配置测试中的 5 项既有失败仍保留，不能称整个仓库测试全绿。
本次没有连接实际飞控、改变主机时钟或进行实机飞行。

## 拆桨台架与真实 GNSS 验证

1. 冷启动至少三次，记录首次校时和启动顺序：

   ```bash
   timedatectl show --property=NTPSynchronized --value
   journalctl -b -u systemd-timesyncd --no-pager
   agi_ros2/scripts/launch.sh mode:=hardware shadow_only:=true \
     bench_fixed_gps:=false record_bag:=true clock_sync_timeout:=90
   ```

   预期：校时确认之前没有飞行组件启动；超时会明确退出。
   若机器离线且未同步，先解决时间源问题。门禁只检查同步状态，不会主动改时钟。
   为检查“未同步时拒绝启动”，使用上面的自动测试，或在专用台架冷启动时观察；
   不要在运行中的飞行系统上执行改时命令。

2. ARM 和 AUTO 保持低位，真实 GNSS 天线置于适合接收的位置，静置至少五分钟并录包。
   如需逐条创新诊断，在一份位于原 config 目录的验证配置副本中设置
   `fusion.navigation_diagnostics: true`，用 `runtime_config:=绝对路径` 启动。
   验证后保留副本随 bag 归档，不直接改 Q/R 或精度门限。
   `bench_fixed_gps:=true` 只能检查链路，不能验证真实导航精度或漂移。

   另开终端分别观察：

   ```bash
   ros2 topic echo /fused_state --field readiness_reason
   ros2 topic echo /fusion/baro/status
   ros2 topic echo /health/status --field data
   ros2 topic echo /authority
   ```

   预期：完整初始化后 `initialized/imu_ready/navigation_ready/estimator_ready` 均满足；
   稳定阶段 `reset_counter` 不增长，`imu_timing_resets` 不增长，`clock.recovery_pending=false`；
   配置及 RC/STATUS 保持新鲜。起始阶段允许导航原点/会话建立带来的 reset。
   若不 ready，按明确原因定位；例如 GNSS 水平精度 2.3–2.5 m 仍会被现有 2 m 门限拦住，这是预期行为。

3. 台架确认 AUTO 低→高→低、RC 丢失、MSP 停止响应后的撤权行为。
   时钟变化与旧消息重放优先用软件测试复现；真实串口中断继续要求处理故障并重启相关会话，
   不应因校时逻辑自行解除。时钟恢复后的 authority 有效也不等于 estimator ready 或 output active。

4. 保存完整 bag、运行时配置、Git 差异及 timesyncd 日志。
   对 IMU 采样间隔、单调时钟接收间隔、回调延迟分别统计最大值/P99，不能只看平均 Hz。
   现有时序工具可补充分析：

   ```bash
   python3 agi_ros2/scripts/analyze_pipeline_timing.py /绝对路径/新bag \
     --output /绝对路径/时序分析目录
   ```

## 仍需实验后决定的调整

| 项目 | 先采集什么 | 再决定什么 |
| --- | --- | --- |
| IMU 加速度尺度、零偏、温漂、振动 | 多姿态静置、温度、拆桨/带载对照和原始 IMU | 校准值、静止门限、过程噪声 |
| GNSS 速度抖动与水平精度 | 真实静止 GNSS 的速度/accuracy 分布、环境和卫星状态 | 是否调整 0.3 m/s 初始化速度限制及导航精度门限 |
| NIS、Q/R 与延迟 | 逐组创新/创新协方差、拒绝率、估计误差与采样延迟 | 噪声模型和门限；不能只为减少拒绝而增大 R |
| 291 ms IMU 断档 | FC 采样时刻、串口收包时刻、回调时刻、CPU/串口负载 | 调度/串口修复；随后单独设计短时预测与飞行中恢复 |

通过台架后，再进行受控低空短悬停并逐步延长，检查上述计数、位置/速度/航向误差和分组创新。
不要在飞行中主动注入时钟跳变或 IMU 断档。
如果再次出现 `IMU sample gap exceeds 25 ms`，本版本仍会撤销 ready 并等待落地 disarmed 后重新初始化。
这是后续实验的重点，不应把这次软件修复视为断档恢复已经完成。
