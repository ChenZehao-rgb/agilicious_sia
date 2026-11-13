# Betaflight FC fused state → Agilicious ROS 2

以下基线记录对应 2026-10-26；2026-11-01 的两仓库扩展见“FC 地理原点与在线目标”节。

2026-10-26，本地软件验证；没有连接物理 UART、解锁、飞行、刷写、commit 或 push。

## 2026-11-13：硬件授权与电池遥测

硬件 `mavlink_sensor_node` 在 `fc_fused` 与 `ros_ekf` 下均请求 RC_CHANNELS_RAW、HEARTBEAT、
SYS_STATUS 各 25 Hz 和 BATTERY_STATUS 2 Hz，发布 `/sensors/fc/telemetry`。
`msp_evidence.py` 保留可执行文件名，节点名为 `hardware_evidence`，是硬件 `/authority` 和 `/health` 的唯一发布者。
CH5 请求解锁、CH8 请求 AUTO，有效区间 `[1875,2100)`；实际 armed 只取 HEARTBEAT，CH9 不读取。
输出同时要求 CH5 请求和实际 armed；取消所有硬件 FC 配置、通道映射、模式/profile 和 override 参数读回核验。
`Health.config_verified=false` 表示未核验，本地硬件策略不再要求该字段，不影响 SITL 原有策略。

RC 源时间由 FC `time_boot_ms` 通过 TIMESYNC 映射；其余三类消息仅有本地接收时间。
快照的发布 header 不代表新观测；重复发布、健康汇总和地面站转发均不刷新各分量年龄。
RC/HEARTBEAT/SYS_STATUS 独立 500 ms，电池 1.5 s；故障和新会话后需要新的有效 AUTO 低→高。
HEARTBEAT 的 CRITICAL 可报告已解锁时 failsafe，但这些消息不能完整判断未解锁 RXLOSS。
SYS_STATUS 传感器位只用于传感器可用性摘要，不证明校准完成。未知电池保持 NaN；默认 100% 不证明电池存在。

正常、shadow 和 diagnostic 硬件启动均不产生 MSP 查询。`msp.launch.py` 只排空串口接收，不产生授权。
输出节点只发送 MSP_SET_RAW_RC，持续排空 ACK 并记录写入/负 ACK/断链故障。独立显式 bench 工具保留原行为。
以下章节是历史协议/测试记录；涉及 MSP 授权和配置回读的旧描述由本节替代。

### 本次本地软件验证

- ROS 2 Release 构建通过（Gazebo 关闭；FC/timing 测试目标开启）。未打开真实 UART，未操作真实电机。
- 新硬件双 PTY 端到端 5/5：解锁请求/实际 armed、AUTO 退出、消息中断、RC 冻结、电池丢失、
  FC 重启、TIMESYNC 中断、负 ACK、断链，以及 MSP 发送帧只有 code 200、单一 authority/health 发布者。
- 解码/健康/电池单元 18/18；monitor 无查询/ACK 排空 3/3；新增配置/启动检查 2/2；
  ROS EKF 硬件回归 3/3；直接输出/UDP/推力映射回归 5/5；shadow 2/2；地图/遥测转发 17/17；
  遥测解码及全话题录包/摘要 2/2。独立 bench 和 C++ MSP 写入期限/部分写入/errno 测试通过。
- 维护中的 `fc_control_timing_test`（含新硬件策略与 bridge 禁止查询用例）通过。
- 旧 MAVLink 套件 24 项中 4 项失败，用 HEAD 接收器在 `/tmp` 重建后全部复现：GPS 无效/航向撤销、
  气压时序断言和旧启动假设。旧配置套件 29 项中 4 项失败，用 HEAD 配置复现，涉及已变更的质量/限幅与零占位假设。
- 可选本地 FC 套件仍有旧方差幅值上限断言失败；HEAD `fcFailure` 也接受对应有限数值，未在本次改动估计器策略。
  额外气压集成测试仍预期 5 维诊断而收到 4 维；未修改气压融合逻辑。
- `git diff --check`、修改行 140 列（Tab=8）检查及 Python/JS 语法检查通过。
  环境没有 clang-format 和 Playwright，未宣称自动格式检查或浏览器测试通过。

复测主入口：先 source ROS 和 `install/agi_ros2/local_setup.bash`，再运行
`python3 agi_ros2/test/test_hardware_mavlink_override.py`、
`python3 agi_ros2/test/test_msp_evidence.py`、`python3 agi_ros2/test/test_msp_poll_scheduler.py`。
ROS 进程测试需要本机 DDS/UDP 权限，测试串口始终由 `pty.openpty()` 创建。

## 范围与来源

Agilicious 分支 `betaflight-bridge`，修改前 HEAD
`820c045e2f01d6d1645b8942c076d3933788ed9d`。原有 `.gitignore` 中的
`experiments/` 修改完整保留，旧 EKF 源码、测试和实验记录保留。

对照了本地 `/home/sia/betaflight/docs/development/mavlink_odometry_protocol.md`
及 `src/main/telemetry/mavlink_odometry.c` 的实际编码实现。该仓库当前为
`mavlink_odometry`，HEAD `6dbc4218fd6bc33bf16ea32c670304d4f89321d5`
（2026.6.1 基线，协议实现包含在未提交工作区中）。没有修改该仓库。
本次 ROS 测试使用确定性合成协议数据，没有把另一仓库文档中的 SITL/固件测试当作
本次 ROS 或实机验证。

## 数据链路与启动

```text
MAVLink2 ODOMETRY 331 / 100 Hz
  → mavlink_sensor_node（串口、来源、TIMESYNC、去重、坐标转换）
  → /sensors/fc/estimated_state : FcEstimatedState
  → fc_state_adapter_node（格式、会话/重置、时效、健康和方差检查）
  → /fused_state : FusedState → control_node → HardwarePilot → 原 MPC / GEO
  → ControlCommand（携带不可刷新的原始 FC 时间证据）
  → command_output_node → 原 MSP bridge

MAVLink2 V2_EXTENSION 248 / 50 Hz，message_type 0xBF01
  → /sensors/fc/estimator_status : FcEstimatorStatus → 同一个适配器

适配器同时输出 /state : nav_msgs/Odometry。
MAVLink RC/HEARTBEAT/SYS_STATUS/BATTERY_STATUS → msp_evidence.py → Authority / Health；
OutputStatus 独立提供控制输出、映射与传输证据。
```

`hardware.yaml` 默认 `state.source: fc_fused`，仍为 `shadow_only: true`。
`mavlink.device: /dev/ttyAMA2`、921600、`odometry_rate_hz: 100`、
`estimator_status_rate_hz: 50`。另按 `gps_rate_hz`（默认 10 Hz，0 禁用）请求
`GLOBAL_POSITION_INT`，仅用于原始录包和地图显示，不输入控制状态估计。
不请求旧 IMU、GPS_RAW_INT、气压或姿态消息；其余旧配置项供 `state.source: ros_ekf` 使用。

`/sensors/gps/global_position : agi_ros2/msg/GlobalPositionInt` 保留消息 33 全部字段和原单位，
附带映射后的 ROS 时间戳与接收会话。时间为 FC `time_boot_ms`，不是接收机 PVT 采样时间。
GPS 未定位也发布原始数据；默认 `ros2 bag record --all --include-hidden-topics` 自动记录。
消息 33 本身没有 fix 标志；地图借助同会话的新鲜 FC flags 确认定位，参见
[地图说明](MAP_UI.zh-CN.md#未解锁时的-gps-位置)。

完整新模式只有五个业务节点：mavlink_sensor_node、fc_state_adapter_node、control_node、
command_output_node、msp_evidence.py。启动图不包含 state_fusion_node、gnss_adapter.py 或旧气压 TF。
记录器和系统时钟启动检查仍保留。SITL 保持旧路径。

实时启动器识别四个 C++ 进程，主线程优先级依次为 30 / 25 / 15 / 20；
FC 适配器继承旧融合节点的 25，支持线程仍为 5。它拒绝已有这些节点、重复节点、
错误估计器进程、退出或重启；只清理自己启动的进程组。适配器还拒绝冲突的控制状态发布者。
使用普通 launch 前仍应停止旧运行实例，不能并行运行两套状态发布器。

供后续人工硬件影子验证的命令（本次没有对物理设备执行）：

```sh
./agi_ros2/scripts/launch.sh mode:=hardware shadow_only:=true
# 保留旧融合兼容路径：
./agi_ros2/scripts/launch.sh mode:=hardware state_source:=ros_ekf shadow_only:=true
# 如需实时调度，使用 launch_realtime.py，保持 shadow_only:=true。
```

## 固定 V1 协议

ODOMETRY 必须为 MAVLink2、LOCAL_NED(1) / BODY_FRD(12)、AUTOPILOT(8)。
保留 FC uint64 启动微秒、reset_counter 的低 8 位、quality 和原始 21 元素协方差数组。
拒绝错误来源、无效枚举、非有限位置/速度/角速度、非单位四元数及 quality 超出 [-1,100] 范围。
2026-10-27 起，ROS 接受并保留 quality=-1，由其余独立检查决定状态能否用于控制，无需修改飞控固件。
四元数平方范数容差 1e-3；接受后只消除该浮点范数误差。
quality=0 在该固件中表示没有校准的质量评分，不能单独证明有效。
FC 路径的 heading_valid 和 mag_heading_fresh 只作为诊断信息，持续失效也不会单独清零 warm 或撤销接管。
XY/Z、融合启用、快照存在、数值、时间、方差、会话及接收机/输出检查仍生效；旧 ROS EKF 路径仍要求航向有效。
未知整块协方差（首元素 NaN）保留未知，有限协方差检查数值及对角线。

`BF_ESTIMATOR_HEALTH_V1` 是 V2_EXTENSION 私有类型 `0xBF01`，目标 network/system/component 都为 0。
249 字节 payload 前 68 字节有效，余下为零；MAVLink2 的尾零截短由生成解码器补回。
内部版本必须为 1，内部长度必须为 68。逐字节小端解码，不使用未对齐结构体强转。

| 偏移 | 类型 | 含义 |
|---|---|---|
| 0 / 1 | uint8 / uint8 | version=1 / payload_length=68 |
| 2 | uint16 | flags |
| 4 / 8 | uint32 / uint32 | state_sequence / 完整 reset_counter |
| 12 | uint64 | state_time_usec，FC 估计更新 epoch |
| 20 / 24 | uint32 / uint32 | XY / Z 已接受观测年龄，微秒 |
| 28 / 32 | float32 / float32 | trust_xy / trust_z，不是位置精度 |
| 36 | uint32 | 此次更新使用的 source_flags |
| 40 / 41 | uint8 / uint8[3] | gps_fix_type / reserved=0 |
| 44 | float32[3] | ENU 位置方差，m² |
| 56 | float32[3] | ENU 速度方差，m²/s² |

flags bit0–7：xy_valid、z_valid、heading_valid、mag_heading_fresh、armed、gps_fix、
xy_fusion_enabled、state_snapshot_present。其余位必须为 0。
source_flags bit0–4：GPS XY、GPS Z、barometer Z、rangefinder Z、optical-flow XY；其余位必须为 0。
fix_type 只接受 V1 的 0/1/3；不推断 RTK。未知观测年龄 UINT32_MAX 不会被视作新鲜。
NaN 方差可作为未知解码，但不能通过控制安全检查；Inf/负方差和超范围 trust 被拒绝。
无 snapshot 的 NaN trust 是协议允许的未知；有 snapshot 时要求有限的 [0,1] trust，
但不使用 trust 推导任何协方差、accuracy 或权重。

## FC 地理原点与在线目标（2026-11-01）

新增 `BF_GEOGRAPHIC_ORIGIN_V1`，V2_EXTENSION 私有类型 `0xBF02`，目标地址均为 0。
0xBF01 health 的 68 字节格式不变。接收器先按 message_type 分派，未知类型忽略；
原点解析失败只清除原点有效性，不调用 invalidateFc，不刷新状态、health 或 TIMESYNC。
独立原点载荷全部小端，249 字节数组的前 32 字节如下，余下全零；MAVLink2 尾零由解码器补齐。

| 偏移 | 类型 | 含义 |
|---|---|---|
| 0 / 1 | uint8 / uint8 | version=1 / payload_length=32 |
| 2 | uint16 | bit0 XY 基准有效，bit1 GPS Z 基准有效，其余零 |
| 4 | uint32 | 完整 FC resetCounter |
| 8 | uint64 | 已完成估计快照 timeUs，FC boot 微秒 |
| 16 / 20 | int32 / int32 | armLocationGps 纬度/经度，degE7 |
| 24 | float32 | 该次更新实际 getGpsCosLat() |
| 28 | float32 | 独立 gpsAltOffsetCm，MSL cm |

固件快照一次捕获上述字段，发送时不拼接实时 GPS/home。固定 1 Hz 重发，变化后在空闲调度槽尽快发送。
ID 248 调频仍只改变 health 周期；禁用 248 同时禁用 health 和原点。没有第二条同 ID stream entry。
ODOMETRY 100 Hz 优先级和 health 50 Hz/5 ms 错峰保留。

接收后发布 transient_local/depth 1 的 `/navigation/origin`：frame=odom，session_id=receiver source_session，
latitude/longitude 为 degree，altitude 为真正 GPS Z offset 的 MSL m，altitude_reference=msl；
另带 xy_valid、z_valid、longitude_scale、fc_reset_counter、fc_time_usec。
header.stamp 只记录接收时刻，不充当状态时间证据。旧 OriginBuilder 发布端补齐 XY/Z 有效标志，
其 FC 时间/身份保持默认，不可用于本接口。

ControlNode 仅在 `state_source=fc_fused`、当前状态有效、session/完整 FC reset 相等，且
origin.fc_time_usec <= state.fc_time_usec 时转换。`FusedState.reset_counter` 是 adapter generation，
不能代替 `fc_reset_counter`。迟到旧原点不覆盖新映射；重连、状态失效和 reset 沿用现有撤权/清理路径，
清掉 pending。低频元数据不套用 15 ms 门限，缺失或坏原点不增加全局授权门槛，本地目标继续工作。
AUTO low/high 不更换地理基准。

`/go_to_geographic_position` 使用 `agi_ros2/msg/GeographicGoal` 的三个 float64 字段：
latitude、longitude（WGS84 degree），altitude_msl（绝对 MSL m）。不接受椭球高、AGL 或相对高度。
原生投影 K 来自 FC 的 float `EARTH_ANGLE_TO_CM` 与 `GPS_DEGREES_DIVIDER`，当前精确值为
111319.49424743652 m/degree；E=wrap(lon-lon0)*K*longitude_scale，N=(lat-lat0)*K，
U=altitude_msl-origin.altitude。经度先在 double 中相减再跨日界线处理，无额外地理库、距离限制或旋转。

转换与共同目标受理都在 ControlNode 单一 owner 线程完成，使用同一个 pending slot、围栏、稳定悬停、
AUTO/输出故障检查和 HardwarePilot/CapturedReference 轨迹链，不经过裸 PointStamped 异步转发。
日志 `geographic input` 给出输入与原点/session/reset，`geographic converted ... pending=true` 表示入槽，
最终仍以 tick 的 `go_to_position accepted` 为准。转向后直线平移、竖直/零位移保持航向、到点悬停、
同一 AUTO 接收下一点、不抢占/不排队、CSV 和 shadow_only 行为保持。

本地固件硬件默认 GPS provider=UBLOX，解析 hMSL；SITL provider=GPS_VIRTUAL，来自 Gazebo 地理位置。
FC 默认 altitude_source=DEFAULT（允许 GPS Z），不是由 ROS altitude_source 参数决定。
未连接硬件读取持久化 CLI；如果实际仅用气压/测距、没有 GPS Z offset，地理目标解释性拒绝，
不改融合配置。高度对齐只保证当前 FC 坐标定义一致，不承诺 GNSS 海拔或融合绝对精度。

### 地理目标的本次仿真证据

2026-11-01，本地隔离 SITL 的完整记录保存在
`agi_ros2/analysis/fc_geographic_20261101/geographic_sitl/`（本地 analysis 按既有规则不入 Git）。
`result.json` 包含原点、发布命令、参考转向、每点持续 15 s 悬停、真值/融合误差及频率；
`flight.log` 区分转换入槽与 tick 接受，`bag/` 保存实际 ROS 流。

该次 FC 原点：latitude=40.4635，longitude=-79.51825，MSL offset=0.19 m，
longitude_scale=0.7608195543289185，receiver session=`14213.348684`，完整 reset=3。
这是仿真收到的原点，不从 Gazebo 世界零点推断。隔离 EEPROM CLI 回读确认
GPS provider=VIRTUAL、altitude_source=DEFAULT、altitude_prefer_baro=100，见 `sitl_config_readback/readback.txt`。

仅对上述仿真原点，对应发布命令如下；新运行先核对 `/navigation/origin`，或运行 README 的
`--geographic` 测试由新原点重新生成。不能将这些坐标作为实机目标。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 \
ros2 topic pub --once -w 2 /go_to_geographic_position agi_ros2/msg/GeographicGoal \
  '{latitude: 40.46351, longitude: -79.5182, altitude_msl: 2.19}'
# 预期绝对 ENU = (4.234702400073, 1.113194942474, 2) m

# 第一目标稳定后，同一 AUTO 内发送第二目标：
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 \
ros2 topic pub --once -w 2 /go_to_geographic_position agi_ros2/msg/GeographicGoal \
  '{latitude: 40.46355, longitude: -79.51827, altitude_msl: 2.19}'
# 预期绝对 ENU = (-1.693880960029, 5.565974712372, 2) m
```

`-w 2` 与已有本地测试相同，等待控制器和录包订阅者；只发现录包节点时的单次发布不能证明控制端收到。
首次测试因此未进入受理，修正测试命令后重跑通过。未放宽控制阈值。
两点各稳定 15 s，全程 AUTO 70.43 s；末尾 3 s 的真值最大位置误差分别为 0.0594/0.0688 m，
最大航向误差 0.268/0.187 degree。参考在转向结束前保持起点，yaw 速率不超过 0.5 rad/s。
state 接收频率 96.7467 Hz、health 50.0304 Hz，最大状态年龄 7.581 ms，最大求解时间 1.850 ms。
原有 FC 本地 `--yaw-chain` 回归也通过：正负 yaw 脉冲与三个本地目标 `(5,0,2)`、`(5,5,2)`、`(0,0,1)`，
同一 AUTO 133.65 s，三个目标均稳定 15 s，真值最大位置误差分别为 0.0416/0.0282/0.0308 m。
FC/真值最大航向差 0.932 degree；state/health 接收频率 97.1107/50.0159 Hz，最大状态年龄 8.026 ms。
记录在同级 `local_sitl/`。这两次 SITL 均保持原时效、位置、速度、航向和求解阈值。
固件确定性调度测试仍精确验证 100 Hz ODOMETRY 与 50 Hz health 的原计数和错峰；
SITL 接收频率是本机运行结果，不能当成物理 UART 频率或飞行验证。

测试覆盖原生常量、独立 GPS Z offset、非重算 scale、完整 reset、首次建立/ResetXY/ResetZ/home scale，
尾零/CRC/坏值/未知子类型、迟到原点、transient_local 后启动订阅、重连及转换到消费间 reset。
真实固件 pack → MAVLink 字节流/CRC parser → ROS decode 的跨仓库测试通过（45-byte 帧）。
旧本地与新增地理 ROS 受理测试共用相同设施；原测试第二段等待同步到已有“先转向再平移”的实际时长，
没有改变稳定门限。ROS 构建、FC SITL 构建及针对性测试通过；未连接真实飞控、刷写或验证目标板飞行。

## 坐标系与消息语义

沿用已有 ATTITUDE 公式，令 `k=sqrt(0.5)`：

```text
position_ENU = [wire.y, wire.x, -wire.z]
q_ENU_FLU = [k*(w+z), k*(x+y), k*(x-y), k*(w-z)]
v_FLU = [wire.vx, -wire.vy, -wire.vz]
v_ENU = R(q_ENU_FLU) * v_FLU
omega_FLU = [rollspeed, -pitchspeed, -yawspeed]
```

健康消息的平移方差已经是 ENU，不能再交换 N/E。
FusedState.velocity 是世界 ENU；nav_msgs/Odometry 的 child_frame_id 是 base_link，
因此其 twist.linear 按 ROS 约定恢复为机体 FLU。只有对角线不能恢复交叉相关项，
也不能据此构造完整机体速度协方差；Odometry 未知项保留 NaN。

适配器没有 IMU 积分、预测、EKF、AHRS、气压融合、位置平滑或外推。
External → FeedthroughEstimator::getAt() 直接返回输入状态，不延长状态时间。
控制器数学算法、Q/R、模型、轨迹、推力映射和 MSP 控制接口均未修改。

MPC 输入状态不使用实测 acceleration；GEO 关闭 drag_compensation 时使用的是
**参考轨迹加速度**。适配器 acceleration=0 只是 QuadState 结构兼容占位，
`acceleration_available=false`。RuntimeConfig 和 FcFused HardwarePilot 均拒绝开启 drag compensation。
不发布虚构 bias、innovation 或 ROS EKF posterior：这些值为 NaN，专用时间为零/NaN，
initialized、imu_ready、rtk_fixed、synchronized 为 false。FC 的有效性由
`state_source=fc_fused`、`fc_state_valid` 和新时间字段表示，不能继续用 initialized 判断这个来源。
旧模式明确标为 ros_ekf，保留原字段含义。

## 时间、安全及恢复

- `fc_time_usec` 是 FC 已完成估计更新的起始 epoch，不是 IMU 曝光、GPS PVT、UART 发送或 ROS 接收时间。
- `fc_state_sample_steady_time` 由原 TIMESYNC 映射一次；接收和发布各自另记。
  接收时刻不会替代采样时刻，状态重复不会刷新任何采样证据。
- 同一 Linux boot clock_id、接收器 source_session、FC 完整 reset_counter 必须一致；
  ODOMETRY 低 8 位必须与健康消息匹配。两种消息不要求逐帧时间戳相等。
- 健康观测年龄属于健康消息的 FC epoch；安全门使用
  `health_sample_steady_time - observation_age`，因此持续发送同一个健康快照不会刷新观测年龄。
- 新计算的状态采样年龄最多 15 ms，保持命令时最多 25 ms（从原始采样时间起算）。
  同时要求有限的 `sample <= receive <= publish <= now`，因此接收、发布时间无需重复检查相同年龄上限。
  命令创建时仍要求采样年龄不超过 15 ms，且状态已经发布；健康采样和接收各自最多 60 ms；
  XY 观测最多 300 ms、Z 最多 250 ms。FcFused 即使 timing_checks=false 也不能跳过这些检查。
- 平移各轴位置方差上限 1 m²、速度方差上限 0.09 m²/s²，沿用当前硬件就绪标准
  （1 m、0.3 m/s），仅为初始工程限值，未经新固件实机精度验证。
- XY/Z/snapshot/XY fusion flags（掩码 0xc3）、有限数值和时钟对齐独立检查。
  heading_valid 原值保留，不参与 FC 控制授权；该策略允许真实航向失效时继续控制。
  mag_heading_fresh 保留为单独证据，不冒充 GPS/PPS 或标定结果。
- RC/STATUS、ARM/AUTO/KILL、接收链路、配置回读、地理围栏、推力映射、电池、输出传输、
  25 ms 命令租约、8 ms 求解预算和原 MSP 写入截止时间仍有效。
- 输出阶段以最新 Authority 的真实 RC/STATUS 请求时间检查 100 ms 新鲜度，
  不再同时要求命令内的旧遥控请求快照保持新鲜；Authority 接收新鲜度、会话及 ARM/AUTO 一致性仍须通过。
  此更新仅替换遥控证据，不刷新命令生成时间或任何 FC 状态时间；KILL、失联、切换会话仍立即撤权。
  RC/STATUS 的规则统一在 `SafetyGate::receiverFailure()`；Health 消息新鲜度统一通过输出传输健康门控。
- ControlCommand → SafetyEvidence 编解码 → output → MSP bridge 均保留 FC 原始证据。
  输出端独立读取最新 FusedState 检查有效性、会话、重置和时效；fc_imu_ready 无法代替这些检查。
  Health 分别记录 fc_estimator_healthy、msp_healthy、control_output_healthy；较慢的汇总消息不刷新状态租约。
- 时间回退、重连、TIMESYNC 失效/跳变和 ROS 时钟跳变使接收会话失效；32 位 micros 回绕由
  固件扩展为共同 uint64 epoch，ROS 不把新协议降成 32 位。
- 接收故障带持久 `validity_epoch`，避免 QoS 深度 1 覆盖瞬时故障。适配器另有递增 generation，
  包含原点/重置/健康丢失/超时等失效。失效后清除参考激活和预热，健康恢复不自动接管；
  仍需 50 个新鲜独立状态的有效计算、健康的 AUTO 低，再物理 AUTO 高。
  新授权重新捕获当前位置/航向，旧绝对参考不会继续执行。
- 重复状态在原始 15 ms 有效期内可被控制 tick 读取，但不算新状态，不推进预热计数；
  原采样时间始终不变。状态停止后由 2 ms 适配器 watchdog、控制和输出各层撤权。

## 自动化验证

完整结果、时序和执行日志见 `test/results/fc_fused_20261026/`。
测试源为 `test_fc_fused.cpp`、`test_fc_fused_pipeline.py`、`test_fc_fused_launch.py`。
测试始终使用合成状态、localhost 和测试创建的 PTY。五节点 ROS 测试保持 hardware/shadow_only=true；
C++ 授权和 MSP transport 单测仅在自己的 PTY 上测试真实编码及授权/撤权。

复现命令（从仓库根目录，先准备 ROS Humble/Jazzy、pymavlink、GTest）：

```sh
./agi_ros2/scripts/build.sh -DAGI_ROS2_GAZEBO=OFF -DAGI_ROS2_FC_TESTS=ON
source /opt/ros/humble/setup.bash
source install/agi_ros2/local_setup.bash
export ROS_LOG_DIR=/tmp/fc-ros-logs
export PYTHONPATH="$PWD/agi_ros2/test:${PYTHONPATH:-}"
ctest --test-dir build/agi_ros2 --output-on-failure -R '^fc_fused_test$'
python3 agi_ros2/test/test_fc_fused_launch.py
python3 agi_ros2/test/test_fc_fused_pipeline.py
python3 agi_ros2/test/test_mavlink_sensor.py
python3 agi_ros2/test/test_node_pipeline.py
python3 agi_ros2/test/test_hardware_pipeline.py
python3 agi_ros2/test/test_msp_evidence.py
# 独立 agilib 已配置 BUILD_TESTS=ON、BUILD_BETAFLIGHT_HW=ON：
CCACHE_DIR="$PWD/build/ccache" CCACHE_TEMPDIR="$PWD/build/ccache/tmp" \
  cmake --build agilib/build --target betaflight_hw_test hardware_pilot_test msp_write_deadline_test tests -j2
ctest --test-dir agilib/build --output-on-failure -R '^(betaflight_hw_test|hardware_pilot_test|msp_write_deadline_test)$'
agilib/build/tests --gtest_filter='EkfImuRtk.*:EkfImuBaro.*:HardwareGeo.*'
```

协议测试使用真实 MAVLink2 pack→字节流→CRC parser→decode，覆盖小端字段、尾零截短、
未知协方差、畸形版本/长度/枚举/数值、四个方位及正负 roll/pitch/yaw、六向速度与角速度转换。
状态测试覆盖健康/状态丢失、冻结重复、时间延迟、FC 时间超过 uint32、回退/重启/重连、
TIMESYNC 丢失/恢复、ROS stamp 异常、完整 reset=256 与低八位=0、会话错配、
XYZ/观测年龄/方差拒绝，以及 quality=-1 和航向健康掉落放行。控制测试实际调用 MPC/GEO、预热、AUTO 边沿、重置后参考重捕获、
ARM/KILL、求解失败证据、MSP 超时与撤权，不修改控制器核心求解器。

## 验证边界与待办

100 Hz 的本机 PTY 结果只说明软件接口和本机调度在该短窗口内可满足约束。
没有测得真实 FC → UART → CM5 的频率/延迟分布、位置或航向精度、长时间运行裕度，
没有做真实新版 ODOMETRY MCAP 回放或飞行。旧原始 IMU/GPS/Baro MCAP 不能证明新协议有效。

固件 V1 的姿态/滤波陀螺是在位置更新完成时读取的最新调度样本，没有各自物理采样年龄；
观测年龄是已接受融合更新的年龄，部分气压/测距/光流更新可能重用旧传感器值。
这不是独立物理传感器新鲜度证明；实际固件健康信息仍有该限制。
此外，只有平移方差对角线，没有姿态/角速度完整协方差；统计一致性和实际精度尚未验证。

收到新版真实数据后，应先录制影子模式，核对 FC artifact、实际波特率/吞吐、TIMESYNC RTT/偏差、
未来/过期计数、采样→接收→发布→控制→命令各阶段延迟和丢帧，再评估 15 ms 限制及精度。
不通过放宽时间、方差门限或重新积分 IMU 来掩盖问题。

## 本次执行结果

见 [完整验证记录](test/results/fc_fused_20261026/README.md)。编译通过；新增 C++ 7/7、
启动 3/3、五节点 3/3 及最终增补故障测试通过。旧配置回归有 5 项基线失败，未宣称全仓库全绿。

`fc_state_sequence` 属于 **50 Hz 健康快照**，ODOMETRY 没有独立状态序列字段；
用 `fc_time_usec` 判断 100 Hz 状态是否重复，不用健康 sequence 当每帧 ODOMETRY 序号。

## 修改文件清单

| 文件 | 修改内容 |
|---|---|
| `.gitignore` | 保留原 experiments/；显式纳入本次测试与证据目录。 |
| `agi_ros2/CMakeLists.txt` | 生成两种 FC 消息、编译安装适配节点，注册协议/控制 CTest，安装协议文档。 |
| `agi_ros2/package.xml` | 声明 GTest 测试依赖。 |
| `agi_ros2/config/hardware.yaml` | 选择 fc_fused，增加 100/50 Hz 配置，保留 shadow 和原控制参数。 |
| `agi_ros2/include/agi_ros2/fc_protocol.h` | 无对齐强转的 V1 解码、MAVLink2 校验和原坐标公式。 |
| `agi_ros2/include/agi_ros2/fc_state_adapter_node.h` | 只含格式/有效性适配的节点接口与会话缓存。 |
| `agi_ros2/include/agi_ros2/fc_state_evidence.h` | 从 FusedState 构造独立 FC 安全证据。 |
| `agi_ros2/include/agi_ros2/runtime_config.h` | 共用 NavigationPolicy 解析，支持 profile 的 fc_fused 默认值。 |
| `agi_ros2/include/agi_ros2/command_output_node.h` | 保存独立最新 FC 状态订阅用于最终输出检查。 |
| `agi_ros2/msg/FcEstimatedState.msg` | FC 采样/接收/发布、会话/有效性 epoch、转换状态与原协议元数据。 |
| `agi_ros2/msg/FcEstimatorStatus.msg` | 完整 V1 健康字段、健康样本与接收时间。 |
| `agi_ros2/msg/FusedState.msg` | 来源、可用性、独立 FC 时间/重置证据；旧字段不伪装。 |
| `agi_ros2/msg/Health.msg` | 分别记录 FC estimator、MSP 和控制输出健康。 |
| `agi_ros2/msg/SafetyEvidence.msg` | 命令跨进程传递不可刷新的 FC 时间/质量/会话证据。 |
| `agi_ros2/msg/ComputationStatus.msg` | 增加 FC epoch、采样年龄、接收年龄和发布延迟。 |
| `agi_ros2/src/mavlink_sensor_node.cpp` | 复用串口/来源/TIMESYNC，接收两种 FC 消息，处理去重/回退/无效 epoch 和诊断。 |
| `agi_ros2/src/fc_state_adapter_node.cpp` | 健康配对、有限性/时效检查、无滤波 FusedState/Odometry 转换及 watchdog。 |
| `agi_ros2/src/fc_state_adapter_main.cpp` | 适配器进程入口。 |
| `agi_ros2/src/control_node.cpp` | 独立 FcFused 路径，重置/失效撤权，使用真正 FC 时间并发布诊断。 |
| `agi_ros2/src/command_output_node.cpp` | 独立检查最新 FC 状态与命令会话/重置，不使用 fc_imu_ready 代替估计状态。 |
| `agi_ros2/src/node_common.cpp` | 新 FC SafetyEvidence 字段对称编解码。 |
| `agi_ros2/src/state_fusion_node.cpp` | 仅标记旧来源及加速度可用性，保留原 EKF 算法。 |
| `agi_ros2/scripts/msp_evidence.py` | 保留物理 MSP 授权，区分 FC/旧 EKF 健康并分别记录。 |
| `agi_ros2/scripts/launch_realtime.py` | 按状态来源选择四个 C++ 节点，继承优先级及监控/清理机制。 |
| `agi_ros2/launch/runtime_launch.py` | 新旧状态节点互斥、配置选择和非法组合拒绝。 |
| `agilib/include/agilib/bridge/betaflight/hardware_safety.hpp` | FcFused 独立安全门和会话/重置后 AUTO 重新授权。 |
| `agilib/include/agilib/pilot/hardware_pilot.hpp` | 保存 FC 会话/代际/重置/上一采样时间。 |
| `agilib/src/pilot/hardware_pilot.cpp` | FC 重置清除参考/预热，重复采样不推进预热，拒绝无加速度的 drag compensation。 |
| `agi_ros2/test/test_fc_fused.cpp` | 7 个确定性协议、坐标、gate、实际 MPC/GEO 和 PTY MSP 测试。 |
| `agi_ros2/test/test_fc_fused_pipeline.py` | 五节点影子链路、故障/时间/重连、状态适配和时序统计。 |
| `agi_ros2/test/test_fc_fused_launch.py` | 新旧节点互斥、实时选择及非法配置测试。 |
| `agi_ros2/test/test_runtime_config.py` | 旧图/bench 测试显式选择 ros_ekf，保留所有旧测试及断言。 |
| `agi_ros2/MAVLINK_FC_FUSED.md` | 协议、语义、安全、复现命令、验证边界与逐文件清单。 |
| `agi_ros2/test/results/fc_fused_20261026/` | 本次构建、测试、基线复现日志和时序 JSON；含首轮失败记录。 |
