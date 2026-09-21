# 实机数据驱动的无输出评估

`shadow.launch.py` 接入两条独立串口：MSP 查询接收机、模式、配置和电池；
MAVLink 接入 IMU 和普通 GPS。它运行本地坐标转换、EKF、悬停/CSV 参考和 MPC，
**不发送 MSP_SET_RAW_RC（code 200）**。不要同时在 MSP 串口启动 `msp.launch.py`。
本入口不提供开启输出的参数；控制端和输出端的 `shadow_only=true` 都是只读参数。
输出端收到伪造允许命令也会在映射/发送之前返回。普通 flight 的硬件分支现在同样使用 GNSS，
并由 `hardware.yaml` 的 `flight.shadow_only: true` 默认禁止输出；SITL 保留 RTK 路径。

## 启动

先构建并 source，填写 [hardware.yaml](config/hardware.yaml) 的两条 UART 和真实机体模型。
模型、rate、精度与围栏的零值表示尚未配置；shadow 同样拒绝不完整的机体模型。
模型未知时先运行 `./agi_ros2/scripts/launch.sh mode:=hardware diagnostic_only:=true`，
读取传感器与 MSP 而不构造 MPC。无需创建单独参数目录。

```bash
./agi_ros2/scripts/build.sh
source install/agi_ros2/local_setup.bash
# 安装入口读取安装包中的hardware.yaml；源码入口读取源码配置。
ros2 launch agi_ros2 shadow.launch.py
# 或指定同一完整配置：
# ros2 launch agi_ros2 shadow.launch.py runtime_config:=/absolute/path/hardware.yaml
```

修改 `hardware.yaml` 的 `mavlink.altitude_source`、`navigation.heading_confirmed`、
`navigation.heading_correction_rad`、`navigation.fc_declination_applied`，依据实际验证结果填写。

这些字段需要核实后填写：

- `altitude_source=unknown` 为默认值，不产生可用于三维融合的观测；选择 `msl` 须确认
  FC 的 GPS_RAW_INT.alt 是 MSL 高度，选择 `ellipsoid` 须确认真实椭球高字段有效。
- `heading_confirmed=false` 为默认值，适配器不输出本地导航观测。确认 FC 使用已校准
  磁罗盘辅助航向、安装方向与北向基准后才能设为 true。FC `trust_mag=ON` 同样是操作者声明；
  配套固件检查 MAG 存在、未校准中、磁采样与磁航向校正新鲜有效，但不能证明标定质量。
  校准流程结束不等于标定成功；本轮不用 COG 替代静止航向。
- `heading_correction_rad` **加到已转成 ENU 的 FC 航向**；不是直接填入罗盘顺时针
  磁偏角。`fc_declination_applied` 记录 FC 是否已处理磁偏角，不自动修改修正量。
  若 FC 已正确输出真北航向，应使用 0；避免两次修正。此声明本身不构成健康证据。

不要求推力表。提供 `thrust_table` 时仍验证其格式并报告可用性，但不会映射成 RC。
默认空 `trajectory` 使用悬停参考；需要时显式传入绝对 CSV 路径。CSV 在有效实体
ARM 和 AUTO low→high、50 个连续有效计算周期后开始，起点对齐当前机体位置和航向。
KILL、失联、状态失效、输出进程故障或导航会话变化撤销参考执行，重新观察 low→high。
无输出悬停计算可以在健康证据不齐甚至授权未知时继续，不能据此判断允许飞行。

默认录制所有 topic（含隐藏 topic），输出至 `~/agi_bags/hardware_*`；使用 `bag_output`
指定位置，`record_bag:=false` 关闭。任一组件退出时整组退出。

## 导航与融合

数据流：

```text
MAVLink HIGHRES_IMU → /sensors/imu ───────────────────────┐
GPS_RAW_INT + GLOBAL_POSITION_INT（按设备时间配对）        │
  → /sensors/navigation                                │
  → gnss_adapter.py → /sensors/local_navigation ────────┤
                                                       ↓
                                            state_fusion_node
                                                       ↓
                                                /fused_state
                                                       ↓
                                          control_node / MPC
```

原有 GPS、速度、航向和姿态 topic 保留。`Navigation` 原子观测携带 fix 类型、设备解算时间、
高度基准、精度和时钟对齐状态，避免从 1 Hz diagnostics 拼接质量数据。
缺少有效速度时不发布有效原子观测，不能用零速度补齐；显式失效会发带 NaN 的导航撤销事件。
TIMESYNC 只代表时钟对齐，
不代表 GPS 测量 PPS 同步；GPS 时间仍是 FC 完整解算更新时间，模块/滤波/串口延迟仍然存在。

适配器在连续静止样本中建立固定原点，默认至少 3 秒、30 个独立解算、速度不超过
0.3 m/s。重复/乱序样本不累计；超过 0.3 秒间断或无效输入重置原点采集窗口。
重新同步/串口重连保持原点，但变更源会话以重置融合/控制；适配器重启建立新原点与新会话。
椭球高采用 WGS84 ECEF→ENU；MSL 路径的水平坐标以椭球表面计算，上下方向明确为相对 MSL。
原点和修正声明通过持久化、定期重发的 `/navigation/origin` 记录；`/navigation/status` 给出等待原因。

已有原点后，no-fix、无效航向/速度、源时钟/会话失效会生成 `observation_valid=false` 的本地导航事件。
融合立即发布失效状态，控制下一周期撤销参考，证据下一次更新撤销 readiness，输出在收到不健康证据时复查。
不会保留旧好定位直到 300 ms 年龄超时；事件使用检测时间，不刷新最后有效测量时间。
完全停流仍按年龄检查，实际撤销时间包含 ROS 调度与传输延迟。

适配器可通过节点只读参数配置 `origin_duration`、`origin_samples`、`origin_max_speed`、
`horizontal_stddev`、`vertical_stddev`、`velocity_stddev`、`heading_stddev`。
默认噪声标准差下限分别为 1 m、2 m、0.3 m/s、10°；精度未知使用这些评估值并标注未知。
消息中的方差单位是平方单位。FC 航向与 FC IMU 有相关性，不按独立 RTK 航向处理。

硬件融合节点 `navigation_source=gnss` 仅订阅新观测；仿真 `rtk` 路径不变。
GNSS 初始化还要求新鲜未 ARM 授权、至少 3 秒/1000 个静止 IMU 样本，并检查gyro bias、
噪声及重力；无法在已经 ARM 的情况下跳过这次初始化。
`FusedState` 增加导航来源、会话、fix 类型、时钟对齐与观测有效性；
普通 GNSS 的 `rtk_fixed` 和 `synchronized` 保持 false，即使 GPS 报 fix_type=6，
也不声称当前组合满足原双天线 RTK/PPS 契约。

## MSP 解码与证据

只支持已按本地源码核对的 **BTFL API 1.48** 布局；不支持的版本保持配置/授权无效。
MSP RC 和 STATUS_EX 每秒查询 25 次，电池 2 次，配置回读每秒一次。原始事件照常记录。
只读配置包括 API/FC 标识与版本、BOXIDS、模式范围与 EXTRA、RX map/config、rates/deadband。
Override mask、failsafe 和 timeout 通过原生 MSP v2 `0x3010` 查询；发送 API 只允许这三个名字，
请求中不包含 `=`。96 字节 NUL 填充为当前固件回复缓冲区预留空间。未支持此查询的固件
明确失败，不回退到 CLI 或写命令，也不自动修改飞控配置。

配置要求：

- `map AETR1234`，对应 MSP 内部 AERT map `[0,1,3,2,4,5,6,7]`。
- AUX1 ARM、AUX2 MSP OVERRIDE、AUX3 FAILSAFE；默认有效范围 `[1700,2100)`。
  `hardware.yaml` 的 `evidence.arm_aux/auto_aux/kill_aux` 可选择其他三条 AUX（零基 0～13，须互不相同）；
  `aux_low/aux_high` 可调整共同范围，必须与回读一致。
- 各授权模式只允许一个直接 OR 范围，不接受链接模式或 AND 逻辑。
- `msp_override_channels_mask=15`、`msp_override_failsafe=OFF`、`msp_override_timeout_ms=50`、midrc=1500。
  固件默认仍是 300 ms，必须由操作员显式设为 50；`evidence.expected_override_timeout_ms` 本轮固定 50，不能放宽。
  旧固件不支持 timeout 只读查询时拒绝就绪；升级/备份/回读流程见 [README](README.md#飞控超时与授权回读)，程序不自动刷写。
- ACTUAL rates、center/max/expo 和 deadband 与 `hardware.yaml` 的 `bridge` 一致；
  当前 PID/rate profile 与 `evidence.expected_pid_profile/expected_rate_profile` 一致。
- AUTO 时拒绝 ANGLE/HORIZON 等冲突控制模式。

`MSP_RC` 的 AETR 可能来自 Override，不能当实体原始遥控；这里只使用已验证不被覆盖的 AUX。
STATUS_EX 通过 BOXIDS 动态位置解码 ARM、MSP OVERRIDE、FAILSAFE，并读取 RX/FAILSAFE 禁止标志。
这仍是 FC 的接收机状态报告，不是独立射频链路测量。真实硬件的失联与开关行为需录包验证。

`MspEvent` 新增会话、请求 ROS/单调时间和查询名字。授权证据使用匹配请求发送时间，
RC/STATUS 分别检查 100 ms 有效期，组合取较早时间，重发不刷新采集证据。
电池单独允许 1.5 秒有效期；health 的新心跳不会延长电池寿命。
配置/遥测错误按原会话锁止；配置变化也锁止，排查后重启。串口关闭释放独占标志。

`msp_evidence.py` 发布 `/msp/decoded_state`、`/authority`、`/health` 和 `/health/status`：

- ARM 同时要求实体 AUX 请求与 FC 实际 armed；AUTO 必须与 FC 模式位一致。
- KILL、RC/STATUS 陈旧、模式矛盾或配置未知，产生撤销状态。
- 电压来自电池回传；传输健康还结合输出进程状态。
- 推力表来自输出节点检查；默认没有推力表时 false。
- 可通过 `geofence_min/geofence_max` 配置本地包围盒，默认未配置为 false。
- 旧 `imu_calibrated/converged` 仍为 false，保留其原有含义。新 `imu_ready` 合并 FC 状态和
  静止统计检查；`estimator_ready/navigation_ready` 来自融合质量、创新、连续更新及精度门限。
  配置中精度/协方差门限为 0 时不授予 ready；不会用 initialized 代替质量判断。
  Shadow 的有效状态可以供 MPC 计算，但不完整健康证据不能解释为允许飞行。

## 观察和验证

`/status` 报告当前状态与原因；缺失质量或配置证据时应结合 `/health/status` 和
`/fused_state.readiness_reason` 定位。普通 GPS 不再被伪装为 RTK；硬件使用独立 GNSS 就绪策略。
`/computation_status` 独立报告 state_valid、controller_type、controller_success、warm_cycles、AUTO 参考活动状态、
reference_elapsed、solve_seconds、cycle_seconds 及状态/IMU/导航年龄。`trajectory_active` 表示 AUTO 参考已激活，空 CSV 时为悬停。
`/control_command.permit_override` 和 `/output_status.override_active` 在本入口始终为 false。

```bash
source install/agi_ros2/local_setup.bash
/usr/bin/python3 agi_ros2/test/test_shadow_support.py -v
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=86 /usr/bin/python3 agi_ros2/test/test_shadow_pipeline.py -v
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=88 /usr/bin/python3 agi_ros2/test/test_mavlink_sensor.py -v
./agi_ros2/scripts/test.sh
```

进程测试仅使用合成传感器、DDS、回环 UDP 和 PTY；需要本地 socket 权限。
输出隔离测试包含伪造授权命令、运行中参数修改拒绝、AUTO/KILL、停流和输出进程重启，
检查实际捕获字节中 code 200 数量为零。硬件需要在目标机另行执行只读录包，不能用这些
软件测试代替物理 UART 吞吐、磁罗盘真北一致性、普通 GPS 漂移和时延验收。

录包完成后可生成可检查的 JSON 统计：

```bash
ros2 run agi_ros2 summarize_shadow_bag.py /absolute/path/to/shadow_bag > shadow_summary.json
```

统计包含独立采集时间频率/间隔、bag 接收年龄、控制使用的 IMU/GPS/RC 年龄、
TIMESYNC RTT、MPC 耗时/成功数/预热、MSP 错误、位置范围和航向变化，以及原点和配置快照。
仅静止实验中的位置范围可用于讨论漂移；没有独立真北参考时不能把航向变化当绝对误差。
记录间断不等于可证明的线上丢包率，DDS discovery 漏掉启动消息也不等于没有发生输出。
报告中的 code 200/授权计数必须结合录制覆盖范围和 PTY 测试解读。

## 历史自动验证记录（2026-09-21，改动前）

以下为影子入口早期版本记录，不是此次硬件悬停改动的回归结论。该版本使用本机 ROS 2 Humble、合成数据与伪串口完成：

| 检查 | 结果 |
|---|---|
| ROS 2 构建 | 通过 |
| `test_shadow_support.py` | 8 项通过：坐标、原点、配置、模式、时效与失效 |
| `test_shadow_pipeline.py` | 4 项通过：GNSS/EKF、MSP/输出隔离、影子 CSV/MPC、航向未确认阻止初始化 |
| `test_shadow_bag.py` | 1 项通过：合成 bag 写入、读取和统计 |
| `test_mavlink_sensor.py` | 9 项通过，含新增原子导航质量/会话测试 |
| `test_node_pipeline.py` | 原有 10 项通过 |
| `test_msp_node.py` | 通过：伪串口频率、topic、ACK、超时隔离和 rosbag |
| C++ 修改范围 clang-format、`git diff --check` | 通过 |

影子串口测试测量 RC/STATUS 查询约 25 Hz，影子路径实际发送的 code 200 数量为零。
独立 bench 回归在伪串口上发送测试 RC 是原测试的预期行为，不能与影子入口混淆。
本次未访问真实串口、刷写飞控或执行真实飞行；真实数据录包及误差/时延验收尚待目标硬件执行。

MPC/GEO 可通过统一入口 `controller:=GEO` 或配置 type 选择；两者的 shadow 都禁止 MSP 200。
旧 `mpc_success` 是 controller_success 的兼容别名，GEO 的 solve_seconds 指控制计算耗时。
模型要求按所选控制器区分，详见 [README 控制器选择](README.md)。
