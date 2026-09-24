# ROS 2 Betaflight 控制入口

标准入口只维护两份完整配置：[simulation.yaml](config/simulation.yaml) 和
[hardware.yaml](config/hardware.yaml)。仿真保留已经使用的分层部署方式：开发机运行
Gazebo、Betaflight SITL 和传感器适配器，CM5、Jetson 或开发机运行融合、控制、输出三节点。
实机使用两条独立 UART：MAVLink 接收 IMU/GPS，MSP 查询实体授权、配置和电池并发送 AETR Override。

```text
仿真：Gazebo → gazebo_sensors → IMU / RTK ────────────────────┐
实机：FC MAVLink → mavlink_sensor_node → IMU ─────────────────┤
                                    → GPS → gnss_adapter.py ┤
                                                           ↓
                                                  state_fusion_node
                                                           ↓ /fused_state
                                                control_node / MPC 或 GEO
                                                           ↓ /control_command
                                                  command_output_node
                                             仿真 UDP / 实机 UART MSP
                                                           ↓
                                                  Betaflight 角速度内环

实体接收机 → FC → MSP 遥测 → msp_evidence.py → /authority、/health
                             ↑ 配置/模式/电池       ↓ 同时供控制与输出独立复查
```

本次软件适配的目标是**先人工起飞，再通过实体 AUTO 开关捕获当前位置和航向、执行 MPC 或 GEO 悬停**。
入口不自动 ARM、不自动起飞或降落。默认空轨迹；`hardware.yaml` 默认 `shadow_only: true`，
只计算与录包、不发送控制帧。实机机体、精度门限、围栏和推力数据需要填入实测值，
仓库中的零值是未配置标记，不是飞行参数。历史 SITL/伪串口结果不代表当前固件已刷入飞控，
也不代表已经完成 CM5 时延验收或真实飞行验证。

## 编译和配置来源

在仓库根目录：

```bash
source /opt/ros/humble/setup.bash  # 目标机使用实际安装版本，例如 jazzy
./agi_ros2/scripts/build.sh
source install/agi_ros2/local_setup.bash
```

脚本只选择 `agi_ros2`，生成目录是工作区 `build/agi_ros2`，安装目录是 `install/agi_ros2`；
不执行测试、不启动或刷写飞控。C++17、Eigen/acados 和原有依赖不变。
CM5/Jetson 需使用目标架构的 acados 库，不能复制 x86 二进制：

```bash
ACADOS_ROOT=/path/to/arm64/acados ./agi_ros2/scripts/build.sh \
  -DAGI_ROS2_GAZEBO=OFF -DAGILIB_ARM_CPU=cortex-a76
```

`cortex-a76` 是 CM5 选项；Jetson 根据实际 CPU 选择或省略该选项。
ARM64 默认不构建 Gazebo 适配器，核心三节点和 MAVLink/MSP 节点仍正常构建。

- `./agi_ros2/scripts/launch.sh` 加载源码 launch 和源码 `agi_ros2/config/`。修改配置后重启即可；
  修改 C++、Python 节点或消息后仍须重新构建安装。
- `ros2 launch agi_ros2 flight.launch.py` 加载安装包内 launch 和同包 `config/`；修改源码后重新安装。
- `runtime_config:=/absolute/path/simulation.yaml` 可以选择另一份同结构配置；启动日志打印最终绝对路径。
  相对轨迹、推力 CSV 和 bag 路径都相对该配置所在目录解析。
- 配置内直接包含 Pilot、机体、MPC/GEO、桥接和融合参数，不生成拆分的 Pilot/controller/quad YAML。
  `params_dir/pilot_config/bridge_config` 保留给旧直接节点入口，统一 launch 拒绝与新配置混用。
- `flight.launch.py`、`shadow.launch.py`、`msp.launch.py`、`mavlink_sensors.launch.py` 共享同一组装代码；
  后三者是诊断/影子薄封装，不拥有第三套参数。旧 `agilib/params` 继续供 standalone/历史工具使用。

配置段对应关系：

| 段 | 内容及读取者 |
|---|---|
| `flight` | 轨迹、推力表、shadow/diagnostic 开关、SITL 延迟实验、录包；launch 读取 |
| `pilot` | 内嵌 `quadrotor` 和 `pipeline.controller.parameter_sets`；按 type 选 MPC/GEO 参数，输出端采用相同模型校验 |
| `bridge` | ACTUAL rates/deadband/min_check；输出端映射、MSP 配置回读和 SITL EEPROM 共用 |
| `fusion` | IMU 噪声、初始化及导航质量门限；launch 传给融合节点 |
| `output` | MSP UART 和查询周期；控制输出/只读 MSP 节点 |
| `mavlink`、`navigation` | 传感器 UART/消息频率、高度/航向声明、原点和精度门限 |
| `evidence` | 实体 AUX/RX map、PID/rate profile、围栏和电池时效 |

数组请写为 `[x, y, z]`；Agilib 的现有 YAML 读取器不支持 PyYAML 默认的无额外缩进多行数组。

## 选择 MPC 或 GEO，以及对应参数

两份配置各自保留两组控制参数。`pilot.pipeline.controller.type` 是默认选择，仓库默认仍为 `MPC`；
`parameter_sets.MPC` 保存 MPC 权重，`parameter_sets.GEO` 保存 GEO 增益和倾角限制。
改变 type 时自动选取同名参数组，不需要手动改文件路径、复制参数或增加第三份 YAML。

```bash
# 仿真，任选一个控制入口；模拟器仍由 run.py 单独运行。
./agi_ros2/scripts/launch.sh controller:=GEO
./agi_ros2/scripts/launch.sh controller:=MPC

# 实机 GEO 影子计算；硬件仍默认 shadow，不发送控制帧。
./agi_ros2/scripts/launch.sh mode:=hardware controller:=GEO

# 参数与台架验证完成后，选择控制器并显式允许输出。
./agi_ros2/scripts/launch.sh mode:=hardware controller:=GEO shadow_only:=false

# 安装入口使用相同选项，读取安装包内配置。
ros2 launch agi_ros2 flight.launch.py mode:=hardware controller:=GEO
```

不传 `controller` 就使用配置中的 type；启动覆盖支持 `mpc/geo` 或 `MPC/GEO`。
控制与输出节点收到同一个选择，启动日志打印最终绝对路径和所选参数组。
`controller` 是只读启动参数，运行中 `ros2 param set` 会被拒绝；更换控制器必须停止原进程后重新启动，
重新完成预热和 AUTO low→high。禁止同时运行两个 flight 控制入口。
`run.py --ros2 --controller geo` 只把所选控制器写入其打印的独立 flight 启动命令，不代替启动控制节点。

GEO 使用位置/速度反馈、参考加速度和 yaw，输出与 MPC 相同的机体角速度＋质量归一化推力。
融合、参考捕获、MSP 映射、导航准入、50 次健康计算预热、100 Hz、8 ms 计算预算及输出期限共用。
它限制期望推力方向的倾角；这个限制不代表实际机体倾角在扰动下绝不会超过该值。

| GEO 参数，位于 `pilot.pipeline.controller.parameter_sets.GEO` | 含义 |
|---|---|
| `kpacc: [x,y,z]` | 位置误差到加速度增益，s⁻² |
| `kdacc: [x,y,z]` | 速度误差到加速度增益，s⁻¹ |
| `kpatt_xy`、`kpatt_z` | 姿态误差到 roll/pitch、yaw 机体角速度的增益，s⁻¹ |
| `p_err_max`、`v_err_max` | 逐轴参与反馈的位置误差上限 m、速度误差上限 m/s；不是围栏 |
| `max_tilt_rad` | 期望推力方向相对世界竖直轴的最大倾角，rad，必须在 `(0, π/2)` |
| `drag_compensation` | 当前 ROS2 路径没有电机 RPM，必须为 false；true 拒绝启动 |
| `filter_sampling_frequency`、`filter_cutoff_frequency` | 保留滤波参数，Hz；关闭补偿的最小模型路径不运行电机/加速度滤波 |
| `kprate` | 保留旧接口兼容；当前 Betaflight rates 输出不使用该增益，也不替代飞控 PID |

当前 rates/thrust MPC 与 GEO 的机体最小数据均为 `mass`、`omega_max`、`thrust_min/max`。`omega_max` 为逐轴 rad/s；
推力字段继续沿用单电机等效 N 的单位，两种控制器的总推力被限制在 `[4*thrust_min, 4*thrust_max]` N。
实机请根据标定表和允许工作范围填写，而非直接把电机规格最大值当作测试范围；
整个总推力范围应落在计划带载电压下的标定包络内，输出端仍拒绝表外值、不外推。
惯量、力臂、kappa、电机转速/时间常数/推力多项式在这两种 ROS2 外环模型中均不加载，可以省略；
内部将未使用项标记为未知。Gazebo 的物理机体和旧内置动力学估计器仍需要自己的完整模型。

GEO 仍要求真实质量、RC 推力表、飞控 rates 回读、导航/融合质量、围栏及实体授权。
配置里的 GEO 增益是调试起点，未通过本机体飞行验收；当前 rates 输出没有 jerk/角速度前馈和位置积分。
零/向下推力方向、无效四元数和姿态计算奇点会拒绝本周期并撤权。
MPC 参数保留在 `parameter_sets.MPC`，具体模型与权重见下一节。
选择 GEO 不会移除现有 acados 构建依赖。消息增加了控制器诊断字段，仿真机和伴随计算机需统一重编译。

## MPC：直接优化总推力和机体角速度

当前 MPC 的状态为 `x=[p(3),q(4),v(3)]`，输入为 `u=[c,ωx,ωy,ωz]`，
其中 `c=T/m` 的单位为 m/s²，三轴角速度为机体 FLU rad/s：

```text
p_dot = v
q_dot = 0.5 * q ⊗ [0, ω]
v_dot = R(q) * [0, 0, c] + [0, 0, -9.8066]
```

控制周期仍为 10 ms，预测 20 个 50 ms 区间，总时域 1 s。acados 直接给出本周期的 `c` 和 `ω`；
不再优化四个电机推力，也不再取下一预测状态的角速度作为当前命令。
控制节点把 `c` 乘真实质量转成总推力 N，输出节点查电压—RC 油门表，角速度通过 ACTUAL rate 逆映射交给飞控。
`Command.thrusts` 保持未知，避免下游误认为得到了电机分配结果。自定义旧电机直出桥（例如 MSP SET_MOTOR、Laird 单电机模式）
不能直接接这个新 MPC，必须有匹配的内环分配器；本次 ROS2 Betaflight 路径使用 rates/thrust 接口。
参考轨迹仍可提供位置、姿态、速度与角速度，
CSV 原机体推力先按源质量换算为 `c_ref`；未改动原 CSV 的时间、翻转或速度。
按本次要求，三条 CSV 统一保留 14 字段短表头；每个数据行仍保留原有 30 列（包括加速度、推力、jerk、snap），
不是标准的等宽命名表，需用项目轨迹加载器读取，不能依赖普通 DictReader 推断后 16 列名称。

这是一种**理想内环模型**：假设 Betaflight 能及时跟踪角速度与推力指令。
没有加入实测内环时间常数、通信纯延迟、电机迟滞或气动阻力，也不保证独立的总推力/角速度约束联合可实现。
因此它减少了外环的机械参数需求，但不能据此认定在真实机体上已具备 50 m/s 跟踪能力。
实机至少还要测量指令与实测角速度的阶跃响应、推力映射和端到端延迟；误差显著时再辨识并加入内环动态。

| `parameter_sets.MPC` 参数 | 含义 |
|---|---|
| `Q_pos_x/y/z` | 三轴位置残差权重 |
| `Q_att_x/y/z` | 四元数姿态的三维残差权重 |
| `Q_vel` | 三轴速度残差权重，标量或三元素数组 |
| `R_collective_thrust` | 输入 `c-c_ref` 的权重；其单位基于 m/s²，不能沿用单电机 N 的旧 R |
| `R_body_rates` | 三轴 `ω-ω_ref` 权重，三元素数组 |
| `exp_decay` | 时域阶段权重衰减，末端同样应用；1 表示不衰减 |

旧单电机 `R`、`Q_omega_*` 和 CoG 适配参数不再静默使用，加载时提示迁移。
ROS2 两份配置和仍保留的旧入口 MPC 文件已经同步迁移；没有新增第三套运行配置。
生成器为 `agilib/externals/acados_code_generator/drone_model.py`，重新生成和验证步骤见
[rate MPC 验证记录](RATE_MPC_VALIDATION.zh-CN.md)。更改求解器后所有使用此库的主机必须重新构建并安装。

## SITL：保持分层部署

终端 1，在开发机：

```bash
python3 betaflight_sitl/run.py
# 后续可使用 --no-build；--no-gazebo 关闭 GUI；--duration 限制模拟器墙钟运行时长。
```

只启动 Gazebo、Betaflight SITL、`gazebo_sensors`，不会启动 flight 或自动 ARM。
ROS2 分支从 `config/simulation.yaml` 的 `bridge` 段写入并回读**隔离的 SITL EEPROM**。
如使用自选配置，同时给模拟器和控制机指定同一份配置内容：

```bash
python3 betaflight_sitl/run.py --runtime-config /absolute/path/simulation.yaml
./agi_ros2/scripts/launch.sh runtime_config:=/absolute/path/simulation.yaml
```

终端 2，在运行融合/控制/输出的主机：

```bash
./agi_ros2/scripts/launch.sh
```

默认 `mode:=sitl`、空轨迹、正常时效检查。不同主机共享 ROS_DOMAIN_ID 和 DDS 网络设置；
控制机 `simulation.yaml` 的 `bridge.host` 指向运行 Betaflight 的开发机 IP。
不要把运行于 CM5/Jetson 的联合仿真改成 hardware 模式；真实传感器来源才决定切换。
融合、控制、输出三节点必须共处同一 Linux 主机，因为它们交换 boot ID 和单调时钟证据；
Gazebo 与传感器适配器可以在另一台主机，用 `/clock` 统一仿真时间。

终端 3，启动唯一模拟接收机：

```bash
source install/agi_ros2/local_setup.bash
ros2 run agi_ros2 sim_rc.py --ros-args -p use_sim_time:=true
```

随后在已 source 的终端操作：

```bash
ros2 topic echo /status
# 在另一终端逐项执行：
ros2 param set /sim_rc kill false
# 等 Betaflight 启动校准完成，AUTO 保持 false，低油门 ARM。
ros2 param set /sim_rc armed true
ros2 param set /sim_rc throttle 1420  # 仅当前仿真模型示例，不是实机油门
# 到达合适高度，确认 AUTO_STANDBY 后：
ros2 param set /sim_rc auto_switch true
# 撤销时先设好人工油门，再 AUTO low；KILL：
ros2 param set /sim_rc kill true
```

需要 CSV 时在 `simulation.yaml` 的 `flight.trajectory` 填路径，或传 `trajectory:=路径`。
轨迹在**授权成功的 AUTO 边沿**对齐当前时刻、位置和航向；未 ARM、启动 AUTO high 或
故障恢复时不会提前消耗轨迹时间。恢复须健康预热并重新观察 AUTO low→high。

`flight.sitl_delay_test` 默认 false。显式 `sitl_delay_test:=true` 仅用于保留的仿真延迟实验：
取消多项年龄/耗时/停流检查并重发旧命令，不能用该运行结果作为严格保护验收。
hardware 模式拒绝启用此开关。旧独立控制器仍可用 `run.py --no-ros2`。

## CM5 实机：诊断、影子、悬停

先在 `hardware.yaml` 填写 `output.device` 与 `mavlink.device`，使用两条独立 UART，
不能是同一设备的两个软链接。飞控连接及 MAVLink 时间/单位细节见 [MAVLINK_SENSORS.md](MAVLINK_SENSORS.md)。

1. **模型尚未准备好时读取实机数据：**

   ```bash
   ./agi_ros2/scripts/launch.sh mode:=hardware diagnostic_only:=true
   ```

   启动 MAVLink、GNSS adapter、融合、MSP monitor、证据节点，不构造控制器、不启动输出节点，
   不发送 MSP code 200。原始 IMU/GPS/MSP 可先读；航向、高度、配置或静止初始化条件未满足时，
   本地导航/融合会继续报告等待原因。此时没有 output_status，health 不会假装整条飞行链已就绪。

2. **所选控制器的真实模型填完后运行影子计算：**

   ```bash
   ./agi_ros2/scripts/launch.sh mode:=hardware
   # 等价只读封装，仍读取同一hardware.yaml：
   # ros2 launch agi_ros2 shadow.launch.py
   ```

   hardware 默认 shadow。核心三节点与 MAVLink/GNSS/evidence 合共六个节点；输出节点只查询 MSP。
   `shadow.launch.py` 即使接到 `shadow_only:=false` 也拒绝，运行中不能切换只读参数。
   推力表可暂缺，真实模型不可缺；详见 [SHADOW_EVALUATION.md](SHADOW_EVALUATION.md)。

3. **实测模型、推力表、质量门限和飞控回读全部就绪后，显式启用输出入口：**

   ```bash
   ./agi_ros2/scripts/launch.sh mode:=hardware shadow_only:=false
   ```

   这仅允许程序在证据满足时发送四通道 AETR，不会 ARM。静止、未解锁时先完成原点和 IMU 初始化；
   观察导航/估计/配置/推力/围栏就绪以及 50 个健康控制预热周期；人工起飞到目标高度，
   在实体 AUTO low 状态确认 AUTO_STANDBY 后切 high。空轨迹捕获当前三维位置和 yaw 悬停。
   退出 AUTO、KILL、失联、导航/控制/输出故障时停止 Override；恢复需新的健康 low→high。
   实体接收机的当前 AETR、飞控模式及 failsafe 决定停止 Override 后的真实行为。

单独检查时可用 `msp.launch.py`（只读 monitor）或 `mavlink_sensors.launch.py`，均读同一 hardware 配置。
完整 flight 已独占这两条串口，不应同时再开诊断节点。原有 bench 固定 RC 能力仍在
`betaflight_msp_node` 直接入口，需要显式 `mode:=bench` 和 `props_removed:=true`；不属于飞行 launch。

## 实机配置必须填写的数据

不能把零占位替换为 Iris 参数来绕过检查。启动聚合报告不合法模型字段；真实模型缺失时请使用 diagnostic。
当前 MPC/GEO 外环的机体必填项只有质量、机体角速度限制及推力限制。
串口、飞控、导航、标定及授权数据仍然共用；未参与外环的机械参数不作为启动占位。

| 位置 | 单位/来源 |
|---|---|
| `pilot.quadrotor.mass` | 含电池/负载的起飞总质量，kg |
| `thrust_min/max` | **每电机等效**推力界限（N）；总推力范围为四倍，需受实际 RC 标定包络约束 |
| `omega_max` | MPC 约束/GEO 输出限幅的机体角速度上限，rad/s；不能超过实际 rate/profile/机体能力 |
| `bridge.center_rate_deg_s/max_rate_deg_s/expo_percent` | FC ACTUAL rate 三轴参数，deg/s、deg/s、%；必须与当前回读 profile 一致 |
| `bridge.deadband/yaw_deadband/min_check` | FC RC deadband/min_check 原始设置；readback 精确匹配 |
| `flight.thrust_table` | 实测电压—PWM—**全机总推力 N** 表的路径，格式见下 |
| `mavlink.altitude_source` | 确认后填 `msl` 或 `ellipsoid`；默认 `unknown` 不提供可用三维高度 |
| `navigation.heading_confirmed/heading_correction_rad` | 确认绝对航向来源与安装方向后声明；修正量加在 ENU 航向上，rad |
| `navigation.fc_declination_applied` | 记录 FC 是否已经处理磁偏角，避免重复修正；本身不授予导航健康 |
| `navigation.max_horizontal_accuracy/max_vertical_accuracy/max_velocity_accuracy` | 允许的接收机报告精度，m/m/(m/s)；任一为 0 表示未配置，不授权 |
| `fusion.max_horizontal_position_stddev/max_vertical_position_stddev/max_velocity_stddev/max_heading_stddev` | 允许的 EKF 后验标准差，m/m/(m/s)/rad；0 表示未配置，不授权 |
| `evidence.geofence_min/geofence_max` | 本地 ENU 包围盒，m；每轴 min<max，包含原点和预定悬停范围 |
| `evidence.expected_pid_profile/expected_rate_profile` | 当前期望 FC profile 的零基编号；飞行中改变 profile 撤销授权 |
| `evidence.expected_override_timeout_ms` | ms；本轮硬件策略固定为 50，不能通过参数放宽，必须与固件回读相等 |
| `evidence.arm_aux/auto_aux/kill_aux` | 零基 AUX 序号，0～13 对应 AUX1～AUX14，三者须不同；默认 0/1/2 |
| `evidence.aux_low/aux_high/rx_map` | 实体 ARM/AUTO/KILL 范围及接收机映射，与飞控回读一致 |

硬件 `bridge.host/port/hover_throttle/motor_idle` 仅为既有映射类型的兼容字段；
硬件油门始终查实测表，不用仿真的平方根/悬停油门模型。MPC 权重只是起始调参值，未经过真实机体飞行验收。

推力 CSV 是纯数字矩形网格，不带字符串表头：

```text
0,PWM_1,PWM_2,...,PWM_n
电压_1,总推力_11,总推力_12,...,总推力_1n
电压_2,总推力_21,总推力_22,...,总推力_2n
...
```

这里的中文/符号是格式说明，实际文件必须填写数字。至少两个正电压与两个 PWM；电压和 PWM 严格递增，
PWM 在 1000～2000；每个电压行推力非负、有限且严格随 PWM 增长。查表按电压和推力插值；
电压或指令超出测量范围就拒绝输出，不外推，不静默截断。表内必须覆盖实际电池电压和所需悬停/纠偏推力。

CSV 的 PWM 列实际是发给 Betaflight 的 **RC 油门通道值**，不是电调 PWM/DShot 输出。
单电机推力台的电调输入不能直接当作这列：必须标定同一 FC 油门曲线、限幅、怠速与混控设置下的
RC 输入到全机总推力关系。以克力记录时用 `N = gf × 0.00980665`；只有测量对象是单个电机且
已经确认四套动力一致、整机映射成立时，才可用单电机结果乘 4。整机称重/总推力测量不能再次乘 4。
在实验记录中保存电机、桨、电调/固件、电池与负载、测量电压、测量对象和夹具、FC 版本/profile、
油门曲线/限幅/怠速/混控设置及单位换算。CSV 不添加文字元数据行；配置变更后重新确认表是否仍适用。

## 飞控超时与授权回读

本地 `/home/sia/betaflight` 的配套修改增加 `msp_override_timeout_ms`：固件保留默认 **300 ms**，
本轮 ROS 硬件策略要求操作员显式设置为 **50 ms**。接收机刷新时动态检查最近完整 AETR MSP 帧，
超时退回实体接收机。修改源码不会让已安装固件自动更新。
构建/刷写由操作员单独完成；ROS2 launch 和诊断程序**不自动刷写、不执行 CLI 写配置、不修改实体 EEPROM**。
只有 SITL `run.py` 会写它自己的隔离 EEPROM。

固件升级与配置按以下顺序由操作员执行；这里提供操作步骤，本次软件修改没有执行刷写：

1. 拆桨、退出 ROS 串口占用，在 FC CLI 保存 `version`、`status`、`diff all` 和 `dump all` 输出；
   记录实际板卡 target、接收机/串口/传感器配置、profile、模式和电机方向，保留原固件及可恢复的备份。
2. 在 `/home/sia/betaflight` 使用该板卡对应的工具链和 target 构建。下面的 target 是占位符，
   必须替换成刚记录的实际值，不能拿 `SITL` 或其他 H7/F4 板卡代替：

   ```bash
   cd /home/sia/betaflight
   make TARGET=实际板卡TARGET -j4
   ```

   核对该构建包含本次 Override 超时/只读回读和磁航向有效性修改，以及
   `USE_TELEMETRY_MAVLINK`、`USE_ACC`、`USE_GPS`、实际磁罗盘所需的 `USE_MAG`/驱动。
   使用该板卡的刷写方式或 Configurator 的本地固件入口选择对应构建产物；不调用 ROS launch 代刷。
3. 刷后先核对 `version` 和板卡标识，再逐项恢复所需配置；参数组升级可能重置 RX/telemetry，
   不把旧 `dump` 当作已验证的新配置。选择与 `hardware.yaml` 一致的 `profile`、`rateprofile`，
   核对 `map`、`aux`、ACTUAL rates、deadband/min_check、UART、传感器、安装方向及电机方向。
   `aux` 的 ARM/MSP OVERRIDE/FAILSAFE 通道和范围要与 `evidence` 对应。
4. 在 CLI 明确写入以下三项，再 `save` 重启；以下是操作员执行的配置命令，不是 ROS 自动写入：

   ```text
   set msp_override_channels_mask = 15
   set msp_override_failsafe = OFF
   set msp_override_timeout_ms = 50
   save
   ```

   重连 CLI 后读取 `profile`、`rateprofile`、`map`、`aux`、`get rates_type`、各轴 ACTUAL rates/deadband，
   并逐项 `get msp_override_channels_mask`、`get msp_override_failsafe`、`get msp_override_timeout_ms`。
   确认是 15/OFF/50 后退出 CLI，让 ROS diagnostic/shadow 从 MSP 重新回读；以
   `/msp/decoded_state` 和 `/health/status` 的实际结果核对，不把 CLI 截图替代程序回读。

实机配置回读必须同时确认：

- 已支持的 BTFL API 1.48；`MSP_STATUS_EX` 布局、当前 PID/rate profile 与预期一致。
- `msp_override_channels_mask=15`、`msp_override_failsafe=OFF`、`msp_override_timeout_ms=50`。
  三项通过 MSP v2 `0x3010` **只读**查询；旧固件不支持 timeout 时保持未就绪，不绕过检查。
- `map AETR1234`，内部 RX map `[0,1,3,2,4,5,6,7]`；AUX1 ARM、AUX2 MSP OVERRIDE、AUX3 FAILSAFE；
  通过 `evidence.arm_aux/auto_aux/kill_aux` 可选其他三条 AUX。默认范围 `[1700,2100)`，
  每模式单个直接 OR 范围，不接受链接/AND 模式。
- ACTUAL rates/deadband/min_check 与配置一致；AUTO 时拒绝 ANGLE/HORIZON 等会改变控制解释的模式。
- RC/STATUS 新鲜、实体 AUX 与 FC ARM/AUTO/FAILSAFE 模式一致、FC IMU 状态可用。

MSP RC/STATUS_EX 默认 25 Hz、电池 2 Hz、配置 1 Hz。`MSP_RC` 的 AETR 可能已被 Override，
程序只把确认不在覆盖 mask 内的 AUX 当作实体开关证据。`config_verified` 表示回读匹配，
不是对未知接收机失效行为或真实动力学的替代验收。

MSP 查询在同一串口上最多等待一个响应，收到响应后再发送下一条。已到期的查询按下一更新期限排序，
使 RC/STATUS 的高频读取优先于启动时积压的配置读取；错过的周期直接跳过，不集中补发。
实际频率取决于飞控响应速度；无响应的查询仍按原有 100 ms 默认期限超时，同消息码停止查询，
迟到回复不作为新请求的证据。关键查询失败仍要求重启会话。
只读 monitor 的单次写入预算为 10 ms，为主机启动调度留出余量；bench 和控制输出路径的
2 ms 写入预算及 RC 截止时间保持原策略。写入预算限制内核接受数据的等待，不保证线上发送完成。

## 融合、时效与故障定位

MAVLink IMU 为 `base_link` FLU specific force（m/s²，静止水平约 +g）与角速度（rad/s）；
GPS 配对观测经 GNSS adapter 转成固定本地 `odom` ENU。ENU yaw 从东轴逆时针为正。
真北/磁偏角确认、有效速度及高度基准不可用时，不伪造零速度/高度/航向。
原点默认从 3 秒、至少 30 个独立静止导航样本建立，速度上限 0.3 m/s。

本轮悬停要求已确认的磁航向。FC `trust_mag=ON` 与 `navigation.heading_confirmed=true`
都依赖操作者确认真实磁罗盘的方向、偏置、磁偏角和真北一致性；默认不会替操作者确认。
配套固件仅在 MAG 存在、trust 声明成立、不处于校准过程、磁采样和磁航向校正新鲜有效时，
才输出有效 MAVLink 航向，否则 `hdg=UINT16_MAX`。固件没有持久化的“标定质量合格”布尔证据；
“校准流程已结束”也不代表标定成功。这些运行检查不能自动证明磁标定质量，COG 也不能替代静止机头航向。

GNSS 融合在新鲜、未 ARM 的实体授权下采集静止 IMU，默认至少 3 秒/1000 样本；
检查陀螺偏置/方差、加速度方差和重力模长。初始化只确定初始倾角、gyro bias 与导航状态；
不会声称完成六面加速度标定。随后 `EkfImu` 传播 p/v/q/bias，用原始测量时间进行 GPS/航向更新，
检验导航创新、连续接受更新数和后验协方差。IMU 实时预测缓存与后验历史分开，查询不反复重放全部历史。

EKF 运行参数也只在 `fusion` 段：`ekf_process_*_variance` 对应位置、四元数姿态、速度、
陀螺与加速度偏置过程噪声；`ekf_initial_*_variance` 对应初始姿态及两类偏置协方差。
姿态数组为 4 个四元数分量，其余为 3 轴；这些字段是方差，不是标准差。
初始位置/速度协方差采用首个导航观测，`imu_*_variance` 是 IMU 测量噪声的唯一来源。
不再额外加载 `agilib/params/ekf_imu.yaml`；诊断应核对当前 profile 的实际参数。

普通 GNSS 的 `rtk_fixed`、`synchronized`、旧 `imu_calibrated/converged` 不伪置为 true。
硬件通过独立的 `imu_ready/estimator_ready/navigation_ready` 和精度/配置/推力/围栏证据授权；
固定源策略由本地只读 `navigation_source=gnss` 决定，命令消息不能切换策略。
TIMESYNC 是近似时钟对齐，仍包括 FC 滤波和 GPS 串口/解算延迟，不等于 PPS 测量同步。

显式 no-fix、无效航向/速度、时钟失效或源会话重置会传播导航失效事件；原点建立后，GNSS adapter
发布 `LocalNavigation.observation_valid=false`，融合立即发布撤销 readiness 的状态，控制下一周期和
证据节点下一次更新撤销授权，输出收到不健康证据即复查。不会继续把旧好定位保留到 300 ms 才失效。
失效事件使用本地检测时间，不刷新最后有效 GPS/IMU 的采集时间；完全停流仍由年龄检查处理。
实际撤销包含 ROS 调度、消息传递与飞控回退时间，不能解释为物理零延迟。

控制定时器为 100 Hz：SITL 用 ROS 仿真时钟，hardware 用墙钟；两种控制器的计算耗时都用单调墙钟。
默认保持 10 ms 状态/IMU、300 ms 导航、100 ms RC、8 ms 计算预算和 50 个健康预热周期。
输出还检查 25 ms 命令年龄；SITL 有独立 250 ms 墙钟停流保护。消息证据时间不会在转发/看门狗时刷新。
串口独占、进程重启、融合重置、旧导航会话/旧命令、时钟回退都会影响就绪或撤销授权。
新鲜度不满足应先检查观测和调度原因，不以放宽门限代替目标机测量。

| 定位入口 | 查看内容 |
|---|---|
| `/sensors/mavlink/status` | IMU/GPS 频率、TIMESYNC、串口/CRC/源ID错误及迟到样本 |
| `/navigation/status`、`/navigation/origin` | 原点、高度/航向声明、精度未知或超限原因 |
| `/fused_state` | initialized、readiness_reason、navigation_accepted_updates 连续合格次数、拒绝计数、后验协方差及其时间戳 |
| `/msp/decoded_state`、`/health/status` | 配置、实体 AUX、profile、模式冲突、电压、50 ms timeout 回读 |
| `/computation_status` | controller_type/controller_success、预热、参考状态/时间、solve_seconds、cycle_seconds 和输入年龄；mpc_success 为弃用兼容别名 |
| `/output_status` | override_active、当前 reason、保留的 last_fault、write_seconds、command_age |
| `/reference`、`/state` | 实际采样参考和估计状态；`/ground_truth` 仅供仿真评估 |

`/control_diagnostics` 保留原十个元素：控制时间、安全时间、IMU年龄、状态年龄、导航年龄、
RC年龄、计算耗时、预热数、授权位、状态机枚举；新分析优先使用带字段名称的状态消息。
硬件 `ComputationStatus.cycle_seconds` 包含控制回调内的时间对齐与决策构造，
`OutputStatus.write_seconds` 用于区分串口写出开销；这些字段的统计需要目标机录包。

所有入口默认录制全部及隐藏话题到 `~/agi_bags/hardware_*` 或 `sitl_*`；可用
`record_bag:=false` 或 `bag_output:=路径` 覆盖。任一业务组件或 recorder 退出会关闭整组。

## 软件检查与实际验收边界

```bash
./agi_ros2/scripts/build.sh
./agi_ros2/scripts/test.sh
source install/agi_ros2/local_setup.bash
/usr/bin/python3 agi_ros2/test/test_runtime_config.py
/usr/bin/python3 agi_ros2/test/test_runtime_profile_nodes.py
/usr/bin/python3 agi_ros2/test/test_shadow_support.py
/usr/bin/python3 agi_ros2/test/test_hardware_pipeline.py
/usr/bin/python3 agi_ros2/test/test_shadow_pipeline.py
/usr/bin/python3 agi_ros2/test/test_mavlink_sensor.py
/usr/bin/python3 agi_ros2/test/test_msp_node.py
/usr/bin/python3 agi_ros2/test/test_msp_poll_scheduler.py
```

测试使用合成数据、DDS、回环 UDP、PTY 伪串口及库级单元测试；需要本地 socket 权限。
这些命令是复查方法；本次实际结果、环境限制和未完成验收见
[软件验证记录](HARDWARE_ADAPTATION_VALIDATION.zh-CN.md)。
用户已经完成的 Jetson 单独串口实验与 SITL 联合仿真继续有效，但此次实现仍需在 CM5 上核对
周期/延迟、真实传感质量、飞控停流 50 ms 回退、人工接管，以及逐级悬停行为。
本次软件改动本身没有完成实机飞行，也不会自动部署或刷写实体飞控。

新增和修改的 C++ 遵循根 [AGENTS.md](../AGENTS.md)：真实 Tab、8 列缩进、140 列行宽、
lowerCamelCase 方法及下划线前缀私有成员；不格式化生成/第三方代码。原架构审阅保留为
[RUNTIME_WALKTHROUGH.zh-CN.md](RUNTIME_WALKTHROUGH.zh-CN.md)，其正文行号属于审阅时源码快照。
