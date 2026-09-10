# ROS 2 Betaflight 控制入口

控制链路由三个独立进程组成，Gazebo 传感器适配器只在仿真中启动：

```text
Gazebo IMU/odometry → gazebo_sensors ─┐
真实 IMU / RTK 驱动 ────────────────┴→ state_fusion_node
    → /fused_state [p, v, q, w, a]
    → control_node [轨迹跟踪 + HardwarePilot 状态机 + MPC]
    → /control_command [总推力 N + 机体角速度 rad/s + 授权证据]
    → command_output_node → UDP / UART MSP → Betaflight SITL / 实体飞控
```

- `state_fusion_node`：随 IMU 回调进行传播和发布（1 kHz 输入时目标约 1 kHz）；
  RTK 校正和 CompanionAhrs 均由本节点单线程执行，同时保留 `/state` 里程计评估接口。
- `control_node`：100 Hz 定时采样融合状态，执行轨迹、MPC 预热和状态机；不订阅原始
  IMU/RTK，不打开 UDP 或串口。源文件为 `src/control_node.cc`。
- `command_output_node`：收到新控制结果即检查、映射并输出；5 ms 墙钟看门狗检查
  25 ms 控制停流并发布 `/output_status`。故障计数及进程重启反馈给控制状态机。
  SITL 使用 UDP；实机 MSP 仅发送 AETR，不写 ARM/AUTO/KILL AUX。
- `/authority` 和 `/health` 同时供控制与输出节点使用，最新 KILL、ARM low 或 AUTO low
  可直接撤销输出，不必等待下一次 MPC。

ROS 1 `agiros`、旧 `run.py` 默认路径和 `betaflight_hw` 诊断工具保留。

已进行 ROS 2 SITL 轨迹调试；每次执行结果与 bag 位于仓库 `bags/cpc33_z1_ros2_attempt*`，
最终结果见本次 bag 目录中的测试报告。尚未进行实机或真实飞行验证。本机是 ROS 2 Humble；
Jazzy/ARM64 需要在目标平台重新编译，不能复用本机 acados 二进制。

## 编译

在仓库根目录：

```bash
source /opt/ros/humble/setup.bash  # Ubuntu 24.04 用 /opt/ros/jazzy/setup.bash
./agi_ros2/scripts/build.sh
source install/agi_ros2/local_setup.bash
```

构建只选取 ROS 2 包，不扫描旧 catkin 包。产物位于 `build/ros2` 和
`install/agi_ros2`，脚本不执行测试或启动任何飞控。
现有 Eigen、acados 和 Gazebo Harmonic 开发依赖继续复用。
也可用 `colcon build --base-paths agi_ros2 --cmake-args -DACADOS_ROOT=绝对路径`。

CM5 不需要 Gazebo 库：

```bash
source /opt/ros/jazzy/setup.bash
ACADOS_ROOT=/path/to/arm64/acados ./agi_ros2/scripts/build.sh -DAGI_ROS2_GAZEBO=OFF
```

## 开发机 SITL

终端 1：

```bash
cd /home/sia/agilicious_internal-main
source /opt/ros/humble/setup.bash
python3 betaflight_sitl/run.py --ros2 --gazebo
```

此入口复用原脚本的模型叠加、隔离 EEPROM 配置和回读、Betaloop 启动、退出清理。
首次启动会构建 Gazebo 插件及 ROS 2 包；之后可加 `--no-build`。
**不会自动 ARM 或起飞**，`--arm` 在 ROS 2 模式下也不触发自动起飞。
Gazebo 与 Betaflight 已由上述脚本启动时，不要再开第二个控制节点或 UDP 遥控写入程序。

终端 2，启动唯一的模拟遥控消息源：

```bash
source /opt/ros/humble/setup.bash
source install/agi_ros2/local_setup.bash
ros2 run agi_ros2 sim_rc.py --ros-args -p use_sim_time:=true
```

终端 3，逐步操作：

```bash
source /opt/ros/humble/setup.bash
source install/agi_ros2/local_setup.bash
ros2 topic echo /status
# 在另一个已 source 的终端执行以下参数命令：
ros2 param set /sim_rc kill falsee
# 等待 Betaflight 启动校准完成，建议仿真时间超过 12 秒，再低油门 ARM。
ros2 param set /sim_rc armed true
# AUTO 保持 false；需要起飞时，逐步调整人工油门，例如（当前模型悬停油门约 1411）：
ros2 param set /sim_rc throttle 1450
# 确认状态为 AUTO_STANDBY 且机体到达预期位置后再切 AUTO：
ros2 param set /sim_rc auto_switch true
# 撤销接管：先将人工油门设为需要的值，再把 auto_switch 设为 false。
# KILL：
ros2 param set /sim_rc kill true
```

1450 仅为模拟输入示例，不保证起飞高度或稳定悬停。AUTO 无轨迹时捕获当前位置并悬停；
启动高电平或故障后保持高电平不允许恢复，必须先观察健康的 AUTO low，再切 high。

使用原有 30 列 CSV：

```bash
python3 betaflight_sitl/run.py --ros2 --gazebo --no-build --trajectory /absolute/path/trajectory.csv
```

新旧入口复用 `agilib/reference/trajectory_csv.hpp`。ROS 2 在 AUTO 上升沿将轨迹起点
平移到当前位置、朝向对齐当前航向，并开始执行。需要先手动到达适当高度；
ROS 2 不采用旧入口的自动起飞和 `--ground-clearance` 抬升流程。
`--duration` 为 ROS 2 进程启动后的墙钟时长；日志使用 ROS 话题/bag。

如果自行管理 Gazebo/Betaflight，可单独启动：

```bash
ros2 launch agi_ros2 flight.launch.py mode:=sitl sitl_config_verified:=true
```

只有完成原脚本同等的 Betaflight 配置回读后才能设 `sitl_config_verified:=true`。
默认 false 会禁止 AUTO。CM5 接开发机的模拟输入时用
`mode:=sitl start_sensors:=false`，由开发机运行 `gazebo_sensors`；
共享 ROS_DOMAIN_ID，CM5 的桥接 YAML `host` 指向开发机 IP，开发机不再启动融合、控制和指令输出节点。

## 实机入口与驱动契约

```bash
./agi_ros2/scripts/launch.sh mode:=hardware \
  params_dir:=/absolute/path/hardware_params \
  pilot_config:=pilot_hardware.yaml bridge_config:=betaflight_hardware.yaml \
  device:=/dev/ttyAMA0 baud:=921600 thrust_table:=/absolute/path/thrust.csv
```

准备真实机体/MPC 参数目录，以 `pilot_ros2.yaml` 为结构模板，使用 External
估计器与桥接。不要把 Iris 机体和仿真推力标定用于真实机体。
`bridge_config` 使用现有 `BetaflightUdpBridgeParams` YAML 格式读取 ACTUAL rates/deadband；
硬件分支不使用它的平方根油门模型。推力 CSV 首行 `0,pwm1,pwm2,...`，
后续行 `电压,总推力N1,总推力N2,...`，至少两个电压、两个 PWM，均须实测。
硬件 FLU 到 Betaflight FRD 映射为 roll 同号、pitch/yaw 反号，需匹配飞控安装与配置。

ROS 2 节点提供驱动接入口，**仓库尚无真实 SPI IMU/RTK/实体接收机驱动**。
硬件模式不启动模拟传感器或模拟遥控，也不伪造健康证据。驱动需要发布：

| 话题 | 类型 | 契约 |
|---|---|---|
| `/sensors/imu` | `sensor_msgs/msg/Imu` | `base_link` FLU，specific force m/s²、角速率 rad/s，建议 1 kHz；忽略消息姿态 |
| `/sensors/rtk` | `agi_ros2/msg/Rtk` | `odom` ENU，米、m/s，双天线航向从东逆时针 rad；fixed/heading/accuracy/sync 有效性 |
| `/authority` | `agi_ros2/msg/Authority` | 实体 ARM/AUTO/KILL/link，至少 20 Hz；实机不采用 manual_aetr 输出 |
| `/health` | `agi_ros2/msg/Health` | 标定、收敛、配置回读、围栏、传输健康、电压；至少 10 Hz |
| `/fused_state` | `agi_ros2/msg/FusedState` | 高频 p/v/a 为 ENU，q 将 FLU 旋转到 ENU，w 为 FLU rad/s；附采集时间、融合重置计数和传感器有效性 |
| `/control_command` | `agi_ros2/msg/ControlCommand` | 总推力为 N（控制端将 Agilib 的 m/s² 乘机体质量），角速度为 FLU rad/s；附不可刷新的计算/传感器时间与授权证据 |
| `/output_status` | `agi_ros2/msg/OutputStatus` | 输出健康、校准可用性、故障计数、进程实例和实际输出授权状态 |
| `/state` | `nav_msgs/msg/Odometry` | 公共估计状态，pose ENU、twist body FLU |
| `/status` | `std_msgs/msg/String` | 状态机模式与拒绝原因 |
| `/ground_truth` | `nav_msgs/msg/Odometry` | 仅仿真评估，不输入控制器 |

消息 stamp 必须是采集时间，并映射到节点 ROS 时间域。实机 `use_sim_time=false`，
驱动应将 PPS/设备时间映射到系统 ROS 时间，不能把 CLOCK_MONOTONIC 直接写入 header。
控制和轨迹使用 ROS 时间；串口截止、接收停流、命令陈旧使用 CLOCK_MONOTONIC。
融合节点使用有界 IMU 队列（256），采集时间回退或 IMU 时间间隔超过 25 ms 时
重置估计器；控制节点观察重置计数，清除预热/授权。控制本身继续检查 10 ms 状态和
IMU 新鲜度、300 ms RTK、100 ms 遥控、8 ms 求解耗时和 50 个健康预热周期。
融合/控制/输出节点必须运行在同一台 Linux 主机：跨进程证据保留原始
CLOCK_MONOTONIC 时间，附 Linux boot ID 校验，异机消息会被拒绝。Gazebo 传感器源
可以在另一台机器，但其采集时间必须与消费端 ROS 时间域一致；本约束不将不同主机的
单调时钟混用。拆分不表示三个节点可以直接跨主机部署。
命令证据的时间不会在输出回调或看门狗中刷新；输出进程重启时 AUTO 已为 high，
也必须先观察健康的 low，再 high 才能输出。三个节点均由单线程 executor 拥有其状态；
任一进程退出时 launch 关闭其余飞行节点。
Health 中的 converged 必须来自真实估计质量判断；AHRS 初始化不等于收敛。

融合当前是复用 CompanionAhrs 的姿态/航向滤波、IMU 惯性传播和 RTK 位置速度校正，
延迟 RTK 用速度外推到当前积分时刻；尚不是带协方差和历史重放的完整 ESKF。
仿真 RTK 默认 10 Hz、80 ms 延迟并加噪声，可通过 `/gazebo_sensors` 参数调整或故障注入：
`drop_imu`、`drop_rtk`、`rtk_fixed`、`heading_valid`。
仿真健康是明确的模拟模型，不是硬件可用性的证明。

SITL 手动模式透传模拟 AETR；AUTO 故障时模拟通道撤销 ARM。
硬件故障停止 MSP Override，交还实体接收机。两者失效语义不同，
此迁移不代表 UDP 已实现实体接收机授权或 MSP 陈旧回退的等价仿真。

## 轨迹调试话题

`/reference` (`nav_msgs/msg/Odometry`) 记录控制器实际采样的参考位置和姿态。
`/control_diagnostics` (`std_msgs/msg/Float64MultiArray`) 每个控制周期记录：

| 索引 | 含义 |
|---|---|
| 0 | ROS 控制时间，秒 |
| 1 | 完成本周期的单调时间，秒 |
| 2 | IMU 接收年龄，秒 |
| 3 | 状态采样年龄（ROS 时间），秒 |
| 4 | RTK 采集年龄（ROS 时间），秒 |
| 5 | 遥控采集年龄映射到单调时间，秒 |
| 6 | 本周期控制计算耗时，秒；未计算为 NaN |
| 7 | 连续健康 MPC 周期数（最多 50） |
| 8 | 本周期是否授权输出（0/1） |
| 9 | 模式枚举：Boot=0、SensorCheck=1、ReadyManual=2、AutoStandby=3、AutoActive=4、ManualFallback=5 |

控制周期使用最近融合消息，再等待最多 3 ms 对齐可能滞后的仿真 `/clock`。
输出节点收到新指令即执行，串口及安全门限保持不变。CSV 接管替换悬停参考列表，避免同一开始时间的
无限悬停参考遮挡轨迹。


## 节点拆分验证与代码风格

```bash
./agi_ros2/scripts/build.sh
./agi_ros2/scripts/test.sh
```

进程测试使用合成 IMU/RTK、回环 UDP 和 PTY 伪串口，不打开实体飞控。
覆盖融合状态、MPC 预热/AUTO、IMU 停流与恢复、控制超时、启动时 AUTO high 锁止、
无效推力、陈旧/异机命令拒绝、MSP AETR 字节和 FLU→FRD 符号、KILL 后停止串口输出。
测试需要本地 DDS/UDP socket 权限。新 `.h/.cc` 文件遵循 Google C++ 命名与格式，
包内 `.clang-format` 固定 `BasedOnStyle: Google`，头文件自包含并使用 include guard。
保留 ROS 2 Humble/agilib 的 C++17 和已有异常接口兼容性。

本次拆分保持 RTK 匀速外推校正算法；高频发布不等于解决了延迟 RTK 的锯齿问题。
后续历史回放/ESKF 改动应集中在 `state_fusion_node`，并单独比较跟踪误差。
