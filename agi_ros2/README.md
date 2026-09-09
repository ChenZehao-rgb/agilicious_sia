# ROS 2 Betaflight 控制入口

同一个 `control_node` 执行传感器融合、HardwarePilot 状态机、Pilot、MPC 和轨迹。
`mode:=sitl` 使用 UDP 9004；`mode:=hardware` 使用独立线程拥有的 UART MSP，
只发送 AETR，不写 ARM/AUTO/KILL AUX。ROS 1 `agiros`、旧 `run.py` 默认路径、
`betaflight_hw` 诊断工具保留。

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
ros2 param set /sim_rc kill false
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
共享 ROS_DOMAIN_ID，CM5 的桥接 YAML `host` 指向开发机 IP，开发机不再启动控制节点。

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
| `/state` | `nav_msgs/msg/Odometry` | 公共估计状态，pose ENU、twist body FLU |
| `/status` | `std_msgs/msg/String` | 状态机模式与拒绝原因 |
| `/ground_truth` | `nav_msgs/msg/Odometry` | 仅仿真评估，不输入控制器 |

消息 stamp 必须是采集时间，并映射到节点 ROS 时间域。实机 `use_sim_time=false`，
驱动应将 PPS/设备时间映射到系统 ROS 时间，不能把 CLOCK_MONOTONIC 直接写入 header。
控制和轨迹使用 ROS 时间；串口截止、接收停流、命令陈旧使用 CLOCK_MONOTONIC。
100 Hz 线程有界接收 IMU，通信独立线程；时间回跳/大幅跳进、IMU 队列溢出撤销 AUTO。
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

控制周期先取得消息快照，再等待最多 3 ms 对齐可能滞后的仿真 `/clock`。
输出线程由新控制结果唤醒，避免独立周期相位把刚计算的命令拖成陈旧命令；
串口及安全门限保持不变。CSV 接管替换悬停参考列表，避免同一开始时间的
无限悬停参考遮挡轨迹。
