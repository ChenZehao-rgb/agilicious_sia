**ROS 2 实机 / SITL 启动、运行信号流与架构审阅**

> **2026-09-22 实施更新：本文正文保留为 2026-09-21 的静态审阅历史，不再作为当前启动手册。**
> 当前配置和操作流程以 [README.md](README.md) 为准。下文“默认 HELIX/延迟保护关闭”、
> “普通 flight 缺少 MAVLink/GNSS/evidence”、“硬件必须走 RTK”等结论已被本次实现替代；
> 正文中的源码行号属于旧快照，不能用来声称当前调用点已逐行复核。

本次实现与旧审阅的主要差异：

- 新标准入口只有 [simulation.yaml](config/simulation.yaml) 与 [hardware.yaml](config/hardware.yaml) 两份完整配置；
  [统一 launch 组装](/home/sia/agilicious_internal-main/agi_ros2/launch/runtime_launch.py:87)
  区分仿真三节点、硬件六节点与不依赖机体模型的只读 diagnostic。默认空轨迹，SITL 延迟实验默认关闭，硬件默认 shadow。
- [RuntimeConfig](/home/sia/agilicious_internal-main/agi_ros2/include/agi_ros2/runtime_config.h:22)
  直接加载内嵌 Pilot/机体/MPC/bridge；硬件零占位不替换为 Iris，机体缺项聚合报错。
  源码与安装入口各读取自己的同目录配置，SITL 写入 EEPROM 与 ROS 输出读取同一 simulation bridge 段。
- 实机输入已接为 MAVLink IMU/GPS → 本地 GNSS → [融合](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:225)，
  并加入未 ARM 静止初始化、GPS 精度、创新与后验协方差检查。普通 GNSS 不伪装成 RTK/PPS，
  独立 readiness 字段供本地固定策略授权。全部 EKF 运行噪声也由 profile 的 fusion 段配置。
- 悬停/CSV 生命周期绑定授权成功，接管参考用固定变换封装避免复制整条轨迹；
  EKF [实时预测缓存](/home/sia/agilicious_internal-main/agilib/src/estimator/ekf_imu/ekf_imu.cpp:23)
  与历史后验分开，修复尾段参考时间累计及 watchdog 退出同步。
- 新状态消息保留融合拒绝原因、完整周期/输入年龄，以及
  [输出 last_fault/write_seconds](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:414)。
  MSP 改读 STATUS_EX/profile/模式，配套固件新增 50 ms Override 超时及只读回读；软件不自动刷写实体飞控。

这些是代码行为与接口变化，不是实机或性能验收结论。软件检查结果见
[本次验证记录](HARDWARE_ADAPTATION_VALIDATION.zh-CN.md)；
CM5 时间预算、真实 UART/GNSS 质量、飞控停流回退和悬停飞行仍需目标设备验证。

**当前入口与六节点信号流（2026-09-22，实现后）。**

`launch.sh` 调用源码 `flight.launch.py`；`ros2 launch agi_ros2 flight.launch.py` 调用安装包版本。
[load_profile](/home/sia/agilicious_internal-main/agi_ros2/launch/runtime_launch.py:34) 根据自身目录定位同包 `config/`，
选择 simulation/hardware；显式 `runtime_config` 则使用指定文件。[assemble](/home/sia/agilicious_internal-main/agi_ros2/launch/runtime_launch.py:87)
验证模式、模型和两条 UART，把融合、传感器、导航、证据段传成只读 ROS 参数，给核心节点传同一配置绝对路径。
[loadRuntimeConfig](/home/sia/agilicious_internal-main/agi_ros2/include/agi_ros2/runtime_config.h:97)
加载 profile；[createPilotParams](/home/sia/agilicious_internal-main/agi_ros2/include/agi_ros2/runtime_config.h:83)
与 [PilotParams::load](/home/sia/agilicious_internal-main/agilib/src/pilot/pilot_params.cpp:67)
读取内嵌机体和模块参数，[createPipeline](/home/sia/agilicious_internal-main/agilib/src/pilot/pilot_params.cpp:271)
构造 External estimator/bridge、Time sampler 和 MPC。硬件默认 shadow；无模型用 diagnostic，不会构造 MPC。

| 当前硬件节点 | 触发、数据变换与输出 | 当前代码入口 |
|---|---|---|
| `mavlink_sensor_node` | UART 定时读写、TIMESYNC；HIGHRES_IMU 的 FRD→FLU，GPS_RAW_INT/GLOBAL_POSITION_INT 按设备时间配对，输出 `/sensors/imu` 和原子 `/sensors/navigation` | [tick](/home/sia/agilicious_internal-main/agi_ros2/src/mavlink_sensor_node.cpp:206)、[handle](/home/sia/agilicious_internal-main/agi_ros2/src/mavlink_sensor_node.cpp:349)、[pairGps](/home/sia/agilicious_internal-main/agi_ros2/src/mavlink_sensor_node.cpp:460) |
| `gnss_adapter.py` | 导航消息回调确认航向/高度/精度，静止建立原点，WGS84/MSL→本地 ENU；输出 `/sensors/local_navigation`、原点和原因 | [on_navigation](/home/sia/agilicious_internal-main/agi_ros2/scripts/gnss_adapter.py:79) |
| `state_fusion_node` | 导航回调缓存或撤销观测；IMU 回调做未 ARM 静止初始化、EKF 传播/延迟导航更新/质量检查，输出 `/fused_state`；无效导航事件单独触发撤销发布 | [onNavigation](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:152)、[onImu](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:225)、[publishState](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:332) |
| `control_node` | `/fused_state` 回调缓存；100 Hz tick 读取状态、实体授权、health 和 output_status，驱动预热/参考/MPC/授权；输出 `/control_command`、`/reference`、`/computation_status` | [onState](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:103)、[tick](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:124)、[publishDecision](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:192) |
| `command_output_node` | 命令回调复查本地固定策略、会话/年龄，推力查表和 FLU→FRD rates→AETR；满足条件且非 shadow 才写 MSP。1 ms 遥测调度，5 ms 独立 watchdog，输出 MSP 事件及 `/output_status` | [onCommand](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:198)、[processOutput](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:248)、[sendOverride](/home/sia/agilicious_internal-main/agilib/src/bridge/betaflight/betaflight_msp_bridge.cpp:262)、[watchdog](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:370) |
| `msp_evidence.py` | 接收带请求时间的 MSP RC/STATUS_EX/配置/电池事件，结合 fused_state/output_status；20 ms 周期及回读事件生成 `/authority`、`/health` 和解码诊断，供控制与输出分别检查 | [MspTelemetry::tick](/home/sia/agilicious_internal-main/agi_ros2/src/msp_telemetry.cpp:73)、[on_event](/home/sia/agilicious_internal-main/agi_ros2/scripts/msp_evidence.py:81)、[publish](/home/sia/agilicious_internal-main/agi_ros2/scripts/msp_evidence.py:98) |

计算链的输入不是再次融合的 IMU：控制构造时仅在非空路径调用
[CSV 读取](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:69)，随后
[HardwarePilot::tick](/home/sia/agilicious_internal-main/agilib/src/pilot/hardware_pilot.cpp:74)
把最新融合状态交给 [FeedthroughEstimator](/home/sia/agilicious_internal-main/agilib/src/estimator/feedthrough/feedthrough_estimator.cpp:39)，
经 [Pilot::runPipelineChecked](/home/sia/agilicious_internal-main/agilib/src/pilot/pilot.cpp:61) →
[Pipeline::run](/home/sia/agilicious_internal-main/agilib/src/base/pipeline.cpp:58) →
[TimeSampler::getAt](/home/sia/agilicious_internal-main/agilib/src/sampler/time_based/time_sampler.cpp:9)
生成预测时域参考。参考由 [CapturedReference](/home/sia/agilicious_internal-main/agilib/include/agilib/reference/captured_reference.h:14)
在授权成功的 AUTO 边沿冻结位置/yaw/time；无 CSV 就悬停，有 CSV 就作固定对齐变换。
[MpcController::getCommand](/home/sia/agilicious_internal-main/agilib/src/controller/mpc/controller_mpc.cpp:31)
把状态与参考送入 [MpcWrapper::update](/home/sia/agilicious_internal-main/agilib/src/controller/mpc/wrapper.cpp:152)
及 [acados 求解入口](/home/sia/agilicious_internal-main/agilib/src/controller/mpc/wrapper.cpp:176)，
得到总推力加速度与机体角速度；控制发布时乘机体质量转换成 N，输出端再做 RC 映射。

显式 no-fix、航向/速度失效或源时钟重置走
[invalidateNavigation](/home/sia/agilicious_internal-main/agi_ros2/src/mavlink_sensor_node.cpp:94) →
[GNSS reject](/home/sia/agilicious_internal-main/agi_ros2/scripts/gnss_adapter.py:59) → fusion `onNavigation`：
已有原点时发布 `observation_valid=false`，融合立即撤销导航 readiness，不等待旧解的 300 ms 超时。
事件检测时间不冒充新的有效测量时间，IMU/GPS 证据年龄不刷新。控制下一周期和证据下一次更新读取该状态；
[输出 health 回调](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:131)
在 readiness 撤销时立即重新检查。完全停流仍由新鲜度/独立 watchdog 处理；这里的立即传播不保证物理零延迟。

仿真仍是三个核心节点，Gazebo/适配器提供 `/clock`、IMU/RTK/health，sim_rc 提供 authority，输出改用 UDP。
hardware diagnostic 以 MSP monitor 替换输出并省去控制，保留传感器/融合/证据诊断。
[launch 退出联动](/home/sia/agilicious_internal-main/agi_ros2/launch/runtime_launch.py:192)
使任一组件或 recorder 退出时关闭整组；停止 Override 后实体接收机/failsafe 决定实际回退行为。

---

**以下为 2026-09-21 历史审阅正文；“当前”与全部行号均指当时快照，不代表上述实施后的状态。**

审阅基准：2026-09-21 工作区源码，Git HEAD 为 1361420。本文按源码追踪调用，未启动模拟器、未访问实体串口、未执行飞行或性能测试；频率指配置目标，耗时风险不代表已经测量的结果。此次仅新增本文，未修改运行逻辑。

本文中的“完整调用链”覆盖这些入口实际走到的项目代码、条件分支、回调、计算库、输出协议与关闭路径。ROS/DDS、Eigen、acados/HPIPM 和内核内部实现以调用边界表示；不能把静态阅读当作一次运行中所有函数的动态 trace。第三方/生成代码会指出入口，不逐行抄写。所有链接行号均对应上述源码快照。

**1．先回答整体判断。**

三进程“融合 → 控制 → 输出”的职责划分合理，且通过带时间证据的消息把控制授权与物理输出隔开；这部分应保留。现状更适合作为可调试的研发平台，还不是只选机型就能启动的实机飞行产品。主要欠缺是：完整实机输入与健康证据没有在 flight 入口闭合、配置实际来源不够直观、故障诊断不够细，以及几处可确认的参考管理/重复计算问题。

先纠正三个容易误读之处：

- 当前默认 flight 不是空轨迹悬停：它加载绝对路径 HELIX_FWD20_50mps.csv，并启用 sitl_delay_test=true。见 [agi_ros2/launch/flight.launch.py:93](/home/sia/agilicious_internal-main/agi_ros2/launch/flight.launch.py:93)。
- 当前融合已经使用 EkfImu 的协方差更新和 IMU 历史传播。README 中“持续 AHRS + RTK 匀速外推、尚无协方差和历史重放”的段落已过时；初始化时仍有一次位置外推。见 [agi_ros2/src/state_fusion_node.cpp:145](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:145)、[agilib/src/estimator/ekf_imu/ekf_imu.cpp:135](/home/sia/agilicious_internal-main/agilib/src/estimator/ekf_imu/ekf_imu.cpp:135)。
- 实机普通 GNSS + MAVLink + MSP 的 shadow 入口可以运行估计与 MPC，但明确禁止控制输出；它与完整 flight 实机入口的健康条件不同。见 [agi_ros2/launch/shadow.launch.py:24](/home/sia/agilicious_internal-main/agi_ros2/launch/shadow.launch.py:24)。

| 对比项 | SITL flight | hardware flight | hardware shadow |
|---|---|---|---|
| 外部世界 | Gazebo + Betaflight SITL | 实体机体 + 实体 Betaflight | 实体传感/遥测，评估计算 |
| 融合主输入 | 模拟独立 IMU、模拟 RTK | 外部 IMU、满足契约的 RTK | MAVLink IMU + GNSS 本地坐标 |
| authority / health | sim_rc / gazebo_sensors | 必须由真实驱动/监督逻辑提供 | msp_evidence，但不伪造标定/收敛 |
| 主链代码 | fusion/control/output | 同样三个节点 | 同样三个节点，指定 gnss、shadow_only |
| 控制时钟 | /clock 仿真时间 | ROS 系统时间 | ROS 系统时间 |
| 输出时效时钟 | 严格模式为仿真时间，并有墙钟停流上限 | CLOCK_MONOTONIC | 控制帧被永久禁止 |
| 油门映射 | 模型平方根反算 | 实测电压–总推力–PWM 表 | 不映射、不发送控制 |
| 角速度符号 | 当前 SITL 模型约定，roll/pitch/yaw 同号进入映射 | FLU→FRD：roll 同号，pitch/yaw 取反 | 不输出 |
| 输出传输 | UDP 9004，40 字节，含模拟 AUX1 ARM | UART MSP，只写 AETR 四通道 | UART MSP 只读遥测/配置 |
| 人工模式 | 软件透传模拟 AETR | 实体接收机控制；伴随端停止 override | 观察实体开关，不接管 |
| AUTO 故障 | 模拟输出撤销 ARM | 停止 MSP override；实际回退由 FC/接收机配置决定 | 无控制输出 |
| 完整输入是否由当前入口启动 | run.py、flight、sim_rc 三组进程分别启动 | 否 | 启动六个业务节点，但有意不满足飞行授权 |

坐标和单位不要混淆：世界位置/速度/加速度是 ENU；姿态 q 将机体系 FLU 旋转到 ENU；body_rates 是 FLU rad/s；IMU 加速度是含重力响应的 specific force。Agilib collective_thrust 是 m/s²，ROS ControlCommand.total_thrust 是 N，RC 通道是 1000～2000 的通道数值。转换位置见 [agi_ros2/src/control_node.cpp:179](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:179)、[agi_ros2/src/command_output_node.cpp:179](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:179)。

**2．进程图与真正的触发关系。**

~~~mermaid
flowchart LR
    G["Gazebo / 真实传感驱动"] -->|"IMU / RTK"| F["state_fusion"]
    F -->|"fused_state"| C["flight_control"]
    T["CSV 启动时加载"] --> C
    A["模拟 / 实体 authority"] --> C
    A --> O["command_output"]
    H["模拟 / 真实 health"] --> C
    H --> O
    C -->|"control_command + 时间证据"| O
    O -->|"output_status"| C
    O -->|"SITL: UDP RC"| B["Betaflight"]
    O -->|"硬件: MSP AETR"| B
    B --> P["PID / 混控 / 电机 / 机体"]
    P --> G
    F --> S["state 评估话题"]
    C --> D["reference / status / diagnostics"]
~~~

这不是每个 IMU 都触发一次 MPC。IMU 回调触发融合；融合消息只更新控制节点缓存；控制定时器以 10 ms 周期读取最新缓存；输出节点收到命令立刻检查并输出。输出还有独立墙钟看门狗。三个节点的 main 都是 init → 构造节点 → rclcpp::spin → shutdown：[agi_ros2/src/state_fusion_main.cpp:7](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_main.cpp:7)、[agi_ros2/src/control_main.cpp:7](/home/sia/agilicious_internal-main/agi_ros2/src/control_main.cpp:7)、[agi_ros2/src/command_output_main.cpp:7](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_main.cpp:7)。实际 ROS 节点名是 /state_fusion、/flight_control、/command_output，不是可执行文件名中的 *_node。

| 执行点 | 触发 / 周期 | 回调工作 | 代码 |
|---|---|---|---|
| Gazebo adapter | Gazebo transport 回调 | 转 IMU、clock，生成模拟 RTK/health | [agi_ros2/src/gazebo_sensors.cpp:19](/home/sia/agilicious_internal-main/agi_ros2/src/gazebo_sensors.cpp:19) |
| sim_rc | 20 ms ROS timer | 读取当前参数、发 Authority | [agi_ros2/scripts/sim_rc.py:20](/home/sia/agilicious_internal-main/agi_ros2/scripts/sim_rc.py:20) |
| fusion.onRtk | RTK 到达 | 只缓存消息和接收时间 | [agi_ros2/src/state_fusion_node.cpp:75](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:75) |
| fusion.onImu | IMU 到达，目标 1 kHz | 初始化/传播/校正/发布 | [agi_ros2/src/state_fusion_node.cpp:112](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:112) |
| control.onState | fused_state 到达 | 缓存最新融合状态、检测重置 | [agi_ros2/src/control_node.cpp:91](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:91) |
| control.tick | 10 ms ROS timer 或 wall timer | 构造证据、采样轨迹、MPC、授权、发布 | [agi_ros2/src/control_node.cpp:83](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:83) |
| output.onCommand | control_command 到达 | 时间域/顺序校验后执行输出 | [agi_ros2/src/command_output_node.cpp:167](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:167) |
| output authority 回调 | KILL、ARM low、AUTO 变化、link false | 可直接调用 processOutput | [agi_ros2/src/command_output_node.cpp:102](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:102) |
| output health 回调 | health 到达 | 只更新缓存，下一次 processOutput 才复查 | [agi_ros2/src/command_output_node.cpp:110](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:110) |
| output MSP timer | 1 ms 墙钟 | 非阻塞收帧、遥测调度、错误检查 | [agi_ros2/src/command_output_node.cpp:114](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:114) |
| output watchdog | 5 ms 墙钟 | 超时检查、状态发布；延迟实验还重发旧命令 | [agi_ros2/src/command_output_node.cpp:318](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:318) |

“单线程”指这三个 ROS 节点的业务回调串行拥有状态，不表示 DDS、Gazebo transport 或运行库没有后台线程。sim_rc 刻意用两个 executor 线程及独立 callback group，使参数服务不阻塞接收机心跳：[agi_ros2/scripts/sim_rc.py:17](/home/sia/agilicious_internal-main/agi_ros2/scripts/sim_rc.py:17)。

**3．严格按启动顺序看。**

**第一步：编译只产生安装包，不会启动控制器。** [agi_ros2/scripts/build.sh:3](/home/sia/agilicious_internal-main/agi_ros2/scripts/build.sh:3) 定位仓库，选择 ROS 环境，配置 ccache，colcon 只选择 agi_ros2，设置 Release、ACADOS_ROOT、FETCH_ACADOS=OFF。[agi_ros2/CMakeLists.txt:9](/home/sia/agilicious_internal-main/agi_ros2/CMakeLists.txt:9) 生成消息，[agi_ros2/CMakeLists.txt:23](/home/sia/agilicious_internal-main/agi_ros2/CMakeLists.txt:23) 引入 agilib，30 行起生成三个可执行文件，52 行起生成独立 MSP 节点，64 行起可选 Gazebo 适配器，72 行起生成 MAVLink 节点。

install/agi_ros2 是显式指定的安装目录。当前 build.sh 没有 --build-base build/ros2；因此 README 宣称固定 build/ros2 与脚本不一致，实际 build base 还取决于 colcon 默认/用户配置。YAML 通过 [agi_ros2/CMakeLists.txt:80](/home/sia/agilicious_internal-main/agi_ros2/CMakeLists.txt:80) 从 agilib/params 复制安装；launch 也是复制安装。

**第二步，SITL：run.py 启动环境和传感器。**

| 顺序 | 实际调用 | 作用 |
|---|---|---|
| 1 | [betaflight_sitl/run.py:605](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:605) main | 参数解析；--ros2 默认 true；ROS2 分支拒绝旧 --arm/--trajectory 等选项 |
| 2 | [betaflight_sitl/run.py:90](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:90) load_betaloop_config | 读取 ~/betaloop/config.txt 的 Aeroloop/world/ELF 路径 |
| 3 | [betaflight_sitl/run.py:458](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:458) ensure_simulator_ports_free | 检查端口占用，防止已有模拟实例冲突 |
| 4 | [betaflight_sitl/run.py:719](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:719) → prepare_assets | 创建模型/世界叠加副本，替换插件、调整电机顺序与模型参数、添加 companion IMU |
| 5 | [betaflight_sitl/run.py:534](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:534) prepare_runtime_betaflight | 复制 ELF/EEPROM 到隔离目录，启动配置实例，写入并回读配置 |
| 6 | [betaflight_sitl/run.py:292](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:292) / 377 / 395 | 从桥接 YAML 生成期望 rates/deadband/PID/通道映射，写入并检查；不改用户原始 EEPROM |
| 7 | [betaflight_sitl/run.py:732](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:732) | 未 --no-build 时构建插件和 ROS2 包 |
| 8 | [betaflight_sitl/run.py:763](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:763) | 调用 Betaloop start.py，指定生成的 world 和隔离 ELF，关闭额外 transmitter |
| 9 | [betaflight_sitl/run.py:874](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:874) | 启动 Betaloop，等待 TCP 5761 就绪 |
| 10 | [betaflight_sitl/run.py:861](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:861)、[agi_ros2/scripts/sensors.sh:9](/home/sia/agilicious_internal-main/agi_ros2/scripts/sensors.sh:9) | 启动 gazebo_sensors，传 config_verified:=true |
| 11 | [betaflight_sitl/run.py:899](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:899) | 监控 simulator/sensor 退出与 --duration；flight 是另一个独立进程组 |

模型创建本身也影响运行：[betaflight_sitl/prepare_assets.py:39](/home/sia/agilicious_internal-main/betaflight_sitl/prepare_assets.py:39) 给 companion IMU 设 1000 Hz；131 行 FDM 500 Hz，135 行 odometry 200 Hz，149 行当前物理步长 0.001 s。部分历史注释写 0.5 ms，当前常量实际为 1 ms。插件构建不是直接编原始 Aeroloop 文件，而是复制后打补丁：[betaflight_sitl/plugin/CMakeLists.txt:28](/home/sia/agilicious_internal-main/betaflight_sitl/plugin/CMakeLists.txt:28)。

**第三步：launch.sh 启动主链。** [agi_ros2/scripts/launch.sh:3](/home/sia/agilicious_internal-main/agi_ros2/scripts/launch.sh:3) 计算仓库路径，source ROS 和 install/agi_ros2/local_setup.bash，然后第 9 行直接 ros2 launch 源码 flight.launch.py，并透传命令行参数。

[agi_ros2/launch/flight.launch.py:101](/home/sia/agilicious_internal-main/agi_ros2/launch/flight.launch.py:101) 先将 FLIGHT_CONFIG 的每一项声明成 launch argument，再用 OpaqueFunction 调用 nodes(context)。[agi_ros2/launch/flight.launch.py:14](/home/sia/agilicious_internal-main/agi_ros2/launch/flight.launch.py:14) 内用 LaunchConfiguration.perform 读取参数，检查 mode，构造各节点的 ROS 参数字典：

- fusion 只拿 mode、use_sim_time、sitl_delay_test；没有拿 pilot_config 或 params_dir。
- control 拿 common 参数和 trajectory。
- output 拿 common、桥接配置、串口、推力表；hardware 另拿 MSP_CONFIG。
- MAVLink 可选且只允许 hardware，并检查它与 MSP 串口不是同一设备；启用后关闭重复 MSP GPS 查询，按需关闭 MSP attitude 查询。
- 第 57 行起，任一业务进程退出会触发整个 flight Shutdown。
- 第 60 行起默认启动 rosbag --all --include-hidden-topics；SITL 加 --use-sim-time。录包进程退出也结束 flight。

这些节点没有按“融合已 ready → 控制已 warm → 输出 ready”串行启动；启动后靠消息和状态机逐步就绪。launch 提供进程退出联动，没有提供完整 readiness 编排。

**第四步，SITL：启动唯一模拟接收机。** [agi_ros2/scripts/sim_rc.py:11](/home/sia/agilicious_internal-main/agi_ros2/scripts/sim_rc.py:11) 默认 armed=false、auto_switch=false、kill=true、throttle=1000。参数变更后，下一次 20 ms timer 将新参数发到 /authority。它只发布 ROS 消息；真正发 UDP RC 的仍是 command_output。正常流程是解除 KILL、保持 AUTO low、人工解锁和起飞、确认预热后切 AUTO。当前没有调用 Pilot::start 自动起飞。

**实机这一步的区别：** 不运行 run.py、gazebo_sensors、sim_rc；运行真实输入驱动，再启动 mode:=hardware 的 flight。仅有串口和推力表还不够，必须提供 /sensors/imu、/sensors/rtk、/authority、/health。当前 flight 没有自动启动实体 authority/health 提供者。后文解释 MAVLink 与 shadow 能补哪些部分。

**4．参数到底从哪里读取。**

| 参数层 | 入口 / 读取点 | 生效范围 |
|---|---|---|
| launch defaults / 命令行 name:=value | [agi_ros2/launch/flight.launch.py:93](/home/sia/agilicious_internal-main/agi_ros2/launch/flight.launch.py:93)、14 | 选择模式、文件、串口、录包等；命令行覆盖 launch 默认 |
| ROS 节点参数 | [agi_ros2/src/control_node.cpp:31](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:31)、[agi_ros2/src/command_output_node.cpp:44](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:44) | 构造时 declare_parameter，缓存到成员 |
| Pilot YAML | [agi_ros2/src/control_node.cpp:50](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:50) → [agilib/src/pilot/pilot_params.cpp:44](/home/sia/agilicious_internal-main/agilib/src/pilot/pilot_params.cpp:44) | 模块类型、MPC 文件、机体文件、控制约束 |
| 底层 YAML 装载 | [agilib/src/base/parameter_base.cpp:7](/home/sia/agilicious_internal-main/agilib/src/base/parameter_base.cpp:7)、[agilib/src/pilot/pipeline_config.cpp:11](/home/sia/agilicious_internal-main/agilib/src/pilot/pipeline_config.cpp:11) | 文件读取、模块子配置和路径解析 |
| 机体 YAML | [agilib/src/pilot/pilot_params.cpp:89](/home/sia/agilicious_internal-main/agilib/src/pilot/pilot_params.cpp:89) | 质量、惯量、力臂、推力范围、角速率限制等 |
| MPC YAML | [agilib/src/pilot/pilot_params.cpp:159](/home/sia/agilicious_internal-main/agilib/src/pilot/pilot_params.cpp:159) → [agilib/src/controller/mpc/mpc_params.cpp:28](/home/sia/agilicious_internal-main/agilib/src/controller/mpc/mpc_params.cpp:28) | 跟踪权重及控制器参数 |
| 输出桥接 YAML | [agi_ros2/src/command_output_node.cpp:69](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:69) | ACTUAL rates、deadband、SITL UDP/油门参数 |
| 融合参数 | [agi_ros2/src/state_fusion_node.cpp:30](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:30)、58 | ROS variance 参数 + EkfImuParameters C++ 默认值 |
| 推力表 CSV | [agi_ros2/src/command_output_node.cpp:135](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:135) | 电压、PWM、总推力标定 |
| MSP 查询参数 | [agi_ros2/src/msp_telemetry.cpp:20](/home/sia/agilicious_internal-main/agi_ros2/src/msp_telemetry.cpp:20) | 各类别 enabled/rate、超时、只读配置回读 |

默认 pilot 配置为 [agilib/params/pilot_ros2.yaml:1](/home/sia/agilicious_internal-main/agilib/params/pilot_ros2.yaml:1)，指定 External estimator、Time sampler、MPC/mpc_betaflight_sitl.yaml、External bridge、betaloop_iris.yaml，dt_min=0.01、outerloop_divisor=1、velocity_in_bodyframe=false、guard=None。这套默认机体质量是 0.54 kg：[agilib/params/quads/betaloop_iris.yaml:2](/home/sia/agilicious_internal-main/agilib/params/quads/betaloop_iris.yaml:2)。实机必须替换实际机体模型和标定。

一条重要路径差异：launch.sh 读取源码 launch，但 FLIGHT_CONFIG.params_dir 默认来自 get_package_share_directory('agi_ros2')，因此指向安装目录的 params。改源码 launch，重启即可；改 agilib/params 下 YAML，如果仍使用默认 params_dir，须重新安装或显式指向源码/外置参数目录。run.py 的 EEPROM 配置则读源码 agilib/params/betaflight_udp.yaml：[betaflight_sitl/run.py:717](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:717)。因此源码 YAML、安装 YAML、隔离 EEPROM 三者可能不一致。

另一处容易调错：fusion 不经过 PilotParams，默认不会读取 agilib/params/ekf_imu.yaml，也不会读取 AHRS YAML。它在 reset() 中创建 CompanionAhrs::Params{}、EkfImuParameters，再用节点参数覆盖部分方差。因此改 ekf_imu.yaml 不会改变此入口的融合器。flight launch 也没有声明这些 variance 的转发参数，需增加统一配置入口或直接给融合节点参数。

一般运行参数是构造时读取，不等于支持在线重载。control/output 对 mode、trajectory、映射等没有动态重配回调；参数服务显示已修改，不能证明内部成员同步更新。fusion 的 RTK 方差在构造时缓存，IMU 方差在 reset 时重读，因此运行中修改的行为也不一致；IMU 消息自带 covariance 没有参与这里的参数读取。sim_rc、gazebo_sensors 在回调中读取参数，故支持其列出的运行调整。shadow_only 显式 read_only，不能通过运行时参数绕过。

HardwarePilot 在构造前强制验证上述组件组合：[agilib/src/pilot/hardware_pilot.cpp:23](/home/sia/agilicious_internal-main/agilib/src/pilot/hardware_pilot.cpp:23)。换 YAML 为 GEO、启用内环/guard、改 dt_min 或把速度标成 body，不是自动切换架构，而是直接拒绝启动。

**5．传感器如何进入，融合究竟算什么。**

**SITL 适配器。** [agi_ros2/src/gazebo_sensors.cpp:35](/home/sia/agilicious_internal-main/agi_ros2/src/gazebo_sensors.cpp:35) 订阅 Gazebo /clock、/model/iris/companion_imu、/model/iris/odometry。onClock 第 54 行转发仿真时间；onImu 第 59 行复制采样时间、specific force 与角速率，frame=base_link，orientation_covariance[0]=-1。

onOdom 第 73 行有两条输出：一条是 ground_truth 评估数据；另一条每约 0.1 s 从真值生成 RTK。第 109 行把 Gazebo body velocity 转到 ENU，第 106/116 行添加位置、速度、航向噪声，第 122 行放入“到期才发布”的队列，实现默认 80 ms 延迟，消息 header 仍保留采集时刻。第 128 行发布模拟 Health：converged 用 t>2，电压固定 16 V，其余部分质量标志为模拟设定。它不等于实机估计质量的测量结果。ground_truth 话题没有被默认控制节点订阅；但模拟 RTK 本身当然由真值生成。

**融合构造。** [agi_ros2/src/state_fusion_node.cpp:22](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:22) 读取参数和 navigation_source，reset() 创建 AHRS/EKF、增加 reset_counter，再建立：

- RTK 分支：sensors/rtk，默认 reliable，depth=10；
- GNSS 分支：sensors/local_navigation，SensorDataQoS；
- IMU：SensorDataQoS，depth=256；
- 输出：fused_state、state，depth=1。

**正常 RTK 每次 IMU 回调的顺序：**

| 步骤 | 文件 / 行 | 输入与输出 |
|---|---|---|
| 1. 接收 IMU | [agi_ros2/src/state_fusion_node.cpp:112](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:112) | 保存本机 monotonic 接收时间，取消息采样时间，要求 base_link、数值有效 |
| 2. 检查时间 | [agi_ros2/src/state_fusion_node.cpp:123](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:123) | 重复时刻丢弃；回退或严格模式正向间隔 >25 ms 时重置 |
| 3. 检查 RTK 缓存 | [agi_ros2/src/state_fusion_node.cpp:136](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:136) | frame=odom，严格模式下采集年龄 ≤0.3 s，实机接收年龄 ≤0.3 s，fixed/accuracy 和是否新观测 |
| 4. 尚未初始化 | [agi_ros2/src/state_fusion_node.cpp:145](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:145) | AHRS 从 IMU 建立初始倾角，RTK 给 yaw、p、v；p 外推到当前 IMU 时刻，gyro bias 取 AHRS |
| 5. EKF 初始化 | [agilib/src/estimator/ekf_imu/ekf_imu.cpp:54](/home/sia/agilicious_internal-main/agilib/src/estimator/ekf_imu/ekf_imu.cpp:54) | 清空历史、设后验状态和初始协方差 |
| 6. 已初始化：收 IMU | [agilib/src/estimator/ekf_imu/ekf_imu.cpp:119](/home/sia/agilicious_internal-main/agilib/src/estimator/ekf_imu/ekf_imu.cpp:119) | 检查顺序，加入历史；不会再每周期调用 AHRS 修姿态 |
| 7. 有新 RTK：校正 | [agilib/src/estimator/ekf_imu/ekf_imu.cpp:135](/home/sia/agilicious_internal-main/agilib/src/estimator/ekf_imu/ekf_imu.cpp:135) | 在 RTK 原始采集时刻传播状态/协方差，再用位置、速度与可选 yaw 更新 |
| 8. 预测到当前 IMU | [agilib/src/estimator/ekf_imu/ekf_imu.cpp:23](/home/sia/agilicious_internal-main/agilib/src/estimator/ekf_imu/ekf_imu.cpp:23) | getAt(time) 沿保存的 IMU 预测当前状态 |
| 9. 发布 | [agi_ros2/src/state_fusion_node.cpp:173](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:173) | fused_state 带 p/v/q/w/a、采样与接收/发布时间、reset_counter、已接受导航时刻和质量标志 |

EKF 内部状态包含位置、四元数、速度、陀螺/加速度偏置。惯性传播的关键关系是：
~~~text
omega = gyro_measured - gyro_bias
a_world = gravity_world + R(q) * (specific_force - accelerometer_bias)
p、v、q 随时间积分
~~~
实现见 [agilib/src/estimator/ekf_imu/ekf_imu.cpp:398](/home/sia/agilicious_internal-main/agilib/src/estimator/ekf_imu/ekf_imu.cpp:398)。RTK 更新计算位置/速度残差，yaw 残差用 atan2(sin Δyaw, cos Δyaw) 折回，计算 K，再用 Joseph 形式更新协方差并处理四元数归一化：[agilib/src/estimator/ekf_imu/ekf_imu.cpp:155](/home/sia/agilicious_internal-main/agilib/src/estimator/ekf_imu/ekf_imu.cpp:155)。初始化后 heading 无效仍可更新 p/v，但严格飞行授权另行要求 heading_valid。

这里应称“具有协方差与延迟观测处理的 EkfImu”，不能扩展成“任意乱序多传感器完整历史回滚”。t≤当前后验、超前于最新 IMU、所需历史已丢弃的 RTK 会被拒绝；历史最多 4096 个 IMU 样本。初始化使用速度外推，不代表运行中的 RTK 更新也继续使用旧外推算法。

fused_state 的 v 是 ENU；另一个 state/Odometry 的 twist.linear 在发布前用 q 的逆变换回 body FLU：[agi_ros2/src/state_fusion_node.cpp:215](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:215)。因此画图时直接比较两者的速度分量会出错。

**6．轨迹什么时候加载、什么时候开始、怎样采样。**

轨迹文件只在 ControlNode 构造时加载：[agi_ros2/src/control_node.cpp:50](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:50)。不存在每个周期重新读 CSV。三个阶段要分开：

1. readTrajectoryRows 读取并校验数值。
2. estimateSourceMass + loadTrajectory 建立“零时刻、零起点、零初始 yaw”的相对轨迹，HardwarePilot::setTrajectory 保存。
3. AUTO 上升沿捕获当前位置/yaw/time，将相对轨迹变为当前飞行参考。

CSV 解析器有历史格式特例：[agilib/include/agilib/reference/trajectory_csv.hpp:44](/home/sia/agilicious_internal-main/agilib/include/agilib/reference/trajectory_csv.hpp:44) 要求表头恰好是 14 个字段的旧字符串，但 [agilib/include/agilib/reference/trajectory_csv.hpp:27](/home/sia/agilicious_internal-main/agilib/include/agilib/reference/trajectory_csv.hpp:27) 要求每条数据有 30 个数。自行生成常规 30 列表头会被拒绝。所有数必须有限，时间严格递增，至少两点，四元数必须非零并会归一化。

| 数据列（0 起算） | 内容 | 后续用途 |
|---|---|---|
| 0 | t | 减去首样本时间，接管时再加当前时间 |
| 1–3 | p | 减初始位置，旋转、平移 |
| 4–7 | q_w q_x q_y q_z | 姿态归一化和左乘 yaw 对齐 |
| 8–10 | v | 世界向量，随 yaw 对齐旋转 |
| 11–13 | w | 机体系角速率，不随世界 yaw 旋转 |
| 14–16 | a | 世界加速度，也用于源机体质量估计 |
| 17–19 | tau | 原样保存，机体系量 |
| 20–23 | u1…u4 | 源机体各电机推力 N，用总和计算归一化 collective |
| 24–26 | jerk | 世界向量 |
| 27–29 | snap | 世界向量 |

[agilib/include/agilib/reference/trajectory_csv.hpp:70](/home/sia/agilicious_internal-main/agilib/include/agilib/reference/trajectory_csv.hpp:70) 用全轨迹最小二乘恢复源质量：
~~~text
s_i = norm([a_x, a_y, a_z + g])
m_source = sum((u1+u2+u3+u4)_i * s_i) / sum(s_i²)
collective_ref = (u1+u2+u3+u4) / m_source
~~~
这样先把来源不同机体的 CSV 推力转成加速度前馈，后续 MPC 再使用当前机体质量。不是直接把 CSV 电机推力原样发给当前飞机。

启动时 [agi_ros2/src/control_node.cpp:58](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:58) 给 loadTrajectory 的 min_altitude=-infinity，所以 ROS2 这条链不会自动抬高最低轨迹高度；旧 standalone 的 ground-clearance 参数不能套用在这里。

设文件首点为 p0、yaw0，实际接管位置为 pc、航向为 yawc、时刻为 tc，综合两次变换后：
~~~text
t_ref = tc + (t_csv - t_csv0)
p_ref = pc + Rz(yawc-yaw0) * (p_csv-p0)
q_ref = Rz(yawc-yaw0) * q_csv
v/a/jerk/snap 同样旋转；body w/tau 不旋转
~~~
第二次变换在 [agilib/src/pilot/hardware_pilot.cpp:133](/home/sia/agilicious_internal-main/agilib/src/pilot/hardware_pilot.cpp:133)。无 CSV 时用 getHoverState 捕获位置和 yaw，再生成 HoverReference。不会自动起飞到 takeoff_height；该配置属于这里未调用的 Pilot::start。

随后每周期 TimeSampler::getAt 按时间从 Reference 列表取预测窗：[agilib/src/sampler/time_based/time_sampler.cpp:9](/home/sia/agilicious_internal-main/agilib/src/sampler/time_based/time_sampler.cpp:9)。CSV 由 [agilib/src/reference/trajectory_reference/sampled_trajectory.cpp:17](/home/sia/agilicious_internal-main/agilib/src/reference/trajectory_reference/sampled_trajectory.cpp:17) 插值，HoverReference 由 [agilib/src/reference/hover_reference.cpp:9](/home/sia/agilicious_internal-main/agilib/src/reference/hover_reference.cpp:9) 提供。轨迹结束后 Pipeline 转到末点悬停：[agilib/src/base/pipeline.cpp:108](/home/sia/agilicious_internal-main/agilib/src/base/pipeline.cpp:108)，不会自动降落。

注意非 shadow 的当前实现是在“观察到 AUTO 上升沿”时启动参考，最终 SafetyGate 授权发生得更晚。尚未 ARM 或未预热时翻 AUTO 也可能启动内部轨迹时间，但不会因此获得输出许可；这是应该改进的参考状态与授权状态耦合问题。

**7．一次 100 Hz 控制周期，从状态到命令。**

**构造期先创建控制器。** [agilib/src/pilot/hardware_pilot.cpp:37](/home/sia/agilicious_internal-main/agilib/src/pilot/hardware_pilot.cpp:37) 构造 Pilot；[agilib/src/pilot/pilot.cpp:15](/home/sia/agilicious_internal-main/agilib/src/pilot/pilot.cpp:15) 经 PilotParams::createPipeline 创建 MPC 与 TimeSampler。External estimator/bridge 由 HardwarePilot 随后注册：

- FeedthroughEstimator：只是融合状态交接，不做第二次 EKF。
- CommandSink：内存中的 BridgeBase 子类，只验证命令格式，不开串口、不发 UDP。
- pilot.enable(false)：选择并激活内存 debug/sink 通道，使流水线能计算；这里 false 不等于“禁用计算”。

MpcController 构造会 updateParameters、reset；reset 调一次 wrapper.update 初始化求解器：[agilib/src/controller/mpc/controller_mpc.cpp:5](/home/sia/agilicious_internal-main/agilib/src/controller/mpc/controller_mpc.cpp:5)、152。这是构造期的一次求解，不是每次 tick 隐藏再算一遍。

**每周期主调用树：**

~~~text
ControlNode::tick
  alignedRosTime                    最多等待 3 ms，让 /clock 追上数据
  缓存 FusedState -> QuadState
  Authority + Health + OutputStatus + 时间戳 -> Evidence
  HardwarePilot::tick
    检查 cadence/state/输入健康
    Pilot::odometryCallback
      FeedthroughEstimator::addState
    必要时 resetHover / 建立 SampledTrajectory
    Pilot::runPipelineChecked
      Pipeline::run
        FeedthroughEstimator::getAt
        TimeSampler::getAt
          HoverReference / SampledTrajectory::getSetpoint
        MpcController::getCommand
          MpcWrapper::setReferences / setReferenceN
          MpcWrapper::update
            设置初值、约束、在线参数
            drone_model_acados_solve
              ocp_nlp_solve -> 生成的动力学/雅可比/代价函数
          生成预测 setpoints
        选择第一控制命令
        管理过期 reference
        BridgeBase::send -> CommandSink::sendCommand
    取 command/reference，检查耗时，累计预热
    SafetyGate::update
  publishDecision
    control_command / computation_status / diagnostics / status / reference
~~~

具体输入校验在 [agi_ros2/src/control_node.cpp:112](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:112)：先检查 output fault/clock 回退，读取融合消息中的 p/v/q/w/a；再组合传感、RC、health、输出状态的新鲜度。异机 clock_id 直接拒绝；融合 reset_counter 变化、输出 session 或 fault_count 变化会清除预热和授权。

[agilib/src/pilot/hardware_pilot.cpp:81](/home/sia/agilicious_internal-main/agilib/src/pilot/hardware_pilot.cpp:81) 在严格模式下检查周期严格递增、间隔≤25 ms，状态年龄≤10 ms、姿态单位四元数误差<1e-3。健康时即使在人工模式也运行 MPC，严格模式下连续 50 个有效且≤8 ms 的周期才视为 warm。名义上至少约 0.5 s，故障或慢周期会重新计数；延迟实验的门限覆盖见第8段。

状态交接见 [agilib/src/pilot/pilot.cpp:540](/home/sia/agilicious_internal-main/agilib/src/pilot/pilot.cpp:540)、[agilib/src/estimator/feedthrough/feedthrough_estimator.cpp:24](/home/sia/agilicious_internal-main/agilib/src/estimator/feedthrough/feedthrough_estimator.cpp:24)。FeedthroughEstimator::getAt 在该文件第39行忽略查询时刻 t，返回保留原 state.t 的最新融合状态；TimeSampler 因此从 state.t 开始采样21点，并未先把状态外推到控制 timer 的 now。Pipeline 取估计→取参考→调控制器→选命令→交给 sink 的主干在 [agilib/src/base/pipeline.cpp:58](/home/sia/agilicious_internal-main/agilib/src/base/pipeline.cpp:58)。HardwarePilot 不调用 Pilot::launchPipeline，因此没有第二个 100 Hz 控制线程；不调用 Pilot::start/land/goToPose，因此那些通用机动生成代码不是默认运行路径。

MPC 的状态和控制输入是：
~~~text
x = [p(3), q(4), v(3), omega(3)]，NX=13
u = [f1, f2, f3, f4]，NU=4，单位 N
N=20；预测步长 0.05 s；21 个状态点；预测约 1 s
实际执行周期仍为 0.01 s
~~~
维数见 [agilib/include/agilib/controller/mpc/acados/acados_solver_drone_model.h:40](/home/sia/agilicious_internal-main/agilib/include/agilib/controller/mpc/acados/acados_solver_drone_model.h:40)，步长见 [agilib/src/controller/mpc/acados/acados_solver_drone_model.c:405](/home/sia/agilicious_internal-main/agilib/src/controller/mpc/acados/acados_solver_drone_model.c:405)。不要把控制周期与预测步长混为一谈。a/jerk/snap 虽在 QuadState/轨迹中存在，并不全作为 MPC 独立状态输入；getCommand 取 state.x 前 13 维。

[agilib/src/controller/mpc/controller_mpc.cpp:50](/home/sia/agilicious_internal-main/agilib/src/controller/mpc/controller_mpc.cpp:50) 构造参考矩阵。当前 CSV 是 rates+collective，先将 collective_ref*m_current/4 分到四路参考推力，把 w_ref 放进状态参考。MPC 使用机体质量/惯量/力臂/气动参数、推力及角速率限制和 Q/R 权重，解出预测状态与四路推力。参考并不是被直接透传。

输出转换见 [agilib/src/controller/mpc/controller_mpc.cpp:109](/home/sia/agilicious_internal-main/agilib/src/controller/mpc/controller_mpc.cpp:109)：
~~~text
collective_cmd = sum(f1,f2,f3,f4) / m_current    [m/s²]
omega_cmd = 第一个未来预测点的 body rate       [rad/s]
~~~
第 125 行把第 0 点指令角速度替换为第 1 点角速度，即 50 ms 后的预测率；它不是原始 CSV 角速度，也不是测得的当前角速度。

Pipeline 将第一条命令通过 BridgeBase::send 交给 CommandSink 校验；HardwarePilot 随后调用 Pilot::getCommand → Pipeline::getCommand，读取 Pipeline 中的 apply_command_。CommandSink 不保存或回传命令，也不做物理发送；ROS control 只在 [agi_ros2/src/control_node.cpp:169](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:169) publishDecision 中发消息。第 180 行 collective*m_current 得到物理总推力 N。

证据中 solve_seconds 从 [agilib/src/pilot/hardware_pilot.cpp:127](/home/sia/agilicious_internal-main/agilib/src/pilot/hardware_pilot.cpp:127) 开始，包含状态交接、参考处理、pipeline 求解和结果提取，不只是 acados 内部耗时；alignedRosTime 的等待在此之前，不包含在这个字段。构造期求解也不计入 50 个 warm 周期。

**8．授权状态机和时间域。**

状态机代码集中在 [agilib/include/agilib/bridge/betaflight/hardware_safety.hpp:25](/home/sia/agilicious_internal-main/agilib/include/agilib/bridge/betaflight/hardware_safety.hpp:25)。Boot 后通常先 SensorCheck；完整输入健康、命令有效、预热完成后，AUTO low 时按 armed 进入 ReadyManual/AutoStandby；观察到健康 low 后再 high，才允许 AutoActive。启动即 high、故障期间保持 high、输出进程重启后仍 high，都不能直接恢复。故障回到 SensorCheck 或 ManualFallback，取决于原状态。

inputsHealthy 要求 IMU、RTK、RC 时间满足范围，以及 rc_link、rtk_fixed、heading_valid、accuracy_ok、imu_calibrated、synchronized、converged、config_verified、thrust_calibrated、geofence_ok、msp_healthy 全部成立。update 另外检查 controller_warm、command_valid、命令年龄和耗时。

| 检查 | 严格模式门限 / 位置 |
|---|---|
| IMU evidence 年龄 | 10 ms；SafetyGate::inputsHealthy |
| 融合状态采样年龄 | 10 ms；HardwarePilot::tick |
| RTK 采样年龄 | 300 ms |
| RC 采样/接收年龄 | 100 ms |
| Health 采样/接收年龄 | 200 ms；control/output 节点 |
| OutputStatus 实机接收/发布时间年龄 | 50 ms；control 节点 |
| 控制调用间隔 | 25 ms；HardwarePilot |
| HardwarePilot 计算段耗时 | 8 ms；不含前置对时等待及后续ROS发布 |
| 命令有效期 | 25 ms；输出节点与 SafetyGate |
| SITL 墙钟停流上限 | 250 ms；node_common.h 与输出节点 |
| IMU 正向间隔触发重置 | 25 ms；融合节点 |

实机的轨迹采样和 ROS header 使用系统 ROS 时间，但安全证据和 UART 截止使用 monotonic。control 将 RTK/RC 的 ROS 采样年龄映射到当前 safety clock。SITL use_sim_time=true 时，控制定时器与证据年龄使用 /clock；求解耗时仍按墙钟。FusedState 接收/发布时间和 OutputStatus.steady_time 始终是主机 monotonic。

[agi_ros2/src/node_common.cpp:14](/home/sia/agilicious_internal-main/agi_ros2/src/node_common.cpp:14) 读取 Linux boot_id，SITL 命令证据再加 ':ros' 标识，阻止把异机 monotonic 或错误时间域混入。因此融合、控制、输出必须同一台主机；Gazebo 传感源可以异机，只要 ROS 采样时间域匹配。

以上门限要加上一个当前默认条件：flight.launch.py 的 sitl_delay_test=true 使三个节点关闭大部分年龄/周期/耗时限制，输出 watchdog 每 5 ms 重发最后命令。它也关闭 250 ms 墙钟停流限制，并非只放宽“网络延迟”。仍有有效性、时间回退、授权撤销等检查，仍须 50 个有效 warm 周期。hardware launch 强制 false。严格 SITL 行为必须显式使用 sitl_delay_test:=false 后讨论。


**9．输出节点如何把控制量发到飞控。**

构造入口 [agi_ros2/src/command_output_node.cpp:35](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:35) 独立读桥接 YAML、Pilot YAML 的机体文件和质量，再建立 BetaflightRcMapper；hardware 读推力表并打开 UART，SITL 打开非阻塞 UDP socket。它没有实例化旧 BetaflightUdpBridge，也没有调用 apps/BetaflightMspClient 的 TCP 客户端。

每条 /control_command 到达，[agi_ros2/src/command_output_node.cpp:167](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:167) 检查 clock_id、时间有限、时间严格前进；被拒绝的消息不刷新停流时间。随后进入 processOutput：

| 顺序 | 代码 | 实际检查 / 作用 |
|---|---|---|
| 1 | [agi_ros2/src/command_output_node.cpp:211](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:211) | shadow_only 直接返回，禁止任何控制帧 |
| 2 | [agi_ros2/src/command_output_node.cpp:222](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:222) | 对齐 ROS 时间，decodeEvidence，读取当前 monotonic；原始采样/计算时间保持不变 |
| 3 | [agi_ros2/src/command_output_node.cpp:228](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:228) | 校验命令 frame、clock_id、接收年龄、原始证据年龄、ROS stamp |
| 4 | [agi_ros2/src/command_output_node.cpp:232](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:232) | 校验 RC、health 的接收和采样时效 |
| 5 | [agi_ros2/src/command_output_node.cpp:238](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:238) | 比较命令计算时与最新消息的 ARM/AUTO；处理两个 DDS topic 到达顺序不同 |
| 6 | [agi_ros2/src/command_output_node.cpp:247](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:247) | 最新kill/RC，以及transport/config/geofence/thrust健康字段与旧证据求交 |
| 7 | [agi_ros2/src/command_output_node.cpp:257](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:257) | 本节点 SafetyGate 再判断，且必须 control 的 permit_override=true |
| 8 | [agi_ros2/src/command_output_node.cpp:259](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:259) | 获授权才执行 mapCommand；映射失败则撤销 |
| 9 | [agi_ros2/src/command_output_node.cpp:275](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:275) | hardware 发 MSP；SITL 发 UDP |
| 10 | [agi_ros2/src/command_output_node.cpp:301](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:301) | 更新 override_active、reason，发布 OutputStatus |

双重/三重安全门不是重复 MPC。控制层知道是否算出合格命令；输出层知道命令抵达后的最新开关、健康与年龄；MSP 底层知道串口最终写之前是否仍有效。这些判断位于不同失效边界，不能为了减少调用而一起删掉。可以共享判定实现和结构化原因，但应保留独立执行。

撤销时机也要准确区分：authority 的指定变化会在其回调中执行 processOutput；health 回调只缓存，遥测timer只锁存transport状态，通常等下一控制消息或命令超时后的 watchdog 才重新执行输出检查。最新 health 的 imu_calibrated/converged 并未在输出节点直接重合并，而由下一控制周期重建证据。不能把这些健康变化描述为即时撤销，也不能把非实时Linux上的定时门限写成严格响应上界。

**角速度→AETR。** [agi_ros2/src/command_output_node.cpp:179](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:179) 先验证总推力/角速率有限且推力非负，再将 rad/s 转 deg/s。hardware 为 roll 同号、pitch/yaw 反号；SITL 使用当前模拟模型已配套的符号。[agilib/src/bridge/betaflight/betaflight_rc_mapper.cpp:6](/home/sia/agilicious_internal-main/agilib/src/bridge/betaflight/betaflight_rc_mapper.cpp:6) 用 48 次二分反解 ACTUAL rate 曲线，再在第 37 行反解 deadband、转通道数值。通道数组顺序是 roll、pitch、throttle、yaw，即 A,E,T,R。

**推力→油门有两套物理假设。**

SITL 在 [agi_ros2/src/command_output_node.cpp:192](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:192) 先 N/质量 得到 collective acceleration，然后：
~~~text
motor_hover = motor_idle + (1-motor_idle)*hover_throttle
motor_target = motor_hover*sqrt(collective/g)
stick = clamp((motor_target-motor_idle)/(1-motor_idle), 0, 1)
throttle = round(min_check + (2000-min_check)*stick)
~~~
参数来自 [agilib/params/betaflight_udp.yaml:45](/home/sia/agilicious_internal-main/agilib/params/betaflight_udp.yaml:45)。

hardware 调 [agilib/include/agilib/bridge/betaflight/thrust_table.hpp:33](/home/sia/agilicious_internal-main/agilib/include/agilib/bridge/betaflight/thrust_table.hpp:33)，使用当前 Health.battery_voltage：先在电压方向插值得到该电压下的推力曲线，再用总推力反查 PWM。至少两个电压、两个 PWM，电压/PWM/推力必须满足递增和有限性校验，超出标定范围拒绝，不进行外推。注意接口表面先 N/m 又乘 m，是沿用 collectiveThrustToRc(acceleration,mass,voltage) 接口，实际目标仍是总推力 N；可以简化接口，但不是多算一次控制器。

**hardware 的串口最终调用链：**
~~~text
CommandOutputNode::processOutput
  BetaflightMspBridge::sendOverride(AETR, evidence, deadline)
    本机 monotonic SafetyGate + 通道范围检查
    四个 uint16 编为 8 字节 payload
    writeFrame(code=200, payload, absolute_deadline)
      MSP v1 帧 + XOR checksum
      write / poll 处理部分写，受截止时间限制
  MspTelemetry::sentRc       发布真实发送字节事件
~~~
[agilib/src/bridge/betaflight/betaflight_msp_bridge.cpp:85](/home/sia/agilicious_internal-main/agilib/src/bridge/betaflight/betaflight_msp_bridge.cpp:85) 打开非阻塞串口、TIOCEXCL 独占、raw mode，支持此实现列出的 115200/460800/921600 波特率；119 行检查调用线程。[agilib/src/bridge/betaflight/betaflight_msp_bridge.cpp:254](/home/sia/agilicious_internal-main/agilib/src/bridge/betaflight/betaflight_msp_bridge.cpp:254) 再检查授权，123 行 writeFrame 支持 v1 和原生 v2；控制 code=200 使用 v1，只发送四通道，绝不拼 ARM/AUTO/KILL AUX。deadline 为调用时刻+2 ms。部分帧/写超时会锁存失败；发送成功只证明内核接收字节，不证明 FC 执行。

hardware 切手动或故障后停止 override 帧。是否回到实体接收机、多久回退，是飞控配置和固件的行为；不会由本程序虚构一个“安全油门”替代实体 RC。KILL 撤销这里的控制权限，真正的 FC KILL/ARM 行为必须由实体授权链实现。

**SITL 的最后一步：** [agi_ros2/src/command_output_node.cpp:288](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:288) 人工模式用 manual_aetr；AUTO 拒绝或 KILL/RC 超时则模拟 ARM low。[agi_ros2/src/command_output_node.cpp:357](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:357) 手工编码 40 字节：
~~~text
8 字节 double monotonic time + 16 路 uint16 RC，均小端
0..3=AETR；4=AUX1 ARM；其余默认1000
~~~
默认目标 127.0.0.1:9004。这里的“sendto 成功”不等于 Betaflight 已收到；UDP 输出 transport_healthy 的含义比硬件遥测健康弱。

**10．遥测反馈、故障恢复、关闭。**

MSP 并非第二个控制器。[agi_ros2/src/msp_telemetry.cpp:9](/home/sia/agilicious_internal-main/agi_ros2/src/msp_telemetry.cpp:9) 声明只读遥测参数和 publisher；默认 attitude/RC=10 Hz、status=5 Hz、analog/battery/GPS=2 Hz，response_timeout=100 ms。

[agi_ros2/src/msp_telemetry.cpp:72](/home/sia/agilicious_internal-main/agi_ros2/src/msp_telemetry.cpp:72) 的 tick 每轮有限数量接收、解析并匹配未完成请求，处理超时，再在剩余时间内发送查询。每个 code 最多一个 pending，请求过期不补发积压周期。底层 [agilib/src/bridge/betaflight/betaflight_msp_bridge.cpp:226](/home/sia/agilicious_internal-main/agilib/src/bridge/betaflight/betaflight_msp_bridge.cpp:226) 非阻塞 receive，[agilib/src/bridge/betaflight/betaflight_msp_bridge.cpp:42](/home/sia/agilicious_internal-main/agilib/src/bridge/betaflight/betaflight_msp_bridge.cpp:42) MspDecoder 有界缓存、识别 v1/v2 并验证校验和。

[agi_ros2/src/msp_telemetry.cpp:42](/home/sia/agilicious_internal-main/agi_ros2/src/msp_telemetry.cpp:42) 发布 /msp/events 和匹配成功的分类 topic，保留原始 payload、ROS/monotonic 请求与接收时间、session、latency、累计错误。RC ACK 只说明协议回复，不证明电机执行或 override 模式生效。默认 flight 的这些 raw topic 不会自动变成 /authority 或 /health。

任一开启的查询类别超时/错误，遥测 healthy 会锁存 false；[agi_ros2/src/command_output_node.cpp:120](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:120) 进一步锁存 transport_healthy=false。它不会仅靠后面几条成功回复自动恢复。没有 GPS 的机器若保持默认 GPS 查询开启，也会影响整个控制输出会话。需要区分“关闭不用的类别”与“掩盖真实关键链路错误”。

[agi_ros2/src/command_output_node.cpp:343](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:343) OutputStatus 回报 transport_healthy、thrust_calibrated、fault_count、session_start、override_active、reason。[agi_ros2/src/control_node.cpp:101](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:101) 发现 session/fault_count 变化就记录输出故障，下一控制周期 [agilib/src/pilot/hardware_pilot.cpp:68](/home/sia/agilicious_internal-main/agilib/src/pilot/hardware_pilot.cpp:68) 清预热、参考和授权状态。这样形成输出→控制的故障反馈回路。

退出时有两层管理：

- flight 任一进程或 recorder 退出，launch 通知同组其余节点 shutdown：[agi_ros2/launch/flight.launch.py:57](/home/sia/agilicious_internal-main/agi_ros2/launch/flight.launch.py:57)、72。
- SITL run.py 独立管理 Gazebo/Betaflight/sensor：[betaflight_sitl/run.py:917](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:917) 先停传感/适配器，再发 fallback disarm，最后停 Betaloop。独立运行的 flight 不归 run.py 直接管理，会通过停流感知环境退出。

[agi_ros2/src/command_output_node.cpp:126](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:126) 析构时 SITL 发送 idle+ARM low 并关闭 socket；hardware 只关闭 MSP 串口，不写 AUX。[agilib/src/bridge/betaflight/betaflight_msp_bridge.cpp:112](/home/sia/agilicious_internal-main/agilib/src/bridge/betaflight/betaflight_msp_bridge.cpp:112) 释放串口独占与 fd。控制节点析构释放 Pilot、参考、控制器；[agilib/src/pilot/pilot.cpp:43](/home/sia/agilicious_internal-main/agilib/src/pilot/pilot.cpp:43) 检查是否有 pipeline thread，此入口未启动它；[agilib/src/controller/mpc/wrapper.cpp:50](/home/sia/agilicious_internal-main/agilib/src/controller/mpc/wrapper.cpp:50) 释放 acados solver/capsule。控制节点退出本身没有物理指令输出，失联处理由输出节点负责。

“退出即联动 shutdown”提高了故障可见性，但 recorder 磁盘错误也会结束飞行节点。研发录包要求下可以理解，实机运行策略应显式区分飞行关键组件和记录组件。

**11．其他入口与实机链路补齐的现状。**

**独立 msp.launch.py。** [agi_ros2/launch/msp.launch.py:11](/home/sia/agilicious_internal-main/agi_ros2/launch/msp.launch.py:11) 默认 monitor、115200；[agi_ros2/src/msp_node.cpp:10](/home/sia/agilicious_internal-main/agi_ros2/src/msp_node.cpp:10) 构造 BetaflightMspBridge 和 MspTelemetry。monitor 只查询；bench 要求 props_removed=true、合法 bench_aetr，通过本地兼容性检查后固定 100 Hz 发送 AETR。[agi_ros2/src/msp_node.cpp:36](/home/sia/agilicious_internal-main/agi_ros2/src/msp_node.cpp:36) 的定时循环不使用 MPC、不订阅 control_command。这是串口/协议台架入口，不能与 flight 同时占同一串口。

**独立 MAVLink。** [agi_ros2/launch/mavlink_sensors.launch.py:9](/home/sia/agilicious_internal-main/agi_ros2/launch/mavlink_sensors.launch.py:9) 创建 mavlink_sensor_node。[agi_ros2/src/mavlink_sensor_node.cpp:36](/home/sia/agilicious_internal-main/agi_ros2/src/mavlink_sensor_node.cpp:36) 读取串口、速率、来源等参数；123 行独占打开串口；188 行 1 ms tick 执行有界收发、CRC/source 检查、同步、配置速率、配对 GPS；489 行发布诊断。

设备时间经 [agi_ros2/src/mavlink_sensor_node.cpp:275](/home/sia/agilicious_internal-main/agi_ros2/src/mavlink_sensor_node.cpp:275) TIMESYNC 对齐到主机 monotonic，再映射 ROS 系统时间；需要至少 5 次有效同步，拒绝超过 20 ms 的 RTT。[agi_ros2/src/mavlink_sensor_node.cpp:350](/home/sia/agilicious_internal-main/agi_ros2/src/mavlink_sensor_node.cpp:350) HIGHRES_IMU 的 Y/Z 取反，从 FRD 转 FLU 后发 sensors/imu。[agi_ros2/src/mavlink_sensor_node.cpp:440](/home/sia/agilicious_internal-main/agi_ros2/src/mavlink_sensor_node.cpp:440) 将 GPS_RAW_INT 与 GLOBAL_POSITION_INT 按同一设备毫秒配对，输出 ENU 速度及原子 Navigation。可选姿态只是独立输出，不是 IMU 融合姿态的直接输入。

这里的 gps_mode=rtk 只影响 fix 要求，不会自动生成旧 sensors/rtk 接口，也不生成双天线 heading/PPS 契约。因此在 flight 打开 mavlink_enabled 只接上可用的传感消息来源，仍需要真正满足 RTK/authority/health 的输入。

**shadow 完整无输出评估。** [agi_ros2/launch/shadow.launch.py:24](/home/sia/agilicious_internal-main/agi_ros2/launch/shadow.launch.py:24) 固定 hardware、shadow_only=true、sitl_delay_test=false，并同时启动：
~~~text
mavlink_sensor_node ── IMU ────────────────────────┐
                   └─ Navigation -> gnss_adapter ─┤
                                                 v
                                          state_fusion(gnss)
                                                 v
                                             control
                                                 v
                                      command_output(shadow)
                                                 |
                                            MSP 只读事件
                                                 v
                                           msp_evidence
                                                 |
                                         authority / health
~~~
实际六个节点见该文件 27–46 行。MSP 与 MAVLink 必须是不同 UART。

[agi_ros2/scripts/gnss_adapter.py:54](/home/sia/agilicious_internal-main/agi_ros2/scripts/gnss_adapter.py:54) 要求 fix、时间、heading 有效且 heading_confirmed。[agi_ros2/scripts/shadow_support.py:31](/home/sia/agilicious_internal-main/agi_ros2/scripts/shadow_support.py:31) OriginBuilder 默认静止至少 3 s、30 样本、速度≤0.3 m/s，建立局部原点；经纬高经 ECEF 转 ENU。GNSS adapter 第 95 行使用精度下限生成方差，产生 LocalNavigation。fusion 的 [agi_ros2/src/state_fusion_node.cpp:80](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:80) 接收它，session 改变会 reset。内部复用 RTK 存储，但不把普通 GNSS 标记成 RTK fixed 或 PPS synchronized。

默认 altitude_source=unknown、heading_confirmed=false，因此不修改真实配置时不能自然形成有效本地导航。它们是明确待确认的输入条件，不能只通过“节点在运行”判断链路 ready。

[agi_ros2/scripts/msp_evidence.py:30](/home/sia/agilicious_internal-main/agi_ros2/scripts/msp_evidence.py:30) 建立只读配置解码器；72 行验证 MSP 会话与原请求时间，再交给 [agi_ros2/scripts/shadow_support.py:103](/home/sia/agilicious_internal-main/agi_ros2/scripts/shadow_support.py:103) 解码；89 行按 20 ms 心跳及接收事件生成 MspState、Authority、Health。只支持代码校验过的 BTFL API 布局与 AUX/override 约定。Authority 的 stamp 来自 RC/STATUS 原请求时间的较早值，不随心跳“续命”。shadow 将 RC/STATUS 查询提高到 25 Hz；普通 flight 的 10/5 Hz 不能简单作为≤100 ms授权的新鲜度保证。

[agi_ros2/scripts/msp_evidence.py:122](/home/sia/agilicious_internal-main/agi_ros2/scripts/msp_evidence.py:122) 明确 imu_calibrated=false、converged=false；默认围栏未配置也是 false。因此它仍不构成完整实机飞行健康生产者。shadow MPC 走独立 navigation_valid 条件，能够计算而不满足严格 RTK gate。[agi_ros2/src/control_node.cpp:184](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:184) 和 [agi_ros2/src/command_output_node.cpp:212](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:212) 两层强制禁输出，不能把“shadow 有 MPC 结果”理解成“实机已具备接管条件”。

**12．下沉到 Betaflight 和 Gazebo 的最后一段。**

本机 Betaloop 配置指向 /home/sia/betaflight 与 /home/sia/aeroloop_gazebo；以下是对当前可读本地源码的补充定位，不证明现有 ELF 或实体 FC 固件一定由这份源码构建。仓库主链的责任边界仍是 UDP/UART。

| 环节 | 本机源码 / 行 | 行为 |
|---|---|---|
| Betaloop 编排 | [betaloop.py:179](/home/sia/betaloop/src/betaloop.py:179) | 创建 Gazebo、Betaflight 及可选代理进程 |
| SITL RC 收包 | [sitl.c:541](/home/sia/betaflight/src/platform/SIMULATOR/sitl.c:541) | UDP 9004 接收完整 rc_packet，再 rxUpdateUdpChannels |
| 硬件 MSP RC 收包 | [msp.c:2887](/home/sia/betaflight/src/main/msp/msp.c:2887) | MSP_SET_RAW_RC 解析后调用 rxMspFrameReceive |
| 接收机/模式处理 | [tasks.c:191](/home/sia/betaflight/src/main/fc/tasks.c:191)、[rx.c:731](/home/sia/betaflight/src/main/rx/rx.c:731) | 处理 RC、模式及 override 配置 |
| PID 调度 | [tasks.c:386](/home/sia/betaflight/src/main/fc/tasks.c:386)、[core.c:1497](/home/sia/betaflight/src/main/fc/core.c:1497) | 独立 FC 内环调度，不由 ROS 100 Hz 直接替代 |
| 角速度控制 | [core.c:1314](/home/sia/betaflight/src/main/fc/core.c:1314)、[pid.c:1048](/home/sia/betaflight/src/main/flight/pid.c:1048) | 飞控自身陀螺反馈和 PID |
| 混控/电机 | [core.c:1415](/home/sia/betaflight/src/main/fc/core.c:1415)、[mixer.c:676](/home/sia/betaflight/src/main/flight/mixer.c:676) | 转各电机输出 |
| Gazebo 已生成插件 | [build/betaflight_sitl/plugin/BetaflightPlugin.cc:651](/home/sia/agilicious_internal-main/build/betaflight_sitl/plugin/BetaflightPlugin.cc:651) | PreUpdate→OnUpdate，收电机、施加动力、发模拟传感 |
| 插件收电机 / 施力 | [build/betaflight_sitl/plugin/BetaflightPlugin.cc:753](/home/sia/agilicious_internal-main/build/betaflight_sitl/plugin/BetaflightPlugin.cc:753)、[build/betaflight_sitl/plugin/BetaflightPlugin.cc:724](/home/sia/agilicious_internal-main/build/betaflight_sitl/plugin/BetaflightPlugin.cc:724) | 接收模拟 FC 电机包，作用到 rotor joints |
| 插件回传 / 真值 | [build/betaflight_sitl/plugin/BetaflightPlugin.cc:863](/home/sia/agilicious_internal-main/build/betaflight_sitl/plugin/BetaflightPlugin.cc:863)、[build/betaflight_sitl/plugin/BetaflightPlugin.cc:818](/home/sia/agilicious_internal-main/build/betaflight_sitl/plugin/BetaflightPlugin.cc:818) | FDM 发往 9003，odometry 供 Gazebo adapter 生成 RTK 与评估数据 |

生成后的插件可能因重新构建而改变行号；维护源是 [betaflight_sitl/plugin/BetaflightPlugin.patch:1](/home/sia/agilicious_internal-main/betaflight_sitl/plugin/BetaflightPlugin.patch:1) 和 prepare_assets.py。acados 同理：飞行时调用生成 C 和库，[agilib/externals/acados_code_generator/drone_model.py:85](/home/sia/agilicious_internal-main/agilib/externals/acados_code_generator/drone_model.py:85) 是模型生成源码，不是在控制周期调用的 Python。

**13．哪些“重复”值得改，哪些应该保留。**

| 现象 | 判断 | 原因 / 处理方向 |
|---|---|---|
| fusion 已估计，Pilot 又有 estimator | 当前不重复融合 | Pilot 的 FeedthroughEstimator 仅存取状态 |
| ROS timer 与 Pilot pipelineThread | 当前不重复跑控制循环 | 后者没有启动 |
| AHRS 与 EKF | 稳态不重复姿态融合 | AHRS 只 bootstrap |
| control、output、MSP 的 SafetyGate | 应保留各边界检查 | 消息传输后状态可能变化，最终写前也需复查；统一诊断与状态语义 |
| EKF getAt 每个 IMU 从 RTK 后验重放 | 明确存在重复状态传播 | 分开实时预测缓存与后验/协方差，延迟观测到来才回放 |
| AUTO接管边沿所在周期复制并变换全CSV，再复制给SampledTrajectory | 明确存在整轨迹处理 | 用相对轨迹+捕获变换，采样时只变换预测窗；或预分配并转移所有权 |
| resetHover 后马上 off/enable 再换 CSV | 明确存在重复参考切换 | 用一次 replaceReference 原子操作表达真实意图 |
| 每周期复制完整参考 vector，只取 front | 明确存在多余数据复制 | 提供当前参考 const 访问或单点 getter |
| 每周期重写静态机体约束 | 可以优化 | 静态约束用 dirty 标记；真正变化的在线参考仍更新 |
| 5 ms watchdog 与 100 Hz命令都发 OutputStatus | 名义约300 Hz，存在重复发布 | 检查频率和心跳发布频率分开，状态变化立即发 |
| ROS2 自写 UDP 与旧 bridge 重复实现 | 跨入口维护重复，不是单次运行双发 | 抽出纯映射/编码函数；不能启动两套发送器 |

EKF 重复传播的确定证据是 [agilib/src/estimator/ekf_imu/ekf_imu.cpp:42](/home/sia/agilicious_internal-main/agilib/src/estimator/ekf_imu/ekf_imu.cpp:42)：getAt 输出后将 prior 重置回 posterior；下一 IMU 又从该后验传播，内部最大积分步长 0.0001 s（382 行）。RTK 10 Hz 且有延迟时，许多段历史反复积分。保留正确后验协方差的设计目的合理，但实现可以有独立的实时预测缓存。节省多少 CPU 必须在目标机测量，不能从调用次数直接下性能结论。

AUTO接管边沿所在周期的全量轨迹操作位于 8 ms 耗时计时内部：[agilib/src/pilot/hardware_pilot.cpp:127](/home/sia/agilicious_internal-main/agilib/src/pilot/hardware_pilot.cpp:127)、137、[agilib/src/reference/trajectory_reference/sampled_trajectory.cpp:9](/home/sia/agilicious_internal-main/agilib/src/reference/trajectory_reference/sampled_trajectory.cpp:9)。长 CSV 在 ARM 平台上可能导致接管第一周期超预算，这是待测风险；不应仅因已预热就假设该周期与悬停 warm 周期开销相同。稳态AUTO周期不会重复加载或变换整份CSV。

**14．架构问题与建议的处理顺序。**

优先级 P0 在本文表示“首先影响启动/验证结论”，不是未经实测的事故等级。

| 优先级 | 已确认的事实 / 风险 | 源码证据 | 建议 |
|---|---|---|---|
| P0 | 普通 hardware flight 没有启动完整 authority/health 生产者；MAVLink 不补 RTK 契约 | [agi_ros2/launch/flight.launch.py:49](/home/sia/agilicious_internal-main/agi_ros2/launch/flight.launch.py:49)、[agi_ros2/scripts/msp_evidence.py:122](/home/sia/agilicious_internal-main/agi_ros2/scripts/msp_evidence.py:122) | 做一个完整实机 profile：明确驱动、来源、质量、机体、推力表、模式；缺少任一项启动前逐项报告 |
| P0 | 默认 SITL 开放宽时效实验，且默认加载特定绝对路径轨迹 | [agi_ros2/launch/flight.launch.py:93](/home/sia/agilicious_internal-main/agi_ros2/launch/flight.launch.py:93) | 常规 profile 恢复严格时效与空轨迹；延迟实验独立命名、明确显示实际保护状态 |
| P0 | 配置有源码/安装双份；EEPROM 根据源码写，flight 可读安装副本 | [agi_ros2/scripts/launch.sh:9](/home/sia/agilicious_internal-main/agi_ros2/scripts/launch.sh:9)、[betaflight_sitl/run.py:717](/home/sia/agilicious_internal-main/betaflight_sitl/run.py:717) | 统一明确的配置根；启动打印解析后绝对路径、配置摘要/hash并录包 |
| P1 | 非shadow轨迹启动先于最终授权，未绑定 ARM/warm 成功转移 | [agilib/src/pilot/hardware_pilot.cpp:133](/home/sia/agilicious_internal-main/agilib/src/pilot/hardware_pilot.cpp:133)、186 | 用统一授权状态变化驱动 capture/start/stop，避免未接管时参考先走 |
| P1 | 轨迹结束分支反复累加时间戳 | [agilib/src/sampler/time_based/time_sampler.cpp:31](/home/sia/agilicious_internal-main/agilib/src/sampler/time_based/time_sampler.cpp:31) | 逐点赋目标时间；补“恰好终点/越过终点/切悬停”的边界回归 |
| P1 | 正常SITL/实机的 trajectory_active 恒false、elapsed为0 | [agilib/src/pilot/hardware_pilot.cpp:195](/home/sia/agilicious_internal-main/agilib/src/pilot/hardware_pilot.cpp:195) | 统一正常/影子参考生命周期，并区分 hover_active 与 csv_active |
| P1 | 输出映射/发送异常原因可能被通用 reason 覆盖；控制异常被吞 | [agi_ros2/src/command_output_node.cpp:263](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:263)、301；[agilib/src/pilot/hardware_pilot.cpp:168](/home/sia/agilicious_internal-main/agilib/src/pilot/hardware_pilot.cpp:168) | 保留 last_fault、fault_code、首次发生时间、solver状态；不要只暴露“health/configuration” |
| P1 | 融合多数无效数据直接 return，缺少等待与重置原因 | [agi_ros2/src/state_fusion_node.cpp:112](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:112) | 增加初始化阶段、RTK接收/接受/拒绝计数、age、reset_reason、bias/协方差/innovation |
| P1 | 实机已开启任一可选遥测超时，整个transport永久失效 | [agi_ros2/src/msp_telemetry.cpp:104](/home/sia/agilicious_internal-main/agi_ros2/src/msp_telemetry.cpp:104)、[agi_ros2/src/command_output_node.cpp:120](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:120) | profile 声明必需与诊断类别；设计有会话边界的恢复，不盲目忽略错误 |
| P1 | 3 ms对时等待、2.5 ms遥测写预算与10 ms IMU门限共用单线程时间预算 | [agi_ros2/src/node_common.cpp:29](/home/sia/agilicious_internal-main/agi_ros2/src/node_common.cpp:29)、[agi_ros2/src/command_output_node.cpp:119](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:119) | 测队列等待、callback、完整tick、最终发送时age；用非阻塞调度/明确优先级保留串口单owner |
| P1 | EKF 每帧重复传播、接管时全轨迹操作 | 上一节 | 先测最坏时延，再优化缓存与轨迹表示 |
| P1 | 内存bridge创建多余电压watchdog，停止标志无同步，快速退出可能等待 | [agilib/src/bridge/bridge_base.cpp:12](/home/sia/agilicious_internal-main/agilib/src/bridge/bridge_base.cpp:12)、[agilib/include/agilib/utils/agi_watchdog.hpp:19](/home/sia/agilicious_internal-main/agilib/include/agilib/utils/agi_watchdog.hpp:19) | 从构造期禁用不需要的线程；同步停止标志并notify，补快速启动/退出回归 |
| P2 | RTK 缓存只有最后一条，缺少session/来源冲突管理 | [agi_ros2/src/state_fusion_node.cpp:75](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:75) | 有界时间有序观测队列，统一来源session；不任意覆盖未处理观测 |
| P2 | RTK更新只查有限/时间/质量位，没有创新统计异常值门控 | [agilib/src/estimator/ekf_imu/ekf_imu.cpp:135](/home/sia/agilicious_internal-main/agilib/src/estimator/ekf_imu/ekf_imu.cpp:135) | 增加创新/协方差门限、拒绝原因；fixed并不保证不跳点 |
| P2 | 初始化可由很短的重力对齐完成，没有充分静止/收敛证据 | [agilib/apps/companion_ahrs.cpp:59](/home/sia/agilicious_internal-main/agilib/apps/companion_ahrs.cpp:59) | 分开 initialized/converged，增加静止窗口与质量指标，正确支持重启条件 |
| P2 | ROS可修改参数与内部生效行为不一致 | [agi_ros2/src/control_node.cpp:34](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:34)、[agi_ros2/src/state_fusion_node.cpp:58](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:58) | 配置要么启动时只读，要么明确实现受控重配；不可只改参数服务器 |
| P2 | 硬件仍使用包含UDP/平方根模型的桥接参数类型并验证无关项 | [agilib/src/bridge/betaflight_udp/betaflight_udp_bridge_params.cpp:33](/home/sia/agilicious_internal-main/agilib/src/bridge/betaflight_udp/betaflight_udp_bridge_params.cpp:33) | 分离 rates标定、硬件推力标定、transport 配置 |
| P2 | timeout/n_timeouts_for_lock 被读入，但ROS2没有使用旧BridgeBase UDP watchdog | [agi_ros2/src/command_output_node.cpp:69](/home/sia/agilicious_internal-main/agi_ros2/src/command_output_node.cpp:69)、318 | 删除无效配置或接入统一安全参数；清楚列出当前25/250ms来自何处 |
| P2 | MPC 一些参数只读不使用；用户会误判调整已生效 | [agilib/src/controller/mpc/mpc_params.cpp:33](/home/sia/agilicious_internal-main/agilib/src/controller/mpc/mpc_params.cpp:33)、61 | threaded_preparation/timing/exp_decay/max_wait 等明确废弃或接入；不能靠参数名猜功能 |
| P2 | 正常 External/None 拓扑仍产生“未创建模块/无Guard”警告 | [agilib/src/pilot/pilot_params.cpp:278](/home/sia/agilicious_internal-main/agilib/src/pilot/pilot_params.cpp:278)、[agilib/src/pilot/pilot.cpp:24](/home/sia/agilicious_internal-main/agilib/src/pilot/pilot.cpp:24) | 正式依赖注入，预期配置用 INFO，只对异常 WARN |
| P2 | bag路径写死 /home/sia/...，record失败会结束主链 | [agi_ros2/launch/flight.launch.py:65](/home/sia/agilicious_internal-main/agi_ros2/launch/flight.launch.py:65)、[agi_ros2/launch/msp.launch.py:27](/home/sia/agilicious_internal-main/agi_ros2/launch/msp.launch.py:27) | 可移植数据目录；飞行与研发分别定义录包故障策略 |
| P2 | float数组索引式诊断与笼统状态字符串难以自动分析 | [agi_ros2/src/control_node.cpp:200](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:200) | 用有字段名/单位的诊断消息，统一门限、当前值、拒绝码 |
| P2 | 多发布者/QoS错误缺少主链统一检测 | [agi_ros2/src/state_fusion_node.cpp:46](/home/sia/agilicious_internal-main/agi_ros2/src/state_fusion_node.cpp:46)、[agi_ros2/src/control_node.cpp:67](/home/sia/agilicious_internal-main/agi_ros2/src/control_node.cpp:67) | 启动检查 publisher数量、QoS、frame、时间域与源身份 |

TimeSampler 末端错误可直接从实现判定：同一个 setpoint 在循环里连续 += t_curr，结果不是 t、t+dt、t+2dt。它会污染参考时间契约；当前 MPC 状态向量不包含这些时间字段，因此不能据此断言必然失控。应修复并用末端边界测试确定影响。

watchdog 问题可达于当前默认控制路径：DebugBridge 与 CommandSink 都先经过 BridgeBase 无条件创建30秒电压watchdog，再在派生构造体disable。enabled_ 是普通bool，工作线程读取与disable写入没有同步；disable也不唤醒wait_for，析构则直接join。见 [agilib/src/utils/agi_watchdog.cpp:5](/home/sia/agilicious_internal-main/agilib/src/utils/agi_watchdog.cpp:5)、[agilib/src/bridge/debug_bridge.cpp:8](/home/sia/agilicious_internal-main/agilib/src/bridge/debug_bridge.cpp:8)。若线程先进入等待，快速退出可能等待剩余超时；长时间运行不一定出现此延迟。不能说每次退出必等30秒，也不能把两个线程的等待简单相加。

还有一项模型层面的评估：MPC优化的是四路电机推力，但最终只向Betaflight输出总推力及未来角速度，飞控再用自身PID实现角速率。这可由 [agilib/src/controller/mpc/controller_mpc.cpp:109](/home/sia/agilicious_internal-main/agilib/src/controller/mpc/controller_mpc.cpp:109) 和输出协议直接看出。因此模型有效性还依赖内环跟踪、延迟与饱和特性；建议在目标机数据上比较实际角速度响应与预测，必要时将内环动态或率/推力接口更明确地纳入预测模型。这是接口导出的待验证建议，不是已证实的控制不稳定结论。

不要把 /status 的“health/configuration”理解成已定位到具体故障。输入健康是多项布尔值和时效的合取；目前很多不同故障映射到同一句话。对实机 debug，先把“缺谁、哪个值、当前age、要求值、最近错误、来源session”完整输出，往往比先大改类层次更有收益。

建议保持三个核心进程，逐步收敛为以下接口：
~~~text
profile配置
  -> NavigationSource (IMU + 有来源/质量/时间的导航观测)
  -> StateFusion (状态 + 估计质量 + 诊断)
  -> ControlCore (state + reference -> command + solver status)
  -> Supervisor (健康/ARM/AUTO状态变化 + reference生命周期)
  -> Output (独立复核 + mapper + 唯一transport owner)
~~~
这不要求新增六个进程；是把已有代码中的责任用明确接口表达。控制算法尽量成为可离线回放的确定性计算部分，ROS node 负责订阅、时间、发布；Supervisor 明确拥有轨迹开始/停止事件；Output 保持最终否决权。

**15．按信号流调试，怎样定位“启动了但不工作”。**

先确认实际运行入口和配置，再逐个边界检查，不要先把所有 health 置 true。以下命令均为观察现有节点，不会发控制指令：

~~~bash
source install/agi_ros2/local_setup.bash
ros2 node list
ros2 param dump /flight_control
ros2 param dump /state_fusion
ros2 param dump /command_output
ros2 topic info /sensors/imu -v
ros2 topic info /sensors/rtk -v
ros2 topic info /fused_state -v
ros2 topic echo /status
ros2 topic echo /output_status
ros2 topic echo /computation_status
~~~

节点运行时参数 dump 只能看到 ROS 参数，不包含 Pilot/MPC/机体 YAML 的全部内容；而且参数值可见不证明构造期缓存已重载。应同时核对前述解析后路径和实际配置文件。

| 观察现象 | 下一步查什么 | 最接近的代码位置 |
|---|---|---|
| 节点直接退出 | FATAL、配置路径/格式、缺推力表、串口打不开、硬件模式与sim_time冲突 | 三个 *_main.cpp 的 catch；各节点构造 |
| 有IMU却没有fused_state | frame、stamp递增、RTK/fix/yaw、初始化条件；shadow看原点/高度/heading确认 | fusion.onImu 112–158 |
| topic有发布者但节点无数据 | DDS domain、QoS匹配；尤其 RTK subscriber默认reliable，best_effort驱动不匹配 | fusion构造46–55 |
| fused_state有数据但SENSOR_CHECK | Authority/Health、OutputStatus每项值和age，不只看频率 | control.tick139–165，SafetyGate.inputsHealthy |
| MPC warming长期不结束 | 50周期是否被age/cadence/8ms/输出故障不断打断 | HardwarePilot.tick171–178 |
| AUTO high却无输出 | 是否先见健康AUTO low；ARM、KILL、命令与最新授权是否匹配 | SafetyGate.update34–68，output238–257 |
| CSV已加载但参考不动 | 是否出现正确AUTO边沿；检查/reference，不用正常模式的trajectory_active单字段判断 | HardwarePilot133–155、195 |
| 推力输出被拒绝 | total_thrust有效、电压/推力在表内；查看MSP事件与fault_count | output179–203 |
| 实机一开始正常随后transport false | 有没有未支持的GPS/其他遥测超时，串口写deadline/错误 | MspTelemetry.tick、output120 |
| shadow 无融合 | /sensors/navigation、/navigation/status、原点是否锁定、heading/altitude配置 | gnss_adapter54–105 |
| shadow 有MPC、status拒绝接管 | 此入口本来禁输出，严格健康仍可能不满足 | shadow_only两层互锁、msp_evidence124 |

频率与新鲜度是两回事：重复旧 stamp 的高频消息仍是旧数据；一次匹配RC ACK也不证明物理接管。应录并分析采集stamp、接收monotonic、发布monotonic、计算完成、串口TX之间的差值。

**16．阅读代码时的主路径索引与排除项。**

为避免只读三个 node 就以为看完了，下面列出其下沉依赖；括号内是关键入口，不表示整文件每个函数都执行。

| 分层 | 本入口相关项目文件 |
|---|---|
| 构建与进程入口 | scripts/build.sh、scripts/launch.sh、launch/flight.launch.py、三个 *_main.cpp；SITL另有run.py、prepare_assets.py、sensors.sh |
| 公共时间/证据 | [agi_ros2/src/node_common.cpp:14](/home/sia/agilicious_internal-main/agi_ros2/src/node_common.cpp:14)、[agi_ros2/include/agi_ros2/node_common.h:15](/home/sia/agilicious_internal-main/agi_ros2/include/agi_ros2/node_common.h:15) |
| 消息合同 | [agi_ros2/msg/FusedState.msg:1](/home/sia/agilicious_internal-main/agi_ros2/msg/FusedState.msg:1)、[agi_ros2/msg/Rtk.msg:1](/home/sia/agilicious_internal-main/agi_ros2/msg/Rtk.msg:1)、[agi_ros2/msg/Authority.msg:1](/home/sia/agilicious_internal-main/agi_ros2/msg/Authority.msg:1)、[agi_ros2/msg/Health.msg:1](/home/sia/agilicious_internal-main/agi_ros2/msg/Health.msg:1)、[agi_ros2/msg/SafetyEvidence.msg:1](/home/sia/agilicious_internal-main/agi_ros2/msg/SafetyEvidence.msg:1)、[agi_ros2/msg/ControlCommand.msg:1](/home/sia/agilicious_internal-main/agi_ros2/msg/ControlCommand.msg:1)、[agi_ros2/msg/OutputStatus.msg:1](/home/sia/agilicious_internal-main/agi_ros2/msg/OutputStatus.msg:1) |
| 融合主体与初始化 | state_fusion_node.cpp、[agilib/apps/companion_ahrs.cpp:59](/home/sia/agilicious_internal-main/agilib/apps/companion_ahrs.cpp:59)、[agilib/apps/companion_ahrs.hpp:17](/home/sia/agilicious_internal-main/agilib/apps/companion_ahrs.hpp:17) |
| EKF计算与参数 | [agilib/src/estimator/ekf_imu/ekf_imu.cpp:23](/home/sia/agilicious_internal-main/agilib/src/estimator/ekf_imu/ekf_imu.cpp:23)、[agilib/src/estimator/ekf_imu/ekf_imu_params.cpp:5](/home/sia/agilicious_internal-main/agilib/src/estimator/ekf_imu/ekf_imu_params.cpp:5)、[agilib/include/agilib/estimator/ekf_imu/ekf_imu.hpp:52](/home/sia/agilicious_internal-main/agilib/include/agilib/estimator/ekf_imu/ekf_imu.hpp:52) |
| 控制监督 | control_node.cpp、hardware_pilot.cpp、hardware_safety.hpp |
| 配置/工厂 | pilot_params.cpp、pipeline_config.cpp、[agilib/src/utils/module_config.cpp:9](/home/sia/agilicious_internal-main/agilib/src/utils/module_config.cpp:9)、parameter_base.cpp、[agilib/src/utils/file_utils.cpp:7](/home/sia/agilicious_internal-main/agilib/src/utils/file_utils.cpp:7)、[agilib/include/agilib/utils/yaml.hpp:70](/home/sia/agilicious_internal-main/agilib/include/agilib/utils/yaml.hpp:70) |
| 状态/机体/输入数据结构 | [agilib/src/types/quad_state.cpp:1](/home/sia/agilicious_internal-main/agilib/src/types/quad_state.cpp:1)、[agilib/src/types/quadrotor.cpp:112](/home/sia/agilicious_internal-main/agilib/src/types/quadrotor.cpp:112)、[agilib/include/agilib/types/command.hpp:1](/home/sia/agilicious_internal-main/agilib/include/agilib/types/command.hpp:1)、[agilib/include/agilib/types/setpoint.hpp:1](/home/sia/agilicious_internal-main/agilib/include/agilib/types/setpoint.hpp:1)、[agilib/include/agilib/types/imu_sample.hpp:1](/home/sia/agilicious_internal-main/agilib/include/agilib/types/imu_sample.hpp:1) |
| 状态交接 / Pipeline | Pilot、Pipeline、[agilib/src/estimator/feedthrough/feedthrough_estimator.cpp:24](/home/sia/agilicious_internal-main/agilib/src/estimator/feedthrough/feedthrough_estimator.cpp:24)、[agilib/src/estimator/estimator_base.cpp:1](/home/sia/agilicious_internal-main/agilib/src/estimator/estimator_base.cpp:1) |
| 参考 | trajectory_csv.hpp、reference_base.cpp、hover_reference.cpp、sampled_trajectory.cpp、time_sampler.cpp、sampler_base.cpp |
| MPC | controller_mpc.cpp、mpc_params.cpp、wrapper.cpp、controller_base.cpp；CoG对象构造但默认不运行适配 |
| 生成solver | [agilib/src/controller/mpc/acados/acados_solver_drone_model.c:788](/home/sia/agilicious_internal-main/agilib/src/controller/mpc/acados/acados_solver_drone_model.c:788) 创建、914 solve、923 free；drone_model_model 中ODE/forward sensitivity；drone_model_cost 中阶段/终端代价与雅可比；下沉acados/HPIPM/BLASFEO |
| 内存命令bridge与通用辅助 | hardware_pilot.cpp中的CommandSink、[agilib/src/bridge/bridge_base.cpp:38](/home/sia/agilicious_internal-main/agilib/src/bridge/bridge_base.cpp:38)、[agilib/src/bridge/debug_bridge.cpp:1](/home/sia/agilicious_internal-main/agilib/src/bridge/debug_bridge.cpp:1)、logger/timer/watchdog等通用实现 |
| 真正物理输出 | command_output_node.cpp、betaflight_rc_mapper.cpp、betaflight_udp_bridge_params.cpp、thrust_table.hpp、betaflight_msp_bridge.cpp、msp_telemetry.cpp |
| 可选MAVLink/GNSS/shadow | mavlink_sensor_node.cpp、gnss_adapter.py、msp_evidence.py、shadow_support.py、Navigation/LocalNavigation/Heading等msg和相应launch |
| 独立串口台架 | msp.launch.py、msp_node.cpp、共用MSP bridge/telemetry |
| 仿真物理反馈 | 插件patch/生成插件、Gazebo模型、外部Betaflight收RC/内环/混控源码 |

这些名字存在于仓库，但不是默认 ROS2 flight 的运行分支：

- agiros/ROS1 节点、旧 agilib/apps/betaflight_sitl.cpp 主程序、旧 apps/betaflight_hw.cpp 诊断主程序。
- apps/betaflight_msp_client.cpp 的 TCP client、旧 BetaflightUdpBridge 实例。
- Pilot::launchPipeline/pipelineThread、Pilot::start/land/goToPose、多项式轨迹生成、Guard safety pipeline。
- MockVIO、Agilib 内置其他估计器/控制器/内环；HardwarePilot 强制 External+MPC+Time。
- CoG在线适配在当前默认 cog_enable=false 下不运行。
- acados Python 模型生成器、build.sh、配置复制流程不会在每个控制周期执行。

既不能把“被编译进 agilib”当作“当前必经调用”，也不能把没有启动的旧控制器当成重复运行证据。

**17．审阅结论的验证边界。**

本次完成源码互相对照与路径/行号核对，形成可追踪的静态说明；没有重新运行 README 中记录的历史测试，不能把其中“通过”记为本次结果。没有基于静态检查给出实机 UART 吞吐、MPC最坏耗时、估计误差或真实回退时延的验收结论。

后续修改建议配套的针对性验证是：轨迹末端时间戳与切悬停测试、未ARM/未warm AUTO边沿测试、正常模式参考活动状态测试、源码/安装配置一致性检查、观测延迟下EKF增量缓存与现有结果对比、缺失可选遥测测试，以及目标机采样→估计→MPC→UART的端到端时延录包。现有合成DDS/PTY测试入口在 [agi_ros2/scripts/test.sh:1](/home/sia/agilicious_internal-main/agi_ros2/scripts/test.sh:1) 与 agi_ros2/test，真实设备验证仍需单独完成。
