# CM5 + GEPRC F722 实机接入与部署

当前交付是**可编译、可做无桨串口诊断的硬件接入基础版，不是可飞的自动驾驶程序**。
用户已确定 GEPRC F722、Betaflight 2026.6；RTK 尚未确定，IMU 尚未采购。
状态机与真实 Pilot/MPC 的软件协调层已补充，具体调用见 [控制链路说明](CONTROL.zh-CN.md)。
这些信息不足以实现、验证真实传感器驱动及闭环，所以程序只提供 `--monitor`，没有 `--arm` 或 `--auto`。
不要用 SITL 的 `--arm`、UDP 参数或 MockVIO 绕过这个限制。

## 1. 已实现与未实现

| 模块 | 当前状态 |
|---|---|
| 独立硬件入口 | `agilib/apps/betaflight_hw.cpp`，只读 MSP；无 ROS/Gazebo/acados 运行依赖 |
| 串口传输 | `agi::hardware::BetaflightMspBridge`：独占串口、单线程归属、非阻塞读写、单调时钟绝对截止时间 |
| MSP 帧 | 发 v1；接收 v1 XOR/native v2 CRC8-DVB-S2，检查方向、长度、错误回复；不支持 v2-over-v1 |
| RC 覆盖基础接口 | 仅 4 个 AETR 通道，拒绝越界值；入口程序不调用该接口 |
| 公共角速度映射 | `BetaflightRcMapper` 被 SITL 复用；ACTUAL 反解和 deadband 反解 |
| 实机推力接口 | `ThrustTable`，实测总推力 N、PWM、电压二维表；超出标定范围抛错，不外推 |
| 授权门控 | `SafetyGate`，未知证据默认拒绝；启动/故障恢复必须重新观察 AUTO 低→高 |
| ARM/KILL | 程序从不写 AUX；由实体接收机及 FC 实现 |
| 状态与配置诊断 | 读版本、build info、RX map、RC tuning、STATUS/RC/ANALOG 的原始回复；**没有自动判定配置通过** |
| Pilot/MPC 协调层 | `HardwarePilot::tick()` 已实现真实 MPC 调用、影子预热、定点接管、失败返回和状态机联动 |
| 硬件完整闭环 | **未实现**：真实 RTK/SPI IMU 驱动、PPS 同步、估计器集成、串口 mailbox、100 Hz 工作线程和日志队列 |
| 飞行前验证 | **未完成**：精确固件配置核验、推力表文件加载、机体标定、实机轴向/接管/超时测试 |

通信类不是旧的 `agi::MspBridge`，也暂未继承 `BridgeBase`。旧类的自动低油门/AUX 行为不符合这里的权限设计。
`SafetyGate` 已接入 `HardwarePilot`，但不代表硬件信号已经读入；不得人为把证据字段全部设为 true 进行实飞。
当前采用更保守的策略：一次 MPC 超过 8 ms 或一次串口发送/查询超时即停止覆盖；未实现上一安全命令复用、RTK 降级悬停或 3–5 帧容错。
Linux 停发后 FC 仍可能保持最后一帧一段时间，停发不等于立即接管，更不等于急停。

## 2. 硬件选择与接线

### 飞控版本先确认

“GEPRC F722”包含多种板型。官方 [HD](https://geprc.com/product/gep-f722-hd-flight-controller/)、
[HD V2](https://geprc.com/product/gep-f722-hd-v2-flight-controller/)、
[BT HD V2](https://geprc.com/product/gep-f722-bt-hd-v2-flight-controller/) 的接口/target 不完全相同。
先看 PCB 完整丝印，按 [GEPRC 官方手册目录](https://geprc.com/electronics/fc-manual/) 获取对应接线图。
不要凭 F722 名称写入 UART 编号或刷另一个 target。

在拆桨状态下用 Configurator 导出并保存：

```text
version
status
diff all
dump all
serial
resource
map
aux
get msp_override
get rates_type
get roll_rc_rate
get pitch_rc_rate
get yaw_rc_rate
get roll_srate
get pitch_srate
get yaw_srate
get roll_expo
get pitch_expo
get yaw_expo
get deadband
get yaw_deadband
get mid_rc
get min_check
get max_check
get thr_mid
get thr_expo
```

记录 `version` 的完整 commit、target、build options；“2026.6”本身不能证明带有所需 override 超时实现。
上述查询中若有未知参数，保存输出，按精确版本源码核对，不直接执行旧固件的设置脚本。

### IMU 建议

体积/重量优先，可选**正规来源 ICM-42688-P + 小型自制 SPI/DRDY 板**作为第一版候选。
其封装为 2.5×3×0.9 mm，典型陀螺噪声密度 2.8 mdps/√Hz，见
[TDK 数据手册](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000347-icm-42688-p-v1.6.pdf)。
这不等于它在所有精度指标上“最好”；板级稳压、时钟、温漂标定和安装振动会影响最终精度。
若优先考虑出厂标定与温度性能，可比较
[ADI ADIS16470](https://www.analog.com/en/products/adis16470.html)，代价是封装和整板尺寸增加。
本版本没有实现任一器件驱动，也没有验证具体模块重量；采购前确定器件、模块电路和引脚表。

伴随 IMU 独立于飞控 IMU，SPI + 数据就绪中断；不从 FC 的 10–20 Hz MSP IMU 提供外环姿态。
IMU 板靠近重心、刚性安装、记录旋转矩阵和杆臂。SPI 导线短，电源/地回路避开 ESC 大电流。

### RTK 采购约束

选择真正支持双天线 moving-baseline/静止航向的接收机，确认输出协议可同时提供：

- 位置、世界系速度，10–20 Hz；固定解状态、卫星数、位置/速度协方差或精度估计。
- 双天线航向及其有效性、精度、基线长度；不能把单天线 course-over-ground 当作静止航向。
- GNSS 测量历元、PPS、时间有效性；明确时标是 GPS/UTC 以及闰秒处理。
- 差分改正输入方式、天线规格、基线安装要求、最大动态能力和原始日志。

同时评估两个天线、地板、线缆和基线支架的总重量。接收机定型后才能写具体协议驱动。

### 接线

```text
ELRS/CRSF 接收机 TX → FC 独立 UART RX
ELRS/CRSF 接收机 RX ← FC 同一 UART TX（遥测，按模块说明）
CM5 硬件 UART TX    → FC 另一个 UART RX（MSP）
CM5 硬件 UART RX    ← FC 同一 UART TX
CM5 GND            — FC GND
RTK                → CM5 独立 UART/USB，PPS → 合适电平的 GPIO
伴随 IMU           → CM5 SPI + DRDY GPIO
```

CM/FC 信号电平按各载板确认，通常用 3.3 V TTL；不可把 RS-232 或 5 V 信号接入。
CM5 用独立、带瞬态裕量的 BEC，并与飞控共地；不默认由 FC 的 5 V 焊盘供电。
检查 USB 与电池同时接入是否回灌。CM5 配可靠散热、固定线束与接插件。

## 3. Betaflight 配置

1. Ports 中给接收机对应 UART 开启 Serial RX；Receiver 使用 CRSF。
2. 给 CM5 对应的另一 UART 开启 MSP，起始可用 115200；验证稳定后再用 921600，程序波特率必须一致。
3. 保留实体 ARM（AUX1）、AUTO/MSP OVERRIDE（AUX2）、KILL/FAILSAFE（AUX3）。
4. Modes 中配置 MSP OVERRIDE 由 AUX2 控制，范围不要与 ARM/KILL 重叠。
5. 在确认固件存在以下参数之后设置并保存：

```text
map AETR1234
set msp_override_channels_mask = 15
set msp_override_failsafe = OFF
save
```

6. 重启后回读 `map`、`aux`、上述参数，确认 AUX1/2/3 永远来自实体链路。
7. 建议统一 ACTUAL rates，并记录三轴 center/max/expo、rate limit、mid_rc、deadband。
   方案中的 70/1000 deg/s 是配置目标，不要把 Configurator 的显示值直接当成 CLI 存储整数。
8. 明确 Acro/rate 控制模式，检查 Angle/Horizon、Headfree、Rescue 等开关不会在 AUTO 时改变控制语义。
9. 不复制 SITL PID、motor idle、悬停油门、质量、惯量、阻力、混控方向。先按实机完成手动飞行与内环调参。

当前官方源码的 [override 条件](https://github.com/betaflight/betaflight/blob/master/src/main/rx/msp_override.c)
要求模式开启、通道掩码匹配且覆盖样本新鲜；[MSP 新鲜度代码](https://github.com/betaflight/betaflight/blob/master/src/main/rx/msp.c)
使用 300 ms 窗口。**这些 master 链接不是你的已安装固件证明**。
必须检查对应 commit 是否包含相同实现及 `USE_RX_MSP_OVERRIDE`，并测量拔掉 MSP 后的真实回退时间。
如该版本仍无限保持 MSP 值，则不得进入自动飞行，应先修复/更换并验证固件。

## 4. CM5 上编译无桨诊断程序（现在可执行）

安装载板支持的 64 位 Linux。先检查 `uname -m` 应为 `aarch64`。
本诊断程序不要求 ROS、Gazebo、acados 或 RT 内核。

```bash
sudo apt update
sudo apt install build-essential cmake git
cd /path/to/agilicious_internal-main
cmake -S betaflight_hw -B build/betaflight_hw -DCMAKE_BUILD_TYPE=Release
cmake --build build/betaflight_hw -j 4
ctest --test-dir build/betaflight_hw --output-on-failure
build/betaflight_hw/agilicious_betaflight_hw --help
```

开发机如有 Python 3，还可运行无硬件端到端检查：

```bash
python3 betaflight_hw/test_monitor.py build/betaflight_hw/agilicious_betaflight_hw
```

根据 CM5 载板文档开启硬件 UART，取消该端口的登录串口/console，并重启。
不要假设 `/dev/serial0`、`/dev/ttyAMA0` 或某个 overlay 对所有载板都一样。
核对 device-tree、`/boot/firmware/cmdline.txt` 和串口 getty，避免两个进程占用同一设备。
把运行用户加入 `dialout`，退出登录后重新登录：

```bash
sudo usermod -aG dialout "$USER"
ls -l /dev/serial/by-id /dev/serial0 /dev/ttyAMA*
```

设备列表中部分路径不存在是正常的；选择实际接线对应的 UART。
关闭占用该串口的 Configurator、串口终端及其他服务，再运行（此处设备名仅为示例）：

```bash
build/betaflight_hw/agilicious_betaflight_hw --monitor /dev/ttyAMA0 921600
```

输出为 `monotonic_s,msp_code,payload_hex,error_count`，首行会明确 `MONITOR ONLY`。
启动时查询 1/2/3/5/64/111，之后约 10 Hz 查询 STATUS(101)、RC(105)、ANALOG(110)。
这是原始诊断数据，不能把十六进制字段误读成已核验的 arming flags/电压。
单个请求截止时间 50 ms；任何查询超时或错误回复使进程退出，返回码 1。
不要反复自动重启掩盖串口错误；先排查波特率、引脚、串口占用、固件功能和供电。
Ctrl-C 正常退出；任何退出路径均不发送 ARM、AUX、低油门或 CLI 写入。

## 5. 编译完整 Agilib（后续 MPC 集成准备）

保留仓库指定的 acados commit，不用随意升级的 master。下面是在 **CM5 本机**重新编译依赖的示例。
使用独立目录，避免覆盖开发机已有的 x86 二进制；需要网络可用。

```bash
sudo apt install libeigen3-dev
mkdir -p dependencies
git clone https://github.com/uzh-rpg/acados.git dependencies/acados-cm5
git -C dependencies/acados-cm5 checkout 78cb9a72975ce2b5616c35936c9ac504a3ed7e5a
git -C dependencies/acados-cm5 submodule update --init --recursive
cmake -S dependencies/acados-cm5 -B build/acados-cm5 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/dependencies/acados-cm5" \
  -DBLASFEO_TARGET=GENERIC -DHPIPM_TARGET=GENERIC
cmake --build build/acados-cm5 -j 4
cmake --install build/acados-cm5
file dependencies/acados-cm5/lib/libacados.so
cmake -S agilib -B build/agilib-cm5 \
  -DCMAKE_BUILD_TYPE=Release -DAGILIB_ARM_CPU=cortex-a76 \
  -DUNSAFE_MATH=OFF -DBUILD_TESTS=OFF \
  -DBUILD_BETAFLIGHT_SITL=OFF -DBUILD_BETAFLIGHT_HW=ON \
  -DFETCH_ACADOS=OFF -DACADOS_ROOT="$PWD/dependencies/acados-cm5"
cmake --build build/agilib-cm5 -j 4
ctest --test-dir build/agilib-cm5 -R betaflight_hw_test --output-on-failure
```

CM4 使用 `-DAGILIB_ARM_CPU=cortex-a72`。空值采用通用 ARMv8；不再强制 cortex-a57。
GENERIC 数学库是可移植起点，不保证 MPC 时限；优化 BLASFEO 前先核对该旧版本支持的目标并做数值/周期回归。
CMake 已有 ELF 架构检查，错误地使用 x86 `.so` 会在配置时失败。
这组 ARM 命令尚未在真实 CM5 执行；本次测试环境是 x86-64 Linux。

## 6. 真实闭环接入必须完成的工作

硬件确定后按以下接口实现，不从 MSP 或 Gazebo伪造状态：

1. **IMU 驱动**：WHO_AM_I、复位、量程/滤波器/ODR、FIFO/DRDY、字节序、溢出/丢样检测、温度与校准。
   输出 FLU 的 rad/s、m/s²、采样时刻和序号；静止加速度在机体水平时应约为 +Z 的 9.81 m/s²。
   偏置校准需要静止判据和足够持续时间，不能只取第一帧。
2. **RTK 驱动**：按选定协议校验帧和测量历元，将纬经高转换为固定局部原点 ENU；
   NED 速度需换为 ENU，北起顺时针 heading 转为 ENU yaw（并应用天线基线与机头夹角）。
   输入位置/速度协方差、fixed、heading 精度，不把经纬度直接赋给 `QuadState::p`。
3. **时间同步**：PPS/GNSS 时间关联到 `CLOCK_MONOTONIC`；记录测量延迟、不确定度、乱序和丢样。
   本代码使用 MONOTONIC 而非 RAW，所有组件必须一致；不能混用 Unix 时间、仿真时间或接收时间。
4. **估计器**：可复用 `CompanionAhrs` 的 `setHeading/setVelocity/addImu`，但其 `initialized()` 不代表收敛。
   增加 IMU 位置/速度传播、延迟 RTK 更新和残差健康检查；验证重力、轴向、杆臂和大动态表现。
   长期使用包含 gyro/acc bias 的误差状态 EKF。尚未定位的 SITL 航向异常也须回归。
5. **Pilot**：使用已实现的 `HardwarePilot::tick()`，硬件专用配置，禁止 MockVIO、SITL quad 和 UDP bridge；从估计器输入带采样时间的 `QuadState`。
   100 Hz 控制线程取得决策，再检查实机最大倾角/角速率、推力和地理围栏；不能绕过 `runPipelineChecked()` 的失败判断直接取旧命令。
   先做在手动悬停点接管的定点参考，不自动解锁/起飞，不加载高速 CPC33。
6. **串口 worker**：唯一线程创建/销毁 `BetaflightMspBridge`；控制线程通过有界最新值 mailbox 提交命令，
   禁止陈旧命令积压。每 10 ms 最多一帧 RC，监控穿插、预算不能侵占下一个控制周期。
   库中的同步 `request()` 不是 100 Hz 调度器；不能直接复用诊断入口的 50 ms 查询预算。
7. **真实授权**：读取精确固件的 AUX、有效接收机状态/FAILSAFE、ARM 和 arming-disable flags，监控中断即失效。
   AUX 通道值不等于 RF 链路健康，MSP ACK 也不等于电机已执行。
   从精确版本解析配置回读、mask、failsafe、rate profile/map/deadband；运行中变更或 FC 重启必须退出 AUTO。
8. **映射**：共用 ACTUAL 反解；实机 FLU 到 BF body-rate 的符号按安装实测，不能复制仿真中“三轴都同号”。
   把 `collective_thrust`（m/s²）乘整机质量 kg 得总推力 N，使用 `ThrustTable` 的真实表及新鲜电压。
   表中必须包含期望工作范围；越界、失效电压、异常质量应退出 AUTO。
9. **状态机**：把以上真实证据送入 `SafetyGate`，外部禁止把默认 false 改为 true 作为“配置通过”。
   当前门限 IMU 10 ms、RTK 300 ms、RC 状态 100 ms、命令 25 ms、MPC 8 ms。
   当前策略立即回退；若增加短时悬停/制动，必须另行设计并测试位置/速度有效性。
10. **日志**：有界队列记录采样/提交/实际发送时刻、周期抖动、求解耗时、命令、RC、状态机原因、RTK精度、
    电压和串口错误。磁盘满或慢写不能阻塞控制；诊断 stdout 不可直接用作飞行实时日志。

## 7. 标定和实时化

推力台至少覆盖 10–15 个 PWM 点、多个工作电压，固定桨、电机、ESC、协议、idle 与滤波配置。
保存总推力、实际负载电压、温度、转速及重复测量；不能把单电机 N 直接作为整机 N。
`ThrustTable` 当前接收内存矩形网格，不读取 CSV；应在启动阶段加载审计后的标定文件并验证单调性。
重新称重并辨识惯量、推力/扭矩系数；仿真模型不能作为默认硬件模型。

接好闭环后再进行实时化：载板支持的 PREEMPT_RT、`mlockall()` 及失败检查、预触碰栈、预分配缓冲、
控制/IMU/串口线程 CPU affinity 与经测量设计的 SCHED_FIFO 优先级、低优先级日志线程。
不要直接把会阻塞 I/O 的诊断进程设成最高实时优先级。

满负载和最高环境温度下连续运行至少 30 分钟，测量：

| 项目 | 初始验收目标 |
|---|---|
| MPC 周期 | 10 ms |
| 求解耗时 p99 / 最大值 | <5 ms / <8 ms |
| 周期抖动 p99 | <1 ms |
| 连续超时、FIFO 溢出、陈旧控制发送 | 0 |
| CPU 热降频 | 无 |

systemd 可在之后部署完整进程，但每次启动必须重新等候实体 AUTO 低→高。
watchdog/Restart=on-failure 只能重启 Linux 服务，不能自动解锁或自动恢复覆盖。
当前诊断阶段建议前台运行；没有提供伪装为飞行服务的开机自启动模板。

## 8. 台架到实飞的验收顺序

**现在可做**：拆桨接线、固件备份、实体通道配置、编译测试、只读 MSP 查询。
以下控制注入项目需要第 6 节闭环集成完成，当前 `--monitor` 不能执行：

| 阶段 | 操作与通过条件 |
|---|---|
| 无桨坐标检查 | 手持机体绕三轴转动，估计器姿态/角速率符号、静止重力、ENU位置方向一致 |
| 无桨授权检查 | 启动时 AUTO 高不覆盖；先低再高才允许；KILL/ARM 不受 CM 输出影响 |
| 无桨速率检查 | 小幅 ±roll/pitch/yaw，Blackbox 的 rate setpoint 与目标数值/符号一致 |
| 无桨失联检查 | 停进程、SIGSTOP、拔 MSP、CM 断电，测量最后一帧至真实 RC 恢复的延迟；AUTO 拨低可独立接管 |
| 无桨 RF 检查 | 关闭发射机，确认 FC 按配置进入 failsafe；持续 MSP 不掩盖接收机断链 |
| 传感器故障注入 | IMU停流、RTK float/超时、heading失效、时钟跳变、超时解算都停发；恢复不自动重入 |
| 推力台 | 用独立防护设施完成推力/电压标定 |
| 系留/手动 | 完成实机 Betaflight 内环调参，人工接管方向正确，电源和热性能合格 |
| AUTO定点 | 在人工悬停状态接管，小范围地理围栏，先定点，无自动起飞 |
| 低速轨迹 | 直线→圆→8字，先约2 m/s，再逐级增加，逐次检查日志 |

掉电和拔线这类故障注入先在无桨环境完成，不在首次自由飞行中随意断电。
接管时人工油门必须与当前悬停状态相容；停发不会使飞机自行保持高度。
FC KILL 的动作应明确为立即 disarm 还是触发 Stage-2 failsafe，两者不可混称；按场地和飞行阶段设置。

高速 CPC33 最后再做。时间缩放后速度/加速度/jerk/snap 必须一致，姿态和 body-rate 必须从缩放后的
加速度与 yaw 重新生成；多旋翼存在重力项，**不能机械地仅把原 body-rate 除以时间缩放系数**。
重新生成前馈推力并以实机质量/推力余量校验，不从原 CSV 猜质量后直接飞行。

## 9. 本次验证边界

本地 x86-64 编译了诊断程序、测试和完整 SITL 目标。伪串口测试覆盖四通道帧、退出停发、线程独占、
查询超时锁闭；单元检查覆盖 v1/v2、坏校验恢复、缓存上限、重启 AUTO 高、故障恢复、超时和推力插值。
ACTUAL 三轴/expo/deadband 反解有回归测试；Python 端到端伪飞控检查诊断握手、只读请求及 SIGINT 退出。
这些结果不证明真实 FC 固件回退、ARM/KILL、RTK/IMU、真实电源或实飞已经通过。
下一次硬件集成需提供 PCB 完整型号、`version`/`diff all`、RTK协议、IMU模块原理图/引脚和机体参数。
