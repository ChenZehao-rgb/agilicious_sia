# Agilicious × Betaflight 2026.6 SITL 闭环

这个适配器不依赖 ROS。数据链为：

```text
Gazebo BetaflightPlugin 发布的真值里程计 (200 Hz)
  -> agilib Feedthrough + MPC 外环 (100 Hz)
  -> body-rate + collective thrust
  -> Betaflight ACTUAL-rates 反映射
  -> UDP 9004 / AETR + AUX1
  -> Betaflight 角速度/PID/电机内环
  -> UDP 9002 -> Gazebo，Gazebo状态 UDP 9003 -> Betaflight
```

## 一条命令启动

在项目根目录执行：

```bash
python3 betaflight_sitl/run.py --gazebo --arm
```

建议首次限定 30 秒：

```bash
python3 betaflight_sitl/run.py --gazebo --arm --duration 30
```

不带 `--arm` 时只发送 `throttle=1000、AUX1=1000`，用于检查状态链而不会解锁：

```bash
python3 betaflight_sitl/run.py --gazebo
```

默认使用 MPC。需要回退到几何控制器时加 `--controller geo`。

## 起飞后跟踪 CSV 轨迹

```bash
python3 betaflight_sitl/run.py --gazebo --arm \
  --trajectory miscellaneous/datasets/ref_trajs/open_source/LOOP14_38.csv \
  --log build/loop10.csv
```

程序先执行 1 m 平滑起飞，再进入 CSV 的 `SampledTrajectory`。

- CSV 首个位置平移到起飞终点，轨迹形状、姿态、速度、角速度和时间尺度不变。
- CSV 的 `u_1..u_4` 是**生成该数据集的飞行器**的单桨推力。生成质量默认由文件
  自身最小二乘反解（`u_sum = m·|a − g|`）。本仓库自带的两组数据来自不同飞行器
  （LOOP 为 0.700 kg，CPC 为 0.857 kg），用同一个常数会让前馈推力最多偏 20%。
  需要覆盖时用 `--trajectory-source-mass KG`。
- 轨迹整体会被抬高，保证最低点距起飞点不低于 `--ground-clearance`（默认 0.8 m）。
- `--log FILE` 记录参考与实际状态的逐周期 CSV，供 `validate.py` 评分。

同一个 CSV 还记录 `/model/iris/aerodynamics` 的最新样本：机体系空速、机体阻力、
总气动力/力矩，以及每个旋翼的转速、推力、反扭矩、盘内阻力、诱导速度、前进比、
桨尖 Mach 数和入流求解状态。`aero_age_s` 是该气动样本相对当前控制周期的仿真时间
延迟，`aero_valid=1` 表示已经收到有效样本。新增列不会影响 `validate.py` 对旧列的评分。

## 全部参考轨迹的验证

```bash
python3 betaflight_sitl/validate.py            # 跑 miscellaneous/datasets 下全部轨迹
python3 betaflight_sitl/validate.py <某个.csv> # 只跑指定轨迹
```

只统计 CSV 轨迹真正作为参考的区间，起飞和收尾悬停不计入。旧 `LiftDrag` 模型的
成绩不能与新模型比较，修改气动参数后应重新运行本命令建立基线。

## 速度包线

覆盖层不再使用 Aeroloop 的两个点式 `LiftDrag` 元件，而使用盘平均 BEM：12 个径向
单元、16 个方位角单元、Prandtl 根部/尖部损失、诱导速度迭代、亚声速修正和跨声速
阻力增量。`num_blades=3` 明确表示乾丰 5136 三叶桨；Gazebo 里的桨叶网格只负责显示，
不参与气动力计算，因此无需把可视网格重画成三叶。

静态绝对推力和功率由用户提供的十个 RPM 台架点插值；BEM 只给出前飞/轴向入流相对
静态的变化。转速上限是 29280 rpm，超过台架范围不外推电机能力。机体阻力为
`0.5*rho*CdA*v*|v|`，MK5 外廓按 `Cd=1.04` 得到三个机体系方向的
`CdA=[0.007338, 0.009348, 0.037390] m^2`。

可独立于控制器计算稳态平飞包线：

```bash
build/betaflight_sitl/plugin/aero_envelope
```

当前假设下，35 m/s 有稳态解（约 77.1°俯角、28469 rpm），40 m/s 起没有同时满足
水平力平衡和 700 g 重力的解。100 m/s、满转速、桨盘未倾斜的诊断点为：单桨推力
27.1 N、单桨盘内阻力 10.0 N、机身 x 向阻力 44.2 N、前进比 0.504、推进侧桨尖
Mach 0.871。这个点已经进入跨声速近似区，只适合趋势和控制鲁棒性验证，不能当作
精确性能预测。

修正悬停油门前馈后，`aggressive_100mps.csv` 的闭环测试仍在 22.26 m/s 前后姿态内环
失稳并触地。它证明当前控制参数不能跟踪该轨迹，不证明模型计算失败，也不能证明实机
可达 100 m/s。

## 控制回路走仿真时间，而不是墙钟

整个适配器的时钟取自里程计消息头里的仿真时间戳，包括 Pilot 自己给起飞多项式、
悬停和采样轨迹打的时间戳（`Pilot` 用同一个时钟函数构造）。桥的看门狗仍然走墙钟
—— 那是真实的安全定时器，不属于被仿真的动力学。

这一点是必需的：Gazebo 的实时倍率并不恒为 1，开着 GUI 时也可能下降。如果参考按
墙钟推进，参考会跑赢飞机，得到的是计时错误而不是动力学结果。

副作用是实时倍率不再是硬约束：想用更小的步长（例如 0.1 ms，实测 0.84 倍）只是
跑得比真实时间慢，结果依然正确。`--duration` 现在也是仿真秒。

## 动力学标定

`calibrate_plant.py` 直接讲 Betaflight ↔ Gazebo 的 UDP 协议，不经过 Betaflight，
因此测到的是**被控对象本身**：

```bash
python3 betaflight_sitl/calibrate_plant.py
```

它先用二分法搜索悬停电机指令，再逐点测量归一化推力。当前台架插值模型的理论值为：

```text
hover rotor speed   ≈ 1006 rad/s
hover motor command ≈ 0.361（含速度伺服负载补偿）
hover stick         ≈ 0.324（含 5.5% idle）
maximum static T/W  ≈ 9.85
```

这两个数分别决定 `betaflight_udp.yaml` 的 `hover_throttle` 和
`quads/betaloop_iris.yaml` 的 `thrust_max` / `thrust_map`。换模型后必须重跑。

## 为什么要改 Gazebo 的动力学参数

Aeroloop 原始 Iris 的两个 `LiftDrag` 点既不是完整 BEM，也没有机体阻力；其几何和
转速也不对应 5.1 英寸三叶桨。`prepare_assets.py` 会删掉八个原始叶素插件，加载本
仓库的 `AgiliciousAerodynamicsPlugin`，同时把质量、轴距、桨半径、桨惯量和机体碰撞
外廓改成 MK5 数据。原始 `~/aeroloop_gazebo` 不会被修改。

同一份覆盖层还把里程计换成插件直接发布的真值：`gz-sim` 的 `OdometryPublisher`
用位姿差分估速度，而且三维模式下发的是**欧拉角速率**而非机体角速度，只在悬停
附近成立。

## Betaflight 内环增益

`betaflight_udp.yaml` 里带了 Betaflight 的 PID，由 `run.py` 写进隔离的 EEPROM。
横滚和俯仰保持 Betaflight 默认值。

偏航使用台架功率除以角速度得到的桨轴反扭矩，并随 BEM 的前飞负载变化；关节阻尼已
降到 `1e-5`，不再把伺服阻尼当成主要偏航来源。当前温和的偏航 PID 已通过 15 s 悬停
检查，但尚未使用实机数据辨识。

**这组增益不能替代桥里的偏航符号。** 桥对三个轴一视同仁地直接传递角速度符号：
Gazebo 模型的 IMU 带 180° 横滚安装，Betaflight 收到的陀螺已经在它自己的杆量坐标
系里，桥再对偏航取反就会把速率环变成正反馈。这个故障很容易被
`yaw_motors_reversed` 掩盖 —— 翻混控能恢复指令方向的符号，却翻不了陀螺反馈，于是
飞机能飞，但偏航速率会缓慢卷绕到一个与指令反号的恒定转速。降低增益只会把卷绕时间
从秒级拉长到几十秒，看起来像"慢慢偏航"。

## 隔离和兼容处理

启动器读取 `~/betaloop/config.txt`，但不会改动 Betaloop、Aeroloop 或原始
Betaflight EEPROM。每次运行都会在 `build/betaflight_sitl/runtime` 中：

1. 复制 SITL ELF 和 `eeprom.bin`；
2. 只在副本中写入 `ARM = AUX1 1700–2100`、AETR 映射、ACTUAL 速率曲线、
   死区、油门曲线和 PID —— 这些值全部来自 `betaflight_udp.yaml`，改配置文件即可，
   不需要手工编辑 EEPROM；
3. 写完之后再读回校验，任何一项不符或 CLI 响应不可解析都会拒绝解锁；
4. 生成 Aeroloop 模型覆盖层（电机顺序、桨叶气动、真值里程计）；
5. 把物理步长设为 1 ms，并把喂给 Betaflight 的 FDM 速率**独立**固定在 500 Hz；
6. 加载项目内的兼容插件，按 Betaflight 2026.6 的 144 字节格式收发 FDM 包，
   并避免物理线程长时间等待 UDP。

物理步长受 gz-sim 串行步进能力限制：实测这台机器 0.2 ms 仍能保持实时倍率 1，
0.1 ms 只有 0.84。因为控制回路走仿真时间，低于 1 只是跑得慢，不影响结果。

FDM 的收发都按 `fdmPublishFrequency` 的节拍进行，与物理步长解耦。插件的接收是阻塞
的（1 ms 超时），若每个物理步都去收，在 Betaflight 尚未应答的步上就会白等满超时；
步长小于传感器周期时这会把实时倍率拖到 0.32。电机指令在两次接收之间保持不变，正如
真实 ESC 保持上一个设定值。

插件的 FDM 发送**不受连接状态门控**。Betaflight SITL 只在收到 FDM 包之后才会
发出电机包，所以一旦把 `SendState` 放进 `betaflightOnline` 分支里，一次连续丢
包就会让两边互相等待、永久死锁：现象是稳定悬停中四个桨突然停转
（`rotor_w=[0,0,0,0]`）而飞控仍处于解锁状态并在加油门。同时把
`connectionTimeoutMaxCount` 从 5 提到 100，让 10 ms 的调度抖动不再触发切桨。

预启动进程保存 EEPROM 副本后会完全退出，正式飞行使用新进程，因此 CLI 设置的
arming-disable flag 和 TCP 5761 单客户端连接都不会带入飞行。

## 输出判定

正常启动后会依次看到：

```text
[WAIT] ...
[DISARM] ...
[PREARM] ...
Takeoff reference started.
[FLIGHT] p=[...] v_body=[...] thrust=... omega_cmd=[...]
```

`[DISARM]` 默认保持 6 秒，让 Betaflight 先看到 ARM 开关为低并越过其 5 秒上电
解锁保护；随后 `[PREARM]` 低油门解锁 2 秒。悬停稳态应为
`p_z≈1.0 m`、`thrust≈9.8`、`rotor_w≈±1000 rad/s`、油门 PWM 约 1330。
状态超过 250 ms 未更新时程序会立即解除 AUX1 并退出。

`rotor_w` 的正负号来自 CW/CCW 方向，不是故障。

## 常用调整

- 改过 `prepare_assets.py` 或插件补丁后只跑 `cmake --build` 是**没用的**：补丁是在
  CMake configure 阶段用 `configure_file` + `patch` 应用的，必须重新 configure
  （`run.py` 不带 `--no-build` 时会自动做）。
- 悬停高度持续上升或下降：重跑 `calibrate_plant.py`，用它给出的悬停电机指令
  换算 `betaflight_udp.yaml` 的 `hover_throttle`
  （`hover_throttle = (motor − motor_idle) / (1 − motor_idle)`）。
- 改过 Betaflight rate profile：改 `betaflight_udp.yaml` 即可，`run.py` 会写入
  并校验。适配器实现的是 ACTUAL rates 的精确逆曲线。
- 轨迹跟踪偏软或发散：调 `mpc_betaflight_sitl.yaml` 的 `Q_pos_*` / `Q_vel`。
  当前值已接近可用上限，再翻一倍会让 `LOOP14_38` 发散触地。
- 已经构建后跳过编译：加 `--no-build`。
- 查看适配器全部参数：

  ```bash
  build/betaflight_sitl/agilib/bin/agilicious_betaflight_sitl --help
  ```

## 手动构建

```bash
cmake -S betaflight_sitl/plugin -B build/betaflight_sitl/plugin \
  -DCMAKE_BUILD_TYPE=Release \
  -DAEROLOOP_PLUGIN_SOURCE="$HOME/aeroloop_gazebo/plugins/BetaflightPlugin.cc"
cmake --build build/betaflight_sitl/plugin --parallel

cmake -S agilib -B build/betaflight_sitl/agilib \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTS=OFF \
  -DBUILD_BETAFLIGHT_SITL=ON \
  -DFETCH_ACADOS=OFF \
  -DWARNINGS_AS_ERRORS=OFF
cmake --build build/betaflight_sitl/agilib --parallel
```

Gazebo 仅链接到这个 SITL 可执行程序，`agilib` 核心和以后 CM4 上的控制进程仍
不依赖 Gazebo 或 ROS。
