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

## 全部参考轨迹的验证

```bash
python3 betaflight_sitl/validate.py            # 跑 miscellaneous/datasets 下全部轨迹
python3 betaflight_sitl/validate.py <某个.csv> # 只跑指定轨迹
```

只统计 CSV 轨迹真正作为参考的区间，起飞和收尾悬停不计入。本机结果：

```text
trajectory         secs    RMSE  max err   v max   v ref   a max  sat %  z min
------------------------------------------------------------------------------
CPC16_Z1.csv       26.0   0.166    0.983   11.80   12.78    25.6    0.0   0.94
CPC25_Z1.csv       20.6   0.288    1.481   16.66   19.26    45.0    0.0   0.87
CPC33_Z1.csv       17.7   0.390    1.807   19.40   22.72    55.7    0.0   0.85
LOOP10_23.csv      40.0   0.023    0.061   10.18   10.16    42.8    0.0   1.18
LOOP14_38.csv      15.0   0.070    0.197   13.74   13.41    55.5    0.0   1.14
```

轨迹跑完之后无人机会完全静止：末 5 秒位置误差 0.0007 m、速度 0.002 m/s、
姿态振幅 0.01°。

外环 100 Hz 循环按墙钟节拍运行，因此逐次结果有几个厘米的抖动；CPC33 是时间
最优轨迹、全程接近满推力，是其中裕度最小的一条。

## 速度上限约 23 m/s

`LiftDrag` 叶素的升力正比于叶素处的合速度平方，而合速度里既有桨自转的
`cp * 转速`，也有机体的平动速度。悬停时叶素速度只有 `0.084 * 279 = 23 m/s`，
所以一旦平动速度接近这个量级，桨就会凭空产生和转速无关的升力。实测在 31 m/s
平飞、油门压到底（`rc_t = 1050`，电机在 5.5% 怠速）时，机体的垂直加速度仍是
**+1.95 m/s²** 而不是 −9.81 —— 也就是有 **1.2 g 的被动升力**，飞机在零油门下
爬升。

后果是垂直方向在高速时失去可控性。`aggressive_50mps.csv`（50 m/s，3.8 g，
轨迹本身可行）能一路跟到 43 s、误差 0.5–1.5 m，然后在减速段因为压不下高度而
发散。CPC33_Z1 峰值 22.7 m/s，正好压在这条边界上，也因此是裕度最小的一条。

要抬高这个上限，需要提高叶素处的自转速度：把 `cp` 从 0.084 增大、`area` 按
平方反比减小可以保持悬停推力不变，同时把被动升力压下去，代价是桨的阻力矩力臂
变化、偏航权限要重新标定。当前没有做这个改动。

## 动力学标定

`calibrate_plant.py` 直接讲 Betaflight ↔ Gazebo 的 UDP 协议，不经过 Betaflight，
因此测到的是**被控对象本身**：

```bash
python3 betaflight_sitl/calibrate_plant.py
```

它先用二分法搜索悬停电机指令（这一项不受气动入流影响），再逐点测量归一化推力。
每次测量都从悬停进入，取穿越零爬升率的样本，避免桨盘轴向入流污染读数。当前
覆盖层模型的结果：

```text
hover motor command = 0.4136
collective thrust   = 56.4 * motor^2  [m/s^2]，满油门 5.75:1 推重比
```

这两个数分别决定 `betaflight_udp.yaml` 的 `hover_throttle` 和
`quads/betaloop_iris.yaml` 的 `thrust_max` / `thrust_map`。换模型后必须重跑。

## 为什么要改 Gazebo 的动力学参数

Aeroloop 的 Iris 把每个桨建成两片 gz-sim `LiftDrag` 叶素，升力系数是
`cla · alpha`，而 `alpha = a0 − 爬升率 / (cp · 桨转速)`。原始 `a0 = 0.025 rad`
让叶片几乎贴着零升力角工作，于是**只要桨盘看到约 0.8 m/s 的轴向入流，推力就
归零**：实测满油门稳态爬升率上限 0.83 m/s，任何带倾角的加速都会让整机失速掉下
去。这就是原来"乱飞"的根本原因，而不是控制器参数。

`prepare_assets.py` 在生成的覆盖层里同比例放大 `a0`、缩小 `area`：零升力入流从
0.8 m/s 移到约 20 m/s，而静态推力和桨的阻力矩逐位不变。在此之上再乘一个推力
系数，把推重比从 2.9:1 提到 5.75:1 —— `LOOP14_38` 光前馈就要 3.8:1，跟踪还需要
在前馈之上留出修正余量。原始的 `~/aeroloop_gazebo` 不会被修改。

同一份覆盖层还把里程计换成插件直接发布的真值：`gz-sim` 的 `OdometryPublisher`
用位姿差分估速度，而且三维模式下发的是**欧拉角速率**而非机体角速度，只在悬停
附近成立。

## Betaflight 内环增益

`betaflight_udp.yaml` 里带了 Betaflight 的 PID，由 `run.py` 写进隔离的 EEPROM。
横滚和俯仰保持 Betaflight 默认值：这副机架在这两个轴上有约 420 rad/s² 的角
加速度，正是默认增益期望的量级。

偏航是例外。偏航力矩只有 `kappa ×` 推力差，本机架上限约 15 rad/s²，大约只有
5 寸穿越机的七分之一，而偏航惯量还大四倍。在符号正确的前提下用默认增益，悬停
时偏航仍会以几 rad/s 振荡。把偏航增益按缺失的权限同比例调小，这个轴就能在一
分钟内稳到 0.06° 以内，另外两个已经正确的轴不动。

**这组增益不能替代桥里的偏航符号。** 桥对三个轴一视同仁地直接传递角速度符号：
Gazebo 模型的 IMU 带 180° 横滚安装，Betaflight 收到的陀螺已经在它自己的杆量
坐标系里，桥再对偏航取反就会把速率环变成正反馈。这个故障很容易被
`yaw_motors_reversed` 掩盖 —— 翻混控能恢复指令方向的符号，却翻不了陀螺反馈，
于是飞机能飞，但偏航速率会缓慢卷绕到一个与指令反号的恒定转速（实测悬停 40 s
后锁定在 −0.175 rad/s，而 MPC 一直在指令 +0.18）。降低增益只会把卷绕时间从
秒级拉长到几十秒，看起来像"慢慢偏航"。

## 隔离和兼容处理

启动器读取 `~/betaloop/config.txt`，但不会改动 Betaloop、Aeroloop 或原始
Betaflight EEPROM。每次运行都会在 `build/betaflight_sitl/runtime` 中：

1. 复制 SITL ELF 和 `eeprom.bin`；
2. 只在副本中写入 `ARM = AUX1 1700–2100`、AETR 映射、ACTUAL 速率曲线、
   死区、油门曲线和 PID —— 这些值全部来自 `betaflight_udp.yaml`，改配置文件即可，
   不需要手工编辑 EEPROM；
3. 写完之后再读回校验，任何一项不符或 CLI 响应不可解析都会拒绝解锁；
4. 生成 Aeroloop 模型覆盖层（电机顺序、桨叶气动、真值里程计）；
5. 把物理步长设为 2 ms；
6. 加载项目内的兼容插件，按 Betaflight 2026.6 的 144 字节格式收发 FDM 包，
   并避免物理线程长时间等待 UDP。

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
`p=[0.000 0.000 1.19]`、`thrust≈9.8`、`rotor_w≈±246 rad/s`、油门 PWM 约 1417。
状态超过 250 ms 未更新时程序会立即解除 AUX1 并退出。

`rotor_w` 的正负号来自 CW/CCW 方向，不是故障。

## 常用调整

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
