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

> 注意：上表是**转速改造之前**（物理步长 2 ms、可达转速 598.6 rad/s）测得的。
> 改造后只重新验证了 `aggressive_50mps.csv`，这五条尚未复测。

## 速度包线

`LiftDrag` 叶素的升力正比于叶素处合速度的平方，而合速度里既有桨自转的
`cp * 转速`，也有机体平动速度。二者之比就是前进比 `mu`，真实旋翼的常规工作区是
`mu < 0.3~0.5`。

Aeroloop 原始模型悬停时叶素线速度只有 21 m/s，平飞 20 m/s 就到 `mu ≈ 1`：实测
在 31 m/s、油门压到底（电机 5.5% 怠速）时垂直加速度仍是 **+1.95 m/s²**，即有
1.2 g 的被动升力，飞机在零油门下爬升，垂直方向彻底失控。

覆盖层把 `maxRpm` 提到 3352（可达转速 2394 rad/s），悬停叶素线速度变成
**84 m/s**，与真实 5 寸桨的 60 m/s 同量级，50 m/s 时 `mu` 只有 0.6。代价是桨叶
方位角的采样：`LiftDrag` 每个物理步只解析一次方位角，2394 rad/s 下需要 0.5 ms
步长才能维持和原来相同的 68.6°/步。

实测 `aggressive_50mps.csv`（55.9 s、50 m/s、3.8 g），**开着 Gazebo GUI**：

```text
RMSE 0.100 m   最大误差 0.219 m   速度峰值 50.1 m/s（参考 50.0）
高度 1.06–1.29 m（参考 1.19）     油门无饱和采样点
指令推力均值 24.6 m/s^2（参考 24.3）
```

改造前这条轨迹在 43 s 的减速段因为压不下高度而发散触地。

## 控制回路走仿真时间，而不是墙钟

整个适配器的时钟取自里程计消息头里的仿真时间戳，包括 Pilot 自己给起飞多项式、
悬停和采样轨迹打的时间戳（`Pilot` 用同一个时钟函数构造）。桥的看门狗仍然走墙钟
—— 那是真实的安全定时器，不属于被仿真的动力学。

这一点是必需的：Gazebo 的实时倍率并不恒为 1，光是开着 GUI 就会间歇性掉到 0.3。
如果参考按墙钟推进，实时倍率一掉参考就跑赢飞机，结果是**丢轨迹**而不是"仿真慢
一点"。同一条 50 m/s 轨迹在墙钟计时下 RMSE 是 3.5 m 并在 GUI 下直接坠地，改成
仿真时间后是 0.100 m。

副作用是实时倍率不再是硬约束：想用更小的步长（例如 0.1 ms，实测 0.84 倍）只是
跑得比真实时间慢，结果依然正确。`--duration` 现在也是仿真秒。

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
hover motor command = 0.4040
collective thrust   = 56.6 * motor^2  [m/s^2]，满油门 5.75:1 推重比
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
横滚和俯仰保持 Betaflight 默认值。

偏航是例外，原因是建模伪迹而非机架本身。本模型的偏航力矩主要来自桨关节伺服对抗
关节阻尼的反作用，它随**转速**而不是随推力变化。为修正前进比把转速提高 4 倍后，
推力不变而偏航权限变成 6.4 倍：实测 0.1 电机差动下偏航角加速度 20.5 rad/s²，改造
前是 3.2。Betaflight 默认增益在这个权限下悬停就会以几 rad/s 振荡，所以把偏航增益
同样按 6.4 缩小（45/80/120 → 2/4/6），悬停偏航稳定在 90.00° ± 0.04°、
`w_z ≈ 0.015 rad/s`。

同样的道理，`kappa` 也不是从 `cda` 推出来的，而是实测值，且在量程内只在两倍范围
内近似恒定 —— 换动力学参数后必须重测。

更干净的做法是缩小桨关节阻尼，让反作用力矩回到物理量级并使 `kappa` 恒定；那会改变
伺服的稳态折损系数，需要把整个动力学重新辨识一遍，当前没有做。

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
5. 把物理步长设为 0.5 ms，并把喂给 Betaflight 的 FDM 速率**独立**固定在 500 Hz；
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
`p=[0.000 0.000 1.19]`、`thrust≈9.8`、`rotor_w≈±246 rad/s`、油门 PWM 约 1417。
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
