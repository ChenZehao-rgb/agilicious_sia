# 2026-09-24 16:30：IST8310 校准后的飞行包

**校准后的磁航向已恢复有效，GPS 报告精度也优于上一包；但 ROS 导航/融合仍未初始化。
本轮还发现约 36.149 s 的系统时钟前跳，触发 MSP 证据会话失效。**
当前结果支持继续完善地面初始化与影子测试准备，不能证明 ROS AUTO 悬停已就绪。

后续已完成[原生 EKF 条件性离线实验](offline_ekf/README.md)：提供轨迹、更新残差、CSV 和 notebook。
末段仍有连续 GPS 拒绝，不能判定定位正确；[时钟与 PID profile 补充说明](offline_ekf/clock_and_profiles.md)
包含具体处理步骤。本文下方的“复算与验证”记录的是最初传感器/MSP 分析，离线程序的构建与验证另见该实验文档。

## 数据与现场信息

- 包：`bags/hardware_20260924_163013_022677/`，154311 条、27 个话题。
  完整读取并校验文件中存储的 CRC，逐话题消息数与 metadata 一致。
- bag 时间起点：2026-09-24 16:30:14.307988，UTC+8；墙钟跨度 130.123 s。
  其中包含一次约 36.149 s 的时钟前跳，不能直接当作真实实验时长。
- 用户确认：GPS M1025、磁力计 **IST8310**，本次为校准后的飞行；含电池/负载总质量仍按此前提供的 0.734 kg。
- 用户进一步确认：已核对真北一致性，**飞控已补偿磁偏角，ROS 修正量为 0**。
  这项依据是现场声明；bag 没有保存 `mag_declination` 数值，也没有真北测量参考。
- [summary.json](summary.json) 保存传感器统计与时间证据；[msp_timeline.json](msp_timeline.json)
  保存原始 MSP 解锁、模式、profile 与时钟对照；[analysis.ipynb](analysis.ipynb) 为可执行复算入口。

## 相比上一包的变化

| 项目 | 15:00 手持行走包 | 16:30 校准后飞行包 |
|---|---:|---:|
| GPS 配对观测数 | 2180 | 883 |
| 有效航向 | 0/2180 | **883/883（100%）** |
| GNSS fix_type | 全部 3 | 全部 3 |
| 水平 accuracy P95 | 3.0773 m | **1.2520 m** |
| 垂直 accuracy P95 | 4.3481 m | **2.1049 m** |
| 速度 accuracy P95 | 1.5861 m/s | **0.5278 m/s** |
| Override mask / failsafe / timeout | 0 / OFF / 300 ms | **15 / OFF / 50 ms** |
| 原始 STATUS 中 RX_FAILSAFE | 全部存在 | **2282 帧均不存在，全部 arming-disable flags 为 0** |
| 导航原点 / LocalNavigation | 0 / 0 | **0 / 0** |
| 融合 initialized | 全部 false | **45386 条全部 false** |

accuracy 是接收机报告的估计量，不是对比真值测得的误差。两包的地点条件、运动和采样时间不同，
不能把 GPS accuracy 的改善归因于磁力计校准。原始 GPS 仍是普通 3D GNSS，不能标记为 RTK fixed。

新包的水平/垂直/速度 accuracy 中位数分别是 **1.206 m / 1.836 m / 0.368 m/s**，
最大值为 **1.261 m / 2.139 m / 0.633 m/s**。这些值可用于后续定义质量要求，
但不能直接把最大值加余量就作为允许实机悬停的门限。

883 个航向观测的 ENU yaw 为 116.8°～144.9°；它是机体朝向，不是飞行方向。
没有独立朝向真值和完整安装标定数据，无法从这份包计算磁偏角残差或确定 `heading_stddev`。
本次不从飞行中的航向变化估计磁噪声，也不从动态 IMU 估计静止零偏。

## 为什么航向有效后仍没有融合

1. **实际录包运行的高度基准仍为 `unknown`**：883 个导航观测的 altitude 全为 NaN。
   MAVLink 诊断中的 MSL 有限，范围 48.23～65.76 m；这是 91 个低频快照的 GPS 海拔，
   不能把其跨度直接解释成离地飞行高度。当前本机配置已设为 `msl`，需同步到 CM5。
2. **GNSS adapter 仍报告航向未确认**：93 次状态为
   `True-north heading source/correction unconfirmed or invalid`。
   同时观测航向全部 valid 且有限，按当前适配器逻辑，运行时的 `heading_confirmed` 未开启是对应阻塞。
   新的现场确认是在本轮分析过程中补充的，不会反向改变这份 bag。
3. **旧 FC_VERSION 解析问题仍出现在运行日志**：API 实际 1.48、BTFL、版本字符串 2026.6.1，
   都与上次合法扩展格式相同。时钟跳变前仍有 5070 条旧版
   `Unsupported FC/API: require BTFL API 1.48`。说明采集时所用安装代码仍存在该错误检查。
4. **只有约 2.6 秒的已记录原始遥控未解锁阶段，然后进入 ARM**。
   当前初始化要求新鲜未解锁实体授权及持续静止 IMU/导航样本；本包不提供足够的地面初始化验证。
   即使 GPS 和航向有效，也不会在已经飞行时凭空补出静止初始化。

包里日志指向 `/home/ubuntu/agilicious_sia/agi_ros2/config/hardware.yaml`，MSP 节点是 `mode: monitor`。
没有控制计算、输出状态或 code 200 发送证据；这份包证明的是实际 FC 飞行中的传感器读取，
不证明 ROS 控制器已接管。融合位置/协方差没有有效初始化，不能拿其中零占位值计算漂移或调 EKF 门限。

## 系统时钟跳变：不是 36 秒 UART 停流

最直接的证据来自同一会话、同一 MSP STATUS 请求的 tx/rx：

| 项目 | 发送 | 接收 |
|---|---:|---:|
| 相对 bag 起点的墙钟秒 | 44.525483492 | 80.684191275 |
| 消息事件单调时钟秒 | 73.240473379 | 73.250305972 |
| 请求单调时间（两者相同） | 73.240451379 | 73.240451379 |

消息 header 墙钟增量与事件单调增量之差为 **36.148773 s**，该请求报告的实际响应耗时仅 **9.852 ms**。
融合节点同一处的 bag 时间增加 36.151 s，而发布单调时间只增加约 **2.126 ms**，相互印证。
大约在墙钟 **16:30:58.83 → 16:31:34.99** 发生跳变；无法单靠 bag 确定具体校时服务或手工操作。

随后：

- +80.679 s 左右先报告 `Configuration readback stale`。
- +80.698 s 左右开始报告 `MSP transport/request failure; restart required`，持续到包结束。
- `/msp/events` 自身没有 timeout/error/late，errors 计数为 0；故不能把证据节点的 failure 当作串口硬件故障。
  证据节点检测 ROS 请求时间与单调时间不一致后，使整个会话失效，这是保护策略。
- MAVLink navigation 的源会话从 `27.884770` 变为 `73.245671`，产生一条无效撤销事件，再重新同步。
  抵扣墙钟前跳后，IMU 最大有效观测间隔约 **0.409 s**、GPS 约 **0.501 s**，仍是实质重同步间隔。

正常采样间隔中位数为 IMU **2.001 ms**、GPS **100.155 ms**，对应约 500 Hz / 10 Hz 的通常节奏。
直接按未修正 bag 跨度得到的 356 Hz / 6.93 Hz 包含了时钟跳变，不应据此判断串口设置错误。
IMU/GPS 的 bag 接收年龄 P95 约 **4.047 ms / 102.822 ms**，含 DDS 和录制延迟，
不是控制回调时延；TIMESYNC RTT P95 约 **2.014 ms**，也不能代替完整传感器延迟测量。

下一轮启动前先在目标机完成时间同步，运行期间避免系统时钟 step，发生跳变后重启整组 diagnostic。
可在 CM5 检查下面的只读信息；服务名以目标机实际安装为准：

```bash
timedatectl status
# 查询本包对应的历史启动；仅当旧日志保留时有效：
journalctl -b 3f96d065-8e18-4ffe-886f-9ce165fdec04 -u chrony -u chronyd -u systemd-timesyncd -o short-monotonic
# 仅在安装 chrony 时：
chronyc tracking
chronyc sources -v
```

使用单调格式日志便于在系统时钟跳变后追查先后关系。不要为了绕过本次失效放宽程序的年龄门限。
若目标机使用 chrony，应核对启动时的 `makestep` 与应用启动顺序；chrony 官方说明建议把 step
限制在依赖连续时间的程序启动前完成。这里没有确认目标机使用 chrony，也没有自动改校时配置。
见 [chrony 官方 makestep 说明](https://chrony-project.org/doc/4.8/chrony.conf.html#makestep)。

## 原始飞控状态与当前配置仍有差异

原始 BOX0 显示解锁开始于 bag +5.256 s、结束于 +127.117 s。
用单调时钟计算，该解锁区间约 **85.712 s**；不能把它当成精确离地飞行时长。
全包原始 PID profile 为 **1**、rate profile 为 **0**；本机配置期望 PID profile 仍为 0。
选择下一次固定使用的 profile 后让两侧一致：使用本次 profile 1 则设置 `expected_pid_profile: 1`，
若计划使用 profile 0，则在 FC 切回并重新核对相应参数。不能用 config 未通过时的 decoded profile=255 判读实际 profile。

| 模式 | 新包实际范围 | 当前 ROS 配置/约束 |
|---|---|---|
| ARM | AUX1 `[1875,2100)` | AUX1 `[1700,2100)`，边界仍不一致 |
| AUTO / MSP OVERRIDE（BOX50） | BOXIDS 已存在，但没有激活范围 | 需要单一直接 OR 范围；当前预期 AUX2 |
| KILL / FAILSAFE（BOX27） | AUX2 和 AUX5 各一个 `[1300,1700)` | 当前预期 AUX3 `[1700,2100)`，不接受重复范围 |
| ANGLE（BOX1） | AUX4 `[1875,2100)`，记录期间一直有效 | 当前手动飞行可用；AUTO high 时会与外环 rates 控制冲突 |

因此即便部署上次的版本解析修复、填写航向，也还会因模式范围/profile 不一致而拒绝授权。
修正遥控模式时应结合实体开关确定用途，避免覆盖已有 AUX3 的 BOX3/BOX11 等配置。
本次只读取这些设置，没有写入 FC 或重排遥控通道。

## 本次已补充的配置与后续顺序

已更新 [hardware.yaml](../../config/hardware.yaml)：

```yaml
navigation:
  heading_confirmed: true
  heading_correction_rad: 0.0
  fc_declination_applied: true
```

依据是用户对真北一致性和 FC 磁偏角补偿的明确确认；ROS 不再额外补偿。
同时记录磁力计型号 IST8310，保留已有质量 0.734 kg、ACTUAL 70/670 °/s、MSL 高度选择。
型号写在注释与分析记录中，运行配置没有磁力计型号参数，芯片驱动由 FC 处理。

下一次按下面顺序推进：

1. 把本工作副本的 `hardware.yaml` 和 FC_VERSION 解析修复同步到 CM5 的实际工作副本，
   在 CM5 重新运行 `./agi_ros2/scripts/build.sh`；核对 launch 打印的配置路径。
   修改源码配置后只重启不足以替换尚未重新安装的 Python 节点。
2. 处理目标机时钟前跳，固定 PID profile 并使三种授权模式的 AUX/范围与配置一致。
   本轮 mask=15、failsafe=OFF、timeout=50 已通过原始回读，不必再把它们作为待修项。
3. 拆桨、遥控器正常连接、ARM/AUTO low，放稳静止至少 60 s 后再移动。
   使用 `./agi_ros2/scripts/launch.sh mode:=hardware diagnostic_only:=true`，先确认
   `/sensors/navigation.altitude_reference=msl`、高度有限、航向有效、`config_verified=true`、
   `receiver_valid=true`、`rc_link=true`、`armed=false`，再确认原点出现和 `initialized=true`。
4. 有了初始化后的静止/移动数据，再结合试验误差容许范围填写 GPS accuracy 与 EKF 后验标准差门限。
   目前仍保留 0，不授予导航 readiness；这不阻止已经满足其他条件的诊断初始化。
5. `omega_max`、每电机等效推力范围、RC—电压—整机总推力表和围栏仍需各自依据。
   本次没有记录可辨识推力模型的输入/输出配对，不能从这份飞行读取包代填这些模型数据。

## 复算与验证

依赖 `mcap`、`mcap-ros2-support`、`numpy`、`pyyaml`；不需要 ROS 实时回放，也不连接串口。
主分析复用上一轮的原始统计函数，但明确替换了实验条件说明，避免把飞行包写成手持行走。

```bash
python3 agi_ros2/analysis/hardware_flight_20260924_163013/analyze.py \
  bags/hardware_20260924_163013_022677 --output /tmp/flight_summary.json
python3 agi_ros2/analysis/hardware_flight_20260924_163013/msp_analysis.py \
  bags/hardware_20260924_163013_022677 --output /tmp/flight_msp_timeline.json
```

本次已完成：全包计数/存储 CRC 校验通过；复算 notebook 的全部代码单元执行成功，
两份结果与保存的 JSON 完全一致；`test_runtime_config.py` 的 16 项测试通过；
分析脚本语法、JSON、文档本地链接、配置声明及差异空白检查通过。
最初的传感器/MSP 分析没有修改节点运行代码或 C++；后续离线实验新增了独立 C++ runner 并已在开发机构建验证，
详见上方离线实验链接。CM5 仍需同步并安装上一轮的节点修复。
没有更改传感器原始记录、部署到 CM5 或发送实体控制命令。
