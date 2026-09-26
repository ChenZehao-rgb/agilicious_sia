# 2026-09-24 实机诊断包：配置补充与下一步

本文记录 15:00 手持行走包的历史结论。后续 IST8310 校准后飞行、航向确认和当前阻塞项见
[16:30 新包分析](../hardware_flight_20260924_163013/README.md)，当前参数以 `hardware.yaml` 为准。

本轮可补入真实质量、FC rates 和 GPS 高度基准，并修复一个 FC 版本回读兼容问题。
**下一步是拆桨地面诊断，尚不具备影子控制或 AUTO 悬停的完整条件。**

## 来源与实验条件

- 原始包：`bags/hardware_20260924_150053_980611/`，启动方式为
  `./agi_ros2/scripts/launch.sh mode:=hardware diagnostic_only:=true`。
- 记录约 224.014 s，共 375359 条消息。数值见 [summary.json](summary.json)，
  可复算代码见 [analyze.py](analyze.py) 与 [analysis.ipynb](analysis.ipynb)。
- 操作者补充：含电池/负载总质量 **734 g**；GPS 模组 **M1025**；已做磁力计校准；
  录包是**手持绕操场行走一圈**。GPS 型号是操作者信息，消息本身没有型号字段。
- 操作者提供的 MS5611 是气压计，不能据此确定磁力计型号，见
  [TE 官方产品资料](https://www.te.com/en/product-MS561101BA03-50.html)。
  原始 MSP sensor mask 为 `0x2f`，确实包含 MAG；还需 FC `status` 给出具体芯片与运行状态。
- 移动实验中的位置跨度、加速度和角速度包含真实运动，不能作为静态漂移、IMU 零偏或测量噪声标定。
  没有 RTK/测量真值，接收机报告的 accuracy 也不等于已实测的绝对误差。

## 已写入 hardware.yaml 的数据

| 参数 | 值 | 依据与限制 |
|---|---|---|
| `pilot.quadrotor.mass` | `0.734` kg | 操作者提供的起飞总质量；换电池/负载后重新称重 |
| `bridge.center_rate_deg_s` | `[70,70,70]` | MSP 111 ACTUAL 曲线回读 |
| `bridge.max_rate_deg_s` | `[670,670,670]` | MSP 111 ACTUAL 曲线回读；不是外环允许角速度 |
| `bridge.expo_percent` | `[0,0,0]` | 回读与原值一致 |
| `bridge.deadband/yaw_deadband/min_check` | `0 / 0 / 1050` | MSP 125/44 回读与原值一致 |
| `mavlink.altitude_source` | `msl` | GPS_RAW_INT.alt 与配套固件使用海拔基准；包诊断保留有限 MSL |
| `evidence.expected_pid_profile/expected_rate_profile` | `0 / 0` | 原始 MSP 150 回读与原值一致 |

配置入口仍是 [hardware.yaml](../../config/hardware.yaml)，没有增加第三份运行配置。
`shadow_only: true` 保持默认。`omega_max`、推力界限/推力表、精度门限和围栏仍待测量/定义。
734 g 对应静态悬停所需总推力约 **7.198 N**、每电机等效约 **1.7995 N**；
这只是 `mg`，不是实测 `thrust_max`、允许推力范围或 RC 油门值，不能用于代填推力表。

MSL 的依据是 [MAVLink GPS_RAW_INT 定义](https://mavlink.io/en/messages/common.html#GPS_RAW_INT)
和本机 `/home/sia/betaflight/src/main/io/gps.c` 的 UBX `hMSL` 转换、
`src/main/telemetry/mavlink.c` 的 `GPS_RAW_INT.alt` 发送代码。
这里选择 GPS 海拔，不是使用 MS5611 气压高度，也不是把本轮海拔写成固定原点。
`/sensors/gps/fix.altitude` 仍可能为 NaN：NavSatFix 高度是椭球高；
当前 MSL 导航应检查 `/sensors/navigation.altitude` 和 `altitude_reference`。
包里 `ellipsoid_valid=false` 受当时 `altitude_source=unknown` 影响，不能证明接收机不支持椭球高。

## 这份包实际说明了什么

以下频率用各流首末消息的独立采样时间计算；P95 为 numpy 线性插值分位数。

| 观测 | 结果 | 解释 |
|---|---|---|
| IMU | 110573 条，约 499.477 Hz；最大采样间隔 3.075 ms | 原始高频输入存在，不等于融合已初始化 |
| GPS 定位 | 2180 个观测，约 9.852 Hz；全部 fix_type=3 | 普通 3D GNSS，不是 RTK fixed |
| GPS 最大采样间隔 | 202.294 ms | 10 Hz 中存在约 200 ms 的间隔，不能据此证明串口丢包率 |
| GNSS 水平 accuracy | 中位数 1.969 m；P95 3.0773 m；最大 3.670 m | 接收机报告，不是与真值对比的误差 |
| GNSS 垂直 accuracy | 中位数 3.466 m；P95 4.34805 m；最大 4.999 m | 不能据此支持亚米级低空高度精度 |
| GNSS 速度 accuracy | 中位数 0.813 m/s；P95 1.5861 m/s；最大 2.267 m/s | 不是机体实际速度，也不是 `origin_max_speed` |
| MSL 高度诊断 | 42.41～58.35 m（221 个低频快照） | 行走时高度变化不能全部归为漂移；不是完整 10 Hz 高度序列 |
| FC 航向 | 0/2180 有效 | 即使将 ROS `heading_confirmed` 改为 true，仍无法通过有效航向检查 |
| 原点 / 本地导航 | 都是 0 条 | 未形成可供融合的本地 ENU 观测 |
| 融合初始化 | 0/110579 | 不能从零占位 position/covariance 计算静态漂移或 EKF 门限 |
| TIMESYNC RTT | 中位数 1.956 ms，P95 2.080 ms，最大 3.081 ms | 近似时钟对齐，不是 GPS 解算/硬件 PPS 的完整延迟 |

`/sensors/navigation` 的 4360 条消息中，2180 条是配对观测，另外 2180 条是航向无效触发的撤销事件。
不能把它当成 20 Hz GPS。配对观测全部 `clock_aligned=true`，但 `heading_valid=false`，
当时的 `altitude_reference=unknown` 也令高度为 NaN。

221 个 MAVLink 诊断快照中，CRC/错误源/重复/迟到/同步拒绝/发送错误计数均为 0；
MSP 没有录到 timeout/error/late 事件，也没有 code 200 发送事件。
这只描述包内证据，不是完整 UART 抓包或物理回退时延验收。
IMU 的 bag 接收年龄 P95 约 3.684 ms，GPS fix 约 95.132 ms；
这些值包含消息传输和录制延时，不应等同于控制回调实时看到的年龄。

### GPS 参数为何没有直接填成“最大观测值 + 余量”

`navigation.horizontal_stddev/vertical_stddev/velocity_stddev` 是观测协方差的下限；
适配器逐条使用 `max(接收机accuracy, 下限)^2`，本轮已经有正的 accuracy，无需伪造固定精度。
`max_*_accuracy` 则是试验允许的质量门限，应由可接受位置/速度误差和场地空间决定。
为了让这份行走包全通过而填写 4 m / 5 m / 2.5 m/s，不能证明悬停适用。

因此保留三项 `navigation.max_*_accuracy=0` 和四项 `fusion.max_*_stddev=0`，不授予飞行就绪。
这不阻止诊断模式记录原始数据；航向/高度/实体未解锁授权满足后，可以建立原点并观察融合过程，
即使 readiness 因质量门限未配置仍为 false。取得静止数据、定义试验误差容许范围后，
再分别填写观测精度门限与 EKF 后验标准差门限，不能把两者互相代替。

## 四个必须先解决的阻塞项

### 1. 新版 FC_VERSION 被旧解析规则误拒绝：本次已修复软件

包中 MSP 1 payload 为 `00 01 30`，即 API **1.48**；MSP 2 为 `BTFL`。
MSP 3 为 `1a060108323032362e362e31`：前三字节版本 26.6.1，后接长度 8 与字符串 `2026.6.1`。
配套 FC 源码明确发送此长度前缀字符串。旧代码仅接受总长度 3，因而错误报告
`Unsupported FC/API: require BTFL API 1.48`。

[shadow_support.py](../../scripts/shadow_support.py) 已兼容该扩展结构并保留旧三字节格式，
测试覆盖有效回复与损坏长度/字符串；API 1.48 和其他授权条件继续严格检查。
因此无需仅为这条误报降级或重刷飞控。诊断消息里的 `pid_profile=255`、`rc_link=false` 等
是在配置未通过时的默认值，实际 profile 应从原始 MSP 150 判读。

### 2. Override 与实体模式尚未配置

| 项目 | 包中回读 | 当前 ROS 策略要求 |
|---|---|---|
| `msp_override_channels_mask` | `0` | `15`（只覆盖 AETR） |
| `msp_override_failsafe` | `OFF` | `OFF` |
| `msp_override_timeout_ms` | `300` | `50` ms |
| MSP OVERRIDE / BOX 50 | BOXIDS 中不存在 | 必须存在且有单一直接 OR 范围 |
| ARM | AUX5，`[1875,2100)` | 默认 AUX1，`[1700,2100)` |
| AUTO / KILL | 无对应 BOX 50 / BOX 27 范围 | 默认 AUX2 / AUX3，`[1700,2100)` |

本地 FC 在 mask 非零时才公布 BOX 50，因此本包缺 BOX 50 可以由 mask=0 解释。
其他活动范围是 ANGLE/AUX6 `[1875,2100)`、BOX3/AUX3 `[1300,2100)`、BOX11/AUX3 `[1900,2100)`；
不要直接覆盖这些已有用途。先备份 `diff all`，根据真实遥控开关在 Configurator 中明确分配
ARM/MSP OVERRIDE/FAILSAFE 三个不同 AUX，保证与 `hardware.yaml` 的 `evidence` 一致。
若保留 ARM AUX5，应设置 `arm_aux: 4`，再确定另外两个 AUX 及统一范围；本次没有擅自重排这些开关。
当前解码要求三种模式使用相同 `aux_low/aux_high`，且没有链接/AND/重复范围。

拆桨、停止 ROS 串口占用后，由操作者在 CLI 执行并回读以下设置：

```text
set msp_override_channels_mask = 15
set msp_override_failsafe = OFF
set msp_override_timeout_ms = 50
save
```

保存后重新连接读取三项，退出 CLI 再启动 diagnostic 验证。以上命令未由本次分析执行。

### 3. 接收机在包中处于 RX_FAILSAFE

5533 个原始 STATUS_EX 都有 RX_FAILSAFE 位；常见 flags 是 `0x04010004`，
还含 MSP 与 ALTHOLD 禁止解锁位；17 帧另有 ANGLE 位。全程 modes 为 0。
这直接说明录包时没有满足当前程序要求的正常接收机授权，不仅是解码默认值问题。
下一次测试需开启已绑定遥控器、确认 FC 实时收到各通道，ARM/AUTO 保持低，退出 CLI。
融合初始化依赖新鲜可信的未解锁实体授权，不能用硬填 `rc_link=true` 代替。

### 4. 校准完成不等于当前 FC 航向有效

先保存以下 CLI 只读输出，确认真实磁力计芯片与设置：

```text
version
status
get mag_hardware
get align_mag
get mag_align
get mag_declination
get trust_mag
```

MS5611 是气压计，实际磁力计型号以 `status` 和硬件资料为准。
当前配套固件发送有效航向需要 MAG 存在、`trust_mag` 声明、校准过程结束、
磁采样和磁航向更新新鲜有效；这份 bag 没有保存能区分这些失败原因的全部信息，
所以不能断言只是 `trust_mag=OFF`。

核对安装方向和机头朝向，完成远离磁干扰的方向验证，确认磁偏角是否由 FC 处理。
满足这些条件后再设置 FC 的 `trust_mag`，并填写 ROS 的
`heading_confirmed`、`heading_correction_rad`、`fc_declination_applied`。
`fc_declination_applied` 只是记录，不会自动修正；ENU yaw 从东轴逆时针为正，
修正量加在 ENU yaw 上。不要把行走方向/COG 当成机头方向，也不要重复补偿磁偏角。

## 下一轮地面诊断步骤

1. 在实际运行的 CM5 工作副本同步这次改动，执行 `./agi_ros2/scripts/build.sh`。
   bag 的日志显示原配置来自 `/home/ubuntu/agilicious_sia/agi_ros2/config/hardware.yaml`；
   本机 `/home/sia/...` 的修改不会自动部署到 CM5。Python 节点改动也需重新安装。
2. 拆桨，完成上面的 FC 回读/模式/接收机/航向检查。所有节点退出 CLI 后才使用串口。
3. 在开阔位置把飞机放稳，ARM/AUTO 保持低，开始录包后至少连续静止 60 s，
   覆盖至少 3 s/30 个独立导航样本及 3 s/1000 个静止 IMU 样本的初始化要求：

   ```bash
   ./agi_ros2/scripts/launch.sh mode:=hardware diagnostic_only:=true
   ```

4. 检查 `/sensors/fc_heading.valid=true`、`/sensors/navigation` 有有限 MSL 高度与速度、
   `/msp/decoded_state.config_verified=true`、`receiver_valid=true`、`rc_link=true`、`armed=false`，
   再确认 `/navigation/origin` 出现和 `/fused_state.initialized=true`。
   GNSS `fix_type=3` 和 `clock_aligned=true` 本轮已经有，仍应检查新包是否持续成立。
   diagnostic 没有输出节点，缺 `output_status`、推力就绪及整链 health 并不表示录包失败。
5. 初始化后再做缓慢平移与定向旋转，每段记下动作和时间，最后静止 60 s。
   第一段用于静止噪声/后验协方差与漂移检查；后续用于方向、尺度和创新检查。
6. 结合新静止包与场地/误差容许范围填写导航和融合门限；补齐 `omega_max`、每电机等效
   `thrust_min/max` 后才能启动默认 shadow。实测 RC—电压—总推力表、围栏、
   人工接管和 50 ms FC 回退验收全部完成后，再考虑输出模式。

## 复算与软件验证

分析直接读取 MCAP 内嵌消息定义，不回放到 ROS，不连接串口，不修改 bag。
本机 Humble 的 rosbag2 不支持该包 metadata v9，且没有 MCAP 插件；使用独立 Python 读取器避免修改原文件。
在已有分析环境安装 `mcap`、`mcap-ros2-support`、`numpy`、`pyyaml` 后，从仓库根执行：

```bash
python3 agi_ros2/analysis/hardware_diagnostic_20260924/analyze.py \
  bags/hardware_20260924_150053_980611 \
  --output /tmp/hardware_diagnostic_summary.json
```

本次已完成：

- 全包读取与存储 CRC 校验（文件中存在的 chunk/data CRC），375359 条及 27 个话题的计数与 metadata 一致。
- `./agi_ros2/scripts/build.sh` 成功；`test_runtime_config.py` 16 项、
  `test_shadow_support.py` 17 项、`test_hardware_pipeline.py` 7 项全部通过。
- 用修复后的解码器离线重读真实 MSP 事件，版本不再误报；仍正确拒绝
  `Override mask must be 15 (AETR only)`，回读超时仍为 300 ms。
- companion notebook 的全部代码单元已执行，复算结果与保存的 summary 完全一致。
- 差异空白检查通过；本次没有修改 C++，无需 C++ 格式化。

上述测试在开发机执行，hardware pipeline 使用伪串口。没有部署到 CM5、写实体 FC 配置或执行飞行；
软件测试不替代真实传感器质量、目标机运行时延和飞控回退验收。
