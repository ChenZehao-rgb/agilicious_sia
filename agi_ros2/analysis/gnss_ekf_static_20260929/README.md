# 2026-09-29 静止硬件 bag：融合停止与 GNSS 漂移

**本次融合停止由 ROS 系统时间向前跳变约 33.3987 s 触发，随后 MSP 证据失败锁存，阻止重新初始化。不是 NIS 门限过严。**
用户说明全程静止；约 17 颗卫星为用户现场观察。bag 没有独立位置真值，也未在记录的导航/诊断 schema 中保留卫星数字段。

源 bag：`bags/hardware_20260929_112215_827325`。全部 286446 条消息计数与 metadata 一致，遍历检查了存储 CRC，按 MCAP 内嵌 schema 解码。原始 bag 未修改。
墙钟记录跨度为 192.859754 s，但其中包含约 33.399 s 的前跳；fused_state 发布单调时钟首末跨度为 159.377009 s。
分析不能把墙钟缺口解释为传感器真实停流 33 秒。

| 事件 | bag 接收时间相对开始 / s | 证据 |
|---|---:|---|
| 第一条有效局部导航 | 5.611 | observation_valid=true |
| 开始有效融合 | 8.614 | initialized=true |
| 最后一个跳时前融合记录 | 42.821 | initialized=true |
| 系统时间前跳后 | 76.227 | 导航撤销、更换 source_session，EKF reset_counter 变为 3 |
| MSP 证据锁存 | 76.230 | MSP transport/request failure; restart required |
| MAVLink 诊断明确记录原因 | 76.489 | ROS clock jumped; resynchronizing |
| 有效导航恢复 | 76.633 | 距撤销约 0.406 s，之后 1112 条局部导航均有效 |
| 后续 | 至 bag 末尾 | authority.rc_link=false，EKF 无法重新初始化 |

融合有效采集时间是 8.610408–42.819749 s，约 34.209 s，共 17085 条状态。跳时后收到的一条最后有效状态携带跳时前的采集时间，不能据接收时间误认为持续融合到 76 s。

**为何会失效、为何没有恢复**

- 同一 MSP code 105 请求/应答的墙钟耗时为 33.407422 s，单调耗时为 0.008745 s，差为 **33.398676986 s**。
  独立用稳定阶段的两时钟偏移中位数相减得到 33.398678486 s。
- [MAVLink 时钟检测](../../src/mavlink_sensor_node.cpp#L207) 发现 ROS 与单调时钟偏移改变超过 50 ms，清空时钟同步、切换 source_session 并发送导航撤销。
- [融合节点](../../src/state_fusion_node.cpp#L152) 因导航源会话改变且导航失效而 reset；后续 IMU 时间戳跨越大间隔又增加一次 reset。
- [MSP 证据处理](../../scripts/msp_evidence.py#L81) 发现该请求跨时钟跳变，内部转换成 transport_error；
  [证据状态机](../../scripts/shadow_support.py#L141) 在同一会话内锁存失败。后续正常回包不会解除它。
- 原始 `/msp/events` 的 20732 条事件仅为 10366 tx + 10366 rx；没有 error/timeout/transport_error 事件，errors 字段为零。
  后续 RC/STATUS 继续正常回包。因此此处 `rc_link=false`、`kill=true` 是证据层默认保护状态，不能认定遥控器 RF 断链、实体 KILL 动作或串口损坏。
- [重新初始化](../../src/state_fusion_node.cpp#L253) 要求新鲜授权、rc_link=true、未解锁，再采集连续 3 秒静止 IMU。全包输出 armed=false，但 rc_link 条件持续不满足，GNSS 恢复后仍无法重新初始化。
- 失效后的融合位置零值是未初始化的占位，不是测得的位置回到了原点。

触发系统校时的进程没有记录在 bag；可能是自动校时或人工改时，但不能从本包唯一确定。需查看当时 CM5 的系统时间服务日志，例如同次开机中的 `journalctl -b -u chrony -u systemd-timesyncd --no-pager`；如已重启，需选取对应历史 boot。

**融合门限和实际权重**

- 通过 `rtk_stamp` 在同一 initialized 段内推进，确认接受了 **317 次导航更新**，不含初始化。
- 全包 navigation_rejections 最大值为 **0**；最大已记录 NIS **0.588787**，中位数 **0.154661**。当前源码配置门限为 **24.322**。
- `navigation_accepted_updates=0` 不等于未融合：该计数还要求 accuracy_ok。当前 `max_*_accuracy` 和 `max_*_stddev` 的零占位会阻止最终 readiness，但不阻止实际 GNSS 更新。
- 有效 LocalNavigation 记录的方差全为 `R_position=[1,1,4] m²`、`R_velocity=[0.09,0.09,0.09] (m/s)²`、`R_heading=0.030461742 rad²`。
  对应标准差：水平 1 m、垂直 2 m、速度 0.3 m/s、航向 10°。
- [GNSS adapter](../../scripts/gnss_adapter.py#L129) 使用 `max(接收机报告标准差, navigation.*_stddev)²`。
  GNSS 路径不以 `fusion.rtk_position_variance=0.0004` 作为最终观测噪声。
- 接收机报告的水平精度范围 0.559–0.701 m，实际 R 已采用更保守的 1 m 标准差。
  全量远端 hardware.yaml 和部署二进制版本没有记录，当前本地配置不应被称为录包配置快照；这里的 R 取自 bag 本身。

**静止漂移与平滑效果**

全包有效局部 GNSS 从第一点算最大水平偏移 **1.9773 m**，任意两点最大水平距离 **2.0191 m**，垂直峰峰值 **2.36 m**。
这支持“水平变化约 2 m”的观察，但不是“绝对定位误差小于 2 m”。卫星数增多与改善相符，单份 bag 不能证明之前误差唯一由卫星数决定。

在共同的有效采集窗口 8.61–42.82 s 内比较：

| 指标 | GNSS 输入 | EKF 输出 |
|---|---:|---:|
| East 峰峰值 / m | 0.441 | 0.323 |
| North 峰峰值 / m | 0.578 | 0.504 |
| Up 峰峰值 / m | 1.430 | 1.066 |
| 相对各自首点最大水平偏移 / m | 0.589 | 0.504 |
| 三维速度模中位数 / m/s | 0.0954 | 0.0362 |
| 三维速度模 P95 / m/s | 0.1749 | 0.0917 |

统计各自原生消息频率；窗口内 GNSS 有 317 条、EKF 有 17085 条，首末采样相差不足 0.1 s。不能用只有前 34 秒有效的 EKF 与全包 GNSS 范围直接比较。
在用户所述静止前提下，速度波动得到抑制；高度仍有慢变漂移，滤波平滑不能去除观测中的慢变偏差。

跳时前 IMU 加速度平均模长 **9.83939 m/s²**，相对代码 g=9.8066 约 +0.0328 m/s²；不能沿用旧包约 9.4 m/s² 的诊断。
另有需核查的后段传感器异常：bag 145.638 s 陀螺仪模长达到 **1.3233 rad/s（75.8°/s）**，全包加速度模长范围 8.9815–11.1929 m/s²。
如果机身姿态也完全不动，这些不是正常的零角速率读数；需区分触碰/振动、设备本身和数据异常。它们发生在融合重置之后，不是本次首次失效原因，也不能据此认定飞机发生了平移。

**如何让估计更可信**

1. 先解决 CM5 启动校时顺序，完成校时后重新启动整套 diagnostic 栈，使 MSP 会话重建。保持当前 R/NIS 做完整静止复测。
   若目标机使用 chrony，可在 ROS 启动前执行 `chronyc tracking`、`chronyc sources -v`，然后 `chronyc waitsync 60 0.01 0 1`；确认成功后启动。
   该命令最多检查约 60 次，每次间隔 1 秒，要求剩余校正小于 10 ms；它是启动门槛，不能保证后续永不跳时。
   大幅 step 校时应在节点启动前完成，运行中采用渐进校时并查明本次 step 来源。
   参考 [chrony 官方说明](https://chrony-project.org/doc/4.6/chronyc.html#waitsync)。
2. “提高预测权重”与“放宽接受门限”是两件事。增大 GNSS 位置 R 会减小位置校正增益，但同时更依赖 IMU 积分；放宽 NIS 只改变接受/拒绝条件，不能让估计自动更准。
   本包 NIS 很低，没有证据支持调大 NIS 或扩大队列；不能仅凭本次短静止段给出已验证的新 R/Q。
3. 若目标是确认静止时位置更稳定，应将可靠的静止信息用于零速更新（ZUPT）/偏置估计，并在移动时解除。
   当前静止判定用于初始化，未提供持续 ZUPT；仅“把位置固定”不是定位精度验证。
4. 若目标是实际飞行的绝对位置更准确，需要更准确的位置约束或独立参考（例如 RTK/外部定位）验证。
   单靠更信任 IMU 不能消除 GNSS 长期偏差。后续再核查 Q/R 的单位与离散化，使用独立静止和动态数据评估。

复算：

```bash
# 独立依赖，不修改 ROS 环境
python3 -m pip install --target /tmp/mcap-analysis-20260929 mcap mcap-ros2-support
PYTHONPATH=/tmp/mcap-analysis-20260929 python3 agi_ros2/analysis/gnss_ekf_static_20260929/extract.py
python3 agi_ros2/analysis/gnss_ekf_static_20260929/analyze.py
PYTHONPATH=/tmp/mcap-analysis-20260929 python3 agi_ros2/analysis/gnss_ekf_static_20260929/extract_msp_events.py
python3 agi_ros2/analysis/gnss_ekf_static_20260929/clock_audit.py
```

数值结果：[summary.json](summary.json)；融合转换：[fusion_transitions.csv](fusion_transitions.csv)；
每次接受更新：[accepted_updates.csv](accepted_updates.csv)；独立双时钟审计：[clock_audit.json](hardware_20260929_112215_827325/clock_audit.json)。
没有修改生产代码、hardware.yaml 或原始 bag；没有在硬件上执行校时或重启。
