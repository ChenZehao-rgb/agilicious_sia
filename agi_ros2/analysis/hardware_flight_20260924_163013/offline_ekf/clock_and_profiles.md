# 系统时钟与 PID profile 说明

适用数据：`bags/hardware_20260924_163013_022677/`。本说明依据 bag、仓库代码及下列官方资料；未取得 CM5 系统日志，未修改 CM5 时钟、服务或飞控配置。

**时钟发生了什么，原因能否确定？**

同一次 MSP STATUS 请求的墙钟增加 **36.158606 s**，事件单调时钟仅增加 **9.833 ms**，请求实际响应耗时 **9.852 ms**。两种时间增量之差说明系统实时时钟/ROS 时钟额外前跳 **36.148773 s**，不是 UART 停流 36 秒。跳变约发生于当地时间 `16:30:58.83 → 16:31:34.99`、CM5 本次启动约 73 s 时。原始数字见 [msp_timeline.json](../msp_timeline.json)。

启动后联网首次校时是合理候选，也可能是其他校时程序或人工设置，**bag 本身不能确定具体服务**。chrony 可按配置进行大步校时；systemd-timesyncd 也会对大偏差进行 step。仓库 MAVLink TIMESYNC 代码只估计传感器时间与主机时间的映射，没有设置 Linux 系统时钟；不能把磁力计校准或 GPS 定位直接认定为前跳原因。[chrony 官方配置说明](https://chrony-project.org/doc/4.8/chrony.conf.html#makestep)、[systemd-timesyncd 官方说明](https://github.com/systemd/systemd/blob/main/man/systemd-timesyncd.service.xml)

**严重性取决于是否启用了 ROS 控制。**

- 本次 `diagnostic_only:=true` 不启动控制节点和控制输出节点，MSP 只监测；因此这次时间异常不会由 ROS 发出错误控制指令。
- 异常已使 MSP 证据会话持续报告 `restart required`，并触发 MAVLink 重同步。扣除墙钟跳变后，IMU/GPS 仍有约 **0.409 s / 0.501 s** 的实际有效观测缺口。
- 以后启用 ROS AUTO 时，这种跳变足以触发控制故障、撤销 override。接收机有效时设计上返回 RC；具体后续行为还受飞控模式和 failsafe 影响。它是自动飞行前必须解决的时序问题，不能保证不会影响飞行，也不能仅凭本包断言必然失控。

代码依据：[`runtime_launch.py`](../../../launch/runtime_launch.py)、[`msp_evidence.py`](../../../scripts/msp_evidence.py)、[`mavlink_sensor_node.cpp`](../../../src/mavlink_sensor_node.cpp)、[`control_node.cpp`](../../../src/control_node.cpp)、[`command_output_node.cpp`](../../../src/command_output_node.cpp)。离线回放需要单独修正时间并保留真实缺口和会话边界；回放修正不等于修好了实机时钟。

**在 CM5 上排查和处理。**

先只读检查实际使用的校时服务及历史日志：

```bash
timedatectl status
systemctl status chrony chronyd systemd-timesyncd --no-pager
journalctl --list-boots
journalctl -b 3f96d065-8e18-4ffe-886f-9ce165fdec04 \
  -u chrony -u chronyd -u systemd-timesyncd -o short-monotonic
```

这里的 boot ID 来自本包，重点查看单调时间约 73 s 附近的同步/step 记录。未安装的服务可能报 `unit not found`。若未保留历史启动日志，无法事后据此溯源；今天的 `journalctl -b` 只反映今天这次启动。下次复现时可用当前启动的 `-b`，保存日志与 bag。

**仅当 CM5 使用 chrony 时**，在地面、启动 ROS 前检查并等待同步：

```bash
chronyc tracking
chronyc sources -v
chronyc waitsync 60 0.01 0 1
```

最后一条最多等待约 60 s，要求已同步且剩余系统时间修正小于 10 ms；非零退出时不要启动这一轮。10 ms 是建议的启动检查值。`waitsync` 成功只证明当时满足条件，**不保证以后不发生 step**。[chronyc 官方说明](https://chrony-project.org/doc/4.8/chronyc.html#waitsync)

若使用 systemd-timesyncd，可用 `timedatectl timesync-status` 和 `timedatectl show -p NTPSynchronized --value` 查看同步状态。服务处于 active 不代表首次同步已完成；自动启动 ROS 时可使用 `systemd-time-wait-sync` 等同步等待机制，不能只依赖校时服务已启动。[systemd 同步等待说明](https://github.com/systemd/systemd/blob/main/man/systemd-time-wait-sync.service.xml)

运行原则是：**先在地面完成初始大幅校时，再启动 ROS；运行期间保持连续时间，以渐变 slew 校时。** 若 chrony 配有 `makestep 0.1 3`，其含义是 chronyd 启动后的前 **3 次更新**中，偏差超过 0.1 s 时允许 step，并非启动后 3 秒；必须结合应用启动顺序核对，不能只添加这行就认为已解决。运行期间避免重启校时服务或手工 `makestep`，并核对是否有多套程序同时设置系统时间。[chrony 启动校时说明](https://chrony-project.org/doc/4.8/chrony.conf.html#makestep)

发生跳变后，在地面停止本轮测试、完成校时，再重启整组 ROS，重新建立传感器同步、导航原点及 MSP 会话。不要通过放宽观测年龄门限绕过失效。

**PID profile 1、本地期望 0 是什么？**

这里是从 0 开始的**配置组编号**，不是 PID 系数、EKF 参数或定位精度：

| 项目 | 本包 / 本地值 | 含义 |
|---|---|---|
| 飞控 PID profile | `1` | 第 2 套 PID、部分滤波等内环配置；GUI 通常显示 Profile 2 |
| ROS `expected_pid_profile` | `0` | 本地预期第 1 套，因此不匹配 |
| 飞控 rate profile | `0` | 第 1 套 rates/expo 等输入响应配置 |
| ROS `expected_rate_profile` | `0` | 与飞控匹配 |

PID profile 与 rate profile 独立。编号和界面显示的对应关系见 [Betaflight 官方 Profiles 说明](https://betaflight.com/docs/wiki/guides/current/Profiles)。

[`shadow_support.py`](../../../scripts/shadow_support.py) 将飞控读回编号与本地预期比较；不匹配时设置 `config_verified=false`，阻止 AUTO 授权。**`expected_*` 只检查，不写飞控、不调整 PID，也不能证明 PID 已调好。** 这个编号差异本身不阻止离线 EKF。

若确定今后继续使用本次第 2 套配置，应核对该组参数后把本地 `expected_pid_profile` 设为 `1`；若计划使用第 1 套，则在地面切回 `profile 0` 并重新验证。不要仅为了通过检查而盲目切换飞控配置。可在拆桨、未解锁时通过 Betaflight CLI 保存 `dump all`，并检查 `profile`、`rateprofile`、`dump profile`、`dump rates`；具体命令支持以这块飞控的 `help` 为准。[Betaflight 官方 CLI 说明](https://betaflight.com/docs/wiki/guides/current/Cli)
