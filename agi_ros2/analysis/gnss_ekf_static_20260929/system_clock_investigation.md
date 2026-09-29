# 本机系统时间跳变：systemd-timesyncd 首次 NTP 校时

2026-09-29 补充调查。结合原始 bag 与录包所在开机的系统 journal，已确认此次约 **+33.398677 s** 的跳变由 **systemd-timesyncd 首次 NTP 校时**触发。原分析中“仅凭 bag 无法确定改时进程”的限制，现已由系统日志补齐。

本地时间均为 Asia/Shanghai（UTC+8）。历史 boot ID：`06236981b7cf4ec4af725c23786bf504`。使用固定 boot ID，避免重启后相对 boot 编号发生变化。

## 直接证据

该 boot 的 `systemd-timesyncd[557]` 记录：

```text
11:23:33.424067 Contacted time server 185.125.190.57:123 (ntp.ubuntu.com).
11:23:33.424152 Initial clock synchronization to Tue 2026-09-29 11:23:33.423739 CST.
```

同刻 `systemd-resolved` 记录 `Clock change detected. Flushing caches.`。上述 timesyncd 时间是 journal 的源时间；对应接收单调时间分别为 73.386948 s、73.387086 s。日志存在传输延迟，不能混合源墙钟与接收单调时钟计算时钟偏移。

本次直接读取原始 bag 的全部 **20732 条 `/msp/events`**，核实其墙钟、单调时钟、匹配请求时间、event、code 与原有 CSV 完全一致。唯一跨跳时请求为 MSP code 105：

| 指标 | 数值 |
|---|---:|
| 请求墙钟 | 11:23:00.018586254 |
| 响应墙钟 | 11:23:33.426008092 |
| 请求单调时间 | 73.378566732 s |
| 响应单调时间 | 73.387311584 s |
| 墙钟耗时 | 33.407421838 s |
| 单调耗时 | 0.008744852 s |
| 两者差值，即墙钟前跳量 | **33.398676986 s** |

NTP 首次同步日志的接收单调时间落在该请求/响应区间内。独立用 journal 成对的 `__REALTIME_TIMESTAMP - __MONOTONIC_TIMESTAMP` 在跳变前后作差，得到 **33.398679 s**，与 bag 结果仅相差约 2 微秒。两份独立记录锁定同一个系统校时事件，墙钟缺口不代表串口真实停流 33 秒。

## 为什么启动后才校时

1. **RTC 未提供有效日期。** 内核记录 `rpi-rtc soc:rpi_rtc: setting system clock to 1970-01-01T00:00:12 UTC (12)`。该消息描述 RTC 读数，不能用它在 journal 中稍后补记的日期作为 RTC 读数。
2. **系统先恢复磁盘保存的历史时间。** 开机单调时间 4.042602 s，timesyncd 记录 `System clock time unset or jumped backwards, restored from recorded timestamp: Tue 2026-09-29 11:21:50 CST`。这给系统一个大致日期，并不证明已经与 NTP 同步。
3. **ROS 提前启动。** 历史 launch 日志显示 11:22:15.695270 启动 launch、11:22:15.852762 启动融合进程；bag 第一条记录为 11:22:17.202234261。校时发生在录包开始后约 **42.82 s 的真实经过时间**，墙钟相对时间却直接从约 42.82 s 跳到 76.22 s。
4. **NTP 首次成功较晚。** 三个 `ntp.ubuntu.com` IPv6 地址依次超时，单调时间为 52.645956 s、62.897291 s、73.146146 s；随后 IPv4 `185.125.190.57:123` 成功。系统发现本机时间落后约 33.4 s，执行一次直接前跳校正。

本次链路是：**RTC 日期无效 → 恢复历史时间 → ROS 在准确对时前开始工作 → NTP 经 IPv6 超时后从 IPv4 成功对时 → 系统墙钟前跳。** 网络等待推迟了校时发生的时机；33.4 s 是旧时间与 NTP 时间的偏差，不能直接等同于三个网络超时之和。

systemd 官方说明确认：timesyncd 对较大偏差直接跳时，对较小偏差渐进校正；恢复磁盘时间与等待准确对时是两个阶段。[systemd 官方手册源码](https://github.com/systemd/systemd/blob/main/man/systemd-timesyncd.service.xml)。本机 systemd 255 手册也包含相同机制说明。

## 项目代码及处理建议

当前代码中，hardware 启动明确使用 `use_sim_time=false`；MAVLink TIMESYNC 只更新飞控时间到主机时间的映射，没有修改内核系统时钟。`mavlink_sensor_node.cpp:207` 被动检测跳时并撤销导航，`msp_evidence.py:81` 执行跨时钟请求的保护处理。现有融合重置和 MSP 失败锁存因果链仍成立。

- [runtime_launch.py](../../launch/runtime_launch.py) 当前直接创建节点及 recorder，没有等待准确对时完成的门槛。历史 launch 与 journal 也直接证明此次确实提前启动。
- **首要修复是启动门槛。** 本机使用 timesyncd，应在启动整套 ROS 栈前确认 `timedatectl show -p NTPSynchronized --value` 为 `yes`；`NTP=yes` 或 timesyncd 服务 `active` 仅说明服务启用/运行。自动启动可使用 `systemd-time-wait-sync.service`，让 ROS 服务在 `time-sync.target` 之后启动，并实际拉起等待同步服务。仅写 `After=systemd-timesyncd.service` 不等于等到校时完成。手工启动脚本仍需要自己的检查。
- **核查 RTC 保时。** 检查 CM5 及载板的 RTC 后备供电、连接与日期保持情况。日志证明启动日期无效，但无法区分未接电池、电池耗尽、连接问题或固件/供电策略，不能直接判定某个硬件部件损坏。
- **核查 NTP 可达性。** 三个 IPv6 超时是事实，应检查该网络的 IPv6 UDP/123 可达性，或配置已验证可达的时间源。超时本身不能确定是路由、防火墙还是远端服务问题，也不需要据此关闭整机 IPv6。
- 完成校时后，重新启动整套诊断栈，让 MSP 会话重建再复测。长期运行若要求时间连续，还需明确运行中校时策略；启动同步检查不能保证以后绝不跳时。可进一步审查超时/存活判断是否应以单调时钟为依据，但不能直接取消现有保护。

当前检查时 `systemd-time-wait-sync.service` 为 disabled，当前 timesyncd 使用 `ntp.ubuntu.com`；这些是**调查时状态，不是录包时配置快照**。本次具体改时进程的归因依赖历史 journal，而非当前状态。

## 证据与复核

- [system_clock_evidence.json](hardware_20260929_112215_827325/system_clock_evidence.json)：历史 journal 原始字段、双时钟差分、ROS launch 摘录、采集命令，以及标明时间边界的当前配置观察。
- [system_clock_bag_verification.json](hardware_20260929_112215_827325/system_clock_bag_verification.json)：本次直接读取原始 bag 的独立复算结果。使用本机 `rosbag2_py` 和已安装的 `agi_ros2.msg.MspEvent` 逐条核对原有 CSV 的时间字段；此次未重复全 bag CRC 审计。
- [clock_audit.json](hardware_20260929_112215_827325/clock_audit.json)：原有 bag 双时钟与失效传播审计。其“bag 不含改时进程”边界仍正确，系统日志是本次新增的外部证据。

只读复查历史日志：

```bash
journalctl -b 06236981b7cf4ec4af725c23786bf504 -u systemd-timesyncd --no-pager -o short-precise
journalctl -b 06236981b7cf4ec4af725c23786bf504 -u systemd-timesyncd --no-pager -o short-monotonic
journalctl -b 06236981b7cf4ec4af725c23786bf504 -k --no-pager --grep='rtc|RTC|fixrtc'
```

本次只补充分析与证据，没有修改生产代码、系统校时配置、运行中服务或原始 bag，也未执行改时、重启操作。
