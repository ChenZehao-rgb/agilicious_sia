# AUTO 切换掉落日志诊断

输入：`bags/flight_20260910_200940_089872`，直接以 SQLite 只读查询和 ROS CDR 反序列化检查。
以下时间为仿真秒，不是墙钟时间。bag 未记录实际 UDP 包或 Betaflight ARM 回读，
因此上锁行为由当时的输入、授权结果和输出代码推断。

- 21.650 s：控制器进入 AUTO_STANDBY。
- 25.920 s：人工油门变为 1450，飞机持续爬升；30.490 s 的真值高度约 10.070 m。
- 30.500 s：控制器最后使用的正常 RC 时间戳。
- 30.600 s：RC 年龄超过 100 ms，状态变成 SENSOR_CHECK，MPC 预热清零。
  IMU、RTK、配置验证等标志仍有效。
- 30.842 s：RC 恢复。bag 同一接收时刻还收到了较早时间戳的 RC 消息；
  相邻源时间戳最大间隔为 284 ms。不能把整个 342 ms 直接归因于发布定时器，
  也不能仅凭 bag 区分 Python 调度、参数回调和 DDS 传输阻塞。
- 30.880 s：AUTO 被置高；30.890 s 的控制消息仍为 controller_warm=false、
  permit_override=false。轨迹 reference 开始变化，证明轨迹已装载并被采样。
- 31.340 s：预热完成，但状态要求重新观察健康 AUTO low，再切 high。
- 整份 bag 的 5759 条 control_command 均为 permit_override=false。
- 32.470 s：真值高度约 0.188 m，已落地。

现有 SITL 输出逻辑在 RC 超时、或 AUTO 高而未获授权时发送怠速和 AUX1 上锁；
因此推力中断可能在 AUTO 切换前已开始。此次没有证据支持“轨迹路径没传进去”。
三个 output_status 会话中，两个属于另一个 host clock_id；
本机控制节点忽略它们，未发现本机输出会话故障计数变化。

## 修改与验证

- 模拟遥控 heartbeat 使用独立回调组和多线程 executor，避免参数服务回调占用
  同一回调组而阻塞 50 Hz 发布。此修改处理可复现的阻塞路径，不能证明该路径是
  原始日志中 284 ms 源时间戳间隔的唯一原因。
- 回归测试人为阻塞参数服务 350 ms：原回调组设计期间收到 0 条 heartbeat；
  修改后仍持续发布，测试通过。
- /status 增加 RC timeout 和 RC link unavailable；输出状态明确说明 AUTO 拒绝、
  遥控失联、手动透传，避免始终显示 Waiting for control and authority。
- 轨迹加载时输出文件路径、采样数和时长。
- 未放宽 RC/控制超时，未修改 ARM、KILL 或故障后必须 AUTO low/high 的规则。
- ROS 包构建通过；三个进程级回归测试通过：UDP 超时/非法命令/高电平重启、
  传感器失联/控制进程暂停/仿真时钟回退，以及新增的 342 ms RC 中断与预热恢复。
- 尚未重新运行完整 Gazebo 飞行，不能宣称原始中断已彻底消除。

重新测试需重启 sim_rc 和 flight。先保持 AUTO=false，等待当前状态持续为
AUTO_STANDBY，再切 AUTO；若再次出现 RC timeout，应进一步检查主机调度和 DDS。
