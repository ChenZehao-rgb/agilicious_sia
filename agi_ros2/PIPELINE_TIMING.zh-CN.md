# IMU、融合、控制与 MSP 写入时延定位

所有主机阶段使用同一 Linux 启动会话的 CLOCK_MONOTONIC。ROS 时间只用于匹配原始样本和显示时间，不与单调时钟直接相减。
诊断字段不刷新旧命令证据。硬件模式 IMU 年龄上限为 50 ms，覆盖 15 ms 状态年龄、控制求解以及 25 ms 指令有效期，
避免有效指令在周期之间的异步复查中因原先 10 ms 上限失效。SITL 的 IMU 上限仍为 10 ms；状态 15 ms、指令 25 ms、
控制写入 2 ms 和通信锁定策略保留。Health 中的推力映射使用同一启动会话的已知配置，不因输出诊断缓存超过 50 ms 清零；
控制节点直接检查输出进程心跳，输出进程直接检查写入健康，MSP 明确失败仍使 Health 传输健康失效。

## 采集与分析

重新编译并部署整个 `agi_ros2` 包；消息接口有新增字段，发布端、订阅端必须一起更新。已有启动脚本默认录制全部 topic，包含新增的
`/sensors/imu/timing`、`/output_timing` 和 `/msp/write_timing`。沿用既有测试条件采集日志，不需要新增串口监控进程。

在源代码根目录执行：

```bash
./agi_ros2/scripts/build.sh
python3 agi_ros2/scripts/analyze_pipeline_timing.py /path/to/bag --output /tmp/agi_timing_result
```

离线脚本需要 `mcap` 和 `mcap-ros2-support`，直接使用 bag 内嵌 schema，无需 source ROS。
输出 `pipeline.csv`、`writes.csv`、`faults.csv` 和带 p50/p95/p99/max 的 `summary.json`。
旧 bag 没有新时间戳时对应列留空，不把 rosbag 记录器收到消息的时间当作节点回调时间。
多个 ROS 命名空间使用 `--namespace /your_namespace` 选择一条链路。

## 如何判断延迟发生在哪里

| CSV 列 | 测量范围 | 解释 |
|---|---|---|
| `mapped_sample_age_ms` | 映射后的飞控采样时间到接收节点 | 包括飞控发送、UART、主机读取及时间同步误差，不能视为纯 UART 时延 |
| `sensor_decode_ms` | 完整 MAVLink 帧开始分发到 IMU 发布前 | 解码、校验、构造消息及此期间调度 |
| `sensor_publish_ms` | IMU publish 调用开始到返回 | 可发现发布调用本身的耗时 |
| `imu_to_fusion_ms` | IMU 发布前到融合回调入口 | DDS、队列、executor 等待及发布调用的一部分 |
| `fusion_ms` | 融合回调入口到 fused_state 发布前 | EKF、导航观察处理、状态消息构造及此期间调度 |
| `fusion_to_control_ms` | fused_state 发布前到控制状态回调入口 | DDS、队列及 executor 等待 |
| `control_wait_ms` | 控制状态回调入口到控制 tick 入口 | 状态在控制器中的等待；100 Hz 周期可能产生 0～10 ms 等待，属于正常阶段但占用年龄预算 |
| `control_ms` | tick 入口到控制命令发布前 | 时间对齐、门限检查、求解、命令构造及此期间调度 |
| `command_to_output_ms` | 命令发布前到输出命令回调入口 | DDS、队列及输出 executor 等待 |
| `output_precheck_ms` | 输出回调入口到安全检查取时点 | 复制命令、时间对齐及初始输出处理 |
| `imu_age_output_ms` | 本命令所用 IMU 的融合回调入口到输出检查 | 与硬件 50 ms 门限对照，不能用后来收到的 IMU 刷新它 |
| `output_ms` | 输出命令回调入口到处理结束 | 包括映射、串口写入、诊断发布、状态发布 |

用 `control_session_start + sequence` 关联命令与输出，用 `clock_id + imu_receive_time` 关联融合与控制。
IMU 原始采样 header 关联 MAVLink 接收节点。脚本只关联同一主机时钟域，并单独报告允许接管的命令分布。
`imu_age_control_ms` 对应计算决策时的证据年龄；`control_ms` 包含求解之外的开销，不等于 `solve_seconds`。

先查看 `faults.csv` 的首次原因，再定位故障时间附近的逐帧数据；平均值或正常段 p95 不能替代故障瞬间的最大值。
若 IMU 采样间隔仍约 2 ms，接收、融合或控制等待却突然变长，说明数据在链路中延迟，不能据此判断飞控 IMU 停止采样。

## 串口写入诊断

`writes.csv` 对每次实际写入尝试和锁定后的请求分别记录命令编号、attempt_id、字节数、errno、开始/结束/期限、总耗时、write()
调用累计墙钟耗时、poll() 等待耗时、EAGAIN 次数和线程 CPU 耗时。控制帧 code=200，通过控制会话和 sequence 与 pipeline.csv 关联。

- `bytes_written < frame_bytes` 且 `system_error != 0`：检查对应系统错误和串口驱动。
- EAGAIN 次数与 `poll_ms` 增大：写队列暂时不可写，需要检查队列压力和驱动服务。
- 字节完整、errno=0、总耗时越限：控制写入仍按严格期限失败并锁定；完整只证明内核接受，不保证飞控收到。
- 墙钟耗时大而线程 CPU 时间小：存在等待或抢占。即使延迟落在 write() 调用内，也不能单凭这些数据区分内核等待和线程被抢占。
- `transport_latched`：本次没有执行 write()，`bytes_written=0`；`failure_attempt_id/failure_code` 指向首次致命写入。

下一次查询错误将明确显示类似：

```text
MSP request code 105 blocked by latched transport; original write code=200, attempt=..., bytes=14/14, ...
```

首次故障和本次尝试保存在不同诊断对象里。不会再把上一条控制帧的 14/14 字节当作本次 6 字节查询的结果。

## 需要进一步定位调度时

若逐段时间戳显示 executor 等待或线程非 CPU 时间突增，再在同样的静止测试条件下采集 Linux 调度事件。
例如使用 `perf sched record` 采集短时间 `sched_switch/sched_wakeup`，然后用 `perf sched timehist -p PID` 查看三个节点及 MAVLink
节点的运行、等待和调度延迟。权限和内核支持以目标主机为准；若同时使用 pidstat，关注对应线程的非自愿上下文切换。
确认串口调用本身异常后才考虑短时间 strace，避免高频跟踪系统调用改变原有时序。

新增诊断 topic 本身也有测量开销。不能用桌面机、伪串口测试代替目标机测量；对照测试应保持录包、进程负载及测试条件一致。
不要通过无限延长门限来隐藏实际状态或串口故障，也不要通过更新旧证据时间戳让旧命令重新获准。
