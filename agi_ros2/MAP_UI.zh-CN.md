# 地图地面站：在线准备与离线使用

地图包、网页、Python HTTP 后台都在**地面电脑**。CM5/实际控制主机只增加普通优先级、
`ground_station_telemetry`，负责遥测回传和固定入口启动。浏览器访问本机 `http://127.0.0.1:8080`；
地面电脑的 rclpy 经 ROS 2 / DDS 接收镜像遥测、发布 `GeographicGoal` 与暂停 `Empty` 命令。
因此地面电脑需要 ROS 2 和匹配的项目消息，不是只需浏览器。

支持“运行程序”和“暂停并悬停”；运行按钮只启动进程，不执行 ARM、起飞/降落或 AUTO 切换。关闭网页、后台退出、地面网络断开
不主动取消目标或撤权；控制主机仍按原保护逻辑运行。不引入地面站心跳门控。

## 运行调参

顶部“调参”打开弹窗，读取当前连接控制端的模式、控制器和实际参数，不采用运行区下拉框作为目标。
可飞行中读取和分组应用，不依赖地图、GPS、悬停或 disarm；目标提交仍沿用原准入规则。

- 轨迹组：`go_to_pose_max_vel`、`go_to_pose_max_acc` 严格为有限正值，供**后续新生成的在线轨迹**使用。
  当前航段的 p/v/a/j、结束时间、暂停减速、yaw 和 CSV 不随调参改变。
- MPC 组：Q_pos/Q_att/Q_vel、R_collective_thrust、R_body_rates、exp_decay 应用到活动 MPC，供**后续求解**使用。
  同一实例在完整控制计算之间安装，不 reset、不清 warmup、不退出 AUTO。GEO 时 MPC 组不适用。
  权重允许有限非负值；exp_decay 为有限正值，允许大于 1，但实际指数加权矩阵必须有限。
  Q_vel 标量同步作用三轴；R_body_rates 三输入仍合为 x/y/z 数组。Q_att 保持四元数残差，T/m 单位为 m/s²。
- 每个 MPC 名称和输入框均有原生鼠标悬停说明，文案仅在 `web/tuning.js` 的静态映射维护。
  调大一项不保证响应更快，R 惩罚相对参考的输入而非相邻周期变化，exp_decay 为预测时域缩放。

每项显示控制端“实际值”，编辑后标为“草稿”。“读取当前值”刷新实际值但保留草稿，
“撤销轨迹/MPC 草稿”将对应组重新载入当前实际值。仅显式点击对应应用按钮才提交改变字段；
另一组和未编辑字段不会随旧页面值写回。读取只发生在打开、手动读取、应用后，不增加遥测参数流。
关闭、Esc、刷新及悬停均不提交；同组请求在途时禁用重复应用。

“已确认应用”表示控制端已经安装；明确拒绝则未应用并保留草稿。超时表示**结果未确认，可能已经执行**，
不重试或回滚，可“读取当前值”核实。“已确认应用，回读未完成”仍表示 SET 成功。
会话改变会标记旧实际值过期并要求重新读取，草稿不会自动重发。
**仅本次运行有效，重启按配置文件加载**，不写 hardware.yaml / simulation.yaml。

HTTP 为 `GET /api/tuning`、`POST /api/tuning`，只转发固定分组/字段。
例如 POST `{"group":"mpc","values":{"Q_pos_x":60,"R_body_rates":[100,100,80]}}`。
控制端专用服务采用 `GetParameters` 与 `SetParametersAtomically`，不使用 ROS 节点参数数据库。
在正确 ROS 域/source 环境下手动调试（命名空间部署时给路径加对应前缀）：

```bash
ros2 service call /flight_control/get_runtime_parameters rcl_interfaces/srv/GetParameters \
  '{names: [mode, controller_type, go_to_pose_max_vel, go_to_pose_max_acc, Q_pos_x, R_body_rates, exp_decay]}'
ros2 service call /flight_control/set_runtime_parameters rcl_interfaces/srv/SetParametersAtomically \
  '{parameters: [{name: go_to_pose_max_vel, value: {type: 3, double_value: 1.5}}]}'
ros2 service call /flight_control/set_runtime_parameters rcl_interfaces/srv/SetParametersAtomically \
  '{parameters: [{name: Q_pos_x, value: {type: 3, double_value: 60.0}}, {name: R_body_rates, value: {type: 8, double_array_value: [100.0, 100.0, 80.0]}}]}'
```

`ros2 param set` 不会更新这些运行值。更新部署需重建控制端和地面端包，加载新的控制节点、
map_ui.py 与网页；不会向已经运行的旧进程自动注入功能。
直接回归沿用 `tests/test_map_ui.py`、`tests/test_map_ui_browser.py` 和
`tests/test_fc_control_delivery.py DeliveryTests.test_runtime_parameter_services_install_between_active_cycles`。
隔离 SITL 可运行 `tests/test_sitl_go_to_position.py --output <新目录> --state-source fc_fused --map-ui --geographic --runtime-tuning`，
它会在 A 运动中更新，检查 B 采用新速度、MPC 安装、控制会话、时序与 KILL；必须使用独立 ROS 域和空闲仿真端口。
单线程串行更新仍占用回调时间，不能据此宣称 CM5 实时性或实飞稳定性已验证。

## 一键启动与部署

右侧“运行程序”位于地理目标表单上方。选择实机/仿真，在对应服务在线时点击一次启动。
“结束任务”停止当前模式下本监听实例一键启动的全部入口及其正常清理范围内的后代；
硬件退出 launch_realtime 控制链，仿真退出 sim_rc、flight、SITL/Gazebo/传感器链。
监听服务和地图 UI 保留，以便再次启动；不结束另行手动启动的程序或其他模式/机器的进程。
实机请先落地并上锁后再结束任务；它是退出控制程序，不是暂停悬停或自动降落。
启动中也可结束：立即显示“结束中”，后台停止后续启动并清理本次部分进程；完成显示“已结束”。
结束中的重复启动/停止不会新增进程或清理线程。结束完成后可以再次显式启动。
HTTP 202 只表示结束请求发送，超时/断网不自动重发，是否结束以执行端状态为准。
没有飞机位置、GPS 或控制节点时也可显示服务在线、发起启动；地图不可用不影响启动。
HTTP 202 只表示“启动请求已发送，等待执行端反馈”；真实状态来自执行端 ROS 心跳。
HTTP 超时显示结果未确认，不自动重发。刷新/关闭网页、退出地面 UI、DDS 失联不停止执行端进程。
失联显示最后报告/未确认，不能凭旧的“运行中”判断当前进程仍在运行。

固定入口如下，子进程 cwd 均为定位后的**实际仓库根目录**，不是监听进程碰巧所在的终端目录：

| runtime_mode | 本机实际启动命令与顺序 |
| --- | --- |
| hardware（默认） | `python3 agi_ros2/scripts/launch_realtime.py mode:=hardware shadow_only:=false` |
| simulation | `python3 betaflight_sitl/run.py` → 本次日志出现 `Betaflight SITL is ready` 且进程存活 → `./agi_ros2/scripts/launch.sh` → `ros2 run agi_ros2 sim_rc.py --ros-args -p use_sim_time:=false` |

`runtime_mode` 仅 hardware/simulation，启动时只读。自动定位依次检查当前工作目录及其父目录、
实际加载的 runtime_launcher.py（源码或安装副本）所在目录及其父目录；以 agi_ros2/package.xml
和本模式固定入口文件确认仓库，不假定安装目录层数，不扫描全盘或其他兄弟仓库。
例如安装位于 `/home/sia/agilicious_internal-main/install/agi_ros2/`，会找到仿真仓库
`/home/sia/agilicious_internal-main`；安装位于 `/home/ubuntu/agilicious_sia/install/agi_ros2/`，
会找到实机仓库 `/home/ubuntu/agilicious_sia`。两机分别构建/source 自己的安装环境，不硬编码这两个路径。
状态 summary 的 repo_root 返回实际采用路径。若 install 被单独搬出仓库，需先 cd 到实际源码根目录；
source 安装环境只选择 ROS 包，本身不改变终端目录。更新后须重启监听服务加载代码（会清理其拥有的控制链）。
systemd 仍填写实际 WorkingDirectory，模板内的相对 source 也依赖此设置。后台继承监听进程环境，并 source `/opt/ros/$ROS_DISTRO/setup.bash` 和
`install/agi_ros2/local_setup.bash` 给子进程（未设 ROS_DISTRO 时与 launch.sh 一样优先 jazzy，否则 humble）。
环境 source 单独限时 15 s；构建和 SITL 就绪不设短总超时，使用 run.py 原阶段退出/超时，页面显示最近阶段。
首次编译可能较久。SITL 日志写文件而非 PIPE，不会因管道无人读取阻塞构建。

**实机参数尚未确认：** 当前 hardware.yaml 为 `state.source: fc_fused`、`bench_fixed_gps: false`。
用户提及的 `bench_fixed_gps:=true` 与 fc_fused 冲突，runtime_launch.py 会拒绝；本轮没有加入该参数、
没有切换定位链，严格执行上表固定命令。如果配置被改成冲突组合，错误随入口退出回传 UI。
硬件“启动中”持续到 launch_realtime 报告四个节点 FIFO 配置；“运行中”仍只表示入口进程存活，
不代表传感器/导航就绪、ARM/AUTO 已开启或输出已接管。sudo、时钟等待、节点检查失败均保留实际退出原因。

sim_rc 默认会以 **50 Hz** 发布 Authority：armed=false、auto_switch=false、kill=true、throttle=1000、
roll/pitch/yaw=1500。用户负责后续遥控操作；按钮/启动器不执行 ros2 param set，不解除 KILL，不 ARM/AUTO，
不改变油门。原有入口退出清理保持，包括仿真的既有 disarm 清理。

### 单连接与 ROS 域

同一 ROS_DOMAIN_ID 只运行一个回传实例和一套活动控制链。两个回传节点即使没有重复控制链，
也会各自订阅和转发，导致遥测频率叠加；本机锁只防止同用户、同仓库、同角色重复拥有进程，不能解决重复回传。
CM5 hardware 服务与 PC simulation 服务同时在线时，应分别使用不同 ROS_DOMAIN_ID。
地面 map_ui.py 要在目标系统对应的域启动，保留原有 RMW/DDS 发现设置。
下拉框只选择命令 topic，不改变后台域、不跨域路由；对应模式无服务显示“该模式服务未连接”。
切换独立域需要重新启动地面后台并设置环境，本轮不提供双 context 或热切换。

每个执行端只订阅自身模式的 `ground_station/runtime/{hardware|simulation}/start` 和 `/stop`（std_msgs/Empty），
并发布相应 `/status`（DiagnosticArray：summary + 固定进程项）。命令和状态均 RELIABLE、VOLATILE、depth=1。
`POST /api/runtime/start` 与 `POST /api/runtime/stop` 都只接收 {"mode":"hardware"} 或 {"mode":"simulation"}；
不接收 PID、命令、目录或主机地址。停止只针对本次实际拥有的进程句柄/进程组。
每次点击发布一次，不保留/重放命令；DDS 对已连接订阅者的可靠重传不等于应用自动重试。
启动器状态变化立即回传，心跳复用 slow_rate_hz（默认 1 Hz），即使无飞行数据/内容未变也重复发送。
显示年龄使用地面本地接收时间，3 s 未更新显示失联；很低的 slow_rate_hz 会使显示过期。
位置/普通状态采用 5/1 Hz，GoalStatus 保留即时回传语义。

### 仿真桌面终端与地面 UI

两端各自先按下节构建并 source，消息版本须一致。此处为本机实际 Humble 示例；路径换成各机实际仓库。
在有桌面 DISPLAY/XAUTHORITY、拥有 Betaloop 配置的用户终端中执行：

```bash
cd <实际仓库根目录>
source /opt/ros/humble/setup.bash
source install/agi_ros2/local_setup.bash
export ROS_DOMAIN_ID=<仿真系统对应的域>
# 沿用原有 RMW_IMPLEMENTATION / DDS 配置，不在此覆盖它们
ros2 run agi_ros2 ground_station_telemetry.py --ros-args -p runtime_mode:=simulation
```

地面终端：cd 自己的仓库，source 相同发行版和自己的 install/agi_ros2/local_setup.bash，
设置同一目标 ROS_DOMAIN_ID 与兼容 DDS 环境，再 `ros2 run agi_ros2 map_ui.py`。
打开 http://127.0.0.1:8080，选择“仿真”，确认主机名和在线状态后点击“一键启动仿真”。
GUI 是 run.py 默认行为，不加 --arm/--no-build/--no-ros2；保持 fc_fused、主机时钟和 use_sim_time:=false。
默认三个入口都在监听主机同机运行，不把 sim_rc 拆到时钟不同的电脑。
已有跨主机联合仿真应保留其时钟/设备布局，不能直接用此同机示例覆盖。
手动已有程序不归本次启动器所有；不要在已手动运行控制链时再点启动。

### 进程所有权与日志

硬件只拥有 launch_realtime；仿真拥有 sitl、flight、sim_rc 三个入口，每项显示 PID、退出码和日志路径。
日志目录是执行用户的 `~/.local/state/agi_ros2/runtime/<仓库路径哈希>/<mode>/`，每次入口使用独立日志，
锁文件 listener.lock 保留在该目录；不要求 XDG_RUNTIME_DIR。日志不上传为 20 Hz 遥测，也不提供网页终端。
只传简要阶段、失败尾部和路径；完整上下文在执行主机文件中。日志需按本机运维方式定期清理。

连续/并发点击在 starting 或仍有本批进程存活时不会起第二组；运行中有入口退出会显示失败/剩余实际状态，
不会自动重启或补齐。启动期失败不再启动后续项，并清理本次拥有的部分进程。
原入口自身的重复节点/端口检查保留；锁不能识别所有手动进程，也不会接管或清除它们。
正常退出先给入口 SIGINT 和 25 s 清理时间，再对本次进程组 TERM（5 s）/KILL（2 s）；
run.py 在 EEPROM 阶段也安装 SIGTERM 清理，独立 session 后代仍由其 finally 清理，编译后代属于本次外层组。

**停止或重启控制主机的回传服务会清理它启动的控制链。** 不能当作运行中无影响的遥测刷新。
systemd 使用 control-group 处理剩余后代；监听崩溃恢复只恢复监听，旧 start/目标/RC 动作不持久化或重放。
本方案不承诺监听崩溃与控制进程完全隔离；已按后续需求增加“结束任务”，没有自动重启或独立运行 unit。

### CM5 开机监听服务

模板：[agi-ground-station.service.in](deploy/agi-ground-station.service.in)。本机生成模板不等于 CM5 已部署，
不要在地面电脑直接启用硬件 unit。目标机使用普通用户构建，Gazebo OFF：

```bash
cd <CM5实际仓库根目录>
source /opt/ros/<实际发行版>/setup.bash
./agi_ros2/scripts/build.sh -DAGI_ROS2_GAZEBO=OFF
source install/agi_ros2/local_setup.bash
cp agi_ros2/deploy/agi-ground-station.service.in /tmp/agi-ground-station.service
# 编辑 /tmp/agi-ground-station.service：替换 @USER@、@REPO_ROOT@、@ROS_DISTRO@、@ROS_DOMAIN_ID@
# User 是普通运行用户；WorkingDirectory 是该用户实际仓库绝对路径；不能依赖 .bashrc
systemd-analyze verify /tmp/agi-ground-station.service
sudo install -m 644 /tmp/agi-ground-station.service /etc/systemd/system/agi-ground-station.service
sudo systemctl daemon-reload
sudo systemctl enable --now agi-ground-station.service
systemctl status agi-ground-station.service
journalctl -u agi-ground-station.service -b -f
```

如需既有 DDS 设置，创建 `/etc/agi-ground-station.env`，使用 systemd 的 `KEY=value` 格式，
如 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、实际 DDS 配置文件路径；不支持 source 或 shell 展开。
跨机不要误留 ROS_LOCALHOST_ONLY=1。模板只 After=network.target，不等待位置/GPS/控制节点，网络稍后可发现。
Restart=on-failure、RestartSec=5 只重建监听；KillMode=control-group、TimeoutStopSec=120 给分级清理留时间。
`systemctl active` 只证明监听 unit 活跃，硬件链状态还须看网页/ROS 状态，飞行授权另看既有状态。
取消开机启动且**同时停止本服务及其控制链**：`sudo systemctl disable --now agi-ground-station.service`。
只取消下次开机启动、不影响当前运行：`sudo systemctl disable agi-ground-station.service`。

### 实机关机按钮

上方状态栏右侧提供仅实机模式显示的电源图标，鼠标悬停显示“关闭板载电脑”。执行位置是网页显示的 hardware 监听主机，
不是地面 HTTP 后台；把 PC 配成 hardware 也会请求关闭那台 PC，不按主机型号判断。
请先落地、上锁再关机。仿真角色没有关机订阅，HTTP 也拒绝 simulation 模式的关机请求。

固定路径为 `POST /api/runtime/shutdown`，只接收 `{"mode":"hardware"}`，发布一次
`ground_station/runtime/hardware/shutdown`（std_msgs/Empty，RELIABLE、VOLATILE、depth=1）。
执行端后台线程只运行 `sudo -n shutdown now`；`-n` 禁止等密码，不改变 shutdown now 的关机含义。
没有可传入的命令/参数/主机地址，也没有自动重试或重连补发。权限失败不会主动结束已有控制链。
重复请求在执行中或命令成功后不会再调用 shutdown；失败后允许用户明确重试。

关机状态独立于程序运行状态。网页显示请求中、命令返回成功、sudo 错误或超时未确认，
并给出执行主机的 `~/.local/state/agi_ros2/runtime/<仓库哈希>/hardware/<唯一编号>-shutdown.log`。
15 s 命令等待超时只说明结果未知，不推断没有关机，也不允许自动重发。
HTTP 202 仅说明已发布；即使命令返回 0，也不声称主机已经断电。失联只显示最后报告/未确认。
关机过程中 systemd/操作系统负责停止服务；已有 control-group 与进程清理保持。

在 CM5 上由管理员单独授权固定关机调用。先核对外部命令路径：

```bash
type -P shutdown
sudo visudo -f /etc/sudoers.d/agi-shutdown
```

例如路径为 `/usr/sbin/shutdown`、运行用户为 ubuntu 时，文件内容为：

```sudoers
ubuntu ALL=(root) NOPASSWD: /usr/sbin/shutdown now
```

以目标机实际路径和用户替换。只授权固定的 `now` 调用，不配置 NOPASSWD: ALL。
检查语法后，以实际运行用户查询权限（以下命令不执行关机）：

```bash
sudo visudo -c
sudo -n -l /usr/sbin/shutdown now
```

更新后两端重新构建/source，重启地面 map_ui.py；CM5 在控制链停止后重启监听服务，再刷新网页。
不能只刷新网页：旧 HTTP 后台没有 shutdown 接口，旧监听也没有对应订阅。
本实现沿用现有 DDS 网络边界，没有新增身份认证；ROS_DOMAIN_ID 不构成关机请求的身份校验。
没有在开发电脑自动修改 sudoers，也没有通过测试执行真实 shutdown。

关机功能验证：进程测试 14/14、DDS/HTTP 11/11、Chrome 合成回归通过；构建安装及 Python/JS/差异空白检查通过。
测试仅用替身执行关机分支，实际 shutdown 执行次数为 0。覆盖固定 argv/cwd、hardware-only、重复请求、
sudo 权限失败不停止运行链、显式重试、超时结果未知、HTTP 校验、命令不保留、刷新/断网不重发。
本机只重启了地面 map_ui.py 并保留 ROS 域 2，使用无效空对象验证新关机接口返回 400（不发布消息）；
已有 simulation 监听未重启。CM5 尚需同步构建、配置固定关机权限并更新监听，真实关机未验证。
[验证摘要](test/results/runtime_shutdown_20261102/validation.json)、
[浏览器结果](test/results/runtime_shutdown_20261102/browser.json)。


### 非交互实时权限（目标机核验）

launch_realtime 原样调用 `sudo -n true`，然后 `sudo -n chrt -f -p <priority> <tid>`。
终端一次 sudo -v 不是开机后台长期权限；不运行 root ROS 服务，不在网页输入密码。
由管理员在目标机先用 `command -v true; command -v chrt` 和 `type -P true; type -P chrt` 核对
真实外部可执行文件路径（true 可能是 shell builtin，以 type -P 的路径为准）。
通过 `sudo visudo -f /etc/sudoers.d/agi-realtime` 配置以下最小固定调用集合，替换运行用户名和实际路径：

```sudoers
Cmnd_Alias AGI_REALTIME = /usr/bin/true "", /usr/bin/chrt -f -p 5 *, /usr/bin/chrt -f -p 15 *, /usr/bin/chrt -f -p 20 *, /usr/bin/chrt -f -p 25 *, /usr/bin/chrt -f -p 30 *
<运行用户名> ALL=(root) NOPASSWD: AGI_REALTIME
```

没有授权任意命令或所有 chrt 形式。PID/TID 动态，因此尾参数为通配符；这授权该用户用列出的调度调用操作
其他 PID，并不提供“只允许本用户 TID”的隔离。按目标机现有权限策略审查此权限，不能误称 NOPASSWD: ALL。
核验配置 `sudo visudo -c`；再以实际运行用户测试（清除该终端 sudo 缓存，确保不是靠 sudo -v）：

```bash
sudo -k
sudo -n true
sleep 30 & probe_pid=$!
sudo -n chrt -f -p 5 "$probe_pid"
chrt -p "$probe_pid"
kill "$probe_pid"
wait "$probe_pid" || true
```

只能结束上述自己启动的探针。随后从服务运行环境核对 PATH/source/ROS_DOMAIN_ID 与日志。
权限不足会导致入口非零退出，网页显示 sudo 实际错误及日志；不自动降低优先级、不绕过原检查。

### 后续“结束任务”验证与升级

新增结束按钮后：进程测试 11/11、DDS/HTTP 10/10、Chrome 合成回归通过，构建安装及 Python/JS/空白检查通过。
覆盖启动中取消、运行后停止、并发起停、停止后再启动、保留无关手动进程、硬件假入口退出，
以及 stop 模式路由、严格字段、VOLATILE 不保留、单次点击、HTTP 超时/刷新不重发。
浏览器夹具使用独立临时部署根目录，避免不同测试域之间争抢同一个真实部署锁。
[验证摘要](test/results/runtime_stop_20261102/validation.json)、[结束截图](test/results/runtime_stop_20261102/stopped.png)。
初次实现时因用户仿真占用端口未做真实结束验证；后续反馈“结束无效”后已补验：
实际问题是 map_ui.py 进程仍为 12:08 启动的旧版，结束接口 12:14 才安装，POST /api/runtime/stop 返回 404。
执行端已有 can_stop=true，但 HTTP 层没有发布 stop。保留原 ROS 域 2、工作目录和完整环境重启地面后台后，
无动作探测从 404 变成预期的无效输入 400；页面也新增“重启 map_ui.py”的旧接口提示。
真实隔离 ROS 域 95 的浏览器启动/结束通过：一次 stop，约 1.86 s 后已结束，记录的入口及后代共 17 个进程退出，
SITL 端口释放；同一监听实例仍在线、启动按钮恢复，刷新不重发。全程默认 KILL、未 ARM/AUTO，无遥控参数操作。
[诊断证据](test/results/runtime_stop_verified_20261102/diagnosis.json)、
[真实结束结果](test/results/runtime_stop_verified_20261102/sitl.json)、
[真实结束截图](test/results/runtime_stop_verified_20261102/stopped.png)。未验证真实硬件退出。
`test_runtime_launcher.py --smoke` 已补上实际按钮停止、监听保留和结束后刷新检查，可在端口空闲时按下文复现。

升级后要重启两端的 Python 进程（执行主机 ground_station_telemetry.py、地面 map_ui.py），再刷新浏览器。
旧监听没有 stop 订阅，旧 HTTP 后台也没有 /api/runtime/stop；仅刷新不能更新这些后台代码。
当前旧版一键启动链可在监听终端按 Ctrl+C 结束（会清理该监听拥有的链），然后启动新版监听。
systemd 部署通过停止/重启对应 unit 更新，同样会结束它拥有的控制链。没有自动关闭用户已有程序。

### 本轮验证（2026-11-02）

本机 Humble，Gazebo ON 构建与安装通过；真实 smoke 使用安装后的 `ros2 run` 监听和 UI，
验证同目录模块可导入，cwd 仍是实际源码根目录。未修改 C++、控制配置或 sim_rc 默认行为。

- 固定启动器 8/8：argv/cwd/环境、并发启动、重复监听锁、旧日志不就绪、失败不继续、部分清理、
  运行后退出、显式重试、真实独立 session 假 EEPROM 的 TERM 清理、编译进程组取消。
  sudo/硬件配置错误通过假入口验证摘要回传；实际配置组装另确认 bench_fixed_gps 冲突，不执行硬件动作。
- DDS/HTTP 9/9：模式路由/只读参数、严格请求字段、同源限制、并发 POST、无飞机数据服务在线、
  VOLATILE 不重放、即时事件、重复心跳和 slow_rate_hz 生效、后台 starting 不阻塞遥测。
  原 20/2 Hz 实测约 20.00/1.94 Hz，10/1 Hz 约 10.32/0.97 Hz（有限窗口边界误差）。
- Chrome 合成回归通过：一次启动请求、5 s HTTP 超时不重试、刷新保持模式与状态、断网不停止/重发，
  既有地图、地理目标、暂停、位置文本刷新和失联样式通过，无页面脚本错误/外部请求。
- 独立 localhost ROS 域 95 的真实 UI smoke 通过：无飞机状态时在线可点击；一次按钮依次启动三入口，
  含默认构建/GUI约 26.70 s；Authority 为 ARM=false、AUTO=false、KILL=true、AETR=[1500,1500,1000,1500]。
  刷新后同一组三个 PID；关闭页面/退出 UI 后仍存活；关闭测试监听后入口及 SITL 端口清理完毕。
  全程无 ros2 param set、ARM/AUTO/油门操作，不是飞行验证。原入口的退出 disarm 行为保留。
- Python/JS 语法和 git diff --check 通过。systemd 模板替换本机实际路径后 verify 返回 0，
  同时有宿主机既有 netplan/snapd/backlight unit 警告；未安装/启用 unit，不能据此声称 CM5 启动验证通过。

尚未验证 CM5 部署/开机及重启恢复、目标机非交互 sudo/FIFO、跨机 DDS 和真实硬件链/飞行。
`bench_fixed_gps:=true` 尚未获得用户确认，仍保持 fc_fused + bench_fixed_gps=false。

[验证摘要](test/results/runtime_start_20261102/validation.json)、
[真实 smoke 结果](test/results/runtime_start_20261102/sitl.json)、
[浏览器结果](test/results/runtime_start_20261102/browser.json)、
[启动前截图](test/results/runtime_start_20261102/idle.png)、
[运行截图](test/results/runtime_start_20261102/running.png)。
结果目录沿用现有 Git 忽略规则；原始文件在 `/tmp/agi_runtime_smoke_final_20261102`、
`/tmp/agi_runtime_browser_final_20261102`，入口日志路径记录在 sitl.json。

可复现命令（挑选未占用的隔离测试域和全新输出目录；测试 Playwright/Chrome 不属于产品依赖）：

```bash
python3 agi_ros2/tests/test_runtime_launcher.py -v
source /opt/ros/humble/setup.bash
source install/agi_ros2/local_setup.bash
ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 python3 agi_ros2/tests/test_map_ui.py -v
ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 PYTHONPATH=/tmp/agi_pause_qa:$PYTHONPATH \
  python3 agi_ros2/tests/test_map_ui_browser.py --browser /opt/google/chrome/chrome \
  --output /tmp/agi_runtime_browser_new
# 下面实际启动默认 Gazebo GUI/SITL/flight/sim_rc；要求空闲端口和用户桌面环境，不发送遥控参数操作。
ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=agi_runtime_new PYTHONPATH=/tmp/agi_pause_qa:$PYTHONPATH \
  python3 agi_ros2/tests/test_runtime_launcher.py --smoke /tmp/agi_runtime_smoke_new
```

## 构建与同机 SITL

命令从各机器自己的项目根目录执行。当前开发环境是 Humble；使用本机实际兼容发行版，
两端消息定义须同步，各自按架构构建，不能互拷 ARM64/x86 的 install。

```bash
source /opt/ros/humble/setup.bash
./agi_ros2/scripts/build.sh -DAGI_ROS2_GAZEBO=ON
source install/agi_ros2/local_setup.bash
```

CM5 不需要 Gazebo 适配器，构建使用 `-DAGI_ROS2_GAZEBO=OFF`；仍复用原工作区和构建体系。
不另建项目、不拆消息包。以下终端都先 source ROS 和上述安装环境。

若同机仿真已在运行，只需新增下面的转发与 UI 两个终端。
从头启动仿真则先分别运行：

```bash
# 终端 1：Gazebo、Betaflight SITL、传感器适配器
python3 betaflight_sitl/run.py
# 终端 2：现有控制链
./agi_ros2/scripts/launch.sh mode:=sitl
# 终端 3：模拟接收机，FC 路径使用主机时间
ros2 run agi_ros2 sim_rc.py --ros-args -p use_sim_time:=false
```

继续按 [README 的模拟接收机流程](README.md#sitl保持分层部署)解除 KILL、低油门 ARM、
人工起飞、AUTO low→high。README 中 throttle 1420 只是当前仿真模型示例，不是实机参数或通用起飞油门。
`run.py` 不启动 flight、sim_rc，也不自动 ARM；默认 ROS 2 路径不能用 `--arm` 代替上述流程。
只有 `mode:=sitl` / `hardware`，没有 `mode:=simulation`、`autorun` 或 `enable_output`。
默认 `simulation.yaml` 为 `state.source: fc_fused`、空轨迹；本次未改动这些配置。

手动启动控制链时，新增终端 4，在**实际仿真主机**运行（同机 SITL 即本电脑）；
也可跳过上面三个手动入口，按下节一键启动流程只运行此监听服务：

```bash
source /opt/ros/humble/setup.bash
source install/agi_ros2/local_setup.bash
ros2 run agi_ros2 ground_station_telemetry.py --ros-args -p runtime_mode:=simulation
```

新增终端 5，只在**地面电脑**运行：

```bash
source /opt/ros/humble/setup.bash
source install/agi_ros2/local_setup.bash
ros2 run agi_ros2 map_ui.py
```

浏览器打开 `http://127.0.0.1:8080`。`--host/--port` 仅配置本地 HTTP 监听，
不是 CM5 地址；它不限制 rclpy 的 DDS 通信。源码入口也可用
`python3 agi_ros2/scripts/map_ui.py ...`，仍需 source 匹配的消息环境。
默认在线显示 OpenFreeMap，道路地图无需注册或 API Key。首次无飞机位置时显示沈阳；
这只是浏览视角，不是导航原点。也可拖动地图准备其他地区。

准备离线地图只需在页面操作：

1. 联网打开页面，点击顶部“地图”状态展开管理面板，移到场地，点击“框选区域”后点两个对角，或点“用当前视野”。
2. 填名称（例如“沈阳试飞场”），保留“街道与建筑”，点“下载到本机”。
3. 等进度完成，在“当前底图”选择“离线 · 沈阳试飞场”，然后即可断开互联网。

下载中断可点“继续下载”，已下载部分会保留；未完成的包不会显示成可用离线地图。
框选下载区域不会更改或发送飞行目标。飞行目标仍需在目标表单中明确点击“执行目标”。
下载其他地区时切回在线地图，移动视野并另取地图名称即可。
地图操作不需要控制端在线；实时遥测与目标执行仍需要控制端的 DDS 连接。
地图管理默认收起，顶部保留在线/离线/下载进度或异常状态；再次点击或按“收起”/Esc 关闭面板。
右侧地理目标操作保持原样。控制阶段使用大字号，跟踪为蓝色、收敛为紫色、稳定悬停为绿色，
未确认状态为橙色；状态与警告每条单独一行。数据新鲜度保留。
左下角位置文字每 200 ms 检查一次（5 Hz），文字不变时不重绘；飞机标记与轨迹仍按原速率更新，断流仍会标记过期。
进入 AUTO 后，控制器当前参考满足已有悬停判据时即显示“稳定悬停”，无需先执行在线目标；
尚未稳定时显示“AUTO 控制中”。该状态来自控制器的 `hover_stable`，不改变控制授权或悬停判据。
顶部连接状态区分浏览器更新延迟、HTTP 请求超时和连接失败；浏览器暂时卡顿不再直接报告“UI HTTP 中断”。
遥测数据过期仍单独提示。升级后需使用同次构建的消息，重启控制端、遥测转发和地图服务，再刷新浏览器。

2026-11-01 回归：旧版在浏览器主线程停顿 2.1 秒后误报 HTTP 中断，期间无请求失败；
修复后同样停顿不再误报，真实断连、请求超时和恢复测试通过。
位置文字实测最小更新间隔 199.7 ms，不变时不写入 DOM。
控制链路 3 项、地图服务 5 项检查通过；完整 SITL 验证首次 AUTO 悬停反馈、两个地理目标和 KILL 退出。
构建已通过。注入卡顿复现了误判机制，但未直接记录原交互浏览器的卡顿原因。
证据见 [HTTP / 悬停修复验证](test/results/map_ui_20261101/http_hover_fix/validation.json)。

“UI 状态已收到，位置无效”反复跳动的另一处原因：原点的时间戳刷新立即转发，
会短暂领先限频后的位置快照。遥测转发现对内容相同、仅时间戳前进的原点去重，
保留原始快照；坐标、参考系、有效性或会话/reset 变化仍立即转发，页面时间匹配检查保持不变。
2026-11-01 19:48 录包回放的 180 次不匹配中，179 次周期性误报消除；
剩余 1 次为结束时 reset 3→4、Z 原点失效的真实变化。6 项地图服务测试和浏览器回归通过。
此修复需重启控制端 `ground_station_telemetry.py`；刷新网页不会更新后台转发进程。
证据见 [原点刷新修复验证](test/results/map_ui_20261101/origin_refresh_fix/validation.json)。

使用步骤：保持仿真和模拟接收机运行，按原流程进入稳定 AUTO 悬停；
等待真实 FC 位置/原点，首次海拔自动取当前有效 MSL。点击地图或输入经纬高，检查目标后点击“执行目标”。
选点不发命令，清除待选只清草稿。观察全局最近请求、活动目标与移动→收敛→稳定悬停。
刷新恢复原点/活动目标，不发新目标；轨迹是当前页面最多 2400 点的显示历史，刷新后重新积累。

配套 Betaflight ELF 须含当前 FC 状态和地理原点协议。`run.py` 只复制所选 ELF，不编译飞控。
原点来自真实 FC/Gazebo 配置；缺 `/navigation/origin` 时排查固件/协议，不能添加示例假原点。
`test_sitl_go_to_position.py --state-source fc_fused --geographic` 是会自行启动仿真、发送目标并退出的
独立回归，不是 UI 常驻服务，不与手动 UI 发点混用。

### Gazebo 打印了 GPS，但页面仍显示“位置未知”

`[SITL] ... pos=(404635000,-795182500)` 打印的是 `gpsSol.llh` 原始 GPS，
单位为度乘 10^7，即 `40.463500,-79.518250`。它不是 `/fused_state` 的有效性证明。
当前 FC 水平融合在未解锁时不启用，XY 地理原点也未建立；即使原始 GPS 有 fix，
地图仍不能从该无效融合状态反算飞机位置。页面现在显示具体 FC 状态与 `readiness_reason`，
并区分“没收到遥测”和“收到状态但位置无效”，不会填入地图中心或假原点。

2026-11-01 18:49 的实际 SITL 录包中，6303 条 FC 健康消息均为 `flags=174 (0xAE)`：
GPS fix=1、armed=0、XY fusion=0、XY valid=0；地面转发的原点 `xy_valid=false`，
且没有 `/ground_station/authority` 消息。这说明当次缺少有效模拟接收机授权输入，
地面转发本身已有数据。按上文启动独立 `sim_rc.py`，再按 README 的仿真操纵流程运行；
仅启动 `run.py`、`launch.sh` 和地图，不会自动启动接收机或解锁。
这条解释不是要求实机为查看地图而解锁，也没有改变控制有效性判据。

### 仿真世界的地理位置

`run.py` 可从 `agi_ros2/config/simulation.yaml` 的 `world_origin` 读取
`latitude`、`longitude`（WGS84 度），以及可选的 `altitude_msl`（米）。
只修改生成的 Gazebo 世界副本的地理基准，模型本地出生姿态保持不变；
省略海拔时保留源世界的 elevation。没有 `world_origin` 时沿用源世界位置。
配置变更需要重启 `run.py`，地图中心不会改变模拟 GPS。
真实电脑位置应来自明确的坐标/定位来源，不把 IP 城市位置或离线地图中心当作电脑坐标。
当前默认已经按用户提供的“沈阳市南塔街114号”设置为该园区的地图参考点：
WGS84 `41.7615277,123.4411319`，来源为 OpenStreetMap / Nominatim
[园区要素 813298975](https://www.openstreetmap.org/way/813298975)。
[高德地址条目](https://ditu.amap.com/place/B001805366)用于核对门牌；未把高德坐标直接当作 WGS84。
此点代表园区，不是电脑所在房间的实测定位。未提供实测海拔，保留仿真世界地面海拔 0 m，
因此仿真 MSL 不能当作该地址的真实海拔。

修改后已运行隔离的完整 SITL→FC→ROS→地面转发→HTTP 验证：未启动模拟接收机时
复现 `flags=174` 与位置未知；启动接收机并仅对模拟飞控解锁后，`flags=255`，
页面 API 得到 `41.7615277,123.4411319`。没有启用 AUTO 或发送地理目标，结束时已上锁并退出测试进程。
随后 30 次采样中 29 次有有效位置，1 次为已有的原点快照时间领先状态检查，未放宽该检查。
证据：[SITL 位置恢复结果](test/results/map_ui_20261101/position/result.json)。
同时已下载并选中“南塔街114号周边”离线包：26 块瓦片，约 4.15 MB，覆盖新仿真坐标。

## 跨机 SITL / 地面电脑连接 CM5

在控制主机上运行现有控制链和转发节点；地面电脑运行 UI，地图始终留在地面电脑。
MAVLink 接收、FC adapter、control、output 核心节点仍共处同一 Linux 主机；
其 clock_id、主机时间、单调时钟证据校验均保持不变。

双方先检查当前终端环境，保留已有有效 domain 和 RMW：

```bash
printenv ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY
printenv ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
```

双方局域网须可互通，使用相同、对应当前目标系统的 `ROS_DOMAIN_ID`。
相同 domain 不保证发现成功；两端 ROS/RMW/消息须兼容，优先沿用已验证组合，不能承诺不同发行版任意互通。
当前 Humble 跨机终端如设置了 localhost-only，应在**该启动终端**运行：

```bash
export ROS_LOCALHOST_ONLY=0
```

Jazzy 等版本按实际 RMW/发现配置检查 `ROS_AUTOMATIC_DISCOVERY_RANGE` 和已有 peers，
不能设为仅本机/关闭发现后再期待自动连接。仅在已有网络发现确实不可用时配置所用 RMW 的最少 peers。
检查实际网卡、组播/单播路由、防火墙，不默认更改系统防火墙、持久环境、VPN 或发现服务器。
不沿用独立回归的隔离 `ROS_DOMAIN_ID` / localhost-only 环境。

在地面电脑检查真实数据，而不只看 ping/topic 列表：

```bash
ros2 topic info /ground_station/fused_state -v
ros2 topic echo /ground_station/fused_state --qos-reliability best_effort --once
ros2 topic echo /ground_station/navigation/origin --qos-reliability reliable --qos-durability transient_local --once
ros2 topic echo /ground_station/goal_status --qos-reliability reliable --qos-durability transient_local --once
```

原始高频 topic 只在控制主机本地检查，地面不要订阅原始流、全 topic 桥或全量录包，
否则这些订阅仍会使原始高频流跨网。UI 仅订阅 `/ground_station/*`，命令直接发布
`/go_to_geographic_position`，不会经遥测转发器重新入队。均使用相对 ROS 名称，可用正常 ROS namespace/remap。

若 Gazebo 在电脑而 FC 控制链在 CM5，按 README 将控制主机 `mavlink.device` 的 TCP 地址和
`bridge.host` 指向实际 SITL 主机。`sim_rc` 按当前同机时钟契约部署在控制主机，
不是为了 UI 拆散核心链。本次不自动更改这些地址或仿真配置。

实机既可用本轮一键启动，也可在 **CM5 终端**手动运行原命令（不要重复启动）：

```bash
python3 agi_ros2/scripts/launch_realtime.py mode:=hardware shadow_only:=false
```

CM5 按前文部署 hardware 监听服务，地面电脑启动 UI。ROS/监听以普通用户运行，
仅原实时入口的 true/chrt 使用明确授权的非交互 sudo。本批开发验证不执行真实硬件命令。

## 外发频率、事件与新鲜度

| 输出 topic 前缀 `/ground_station/` | 常规频率 / 语义 |
| --- | --- |
| `fused_state` | fast_rate_hz，默认 5 Hz，最新位置/速度/姿态及 FC 有效性 |
| `authority`, `health`, `output_status`, `computation_status` | slow_rate_hz，默认 1 Hz |
| `navigation/origin`, `goal_status` | 语义更新即时可靠发布，transient_local、depth 1 |
| `runtime/hardware/status` 或 `runtime/simulation/status` | slow_rate_hz 心跳，启动器状态变化即时发送 |

快慢组 BEST_EFFORT、KEEP_LAST(1)，控制端原 topic 频率和 QoS 不变。
普通状态的 ARM/AUTO/KILL、链路、输出故障计数和控制有效性变化同样等待慢组定时器，
不再绕过限频。默认最多约 1 s 才显示一次最新状态；期间的中间变化可能合并，不用于完整故障记录。
机载控制仍接收原始 topic，目标/暂停命令直达控制节点；显示降频不降低控制、估计或撤权频率。
每路仅发送更新版本；源停止后不重新发送最后一帧，改频率也不重新标记缓存。
原 header、frame_id、clock_id、会话/reset、采样时间原样保留。

已有部署可以先在相同 ROS 域下在线降频，无需停止控制链（旧版本仍有状态变化即时发送旁路）：

```bash
ros2 param set /ground_station_telemetry fast_rate_hz 5.0
ros2 param set /ground_station_telemetry slow_rate_hz 1.0
ros2 param get /ground_station_telemetry fast_rate_hz
ros2 param get /ground_station_telemetry slow_rate_hz
```

参数须为有限正数且周期可由 ROS 纳秒计时器表示。单次原子参数请求先验证全部值再替换计时器；
不重启飞行程序、不清空目标、不改变机载控制频率。源更新慢于配置时，实际发布随源更新。
参数修改只对当前监听进程有效。同步新版到 CM5 并按前文构建后，在落地、上锁且控制链停止时
重启 `ground_station_telemetry.py`（systemd 部署为 `sudo systemctl restart agi-ground-station.service`），
才会加载新的默认值和取消旁路的逻辑。监听重启会清理它拥有的控制链；仅刷新地面网页不生效。
检查旧启动命令是否显式覆盖了 fast_rate_hz / slow_rate_hz，删除旧 20/2 覆盖或改成 5/1。

带宽目标是稳态遥测低于 **1 Mb/s = 125 kB/s**，这里没有给整机网卡设置硬限速。
100 Hz 合成输入、普通状态持续抖动的本机 DDS 回归中，位置约 5.10 Hz，普通状态与运行心跳各约 0.98 Hz，
CDR 载荷合计约 **0.038 Mb/s**（5.10 s 窗口，计数边界误差约一个样本）。
这是测试消息的序列化载荷统计，不含 DDS 发现、协议头、可靠重传、其他订阅和原点/目标/启动事件突发。
2026-11-05 本机 DDS/HTTP 15/15、浏览器合成回归及构建安装通过；
[验证摘要](test/results/bandwidth_20261105/validation.json)记录实际频率与字节数。尚未在 CM5 部署或测量无线流量。
因此持续 5 Mb/s 不能仅凭默认镜像频率解释，需测量 CM5 实际网卡流量。
地面电脑用 `ros2 topic info /fused_state -v` 等查询原始 topic 的订阅节点，确认没有远端原始流录包/桥接；
此查询不订阅数据，不要在地面用 `ros2 topic hz /fused_state` 来排查，否则会新增原始高频流量。
还应确认同域没有重复回传实例。若降频后仍高，结合 CM5 抓包区分原始 DDS 数据、发现/重传和其他进程，
不能用镜像 topic 的载荷估算代替整机上行测量。

浏览器 HTTP 快照名义 20 Hz，每次只保留一个请求；网络/浏览器调度可能使实际刷新变慢。
低频显示数据只在新 ROS 回调时变更，接收年龄单独增长。浏览器刷新频率不等于无线外发频率；
无线流量降低发生在控制主机转发器。

每个 topic 使用电脑自己的 monotonic 接收时刻计算更新年龄；不将电脑时间与远端 steady time 相减，
也不要求电脑 boot_id 等于控制主机 clock_id。原点/目标是保留快照，不能用其接收时刻证明控制在线。
持续位置/控制遥测和会话用于标记确认状态；位置 1 s、慢组 3 s 未更新显示过期，
这是显示规则，低速配置可能显示过期，不参与飞行准入。HTTP 中断另行提示。
失联仅保留“最后报告/未确认”，不伪造拒绝、取消、稳定悬停。非有限遥测输出 JSON null，页面显示未知。

## 目标与坐标契约

`/goal_status` 是控制节点保留快照，转发为 `/ground_station/goal_status`。
最近请求含进程序号、来源、原目标、pending/accepted/rejected 和原因；
活动目标独立保留 ENU 端点、原会话/reset/generation 及阶段。拒绝 B 不覆盖活动 A；
到达后仍保留 A，撤权/故障/reset 按原控制清除同步更新。只有 tick 安装成功且本轮检查通过才 accepted。
阶段直接复用 `HardwarePilot` 原有 movingAt、速度 0.2 m/s 和位置误差 0.2 m 判据，没有新增准入阈值。
`trajectory_active` 不被当作正在移动或到达信号。

`POST /api/goal` 只接受有限的 latitude、longitude、altitude_msl，HTTP 202 仅表示发布调用已完成。
输入消息未增加 request_id；全局最近请求不能唯一关联本浏览器，也没有 exactly-once 保证。
仅显式点击发布一次，无自动补发、重试或应用队列。命令 QoS 为 RELIABLE、VOLATILE、KEEP_LAST(1)：
VOLATILE 限制晚加入者获取历史，不禁止 DDS 对既有未确认样本重传。
失联后结果未确认，可能延迟送达；不能把 HTTP 成功当控制接受，也不能保证绝不会延迟送达。

显示使用 `fc_geographic.h` 同一 float 表达式得到的 K=111319.49424743652 m/deg，
实际 longitude_scale，ENU 的 N→纬度、E→经度、U+原点 altitude→MSL。
反算检查 fc_fused、有效性、完整 FC reset/session、原点快照时间及 odom/MSL 契约；
不用 cos(latitude)、heading correction 或 ECEF 替代。比例为零时经度未知；合法的零坐标不作缺失处理。
会话/reset/映射变化清除旧轨迹关联。机头方向来自四元数 ENU yaw：heading=wrap360(90−yaw)，不使用速度方向。
相对原点高度不是离地高度，方向虚线不是完整规划路径。

## 暂停并悬停

地面电脑与控制主机均须重新构建并 source **同版本的 agi_ros2 消息**，然后重启各自节点；
本次 `GoalStatus` 新增 `bool pause_active`，只更新浏览器或 Python 文件不足以升级。
构建、仿真/接收机、遥测转发和 UI 的准确启动命令见本文开头。跨机沿用同一 domain、兼容 RMW
和既有网络配置；暂停命令直达控制节点，不经过遥测定时器。

在稳定 AUTO 中执行目标，转向、平移、升降、终点收敛或已悬停时均可点击橙色“暂停并悬停”。
按钮不依赖目标输入、地图、地理原点或 `hover_stable`，只在自己的 HTTP 请求在途时禁用。
暂停取消旧目标，刹停后持续保持终点及朝向，不自动恢复旧在线目标或底层 CSV。
经纬高与待选点不被暂停改写；实际稳定后再次明确点击“执行目标”，按原有准入规则从新悬停点规划。
暂停不调用 off/kill、不重新 warmup 或进入 AUTO；原授权、状态、围栏、遥控接管与故障逻辑仍有效。

接口：`POST /api/pause`，JSON `{}`，每次有效请求只发布一次。
HTTP 202 的“暂停请求已发送，等待控制端处理”只说明发布调用完成。
网络超时显示结果未确认，无自动重试/重连补发。终端调试使用：

```bash
ros2 topic pub --once /pause_hover std_msgs/msg/Empty '{}'
```

topic 为相对名称 `pause_hover`，RELIABLE、VOLATILE、KEEP_LAST(1)；如使用 `/quad1` namespace，
替换为 `/quad1/pause_hover`，UI、转发和控制节点也应使用对应 namespace。
不同 DDS topic 不提供全局顺序；本实现保证控制端收到后的暂停优先级和无应用层自动重放，
不承诺任意网络延迟下的跨 topic 排序或 exactly-once 交付。

`GoalStatus.request_source=pause_hover`，最近请求仍为 PENDING / ACCEPTED / REJECTED；
只有控制 tick 安装参考并通过该周期检查后才 ACCEPTED。暂停清掉尚未执行的普通目标；
暂停 pending 时拒绝新普通目标，不排队等候。健康 idle 保留暂停到下一有效 FC 控制周期，
撤权/AUTO 退出/disarm/kill/reset/输出故障按原路径清理。
重复 pending 暂停合并为最近序号；已安装后的重复暂停不重规划、不延长时长、不移动停止点，
活动请求沿用首次安装的编号，最近请求正常回报幂等接受。被拒绝的新目标不清除暂停标识。
最近请求 B 被拒绝后，较早 pending 暂停 P 的执行结果只更新活动参考，不覆盖 B 的请求字段。

`pause_active` 独立于最近请求来源；`active_enu` 是实际安装的刹停终点。
MOVING / SETTLING / STABLE 分别显示“减速中 / 悬停收敛中 / 已暂停，稳定悬停”，地图 marker 显示“悬停点”。
段结束用真实控制时刻判断，MPC 未来采样不会提前改变阶段；实际稳定仍要求速度 ≤0.2 m/s、位置误差 ≤0.2 m。
同阶段内暂停标识、活动编号/停止点变化也发布新 revision，现有 relay 即时转发保留状态。
刷新只恢复显示，状态过期或会话不匹配仍显示未确认。

实现复用同一个 `CapturedReference`、`MinSnapTrajectory`、相对时间和末端 hover。
从切换时当前活动参考取 p/v/a/j，使用原 `pilot.brake_deceleration`（配置未变）：
`T=max(1.5, 1.875*|v0|/brake_deceleration)`，
`p_stop=p0+0.5*T*v0+0.1*T²*a0+(T³/120)*j0`，末端 v/a/j=0。
不对运动起点清零导数，不通过 scale 改坏起点边界；候选求解、有限采样和边界校验通过后才替换。
固定位置参考保留已有位置和朝向。停止点通常不等于点击位置。
heading 从既有 tilt×heading 组合中取回；局部标量 yaw 多项式保留未包裹角度，
终止相对角为 `0.5*yaw_rate0*T`，heading/yaw rate 连续，末端 yaw rate=0。

限制：该减速度参数用于时长设计，非零起始 a/j 下不保证严格峰值加速度上限或速度立即单调下降。
yaw acceleration 起点按现有接口取零，不保证任意起始 yaw acceleration 连续。
有限采样与端点围栏检查不等于全路径围栏、避障或形式化动力学保证；失败拒绝并保留仍有效的原参考。
未连接或控制真实飞机，未完成实机飞行、CM5 实际运行或跨机断网延迟交付验证。

相关验证命令（独立测试终端、空闲 domain/仿真端口，不与真实系统或交互仿真混用）：

```bash
source /opt/ros/humble/setup.bash
source install/agi_ros2/local_setup.bash
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
cmake --build agilib/build --target tests -j 3
agilib/build/tests --gtest_filter='CapturedOnlineReference.*:HardwareGeo.*'
python3 agi_ros2/tests/test_map_ui.py
python3 agi_ros2/tests/test_fc_control_delivery.py \
  DeliveryTests.test_pause_priority_idle_sequence_and_reset \
  DeliveryTests.test_online_goals_on_fc_control_thread \
  DeliveryTests.test_geographic_goals_share_local_chain_and_reset_cleanup -v
ctest --test-dir build/agi_ros2 --output-on-failure
# 仅测试需要 Playwright 与 Chrome；输出目录必须不存在。
python3 agi_ros2/tests/test_map_ui_browser.py --browser /opt/google/chrome/chrome \
  --output /tmp/agi_pause_browser_check
GZ_PARTITION=agi_pause_hover_check python3 agi_ros2/tests/test_sitl_go_to_position.py \
  --state-source fc_fused --geographic --map-ui --pause-hover --output /tmp/agi_pause_sitl_check
```

`--pause-hover` 使用实际浏览器按钮、地面后台、ControlNode→HardwarePilot→CapturedReference→MPC→output
和模拟接收机，分别在平移和转向中暂停，等到原目标预计结束之后仍保持，再明确点击新目标。
保留 result.json、CSV、rosbag、日志与截图，最后执行原有 KILL 撤权检查。
测试的浏览器只访问本机，地图底图不作为本次验证条件。

本机 2026-11-02 最终验证：ROS2/Gazebo ON 构建、C++ 14/14、DDS/HTTP/地图 7/7、真实 control_node 3/3、
ctest 2/2 和 Chrome 浏览器检查均通过。修改范围 clang-format、Tab/140 列、命名/空白与 Python/JS 语法检查通过。
浏览器合成测试验证一次点击一次 Empty、空目标也可暂停、移动时可用、5 s HTTP 超时不重试、
原输入保留、刷新不重发、三个暂停阶段、既有 marker tooltip 更新以及源停流后的未确认样式。
快慢组频率回归保持 20/2 Hz；生产 relay 与 YAML 没有修改。

最终真实 FC-fused SITL 的 AUTO/输出连续有效约 **128.67 s**，最大求解 **2.346 ms**、控制周期 **2.436 ms**。
平移暂停时实测速度 **0.608→0.012 m/s**；停止点比点击前参考位置前移约 **0.618 m**。
转向暂停前参考 yaw rate **0.201 rad/s**，末端参考 yaw rate=0。
两次都等待超过旧目标预计结束时刻，随后浏览器明确点击新目标成功；最终 KILL 撤权和活动状态清理通过。
两次暂停最后 2 s 的仿真真值最大位置误差分别 **0.0486 m / 0.0419 m**，速度最大 **0.0764 / 0.0759 m/s**。
实测稳定不等同于计时器结束，合成测试单独覆盖 SETTLING 和实际速度/位置误差未达标。

中间复测修正了两处测试观察方式：等待 FC 待合并样本真正计算后才断言 healthy idle；
用保留型 GoalStatus 判断接受，避免漏收单周期 BEST_EFFORT 文本误报超时。未放宽控制门限。
摘要、实际命令和日志见 [validation.json](test/results/pause_hover_20261102/validation.json)、
[SITL 结果](test/results/pause_hover_20261102/sitl.json)、
[平移暂停截图](test/results/pause_hover_20261102/pause_translation.png)、
[转向暂停截图](test/results/pause_hover_20261102/pause_yaw.png)。
原始 CSV、rosbag 与运行日志在 `/tmp/agi_pause_sitl_verified`；结果目录按现有规则被 Git 忽略，未提交或推送。
测试专用 Playwright 安装在 `/tmp/agi_pause_qa`，本次浏览器命令额外使用 `PYTHONPATH=/tmp/agi_pause_qa:$PYTHONPATH`；
未增加产品运行依赖，也未执行实机启动/飞行验证。

## 地图保存位置与离线范围

默认保存在 `~/.local/share/agi_ros2/maps/`，本机即
`/home/sia/.local/share/agi_ros2/maps/`。页面直接显示实际路径，无需手动选文件。
`online.mbtiles` 是在线浏览缓存；各区域使用独立文件，`selection.json` 保存当前选择。
重新打开页面或重启 UI 后保留所选离线地图。可用 `--map-dir /absolute/path/maps` 指定其他目录。
Windows 11 使用 WSL 运行 UI 时，文件保存在该 WSL 用户的 Linux 主目录，
可从资源管理器 `\\wsl.localhost\Ubuntu-22.04\home\<用户名>\.local\share\agi_ros2\maps` 访问
（发行版名以 `wsl -l -v` 输出为准）。地图不需要复制到控制端。

在线缓存不等于完整离线地图。明确下载会保存所选区域从概览到指定细节的全部瓦片，
以及对应的样式、图标；中文等文字使用本机字体，不依赖外部字体服务器。
离线包固定数据版本，缺失瓦片返回缺失提示，不联网补图；区域外或超出所选细节没有额外信息。
“街道与建筑”保存至原生 14 级，仍可继续放大显示。底图范围不构成飞行围栏，地图不提供地形高度。

采用 [OpenFreeMap](https://openfreemap.org/) 的道路矢量底图，使用
[其公布的离线下载政策](https://openfreemap.org/tos/)：本实现单包最多 3000 瓦片且不超过
50 MB 资源数据，最多 3 个并发网络请求，并发送专用 User-Agent。
超额时应缩小区域或降低细节；需要更大规模时按提供方要求联系授权，不能以拆包绕过限制。
服务端需能联网访问 `tiles.openfreemap.org`；没有配置 Google 瓦片或普通 OSM 公共瓦片批量抓取。

## 兼容已有栅格包

将允许离线使用的道路栅格 MBTiles 存在电脑任意可读绝对路径，通过 `--mbtiles` 指定。
支持 metadata.format 为 png/jpg/jpeg/webp；拒绝 PBF。数据库以只读 URI 打开，
XYZ 的 y 转为 TMS `2^z-1-y`，以实际图片签名返回 MIME。缺包/瓦片明示错误，不切在线源。
包名称、bounds、minzoom/maxzoom、attribution 按原元数据显示，缺失明确标识未提供。
署名按文本显示，不执行包内 HTML。

使用 `ros2 run agi_ros2 map_ui.py --mbtiles /absolute/path/area.mbtiles` 启动兼容模式，
此时不启用在线地图和下载面板。外部 PBF 包不走此栅格入口；应用自己下载的矢量包通过页面选择。

全部 JS/CSS 本地安装：Leaflet 1.9.4、MapLibre GL JS 5.13.0、Leaflet 适配器 0.1.3，
许可证和来源在 `web/leaflet/`、`web/maplibre/`。矢量底图需要浏览器 WebGL 支持和本机中文字体。
包须覆盖实际 FC/Gazebo 区域；无需上传 CM5 或纳入 Git。
断互联网不影响本地底图；断控制网络时底图仍可浏览，遥测会过期。

参考：[Leaflet](https://leafletjs.com/examples/quick-start/)、
[MBTiles 1.3](https://github.com/mapbox/mbtiles-spec/blob/master/1.3/spec.md)、
[ROS QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)、
[Jazzy 发现配置](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html)、
[OSM 瓦片政策](https://operations.osmfoundation.org/policies/tiles/)。

## 简化地图下载验证（2026-11-01）

本轮仅修改地图后台、页面及安装/说明，未修改控制逻辑。
构建通过；地图存储测试 6/6、原 DDS/HTTP/坐标测试 4/4、原栅格浏览器回归通过。
覆盖区域计数/额度、失败与暂停续传、进程中断恢复、TMS 行号、错误瓦片拒绝、
离线资源缺失不联网，以及离线选择持久化。

实际从 OpenFreeMap 下载“沈阳中心（验证区）”：WGS84 边界
`123.41,41.79,123.45,41.82`，0–14 级共 23 瓦片，资源数据 3,987,798 字节。
保存在本机默认地图目录并已选择离线模式。这是中心一小块区域，不是整个沈阳市。
重启后台、禁止后台网络读取及浏览器外部请求后，道路、中文文字和图标可显示；
实际浏览器放大后可见 1831 个渲染要素，无失败 HTTP 请求或页面 JS 异常。

另用该真实地图的副本作为固定夹具，浏览器执行框选→下载→离线选择→重启→刷新→放大：
139 次浏览器请求均为本机，后端外网请求 0，飞行目标发送 0，页面异常/失败 HTTP 请求均为 0。
该流程验证包含下载操作和持久化，但夹具测试自身不访问地图提供方。
Headless Chrome 使用软件 WebGL；上游样式有属性值为空的控制台提示，未阻止已验证区域显示。
未验证 Windows/WSL 的实际运行、全部城市覆盖或硬件飞行。

检查脚本：

```bash
python3 agi_ros2/tests/test_map_store.py
# 下列浏览器检查需 source ROS/项目环境，并已安装测试依赖 Playwright、Chrome。
python3 agi_ros2/tests/test_map_download_browser.py \
  --fixture "$HOME/.local/share/agi_ros2/maps/820c0b58d1aa41e991b846a5fc03e3fc.mbtiles" \
  --output /tmp/map-download-browser-check
```

证据：[下载操作与重启测试](test/results/map_ui_20261101/download/flow-result.json)、
[真实离线地图截图](test/results/map_ui_20261101/download/shenyang-offline.png)。

## 第一阶段控制与 UI 验证（2026-11-01）

本机 Humble / `rmw_fastrtps_cpp`。构建通过（Gazebo ON、FC timing tests ON），
C++ ctest 2/2、原 HardwareGeo/在线轨迹测试 9/9、真实 control_node 的本地/地理目标回归 2/2、
DDS/HTTP/地图/坐标测试 4/4 通过。改动范围 clang-format、Tab/140 列和 diff 空白检查通过。

合成 100 Hz 源下，3.1 秒计数窗口实测：

| 参数 | fused_state 外发 | health / authority 外发 | 原始源 |
| --- | --- | --- | --- |
| 20 / 2 Hz | 20.00 Hz | 1.94 Hz | 99.98 Hz |
| 10 / 1 Hz | 10.00 Hz | 0.97 Hz | 99.97 Hz |

计数窗口边界有约一个样本误差。参数原子失败不部分生效、关键事件及时发送、
源停流不重发旧 payload、会话/reset/时间契约、非有限 JSON、TMS/XYZ 与图片 MIME 均已检查。
真实 control_node 测试覆盖地理转换拒绝、入队拒绝、接受、拒绝 B 保留 A、
收敛/稳定阶段、晚订阅和 reset 清理。未修改控制门限、原始频率或规划公式。

Chrome/Playwright 使用**合成单色 MBTiles**与合成 ROS 消息：262 次请求均为 localhost，
无页面 JS 错误；地图点击/输入、保留用户海拔、一次点击一次命令、刷新/关闭无重发、
源停流过期显示、仅本地资源和窄屏布局通过。所有外部 URL 在浏览器测试中被拦截。
这验证离线加载实现，不代表真实道路地图或覆盖范围已验收。

真实 fc_fused SITL 通过 **HTTP `/api/goal`** 执行两个地理目标，AUTO 连续约 68.72 秒；
每个目标均验证至少 15 秒稳定悬停。最终统计窗口 FC 位置误差最大约 0.0512 m，
仿真真值误差最大约 0.0544 m；最大求解约 2.015 ms、控制周期约 2.031 ms。
拒绝 B 保留 A、HTTP 快照恢复、KILL 后活动目标清除通过。
首个目标段末已稳定，阶段为 moving→stable；第二个为 moving→settling→stable，
符合原判据，不强制插入收敛阶段。浏览器点击测试与真实 SITL HTTP 测试分别进行，未声称浏览器操纵实机。

未验证：CM5/跨机真实 DDS、真实断网后 RELIABLE 待确认样本的重传/延迟送达、真实区域地图、硬件飞行。
本机验证包含 late-join VOLATILE 不接收历史命令，以及控制进程短暂停顿后 reset 清除排队目标；
它们不替代跨机断网测试。原始 SITL rosbag/log 留在 `/tmp/agi_map_sitl_http_20261101b`。
该结果目录沿用现有 `.gitignore` 规则，为本地验证产物；版本纳入由用户决定。
安装入口的页面/资源/API 返回 200，真实 `ros2 param set` 修改 10/1 并恢复 20/2 通过。
摘要和小型证据在 [validation.json](test/results/map_ui_20261101/validation.json)、
[SITL 结果](test/results/map_ui_20261101/sitl_http.json)、
[合成浏览器截图](test/results/map_ui_20261101/browser_desktop.png)。

可复现的直接检查（使用未占用的测试 domain，勿与交互仿真混用）：

```bash
source /opt/ros/humble/setup.bash
source install/agi_ros2/local_setup.bash
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 agi_ros2/tests/test_map_ui.py
python3 agi_ros2/tests/test_fc_control_delivery.py \
  DeliveryTests.test_online_goals_on_fc_control_thread \
  DeliveryTests.test_geographic_goals_share_local_chain_and_reset_cleanup -v
ctest --test-dir build/agi_ros2 --output-on-failure
# 会自行启动模拟器、模拟接收机并发送目标；输出目录必须不存在，SITL 端口必须空闲。
GZ_PARTITION=agi_map_ui_check python3 agi_ros2/tests/test_sitl_go_to_position.py \
  --state-source fc_fused --geographic --map-ui --output /tmp/agi_map_ui_check
# 可选：有 Playwright 和 Chrome 的测试环境；它们不是 UI 运行依赖。
python3 agi_ros2/tests/test_map_ui_browser.py --browser /opt/google/chrome/chrome \
  --output /tmp/agi_map_browser_check
```

这些环境变量只作用于该回归终端；跨机使用请回到原有 domain/网络配置的终端。
