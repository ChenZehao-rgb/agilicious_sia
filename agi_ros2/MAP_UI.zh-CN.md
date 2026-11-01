# 地图地面站：在线准备与离线使用

地图包、网页、Python HTTP 后台都在**地面电脑**。CM5/实际控制主机只增加普通优先级、
只读的 `ground_station_telemetry`。浏览器访问本机 `http://127.0.0.1:8080`；
地面电脑的 rclpy 经 ROS 2 / DDS 接收镜像遥测、发布既有 `GeographicGoal`。
因此地面电脑需要 ROS 2 和匹配的项目消息，不是只需浏览器。

未实现暂停、实机启动按钮、ARM、起飞/降落或 AUTO 切换。关闭网页、后台退出、地面网络断开
不主动取消目标或撤权；控制主机仍按原保护逻辑运行。不引入地面站心跳门控。

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

新增终端 4，在**实际控制主机**运行（同机 SITL 即本电脑）：

```bash
source /opt/ros/humble/setup.bash
source install/agi_ros2/local_setup.bash
ros2 run agi_ros2 ground_station_telemetry.py --ros-args -p fast_rate_hz:=20.0 -p slow_rate_hz:=2.0
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
会短暂领先限频到 20 Hz 的位置快照。遥测转发现对内容相同、仅时间戳前进的原点去重，
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
`/go_to_geographic_position`，不会经只读转发器重新入队。均使用相对 ROS 名称，可用正常 ROS namespace/remap。

若 Gazebo 在电脑而 FC 控制链在 CM5，按 README 将控制主机 `mavlink.device` 的 TCP 地址和
`bridge.host` 指向实际 SITL 主机。`sim_rc` 按当前同机时钟契约部署在控制主机，
不是为了 UI 拆散核心链。本次不自动更改这些地址或仿真配置。

实机仍由用户在 **CM5 终端**手动运行原命令：

```bash
python3 agi_ros2/scripts/launch_realtime.py mode:=hardware shadow_only:=false
```

另开 CM5 终端启动只读转发，电脑启动 UI。后台普通用户运行，不使用 sudo，不新增 systemd、
远程启动 API 或自动服务。本批开发验证不执行上述硬件命令。

## 外发频率、事件与新鲜度

| 输出 topic 前缀 `/ground_station/` | 常规频率 / 语义 |
| --- | --- |
| `fused_state` | fast_rate_hz，默认 20 Hz，最新位置/速度/姿态及 FC 有效性 |
| `authority`, `health`, `output_status`, `computation_status` | slow_rate_hz，默认 2 Hz |
| `navigation/origin`, `goal_status` | 语义更新即时可靠发布，transient_local、depth 1 |

快慢组 BEST_EFFORT、KEEP_LAST(1)，控制端原 topic 频率和 QoS 不变。
ARM/AUTO/KILL、链路、输出故障计数和控制有效性等有意义的离散变化即时额外发送，
不把时间戳、header、age、电压抖动或 reason 文本变化当事件。因此 20/2 是常规上限，非事件流硬上限。
每路仅发送更新版本，事件发送也标记版本；源停止后不重新发送最后一帧，改频率也不重新标记缓存。
原 header、frame_id、clock_id、会话/reset、采样时间原样保留。

运行中修改和恢复：

```bash
ros2 param set /ground_station_telemetry fast_rate_hz 10.0
ros2 param set /ground_station_telemetry slow_rate_hz 1.0
ros2 param get /ground_station_telemetry fast_rate_hz
ros2 param get /ground_station_telemetry slow_rate_hz
# 恢复默认
ros2 param set /ground_station_telemetry fast_rate_hz 20.0
ros2 param set /ground_station_telemetry slow_rate_hz 2.0
```

参数须为有限正数且周期可由 ROS 纳秒计时器表示。单次原子参数请求先验证全部值再替换计时器；
不重启飞行程序、不清空目标、不改变机载控制频率。源更新慢于配置时，实际发布随源更新。

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
