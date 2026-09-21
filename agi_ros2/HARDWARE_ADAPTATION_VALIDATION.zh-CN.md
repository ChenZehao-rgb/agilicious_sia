# CM5 / Betaflight 悬停适配：软件验证记录

本记录对应 2026-09-22 工作区改动。测试主机是当前 x86 Linux / ROS 2 Humble 开发机；
没有访问实体 UART、刷写实体飞控、在 CM5 上测量或进行实际飞行。
实机模型和标定数据仍待填写，`hardware.yaml` 默认 shadow，并保留拒绝飞行的未配置值。

## MPC/GEO 选择适配：本轮新增验证

同一 `simulation.yaml` / `hardware.yaml` 内保存 `parameter_sets.MPC/GEO`；默认仍为 MPC，
启动参数 `controller:=GEO` 选择对应参数和 GEO 最小模型校验。未增加第三份运行配置。
硬件默认 shadow，真实机体数据仍未填写。本轮没有访问实体串口或刷写飞控。

| 检查 | 本轮结果 |
|---|---|
| 核心与 ROS2 Release 构建、安装 | 通过；包括新诊断消息、MPC/GEO 参数选择和最小模型 |
| 核心 GTest | 28 项通过：GEO 控制 10、GEO 接管/故障/影子/显式轨迹 4、参数加载 9、采样 3、Pilot 2 |
| 旧硬件控制测试程序 | `hardware_pilot_test`、`betaflight_hw_test` 通过，保留真实 MPC 路径与门限 |
| 配置/launch/模拟器参数选择 | 16 项通过：默认与覆盖、对应参数组、缺失/冲突拒绝、最小模型、三/六节点 |
| 真实 ROS 核心进程 | 3 项通过：MPC/GEO 空轨迹接管及 KILL、控制/输出禁止运行中切换、缺少参数不得用默认增益启动 |
| 双 PTY 六节点 | 7 项通过：MPC/GEO 普通 GNSS 接管与故障恢复，两者 shadow 均不发 MSP 200，配置/导航/ACK 故障撤权 |
| 录包摘要 | 1 项通过：GEO controller_type/controller_success、计算耗时及原始证据统计；保留弃用 mpc 别名 |
| Gazebo＋Betaflight SITL，GEO | 独立分区复测通过，400 个授权命令，KILL 后输出关闭；未放宽时效检查 |
| Gazebo＋Betaflight SITL，MPC | 通过，402 个授权命令，控制器类型确认 MPC，KILL 后输出关闭 |

GEO 第一次 Gazebo 运行因 TCP 5761 在原有 45 秒启动期限内未就绪而失败，尚无融合/控制数据；
Gazebo 服务日志显示世界与插件加载，但没有定位固件未进入 TCP 就绪的根因。
更换独立分区、启用无缓冲日志后，相同代码复测通过。这与前次验证中出现的启动问题相似，
不能据此声称已修复模拟器启动可靠性。首次记录 `/tmp/agi_geo_sitl_smoke/result.json`，
复测记录 `/tmp/agi_geo_sitl_retry/result.json`；MPC 记录 `/tmp/agi_mpc_selection_sitl/result.json`。

相关日志：`/tmp/agi_geo_core_tests.log`、`/tmp/agi_geo_standalone_tests.log`、
`/tmp/agi_geo_profiles.log`、`/tmp/agi_geo_profile_nodes.log`、`/tmp/agi_geo_hardware_pipeline.log`、
`/tmp/agi_geo_ros_build.log`、`/tmp/agi_geo_bag.log`。日志为本机临时产物，复现命令如下：

```bash
# 先按 README 构建并 source ROS 与本工作区。
export ROS_LOG_DIR=/tmp/agi_geo_ros_logs
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=91
python3 agi_ros2/test/test_runtime_config.py
python3 agi_ros2/test/test_runtime_profile_nodes.py
python3 agi_ros2/test/test_hardware_pipeline.py
python3 agi_ros2/test/test_shadow_bag.py
build/betaflight_sitl/agilib/tests \
  --gtest_filter='GeometricController.*:HardwareGeo.*:RuntimeConfig.*:TimeBasedSampler.*:Pilot.*'
GZ_PARTITION=agi_geo_check python3 agi_ros2/test/test_sitl_smoke.py --controller GEO
GZ_PARTITION=agi_mpc_check python3 agi_ros2/test/test_sitl_smoke.py --controller MPC
```

测试只使用合成数据、伪串口与本机仿真。ROS 进程测试需要 DDS/回环套接字权限；
受限沙箱首次尝试因不允许 socket 而未执行，获准本机通信后通过。
重新配置 CMake 后收集了新增测试；沿用已有构建树直接 CMake 构建，不声称重新验证了 colcon 干净构建。
找到并使用 clang-format 23.1.1，GEO 实现、相关头文件与新增测试完整格式检查通过；
其余历史文件检查本次修改行，Tab 展开后均不超过140列，`git diff --check` 和 Python 语法检查通过。
未运行 clang-tidy。新消息要求仿真机和伴随计算机统一重编译。

以上不证明实机悬停精度、CM5 完整周期或真实飞控回退时间；GEO 初始增益仍需实机调试。
当前 GEO 只验证低速非倒飞控制范围，未引入角速度前馈、位置积分或 RPM 气动补偿。

## 此前完整悬停适配检查（保留记录）

| 检查 | 结果与范围 |
|---|---|
| ROS 2 CMake Release 构建、安装 | 通过；包含新增消息接口、融合/控制/输出、MAVLink/MSP 和 Gazebo 适配器 |
| Agilib 相关 GTest | 23 项通过：内嵌参数/缺失模型拒绝、EKF 延迟更新/异常拒绝/缓存、采样末端、Pilot 和 YAML 空路径 |
| `hardware_pilot_test`、`betaflight_hw_test` | 两个测试程序通过：真实 MPC、接管边沿/位置与 yaw 捕获、故障恢复、相对 CSV、输出门限和 watchdog 关闭 |
| 静止 IMU 初始化 GTest | 3 项通过：完整窗口、gyro bias/倾角、运动/重力异常、时间间断与重置 |
| 配置与 launch | 8 项通过；两份配置、三/六节点组装、缺失实机模型、同一串口别名、新旧配置冲突 |
| 导航/MSP 纯逻辑 | 14 项通过；未知精度不授权、原点、配置/模式/AUX、异步 RC/STATUS 配对、新旧配置冲突和固定 50 ms 策略 |
| 真实三节点 + 默认仿真配置 | 1 项通过；空轨迹启动、50 次预热、AUTO 悬停参考、KILL 清除参考并撤销 |
| 六节点、双 PTY、真实 MAVLink/MSP 解码 | 5 项通过；普通 GNSS + MPC 发出 MSP 200，shadow 不发 200，错误 timeout/profile/模式拒绝，错误 ACK 锁闭 |
| 原始故障原因保留 | 最终错误 ACK 用例单独复测通过；`last_fault` 保留具体通信故障，不被通用撤销状态覆盖 |
| 显式导航失效 | 上述双 PTY 测试覆盖航向未知、坏 fix、无效速度；80 ms 内撤销，且不刷新最近接受导航时间 |
| 原三节点进程回归 | 10 项通过；传感器/控制停流、输出重启时 AUTO high、时钟暂停/回退、延迟实验和 PTY KILL |
| GNSS 融合 / shadow 进程 | 5 项通过；参数实际生效、静止初始化、精度准入、NIS 离群拒绝/恢复、连续合格次数发布/清零、显式撤权与会话重置 |
| MAVLink PTY | 10 项通过；单位/坐标转换、观测配对、时钟对齐/失效、会话重启、精度传播与显式坏观测 |
| MSP 独立 PTY 与录包 | 通过；测试设定的控制发送约 100 Hz，STATUS_EX 5 Hz、姿态 20 Hz；这不是实机默认遥测频率测量 |
| Betaflight 单测 | 23 项通过：RX 2、PG 3、IMU 18；覆盖 50/300 ms、计时回绕、短帧、旧 v4 数据和新字段保存恢复、无效磁航向 |
| Betaflight SITL 固件完整构建 | `make TARGET=SITL -j4` 通过；没有编译尚未指定型号的实体飞控 target |
| Gazebo + Betaflight SITL + 三节点 | 最终 smoke 通过；严格时效、默认空轨迹，401 个授权命令，输出曾激活，KILL 后输出关闭 |

Gazebo smoke 检查闭环启动、MPC 接管与撤销，没有建立悬停误差或长时间稳定性的验收结论。
本轮一次运行因 Betaloop 在 45 秒内未等到 TCP 5761 而失败（没有传感器输出）；
换用独立 `GZ_PARTITION` 后相同代码、默认保护参数复测通过，没有通过延长控制时限绕过问题。
尚未确定这次模拟器启动失败的根因，不把重试成功描述成启动可靠性问题已经解决。
最终原始记录为 `/tmp/agi_sitl_final_retry/result.json`，脚本是 [test_sitl_smoke.py](test/test_sitl_smoke.py)。

其他本地日志：`/tmp/agi_final_ros_build.log`、`/tmp/agi_final_core_unit.log`、
`/tmp/agi_final_node_pipeline.log`、`/tmp/agi_final_hardware_pipeline.log`、
`/tmp/agi_final_profile_nodes.log`、`/tmp/agi_final_shadow_pipeline.log`、
`/tmp/agi_final_fault_reason.log`、`/tmp/agi-betaflight-hardware-tests.log`。
这些是当前机器的临时日志；上述结果和复现入口保存在本文件中。

## 复现入口

先在仓库根目录 source ROS 及本工作区安装环境。ROS 进程测试需要 DDS 和回环 UDP 权限；
在受限运行环境中，将 `ROS_LOG_DIR` 指向可写目录，并允许本地 socket。
下面的进程测试应串行运行；同时构建会影响严格时效测试。

```bash
source /opt/ros/humble/setup.bash
source install/agi_ros2/local_setup.bash
export ROS_LOG_DIR=/tmp/agi_hardware_validation_ros
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=91
python3 agi_ros2/test/test_runtime_config.py
python3 agi_ros2/test/test_shadow_support.py
python3 agi_ros2/test/test_runtime_profile_nodes.py
python3 agi_ros2/test/test_node_pipeline.py
python3 agi_ros2/test/test_hardware_pipeline.py
python3 agi_ros2/test/test_shadow_pipeline.py
python3 agi_ros2/test/test_mavlink_sensor.py
python3 agi_ros2/test/test_msp_node.py
GZ_PARTITION=agi_hover_smoke python3 agi_ros2/test/test_sitl_smoke.py
```

本次环境的 `build.sh` / colcon 调用未正常完成，停止后用已有构建树直接 CMake 构建、安装通过。
轻量诊断在 colcon 捕获空前缀脚本的 `env -0` 输出时偶发复现等待退出通知阻塞，
尚未执行 CMake；`colcon list` 与同步执行相同环境捕获命令正常。没有为此改动项目构建逻辑。
因此不把本次直接 CMake 结果表述为 `build.sh` 或干净环境安装已通过。使用的核心命令：

```bash
export CCACHE_DIR="$PWD/build/ccache"
export CCACHE_TEMPDIR="$PWD/build/ccache/tmp"
cmake -S agi_ros2 -B build/agi_ros2 -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/install/agi_ros2" \
  -DACADOS_ROOT="$PWD/agilib/externals/acados-src" -DFETCH_ACADOS=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DAGI_ROS2_GAZEBO=ON
cmake --build build/agi_ros2 -j2
cmake --install build/agi_ros2
cmake --build build/betaflight_sitl/agilib --target tests hardware_pilot_test betaflight_hw_test -j2
ctest --test-dir build/betaflight_sitl/agilib \
  -R '^(hardware_pilot_test|betaflight_hw_test)$' --output-on-failure
build/betaflight_sitl/agilib/tests \
  --gtest_filter='RuntimeConfig.*:EkfImuRtk.*:TimeBasedSampler.*:Pilot.*:Yaml.*'
g++ -std=c++17 -O1 -fno-finite-math-only \
  -Iagilib/include -Iagi_ros2/include -I/usr/include/eigen3 \
  agi_ros2/test/test_imu_initialization.cpp \
  -lgtest_main -lgtest -lpthread -o /tmp/agi_imu_initialization_tests
/tmp/agi_imu_initialization_tests
make -C /home/sia/betaflight/src/test \
  test_rx_msp_override_unittest test_pg_unittest test_flight_imu_unittest \
  CC=gcc CXX=g++ OBJECT_DIR=/tmp/agi-betaflight-hardware-tests -j4
```

Betaflight 测试使用 GCC；当前默认 Clang 环境缺少 `cstddef`，没有把该工具链错误当作测试通过。
重复生成覆盖率文件曾提示旧 gcov 时间戳不匹配，断言与测试进程均通过。

## 格式与变更边界

找到了环境中的 clang-format 23.1.1：
`/tmp/agi_shadow_format/clang_format/data/bin/clang-format`。
新增 C++ 文件的完整格式检查通过；新增/修改行按 Tab 展开后没有超过 140 列，
Python 变更通过语法编译，两个仓库的 `git diff --check` 通过。

历史 Agilib 部分函数仍保留未修改的两空格缩进。clang-format 的局部行模式会沿用其上下文，
建议把某些新增 Tab 改回空格；这些冲突建议没有采用，新增行遵循 AGENTS.md 手工核对。
没有为让全文件 formatter 归零而批量重排旧实现，也没有运行 clang-tidy 命名检查。

## 必须在目标机和实机补完的验证

- 填写真实质量、惯量、电机位置/模型/限制、ACTUAL rates、推力 CSV、精度/协方差门限和 ENU 围栏。
  格式和单位见 [README 的数据说明](README.md#实机配置必须填写的数据)，不引用 Iris 代替实测数据。
- 用最终飞控 target 构建、备份配置、由操作员刷写，并回读实际 PID/rate profile、AUX、mask/failsafe、
  `msp_override_timeout_ms=50`；测量停发后实际释放旧输入的时间。固件默认值仍是 300 ms。
- 在 CM5 无 Gazebo 环境重建，使用真实 IMU/GPS 测量频率、时间偏差、初始化和失流行为，
  统计完整控制周期、求解、串口更新和回退时间；不能用本机结果替代。
- 确认磁罗盘安装/校准/磁偏角和高度基准，完成影子记录及人工起飞—AUTO 悬停—人工接回的逐级验证。
- 因 ROS 自定义消息改变，开发机、CM5、Jetson 等参与同一 ROS 图的主机必须全部重编。

这份记录证明列出的软件检查结果，不构成首飞完成或已具备实机飞行性能的结论。
