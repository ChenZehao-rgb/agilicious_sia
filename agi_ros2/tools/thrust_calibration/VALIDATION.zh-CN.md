# 实现及软件验证记录（2026-10-29）

工作树：`betaflight-bridge`，开始时 HEAD `030b2aa0b12aaa75577b7f3e2787612a393e7f69`。开始时未提交的 `agi_ros2/THRUST_CALIBRATION_VOLTAGE_PLAN.zh-CN.md` 保持原样；已有 `log_io.py` 在保留适配器的基础上补齐必要的只读输出、整数时间、ULog损坏及 decoder 选项检查。未 reset/clean/stash、提交、刷机或部署。没有连接真实串口，没有电机操作、MSP扫描器或UART传感器移植。

## 文件与行为变化

| 文件/目录 | 之前 | 本次修改及原因 |
|---|---|---|
| `tools/thrust_calibration/log_io.py` | 已有未提交只读适配器 | 保留功能，增加设备/管道拒绝、整数时间保存、ULog损坏拒绝、Blackbox log index检查及输出目录约束，避免写原日志目录；保留decoder hash/帮助/统计，版本未知时明确标记 |
| `semantics.py`, `inspection.py` | 无离线字段契约 | 显式时间/电压/力单位、RX range逆变换、限幅/有效性拒绝、topic/instance枚举、配对及跨split源区间防泄漏 |
| `alignment.py`, `steady_windows.py` | 无双时钟稳态配对 | 带留出共同锚点的稳健仿射映射、原时钟和响应延迟边界、原生采样统计、过渡/gap/dropout/不稳定拒绝、对齐扰动与前后零点 |
| `fit_reference.py`, `analyze_thrust_logs.py` | 无新版建表流水线 | 分块p辨识、RC局部偏差控制、跨压共同域、bootstrap/分箱/p区间检查、候选模型比较、最终测试、PAVA平台、单电机x4一次、完整产物/图表/CLI |
| `reference_thrust_model.h` | 仅旧table/quadratic | 新独立加载器/广义反函数/总N电压补偿；真实报告身份滤波与age校验；synthetic/unvalidated生产拒绝 |
| `command_output_node.h/.cpp` | 旧模型和电压NaN门禁 | 接入reference模型，保留总N与角速度链路、旧模型、SafetyGate；watchdog独立检验参考电压，shadow只计算诊断；FC可观测重置使滤波失效 |
| `Health.msg`, `MspState.msg` | 只有缓存电压/部分时间 | 增加电压会话、序号、原request/receive monotonic时间，心跳不改变样本身份 |
| `shadow_support.py`, `msp_evidence.py` | 解码电压但Health缺报告身份 | 保存并传递MSP130真实新报告及本地clock epoch；未调整轮询或超时 |
| `OutputStatus.msg` | 模型来源及旧诊断 | 增加schema/calibration/provenance、raw/filtered/age、Vref/p/scale、目标/参考N、取整RC、有效域及拒绝原因 |
| `launch/reference_calibration.py`, `runtime_launch.py` | 只校验旧模型 | Python契约与C++相同字段/域/平台/生产来源规则；直接节点同样拒绝无效配置 |
| `config/reference_table.example.yaml` | 无新模型示例 | 明确不可直接启用的片段，路径null、shadow=true；生产hardware.yaml未修改 |
| `CMakeLists.txt`, `package.xml` | 无yaml-cpp加载依赖 | 显式依赖/链接yaml-cpp，安装离线工具；其余构建路径不变 |
| `tests/*`, `test_reference_runtime.py` | 无新版测试 | 合成多日志、ULog二进制多实例、Python/C++契约及真实节点启动前拒绝测试 |
| `test_shadow_support.py`, `test_runtime_config.py`, `.gitignore` | 旧电压和两模型断言 | 增加样本身份/clock reset测试，更新三模型错误文本断言；只为新增runtime测试添加ignore例外 |
| `MANUAL_TEST.zh-CN.md`, 示例yaml/README | 旧自动方案不足以执行新版 | 落实手动预检、有效配置、同步标记、升降/换序/跨电压重复、独立留出和归档；未知实测参数仍为null/unknown |

`msp_telemetry.cpp` 不需修改：现有 MspEvent 已带原 request/receive 时间与会话。本次复用，不重造时间。控制器、MPC、生产SafetyGate及真实硬件配置未改。

## 已执行验证

日志目录：仓库根目录 `build/thrust_reference_validation/`（构建产物，未提交）。

| 检查 | 结果 | 日志 |
|---|---|---|
| 常规 `agi_ros2/scripts/build.sh`，独立build/install，Release，Gazebo关闭 | 完成，包含消息生成及C++节点；最后一次配置/安装3.63s，前次完整变更构建54.2s | `build.log` |
| 离线流程及真实C++加载/映射/滤波器测试 | 30项通过 | `offline_tests.log` |
| 最终导出文件Python/C++逐点比对 | 同一YAML/CSV的30个总N映射一致，生产拒绝该合成产物 | `export_cpp_parity.log` |
| reference launch校验及直接C++节点生产拒绝 | 1项通过，无真实UART | `reference_runtime.log` |
| MSP evidence | 23项通过 | `msp_evidence.log` |
| shadow_support，包括报告身份/重置 | 29项通过 | `shadow_support.log` |
| FC回归 `ctest` | `fc_fused_test`通过，实际执行 | `ctest.log` |
| 旧quadratic | anchors/inverse/rounding/invalid inputs通过 | `quadratic.log` |
| 旧table及MSP/safety/伪终端测试 | 通过；使用PTY，无物理串口 | `legacy_table.log` |
| 原角速度RC mapper | 三轴ACTUAL/expo/deadband/NaN通过 | `rc_mapper.log` |
| 旧runtime配置测试 | 29项中5项失败；未修改HEAD版本同样5项失败 | `runtime_config.log`, `runtime_config_baseline.log` |
| clang-format 23.1.3 +差异/行长检查 | 新C++全文件、现有文件本次范围、Tab展开140列、新增行缩进与`git diff --check`通过；命名人工检查，未运行clang-tidy | `style.json` |

5项既存runtime测试失败名称：`test_geo_hardware_still_rejects_missing_mass_limits_or_thrust_table`、`test_hardware_defaults_to_estimate_without_changing_controller_limits_or_shadow`、`test_hardware_placeholders_reject_control_but_allow_diagnostics`、`test_simulation_matches_existing_model_controller_and_fc_settings`、`test_simulation_remains_three_independent_core_nodes`。涉及当前profile相对历史断言的质量/推力限值/默认轨迹差异。本次未为让这些断言通过而更改飞行配置。

## 合成端到端产物

本地报告：[`../../test/results/thrust_reference_20261029/report.html`](../../test/results/thrust_reference_20261029/report.html)。仓库实际路径为 `agi_ros2/test/results/thrust_reference_20261029/report.html`。目录按现有规则被gitignore，保留为本地可审查产物；生成器及测试源码可重新生成。

- 15段合成BF/PX4配对，345个稳态窗口，207个训练窗口。
- 真实生成参数p=1.6；估计p=1.6002383698；块bootstrap 95%区间[1.59950825,1.60111931]。
- 模型选择为p_fit。对齐±不确定度扰动所得p分别1.60025192/1.60024194。
- 合成有效时钟映射a=1.00008，b=9.245s（含生成的响应延迟及边缘定位，不能称纯时钟同步）。
- 域内最终测试37个窗口：平均相对误差0.171824%，最大0.619243%。这些数值仅验证合成软件链，不能代表真实动力误差。
- 生成CSV与calibration.yaml经Python加载器检查，C++契约测试使用相同格式；全部仍为 `provenance: synthetic`, `calibration_status: unvalidated`, `flight_validation_status: not_performed`。生产模型不接受这些表。
- 图包括两边原时钟对齐前后、四电机/所选稳态窗口、原始及归一化散点、PAVA前后、候选p验证误差与最终残差；已查看生成图。

复现：README提供完整命令；本次生成原始合成CSV位于 `build/thrust_reference_validation/synthetic/`，六个阶段输出检查见 `pipeline.log`。`decode`由模拟decoder argv/统计测试覆盖，ULog由实际pyulog读取合成二进制覆盖，不声称真实定制Blackbox decoder兼容。

## 资格边界

实现和上述软件测试已完成。真实 force topic/instance/字段、单位/静态载荷标定、有效RX range、固件/配置匹配、实际同步及稳态质量、真实p/有效域/误差仍待用户真实日志。对齐要求经过审查的共同边缘；不自动猜周期信号的唯一对应。处理后Q轴的完整非线性稳态反解未启用，缺原始轴证据时保持unresolved，不造表。

单电机乘4的一致性、Blackbox与生产MSP电压的动态差异、真实运行时补偿及飞行验收均未验证。所有未知项保留配置缺口。新消息已在独立安装中构建，原有生产安装未覆盖；后续实际部署必须统一重建消息接口。
