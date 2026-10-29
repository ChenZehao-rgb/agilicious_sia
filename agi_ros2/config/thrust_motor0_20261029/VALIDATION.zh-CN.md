# 本次实施与验证（2026-10-29）

已按真实四份日志完成解码、motor[0]拟合、目标profile转换和运行时接入。基础 `hardware.yaml` 未修改；生成的 trial profile 只改变推力模型/标定路径，保留原 shadow 和控制/授权/接管参数。工作开始时已有的未提交 reference_table 工作被保留并增量完善，未提交、推送或部署。

## 本次新增及完善

- `agi_ros2/tools/thrust_calibration/motor0_trial.py`：真实 motor 轴分析入口，复用 `log_io.py` / `fit_reference.pava`。包括 inspect、fit、export，保留原时钟和数据hash，拒绝跨gap核心，分段线性单调表、p对比、基线敏感性和源文件记录。
- `agi_ros2/config/thrust_motor0_20261029/`：真实数据生成的16点motor表、所选600行RC表、static635行对照、YAML、窗口统计、报告及图；不是合成fixture。
- `agi_ros2/config/hardware_reference_trial.yaml`：由当前基础profile派生，唯一选定 `dynamic_nominal`，保留 `shadow_only: true`。
- `agi_ros2/launch/reference_calibration.py`、`agilib/include/agilib/bridge/betaflight/reference_thrust_model.h`：Python/C++加载相同provisional契约；不要求该名义映射声称 `axis_verified=true`，也不以缺少独立验收拒绝加载。本次真实来源允许使用；旧synthetic仍不能冒充真实表。
- `agi_ros2/src/command_output_node.cpp`、`agi_ros2/msg/OutputStatus.msg`：已有总N电压补偿接入上增加0N idle、正低端饱和和来源/status诊断；高端/电压/过期仍拒绝。未改角速度映射、SafetyGate、MPC限幅或授权条件。
- `tests/test_motor0_trial.py`、既有 `reference_model_probe.cpp` / `test_offline.py`、`agi_ros2/test/test_reference_runtime.py`：真实产物两端一致性、诊断flags、目标单独重导出、年龄/滤波、原有授权配置一致性及边界回归。

## 实际执行结果

日志在仓库 `build/motor0_trial/`，隔离构建在 `build/thrust_reference/`，隔离安装在 `build/thrust_reference_validation/install/`。没有覆盖原 `install/agi_ros2`。

| 检查 | 结果 |
|---|---|
| 四原始日志SHA256，处理前/后 | 与任务说明完全一致，原文件未改 |
| 官方decoder，两日志 | 74848 / 45028行，坏帧0；run1约30ms缺口已排除 |
| ROS2 Release/Gazebo OFF，消息和C++节点 | 构建通过 |
| 离线与纯C++模型测试 | 39项通过；包含真实YAML/CSV、正低端/0N/高端、p0、平台反解、单位和一次乘4 |
| reference启动配置/直接节点拒绝synthetic | 2项通过；无物理UART |
| MSP evidence | 23项通过 |
| shadow_support | 29项通过 |
| FC ctest | 1项通过 |
| 旧quadratic / ACTUAL RC mapper / table+MSP+safety | 3个既有离线probe通过，transport测试仅用伪终端 |
| 通用runtime配置套件 | 29项中24通过、5失败；本次用HEAD版本的测试与launch代码复跑，同样5项失败 |
| clang-format / Tab展开140列 / git diff --check | 新C++文件完整检查和现有文件修改范围检查通过；命名人工检查，未运行clang-tidy |
| 生成曲线图 | 已实际查看，无裁切或单位/轴域混淆 |

5项既有失败：

- `test_geo_hardware_still_rejects_missing_mass_limits_or_thrust_table`
- `test_hardware_defaults_to_estimate_without_changing_controller_limits_or_shadow`
- `test_hardware_placeholders_reject_control_but_allow_diagnostics`
- `test_simulation_matches_existing_model_controller_and_fc_settings`
- `test_simulation_remains_three_independent_core_nodes`

这些失败涉及当前硬件/仿真profile中的质量、推力限值和默认轨迹与历史断言不一致。本次没有为通过旧断言而修改这些飞行参数。对比日志是 `runtime_config.log` / `runtime_config_head_baseline.log`。

解码器仓库：https://github.com/betaflight/blackbox-tools 。本机源码commit和二进制hash已写入 `decode_run1.json` / `decode_run2.json`。第一次误用make隐式target导致链接失败，随后用项目真实 `./obj/blackbox_decode` target成功构建；未改decoder源码。

所有飞控/传感器硬件调用均未运行。shadow/正常硬件profile启动命令仅写入README供后续用户选择。实际飞行、四电机一致性、动态怠速增量、非零PID力矩及快速电压瞬态仍未验证；这不妨碍本次工程初版正常加载和选择。
